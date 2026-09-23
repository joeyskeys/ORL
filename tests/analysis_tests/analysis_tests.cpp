#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_analysis.h"
#include "orl_graph_import.h"
#include "orl_parser.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <vector>

#if defined(ORL_HAS_GRAPH_IO)
#include "graph_serialization.hpp"
#endif

using namespace orlcomp;

TEST_CASE("semantic analysis imports a typed ORL function", "[orl][analysis]") {
    Parser parser(R"(
        export int add(int left, int right) {
            return left + right;
        }
    )");
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);

    SemanticAnalyzer analyzer;
    const AnalysisResult analysis = analyzer.analyze(*parser.Ast(), "analysis_test.orl");
    REQUIRE(analysis.ok());
    const auto* function = analysis.function("add");
    REQUIRE(function != nullptr);
    REQUIRE(function->parameters.size() == 2);
    REQUIRE(function->parameters[0].logical_type == orlgraph::LogicalType::int64());
    REQUIRE(function->pure);

    const auto imported = import_node_definitions(analysis, "analysis_test");
    REQUIRE(imported.ok());
    const auto* definition = imported.registry.find("analysis_test.add");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->inputs.size() == 2);
    REQUIRE(definition->outputs.size() == 1);
}

TEST_CASE("semantic analysis permits read-only solver context",
    "[orl][analysis][solver_context]")
{
    Parser parser(R"(
        export int solve() {
            return solver_context.joint_count;
        }
    )");
    REQUIRE(parser.Parse());

    SemanticAnalyzer analyzer;
    const auto analysis = analyzer.analyze(*parser.Ast());
    REQUIRE(analysis.ok());
}

TEST_CASE("semantic analysis rejects solver context writes",
    "[orl][analysis][solver_context][error]")
{
    Parser parser(R"(
        int invalid() {
            solver_context.joint_count = 1;
            return 0;
        }
    )");
    REQUIRE(parser.Parse());

    SemanticAnalyzer analyzer;
    const auto analysis = analyzer.analyze(*parser.Ast());
    REQUIRE_FALSE(analysis.ok());
    REQUIRE(std::any_of(analysis.diagnostics.begin(),
        analysis.diagnostics.end(),
        [](const AnalysisDiagnostic& diagnostic) {
            return diagnostic.code == "ORL_ANALYSIS_CONTEXT_WRITE";
        }));
}

TEST_CASE("semantic analysis rejects solver context shadowing",
    "[orl][analysis][solver_context][error]")
{
    Parser parser(R"(
        int invalid() {
            int solver_context = 1;
            return solver_context;
        }
    )");
    REQUIRE(parser.Parse());

    SemanticAnalyzer analyzer;
    const auto analysis = analyzer.analyze(*parser.Ast());
    REQUIRE_FALSE(analysis.ok());
    REQUIRE(std::any_of(analysis.diagnostics.begin(),
        analysis.diagnostics.end(),
        [](const AnalysisDiagnostic& diagnostic) {
            return diagnostic.code == "ORL_ANALYSIS_RESERVED_NAME";
        }));
}

TEST_CASE("external ORL source registers node definitions at runtime",
    "[orl][analysis][runtime]")
{
    orlgraph::NodeRegistry registry;
    NodeImportOptions options;
    options.module_name = "external_nodes";
    options.source_name = "external_nodes.orl";
    options.exported_functions.push_back("add_external");

    const auto compiled = compile_node_definitions_to_oro(R"(
        int add_helper(int value) {
            return value + 1;
        }

        export int add_external(int left, int right) {
            return add_helper(left) + right - 1;
        }
    )", options);
    REQUIRE(compiled.ok());
    const auto result = register_orl_node_definitions_from_oro(
        registry, compiled.text);
    REQUIRE(result.ok());
    REQUIRE(result.registered_count == 1);

    const auto* definition = registry.find("external_nodes.add_external");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->implementation.kind
        == orlgraph::ImplementationKind::OrlFunction);
    REQUIRE(definition->implementation.module == "external_nodes");
    REQUIRE(definition->implementation.function == "add_external");

    const auto duplicate = register_orl_node_definitions_from_oro(
        registry, compiled.text);
    REQUIRE_FALSE(duplicate.ok());
    REQUIRE(duplicate.registered_count == 0);
    REQUIRE(registry.find("external_nodes.add_external") != nullptr);
}

TEST_CASE("only exported functions become stage-aware node definitions",
    "[orl][analysis][export]")
{
    const auto imported = import_node_definitions(R"(
        export int solve [[ string stage = "solver" ]] (int value) {
            return value;
        }

        export int deform [[ string stage = "deformer" ]] (int value) {
            return value;
        }

        int helper(int value) {
            return value + 1;
        }
    )", NodeImportOptions{.module_name = "stage_nodes"});

    REQUIRE(imported.ok());
    REQUIRE(imported.registry.find("stage_nodes.solve") != nullptr);
    REQUIRE(imported.registry.find("stage_nodes.deform") != nullptr);
    REQUIRE(imported.registry.find("stage_nodes.helper") == nullptr);

    const auto* solve = imported.registry.find("stage_nodes.solve");
    REQUIRE(solve->allowed_stages == orlgraph::GraphStageMask::Solver);
    REQUIRE(solve->metadata.contains("stage"));
    REQUIRE(std::get<std::string>(
        solve->metadata.at("stage").value) == "solver");
    REQUIRE(imported.registry.is_available(
        solve->id, orlgraph::GraphStage::Solver));
    REQUIRE_FALSE(imported.registry.is_available(
        solve->id, orlgraph::GraphStage::Deformer));

    const auto solver_nodes = imported.registry.definitions_for(
        orlgraph::GraphStage::Solver);
    const auto deformer_nodes = imported.registry.definitions_for(
        orlgraph::GraphStage::Deformer);
    REQUIRE(std::any_of(solver_nodes.begin(), solver_nodes.end(),
        [solve](const auto* definition) {
            return definition->id == solve->id;
        }));
    const auto* deform = imported.registry.find("stage_nodes.deform");
    REQUIRE(std::none_of(solver_nodes.begin(), solver_nodes.end(),
        [deform](const auto* definition) {
            return definition->id == deform->id;
        }));
    REQUIRE(std::any_of(deformer_nodes.begin(), deformer_nodes.end(),
        [deform](const auto* definition) {
            return definition->id == deform->id;
        }));
    REQUIRE(std::none_of(deformer_nodes.begin(), deformer_nodes.end(),
        [solve](const auto* definition) {
            return definition->id == solve->id;
        }));
}

TEST_CASE("export metadata imports a partial evaluation footprint",
    "[orl][analysis][partial]")
{
    const auto imported = import_node_definitions(R"(
        export int solve [[
            string stage = "solver",
            string partial_propagation = "descendants",
            string partial_read_locator_ports = "target_index,pole_index",
            string partial_write_locator_ports = "subject_index",
            string partial_write_locators = "locator.target",
            int partial_sparse = 1
        ]] (int value) {
            return value;
        }
    )", NodeImportOptions{.module_name = "partial_nodes"});

    REQUIRE(imported.ok());
    const auto* solve = imported.registry.find("partial_nodes.solve");
    REQUIRE(solve != nullptr);
    REQUIRE(solve->partial_footprint.has_value());
    REQUIRE(solve->partial_footprint->declared);
    REQUIRE_FALSE(solve->partial_footprint->global);
    REQUIRE(solve->partial_footprint->supports_sparse_dispatch);
    REQUIRE(solve->partial_footprint->propagation
        == orlgraph::PartialPropagation::Descendants);
    REQUIRE(solve->partial_footprint->read_locator_ports
        == std::vector<std::string>{"target_index", "pole_index"});
    REQUIRE(solve->partial_footprint->write_locator_ports
        == std::vector<std::string>{"subject_index"});
    REQUIRE(solve->partial_footprint->write_locators
        == std::vector<orlgraph::StableId>{
            orlgraph::StableId{"locator.target"}});
}

TEST_CASE("semantic analysis rejects unresolved calls and unsupported types",
    "[orl][analysis][error]")
{
    Parser parser(R"(
        vec2 invalid(vec2 value) {
            return missing_function(value);
        }
    )");
    REQUIRE(parser.Parse());

    SemanticAnalyzer analyzer;
    const AnalysisResult analysis = analyzer.analyze(*parser.Ast());
    REQUIRE_FALSE(analysis.ok());
    REQUIRE_FALSE(analysis.diagnostics.empty());
    REQUIRE(std::any_of(analysis.diagnostics.begin(), analysis.diagnostics.end(),
        [](const AnalysisDiagnostic& diagnostic) {
            return diagnostic.code == "ORL_ANALYSIS_UNSUPPORTED_TYPE";
        }));
    REQUIRE(std::any_of(analysis.diagnostics.begin(), analysis.diagnostics.end(),
        [](const AnalysisDiagnostic& diagnostic) {
            return diagnostic.code == "ORL_ANALYSIS_UNKNOWN_CALL";
        }));
}

TEST_CASE("semantic analysis records buffer reads and writes", "[orl][analysis][effects]") {
    Parser parser(R"(
        int update(int input[], int output[], int count) {
            int index = 0;
            while (index < count) {
                output[index] = input[index];
                index = index + 1;
            }
            return count;
        }
    )");
    REQUIRE(parser.Parse());

    SemanticAnalyzer analyzer;
    const AnalysisResult analysis = analyzer.analyze(*parser.Ast());
    REQUIRE(analysis.ok());
    const auto* function = analysis.function("update");
    REQUIRE(function != nullptr);
    REQUIRE_FALSE(function->pure);
    REQUIRE(function->has_loop);
    REQUIRE(function->parameters[0].access == ParameterAccess::Read);
    REQUIRE(function->parameters[1].access == ParameterAccess::Write);
}

#if defined(ORL_HAS_GRAPH_IO)
TEST_CASE("compiled oro documents provide node definitions",
    "[orl][analysis][oro]")
{
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{"custom.parent"};
    definition.qualified_name = "custom.parent";
    definition.implementation.kind = orlgraph::ImplementationKind::OrlFunction;
    definition.implementation.module = "constraint/parent";
    definition.implementation.function = "constraint_parent";
    definition.allowed_stages = orlgraph::GraphStageMask::Solver;

    orlgraph::Port source;
    source.id = orlgraph::StableId{"source"};
    source.name = "source";
    source.cardinality = orlgraph::PortCardinality::Buffer;
    source.type = orlgraph::LogicalType::buffer(orlgraph::LogicalType::matrix());
    source.domain = orlgraph::Domain::buffer();
    source.shape = orlgraph::Shape::one("source_count");
    definition.inputs.push_back(source);

    orlgraph::Port offset;
    offset.id = orlgraph::StableId{"offset"};
    offset.name = "offset";
    offset.type = orlgraph::LogicalType::matrix();
    definition.inputs.push_back(offset);

    orlgraph::Port status;
    status.id = orlgraph::StableId{"status"};
    status.name = "status";
    status.direction = orlgraph::PortDirection::Output;
    status.type = orlgraph::LogicalType::int64();
    status.required = false;
    definition.outputs.push_back(status);

    orlgraph::NodeRegistry compiled;
    REQUIRE(compiled.register_definition(definition));
    orlgraph::GraphModule module;
    module.module_id = "custom.oro";
    const auto serialized = orlgraph::serialize_oro(module, compiled);
    REQUIRE(serialized.ok);

    const auto imported = import_node_definitions_from_oro(serialized.text);
    REQUIRE(imported.ok());
    const auto* loaded = imported.registry.find("custom.parent");
    REQUIRE(loaded != nullptr);
    REQUIRE(loaded->implementation.module == "constraint/parent");
    REQUIRE(loaded->implementation.function == "constraint_parent");
    REQUIRE(loaded->inputs.size() == 2);
    REQUIRE(loaded->inputs[0].id == orlgraph::StableId{"source"});
    REQUIRE(loaded->inputs[0].shape == orlgraph::Shape::one("source_count"));
    REQUIRE(loaded->inputs[1].name == "offset");
    REQUIRE(loaded->outputs.size() == 1);
    REQUIRE(loaded->outputs[0].name == "status");

    orlgraph::NodeRegistry registry;
    const auto registered = register_orl_node_definitions_from_oro(
        registry, serialized.text);
    REQUIRE(registered.ok());
    REQUIRE(registered.registered_count == 1);
    REQUIRE(registry.find("custom.parent") != nullptr);

    const auto rejected = import_node_definitions_from_oro("{not an oro document}");
    REQUIRE_FALSE(rejected.ok());

    const auto path = std::filesystem::temp_directory_path()
        / "orl_custom_parent.oro";
    {
        std::ofstream output(path, std::ios::binary);
        REQUIRE(output);
        output << serialized.text;
    }
    const auto from_file = import_node_definitions_from_oro_file(path.string());
    REQUIRE(from_file.ok());
    REQUIRE(from_file.registry.find("custom.parent") != nullptr);
    std::filesystem::remove(path);
}
#endif

#endif
