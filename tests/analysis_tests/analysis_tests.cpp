#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_analysis.h"
#include "orl_graph_import.h"
#include "orl_parser.h"

#include <algorithm>

using namespace orlcomp;

TEST_CASE("semantic analysis imports a typed ORL function", "[orl][analysis]") {
    Parser parser(R"(
        int add(int left, int right) {
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

TEST_CASE("external ORL source registers node definitions at runtime",
    "[orl][analysis][runtime]")
{
    orlgraph::NodeRegistry registry;
    NodeImportOptions options;
    options.module_name = "external_nodes";
    options.source_name = "external_nodes.orl";
    options.exported_functions.push_back("add_external");

    const auto result = register_orl_node_definitions(registry, R"(
        int add_helper(int value) {
            return value + 1;
        }

        int add_external(int left, int right) {
            return add_helper(left) + right - 1;
        }
    )", options);
    REQUIRE(result.ok());
    REQUIRE(result.registered_count == 1);

    const auto* definition = registry.find("external_nodes.add_external");
    REQUIRE(definition != nullptr);
    REQUIRE(definition->implementation.kind
        == orlgraph::ImplementationKind::OrlFunction);
    REQUIRE(definition->implementation.module == "external_nodes");
    REQUIRE(definition->implementation.function == "add_external");

    const auto duplicate = register_orl_node_definitions(registry, R"(
        int add_external(int value) {
            return value;
        }
    )", options);
    REQUIRE_FALSE(duplicate.ok());
    REQUIRE(duplicate.registered_count == 0);
    REQUIRE(registry.find("external_nodes.add_external") != nullptr);
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

#endif
