#include "graph_resources.hpp"

#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace orlrig
{

using orlgraph::StableId;

namespace
{

std::string scene_binding(std::string_view prefix,
    std::string_view name, std::string_view suffix)
{
    std::string result{prefix};
    if (!name.empty()) {
        result += ".";
        result += name;
    }
    if (!suffix.empty()) {
        result += ".";
        result += suffix;
    }
    return result;
}

orlgraph::Port buffer_port(std::string id, std::string name,
    orlgraph::LogicalType element, orlgraph::PortDirection direction,
    orlgraph::Domain domain = orlgraph::Domain::buffer(), bool required = true)
{
    orlgraph::Port port;
    port.id = orlgraph::StableId{std::move(id)};
    port.name = std::move(name);
    port.direction = direction;
    port.cardinality = orlgraph::PortCardinality::Buffer;
    port.type = orlgraph::LogicalType::buffer(std::move(element));
    port.domain = domain;
    port.shape = orlgraph::Shape::one("vertex_count");
    port.required = required;
    return port;
}

orlgraph::NodeDefinition make_capture_definition(
    const RigGraphResourceIds& resources)
{
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{"orlrig.deformer.lbs.capture_bind"};
    definition.qualified_name = "orlrig.deformer.lbs.capture_bind";
    definition.allowed_stages = orlgraph::GraphStageMask::Deformer;
    definition.implementation.kind = orlgraph::ImplementationKind::Runtime;
    definition.implementation.runtime_name = definition.qualified_name;
    definition.outputs.push_back(buffer_port(
        "inverse_binds", "inverse_binds", orlgraph::LogicalType::matrix(),
        orlgraph::PortDirection::Output, orlgraph::Domain::buffer(), false));
    definition.outputs.front().shape = orlgraph::Shape::one("joint_count");
    definition.effects = {
        {resources.joints, orlgraph::AccessMode::Read, false},
        {resources.inverse_binds, orlgraph::AccessMode::Write, true},
    };
    definition.pure = false;
    definition.inline_policy = orlgraph::InlinePolicy::Never;
    return definition;
}

orlgraph::NodeDefinition make_deform_definition(
    const RigGraphResourceIds& resources)
{
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{"orlrig.deformer.lbs.evaluate"};
    definition.qualified_name = "orlrig.deformer.lbs.evaluate";
    definition.allowed_stages = orlgraph::GraphStageMask::Deformer;
    definition.implementation.kind = orlgraph::ImplementationKind::Runtime;
    definition.implementation.runtime_name = definition.qualified_name;
    definition.inputs = {
        buffer_port("bind_positions", "bind_positions",
            orlgraph::LogicalType::point(), orlgraph::PortDirection::Input),
        buffer_port("inverse_binds", "inverse_binds",
            orlgraph::LogicalType::matrix(), orlgraph::PortDirection::Input),
        buffer_port("weights", "weights", orlgraph::LogicalType::struct_type("Weight"),
            orlgraph::PortDirection::Input),
    };
    definition.outputs.push_back(buffer_port(
        "posed_positions", "posed_positions", orlgraph::LogicalType::point(),
        orlgraph::PortDirection::Output, orlgraph::Domain::buffer(), false));
    definition.inputs[1].shape = orlgraph::Shape::one("joint_count");
    definition.inputs[2].shape = orlgraph::Shape::one("weight_count");
    definition.effects = {
        {resources.bind_positions, orlgraph::AccessMode::Read, false},
        {resources.joints, orlgraph::AccessMode::Read, false},
        {resources.inverse_binds, orlgraph::AccessMode::Read, false},
        {resources.weights, orlgraph::AccessMode::Read, false},
        {resources.posed_positions, orlgraph::AccessMode::Write, true},
    };
    definition.pure = false;
    definition.inline_policy = orlgraph::InlinePolicy::Never;
    return definition;
}

orlgraph::NodeDefinition make_computed_joints_definition() {
    const std::string qualified_name{kComputedJointsNodeDefinition};
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{qualified_name};
    definition.qualified_name = qualified_name;
    definition.allowed_stages = orlgraph::GraphStageMask::Deformer;
    definition.implementation.kind = orlgraph::ImplementationKind::Runtime;
    definition.implementation.runtime_name = qualified_name;
    definition.operation = "computed_joints";
    definition.capabilities = {"stage", "cpu", "cuda"};
    definition.pure = false;
    definition.inline_policy = orlgraph::InlinePolicy::Never;

    auto joints = buffer_port(
        "joints", "joints",
        orlgraph::LogicalType::struct_type("Joint"),
        orlgraph::PortDirection::Output,
        orlgraph::Domain::joint(), false);
    joints.shape = orlgraph::Shape::one("joint_count");
    joints.semantic = "joints";
    definition.outputs.push_back(std::move(joints));
    return definition;
}

orlgraph::Port stdlib_buffer_port(std::string id,
    orlgraph::LogicalType element, std::string shape)
{
    orlgraph::Port port = buffer_port(id, id, std::move(element),
        orlgraph::PortDirection::Input);
    port.shape = orlgraph::Shape::one(std::move(shape));
    port.semantic = port.name;
    if (port.name == "joints") {
        port.domain = orlgraph::Domain::joint();
    }
    return port;
}

orlgraph::Port stdlib_scalar_port(std::string name, orlgraph::LogicalType type) {
    orlgraph::Port port;
    port.id = orlgraph::StableId{name};
    port.name = std::move(name);
    port.direction = orlgraph::PortDirection::Input;
    port.cardinality = orlgraph::PortCardinality::Scalar;
    port.type = std::move(type);
    port.domain = orlgraph::Domain::constant();
    port.shape = orlgraph::Shape::scalar();
    port.semantic = port.name;
    return port;
}

orlgraph::Port stdlib_status_port() {
    auto port = stdlib_scalar_port("status", orlgraph::LogicalType::int64());
    port.direction = orlgraph::PortDirection::Output;
    port.required = false;
    port.semantic = "status";
    return port;
}

orlgraph::ConversionDefinition make_joint_world_matrix_conversion() {
    orlgraph::ConversionDefinition definition;
    definition.id = orlgraph::StableId{
        std::string{kJointWorldMatrixConversion}};
    definition.qualified_name = definition.id.value;
    definition.source = stdlib_scalar_port(
        "index", orlgraph::LogicalType::int64());
    definition.source.direction = orlgraph::PortDirection::Output;
    definition.source.required = false;
    definition.source.semantic = std::string{kSceneArrayIndexSemantic};

    definition.output = buffer_port(
        "xform", "xform", orlgraph::LogicalType::matrix(),
        orlgraph::PortDirection::Output, orlgraph::Domain::buffer(), false);
    definition.output.shape = orlgraph::Shape::one("one");
    definition.output.semantic = "scene.joint_xform";
    definition.output.coordinate_space = "world";

    auto joints = buffer_port(
        "joints", "joints",
        orlgraph::LogicalType::struct_type("Joint"),
        orlgraph::PortDirection::Input, orlgraph::Domain::joint());
    joints.shape = orlgraph::Shape::one("joint_count");
    joints.semantic = std::string{kSceneJointsBinding};
    definition.auxiliary_inputs.push_back(std::move(joints));

    definition.implementation.kind = orlgraph::ImplementationKind::OrlFunction;
    definition.implementation.module = "joint";
    definition.implementation.function = "joint_world_matrix";
    definition.emitter = orlgraph::ConversionEmitterKind::OrlMatrixBuffer;
    return definition;
}

orlgraph::ConversionDefinition make_joint_world_matrix_writeback_conversion() {
    orlgraph::ConversionDefinition definition;
    definition.id = orlgraph::StableId{
        std::string{kJointWorldMatrixWritebackConversion}};
    definition.qualified_name = definition.id.value;

    definition.source = buffer_port(
        "xform", "xform", orlgraph::LogicalType::matrix(),
        orlgraph::PortDirection::Input, orlgraph::Domain::buffer(), true);
    definition.source.shape = orlgraph::Shape::one("one");

    definition.output = buffer_port(
        "joints", "joints",
        orlgraph::LogicalType::struct_type("Joint"),
        orlgraph::PortDirection::Input, orlgraph::Domain::joint(), true);
    definition.output.shape = orlgraph::Shape::one("joint_count");
    definition.output.semantic = std::string{kSceneJointsBinding};

    auto selector = stdlib_scalar_port(
        "handle", orlgraph::LogicalType::int64());
    selector.direction = orlgraph::PortDirection::Output;
    selector.required = false;
    selector.semantic = std::string{kSceneJointHandleSemantic};
    definition.selector = std::move(selector);

    definition.implementation.kind = orlgraph::ImplementationKind::OrlFunction;
    definition.implementation.module = "joint";
    definition.implementation.function = "joint_write_world_matrix";
    definition.emitter = orlgraph::ConversionEmitterKind::OrlFunctionWriteback;
    definition.pure = false;
    return definition;
}

orlgraph::NodeDefinition make_find_definition(
    std::string element, std::string handle_semantic)
{
    const std::string qualified_name = "orlrig.input.find_" + element;
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{qualified_name};
    definition.qualified_name = qualified_name;
    definition.allowed_stages = orlgraph::GraphStageMask::All;
    definition.implementation.kind = orlgraph::ImplementationKind::Runtime;
    definition.implementation.runtime_name = qualified_name;
    definition.parameters.push_back({
        orlgraph::StableId{"name"}, "name",
        orlgraph::LogicalType::string(), true, std::nullopt,
        "scene.element_name"});

    auto handle = stdlib_scalar_port("handle", orlgraph::LogicalType::int64());
    handle.direction = orlgraph::PortDirection::Output;
    handle.required = false;
    handle.semantic = std::move(handle_semantic);
    definition.outputs.push_back(std::move(handle));
    if (element == "joint" || element == "controller"
        || element == "locator")
    {
        auto index = stdlib_scalar_port(
            "index", orlgraph::LogicalType::int64());
        index.direction = orlgraph::PortDirection::Output;
        index.required = false;
        index.semantic = std::string{kSceneArrayIndexSemantic};
        definition.outputs.push_back(std::move(index));
        const bool locator = element == "locator";
        auto transform = buffer_port(
            "xform", "xform",
            locator
                ? orlgraph::LogicalType::struct_type("Locator")
                : orlgraph::LogicalType::matrix(),
            orlgraph::PortDirection::Output,
            orlgraph::Domain::buffer(), false);
        transform.shape = orlgraph::Shape::one("one");
        transform.semantic = element == "joint"
            ? "scene.joint_xform"
            : locator ? "scene.locator_xform" : "scene.controller_xform";
        transform.coordinate_space = "world";
        if (element == "joint") {
            transform.output_adapter = orlgraph::Port::OutputAdapter{
                orlgraph::StableId{std::string{kJointWorldMatrixConversion}},
                orlgraph::StableId{"index"},
                std::optional<orlgraph::StableId>{
                    orlgraph::StableId{
                        std::string{kJointWorldMatrixWritebackConversion}}},
                std::optional<orlgraph::StableId>{
                    orlgraph::StableId{"handle"}}};
        }
        definition.outputs.push_back(std::move(transform));
    }
    definition.capabilities = {"runtime", "scene"};
    definition.operation = "input";
    definition.pure = false;
    definition.inline_policy = orlgraph::InlinePolicy::Never;
    return definition;
}

std::vector<orlgraph::NodeDefinition> make_input_definitions() {
    return {
        make_find_definition("joint",
            std::string{kSceneJointHandleSemantic}),
        make_find_definition("controller",
            std::string{kSceneControllerHandleSemantic}),
        make_find_definition("locator",
            std::string{kSceneLocatorHandleSemantic}),
        make_find_definition("mesh", "scene.mesh.handle"),
    };
}

orlgraph::ResourceEffect parameter_effect(std::string_view function,
    std::string_view parameter, orlgraph::AccessMode access)
{
    std::string resource_name{function};
    resource_name += ".";
    resource_name += parameter;
    return {
        orlgraph::StableId::from("orl.parameter", resource_name),
        access,
        access != orlgraph::AccessMode::Read,
    };
}

std::vector<orlgraph::ResourceEffect> parameter_effects(
    std::string_view function,
    std::initializer_list<std::string_view> reads,
    std::initializer_list<std::string_view> writes,
    std::initializer_list<std::string_view> read_writes)
{
    std::vector<orlgraph::ResourceEffect> result;
    for (const auto parameter : reads) {
        result.push_back(parameter_effect(function, parameter,
            orlgraph::AccessMode::Read));
    }
    for (const auto parameter : writes) {
        result.push_back(parameter_effect(function, parameter,
            orlgraph::AccessMode::Write));
    }
    for (const auto parameter : read_writes) {
        result.push_back(parameter_effect(function, parameter,
            orlgraph::AccessMode::ReadWrite));
    }
    return result;
}

orlgraph::NodeDefinition make_stdlib_definition(std::string category,
    std::string function, std::vector<orlgraph::Port> inputs,
    std::vector<orlgraph::ResourceEffect> effects, bool cuda_capable,
    bool stateful = false)
{
    const std::string qualified_name =
        "orlrig." + category + "." + function;
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{qualified_name};
    definition.qualified_name = qualified_name;
    definition.allowed_stages =
        category == "solver" || category == "constraint"
            ? orlgraph::GraphStageMask::Solver
            : category == "deformer" || category == "auto_weight"
                ? orlgraph::GraphStageMask::Deformer
                : orlgraph::GraphStageMask::All;
    definition.implementation.kind = orlgraph::ImplementationKind::OrlFunction;
    definition.implementation.module = category + "/" + function;
    definition.implementation.function = category + "_" + function;
    definition.inputs = std::move(inputs);
    definition.outputs.push_back(stdlib_status_port());
    definition.effects = std::move(effects);
    definition.capabilities.push_back("cpu");
    if (cuda_capable) {
        definition.capabilities.push_back("cuda");
    }
    definition.inline_policy = orlgraph::InlinePolicy::Never;
    definition.operation = category == "solver" ? "solver"
        : category == "constraint" ? "constraint" : category;
    definition.pure = false;
    definition.stateful = stateful;
    if (category == "solver" && function == "ik_two_bone") {
        orlgraph::PartialEvaluationFootprint footprint;
        footprint.declared = true;
        footprint.propagation =
            orlgraph::PartialPropagation::Descendants;
        footprint.read_joint_ports = {"root", "mid", "end"};
        footprint.write_joint_ports = {"root", "mid"};
        footprint.read_locator_ports = {"target_index", "pole_index"};
        definition.partial_footprint = std::move(footprint);
    } else if (category == "solver") {
        // The remaining solvers currently read/write broad buffers or carry
        // iterative state. They are explicit conservative barriers until a
        // solver-specific footprint is available.
        orlgraph::PartialEvaluationFootprint footprint;
        footprint.declared = true;
        footprint.global = true;
        footprint.stateful = stateful;
        footprint.propagation = orlgraph::PartialPropagation::Full;
        definition.partial_footprint = std::move(footprint);
    }
    return definition;
}

std::vector<orlgraph::Port> closest_weight_inputs() {
    return {
        stdlib_buffer_port("positions", orlgraph::LogicalType::point(),
            "vertex_count"),
        stdlib_buffer_port("joints",
            orlgraph::LogicalType::struct_type("Joint"), "joint_count"),
        stdlib_buffer_port("weights",
            orlgraph::LogicalType::struct_type("Weight"), "weight_count"),
        stdlib_scalar_port("vertex_count", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("joint_count", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("weight_cnt", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("dropoff", orlgraph::LogicalType::float64()),
    };
}

std::vector<orlgraph::Port> envelope_weight_inputs() {
    return {
        stdlib_buffer_port("positions", orlgraph::LogicalType::point(),
            "vertex_count"),
        stdlib_buffer_port("joints",
            orlgraph::LogicalType::struct_type("Joint"), "joint_count"),
        stdlib_buffer_port("radii", orlgraph::LogicalType::float64(),
            "joint_count"),
        stdlib_buffer_port("weights",
            orlgraph::LogicalType::struct_type("Weight"), "weight_count"),
        stdlib_scalar_port("vertex_count", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("joint_count", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("weight_cnt", orlgraph::LogicalType::int64()),
    };
}

std::vector<orlgraph::Port> csr_weight_inputs(bool with_radii) {
    std::vector<orlgraph::Port> result{
        stdlib_buffer_port("positions", orlgraph::LogicalType::point(),
            "vertex_count"),
        stdlib_buffer_port("offsets", orlgraph::LogicalType::int64(),
            "offset_count"),
        stdlib_buffer_port("neighbors", orlgraph::LogicalType::int64(),
            "neighbor_count"),
        stdlib_buffer_port("joints",
            orlgraph::LogicalType::struct_type("Joint"), "joint_count"),
    };
    if (with_radii) {
        result.push_back(stdlib_buffer_port("radii",
            orlgraph::LogicalType::float64(), "joint_count"));
    }
    result.push_back(stdlib_buffer_port("scratch",
        orlgraph::LogicalType::float64(), "scratch_count"));
    result.push_back(stdlib_buffer_port("weights",
        orlgraph::LogicalType::struct_type("Weight"), "weight_count"));
    result.push_back(stdlib_scalar_port("vertex_count",
        orlgraph::LogicalType::int64()));
    result.push_back(stdlib_scalar_port("joint_count",
        orlgraph::LogicalType::int64()));
    result.push_back(stdlib_scalar_port("weight_cnt",
        orlgraph::LogicalType::int64()));
    return result;
}

std::vector<orlgraph::NodeDefinition> make_auto_weight_definitions() {
    std::vector<orlgraph::NodeDefinition> result;
    const auto add_closest = [&result](std::string function,
        bool cuda_capable) {
        const std::string implementation_function = "auto_weight_" + function;
        const auto effects = parameter_effects(
            implementation_function, {"positions", "joints"}, {}, {"weights"});
        result.push_back(make_stdlib_definition("auto_weight", function,
            closest_weight_inputs(), effects, cuda_capable));
    };
    add_closest("closest_joint", true);
    add_closest("closest_distance", false);
    add_closest("closest_hierarchy", true);

    result.push_back(make_stdlib_definition("auto_weight", "envelope",
        envelope_weight_inputs(), parameter_effects(
            "auto_weight_envelope", {"positions", "joints", "radii"}, {},
            {"weights"}), true));

    const auto add_csr = [&result](std::string function, bool with_radii) {
        const std::string implementation_function = "auto_weight_" + function;
        const auto effects = with_radii
            ? parameter_effects(implementation_function,
                {"positions", "offsets", "neighbors", "joints", "radii"},
                {}, {"scratch", "weights"})
            : parameter_effects(implementation_function,
                {"positions", "offsets", "neighbors", "joints"},
                {}, {"scratch", "weights"});
        result.push_back(make_stdlib_definition("auto_weight", function,
            csr_weight_inputs(with_radii), effects, true));
    };
    add_csr("heat", true);
    add_csr("geodesic", false);
    add_csr("harmonic", true);
    add_csr("bounded_biharmonic", true);
    return result;
}

std::vector<orlgraph::NodeDefinition> make_solver_definitions() {
    const auto index_port = [](std::string name) {
        auto port = stdlib_scalar_port(
            std::move(name), orlgraph::LogicalType::int64());
        port.semantic = std::string{kSceneArrayIndexSemantic};
        return port;
    };
    const auto index_buffer = [](std::string name, std::string shape) {
        auto port = stdlib_buffer_port(
            std::move(name), orlgraph::LogicalType::int64(),
            std::move(shape));
        port.semantic = std::string{kSceneArrayIndexSemantic};
        return port;
    };
    const auto read_write_buffer = [](
        std::string name, orlgraph::LogicalType element,
        std::string shape) {
        auto port = stdlib_buffer_port(
            std::move(name), std::move(element), std::move(shape));
        port.access = orlgraph::AccessMode::ReadWrite;
        return port;
    };
    return {
        make_stdlib_definition("solver", "fk",
            {
                read_write_buffer("world",
                    orlgraph::LogicalType::matrix(), "joint_count"),
            },
            parameter_effects("solver_fk", {}, {"world"}, {}), false),
        make_stdlib_definition("solver", "ik_two_bone",
            {
                index_port("root"),
                index_port("mid"),
                index_port("end"),
                index_port("target_index"),
                index_port("pole_index"),
            },
            parameter_effects("solver_ik_two_bone",
                {"target_index", "pole_index"}, {}, {}), false),
        make_stdlib_definition("solver", "hd_id",
            {
                read_write_buffer("history",
                    orlgraph::LogicalType::struct_type("Joint"), "joint_count"),
                index_port("root"),
                index_port("end"),
                index_port("target_index"),
                stdlib_scalar_port("iterations", orlgraph::LogicalType::int64()),
            },
            parameter_effects("solver_hd_id", {"target_index"}, {}, {"history"}),
            false, true),
        make_stdlib_definition("solver", "spline_ik",
            {
                stdlib_buffer_port("chain",
                    orlgraph::LogicalType::int64(), "chain_count"),
                stdlib_buffer_port("spline",
                    orlgraph::LogicalType::point(), "point_count"),
                stdlib_scalar_port("chain_count", orlgraph::LogicalType::int64()),
                stdlib_scalar_port("point_count", orlgraph::LogicalType::int64()),
            },
            parameter_effects("solver_spline_ik",
                {"chain", "spline"}, {}, {}), false),
        make_stdlib_definition("solver", "full_body_ik",
            {
                stdlib_buffer_port("effectors",
                    orlgraph::LogicalType::int64(), "effector_count"),
                index_buffer("target_indices", "effector_count"),
                stdlib_scalar_port("effector_count", orlgraph::LogicalType::int64()),
                stdlib_scalar_port("iterations", orlgraph::LogicalType::int64()),
            },
            parameter_effects("solver_full_body_ik",
                {"effectors", "target_indices"}, {}, {}), false),
    };
}

std::vector<orlgraph::NodeDefinition> make_constraint_definitions() {
    const auto indices = [] {
        auto result = std::vector<orlgraph::Port>{
            stdlib_scalar_port("source_index", orlgraph::LogicalType::int64()),
            stdlib_scalar_port("destination_index", orlgraph::LogicalType::int64()),
            stdlib_scalar_port("source_count", orlgraph::LogicalType::int64()),
            stdlib_scalar_port("destination_count", orlgraph::LogicalType::int64()),
        };
        result[0].semantic = std::string{kSceneArrayIndexSemantic};
        result[1].semantic = std::string{kSceneArrayIndexSemantic};
        result[0].default_value = orlgraph::ConstantValue{
            orlgraph::LogicalType::int64(), std::int64_t{0}};
        result[1].default_value = orlgraph::ConstantValue{
            orlgraph::LogicalType::int64(), std::int64_t{0}};
        return result;
    };
    const auto copy_inputs = [&indices] {
        auto result = std::vector<orlgraph::Port>{
            stdlib_buffer_port("source",
                orlgraph::LogicalType::matrix(), "source_count"),
            stdlib_buffer_port("destination",
                orlgraph::LogicalType::matrix(), "destination_count"),
        };
        result[0].semantic.clear();
        result[1].semantic.clear();
        result.back().access = orlgraph::AccessMode::ReadWrite;
        auto index_ports = indices();
        result.insert(result.end(), index_ports.begin(), index_ports.end());
        return result;
    };
    const auto copy_definition = [&copy_inputs](std::string function) {
        const std::string implementation_function = "constraint_" + function;
        return make_stdlib_definition("constraint", function,
            copy_inputs(), parameter_effects(
                implementation_function, {"source"}, {}, {"destination"}), false);
    };

    auto aim_inputs = std::vector<orlgraph::Port>{
        stdlib_buffer_port("targets",
            orlgraph::LogicalType::matrix(), "target_count"),
        stdlib_buffer_port("subjects",
            orlgraph::LogicalType::matrix(), "subject_count"),
        stdlib_buffer_port("axes",
            orlgraph::LogicalType::vector(), "one"),
        stdlib_scalar_port("target_index", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("subject_index", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("target_count", orlgraph::LogicalType::int64()),
        stdlib_scalar_port("subject_count", orlgraph::LogicalType::int64()),
    };
    aim_inputs[0].semantic.clear();
    aim_inputs[1].semantic.clear();
    aim_inputs[3].semantic = std::string{kSceneArrayIndexSemantic};
    aim_inputs[4].semantic = std::string{kSceneArrayIndexSemantic};
    aim_inputs[3].default_value = orlgraph::ConstantValue{
        orlgraph::LogicalType::int64(), std::int64_t{0}};
    aim_inputs[4].default_value = orlgraph::ConstantValue{
        orlgraph::LogicalType::int64(), std::int64_t{0}};
    aim_inputs[1].access = orlgraph::AccessMode::ReadWrite;
    auto locator_aim_inputs = aim_inputs;
    locator_aim_inputs[0].type =
        orlgraph::LogicalType::buffer(
            orlgraph::LogicalType::struct_type("Locator"));
    return {
        make_stdlib_definition("constraint", "aim", std::move(aim_inputs),
            parameter_effects("constraint_aim", {"targets", "axes"}, {},
                {"subjects"}), false),
        make_stdlib_definition("constraint", "aim_locator",
            std::move(locator_aim_inputs),
            parameter_effects("constraint_aim_locator",
                {"targets", "axes"}, {}, {"subjects"}), false),
        copy_definition("copy_xform"),
        copy_definition("copy_translation"),
        copy_definition("copy_rotation"),
        copy_definition("copy_scale"),
    };
}

void add_interface_input(orlgraph::GraphModule& module,
    const orlgraph::StableId& id, std::string name,
    orlgraph::LogicalType type, std::string shape_symbol)
{
    module.add_input(orlgraph::InterfacePort{
        id, std::move(name), orlgraph::PortDirection::Input,
        std::move(type), orlgraph::Domain::buffer(),
        orlgraph::Shape::one(std::move(shape_symbol)), true, std::nullopt,
        false, id.value, {}, {}});
}

} // namespace

std::string scene_mesh_positions_binding(std::string_view object_name) {
    return scene_binding("scene.mesh", object_name, "positions");
}

std::string scene_mesh_vertex_count_binding(std::string_view object_name) {
    return scene_binding("scene.mesh", object_name, "vertex_count");
}

std::string scene_weight_buffer_binding(std::string_view component_name) {
    return scene_binding("scene.rig.weights", component_name, "buffer");
}

std::string scene_weight_count_binding(std::string_view component_name) {
    return scene_binding("scene.rig.weights", component_name, "weight_count");
}

std::string scene_inverse_bindings_binding(std::string_view component_name) {
    return scene_binding("scene.rig.deformer", component_name, "inverse_binds");
}

std::string scene_controller_xform_binding(std::string_view component_name) {
    return scene_binding("scene.rig.controller", component_name, "xform");
}

std::string scene_controller_count_binding(std::string_view component_name) {
    return scene_binding("scene.rig.controller", component_name, "count");
}

std::string scene_locator_xform_binding(std::string_view component_name) {
    return scene_binding("scene.rig.locator", component_name, "xform");
}

std::string scene_locator_count_binding(std::string_view component_name) {
    return scene_binding("scene.rig.locator", component_name, "count");
}

orlgraph::ValidationResult RigGraph::validate() const {
    return orlgraph::validate(module, registry);
}

RigGraphResourceIds add_lbs_resources(orlgraph::GraphModule& module) {
    RigGraphResourceIds resources{
        StableId{"bind_positions"},
        StableId{"posed_positions"},
        StableId{"joints"},
        StableId{"inverse_binds"},
        StableId{"weights"},
    };
    module.add_resource(orlgraph::Resource{
        resources.bind_positions, "bind_positions",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        orlgraph::Domain::buffer(), orlgraph::Shape::one("vertex_count"),
        orlgraph::AccessMode::Read, false, "bind_positions"});
    module.add_resource(orlgraph::Resource{
        resources.posed_positions, "posed_positions",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        orlgraph::Domain::vertex(), orlgraph::Shape::one("vertex_count"),
        orlgraph::AccessMode::Write, true, "posed_positions"});
    module.add_resource(orlgraph::Resource{
        resources.joints, "joints",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Joint")),
        orlgraph::Domain::joint(), orlgraph::Shape::one("joint_count"),
        orlgraph::AccessMode::Read, false, "joints"});
    module.add_resource(orlgraph::Resource{
        resources.inverse_binds, "inverse_binds",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::matrix()),
        orlgraph::Domain::joint(), orlgraph::Shape::one("joint_count"),
        orlgraph::AccessMode::ReadWrite, false, "inverse_binds"});
    module.add_resource(orlgraph::Resource{
        resources.weights, "weights",
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Weight")),
        orlgraph::Domain::vertex(), orlgraph::Shape::one("weight_count"),
        orlgraph::AccessMode::Read, false, "weights"});
    return resources;
}

bool register_rig_node_definitions(orlgraph::NodeRegistry& registry,
    std::string* error)
{
    const RigGraphResourceIds resources{
        StableId{"bind_positions"},
        StableId{"posed_positions"},
        StableId{"joints"},
        StableId{"inverse_binds"},
        StableId{"weights"},
    };
    bool result = true;
    const auto register_definition =
        [&registry, error, &result](orlgraph::NodeDefinition definition) {
            std::string registration_error;
            if (!registry.register_definition(std::move(definition),
                    &registration_error))
            {
                result = false;
                if (error != nullptr && error->empty()) {
                    *error = std::move(registration_error);
                }
            }
        };
    const auto register_conversion =
        [&registry, error, &result](
            orlgraph::ConversionDefinition conversion) {
            std::string registration_error;
            if (!registry.register_conversion(std::move(conversion),
                    &registration_error))
            {
                result = false;
                if (error != nullptr && error->empty()) {
                    *error = std::move(registration_error);
                }
            }
        };

    register_conversion(make_joint_world_matrix_conversion());
    register_conversion(make_joint_world_matrix_writeback_conversion());
    register_definition(make_computed_joints_definition());
    register_definition(make_capture_definition(resources));
    register_definition(make_deform_definition(resources));
    for (auto& definition : make_input_definitions()) {
        register_definition(std::move(definition));
    }
    for (auto& definition : make_auto_weight_definitions()) {
        register_definition(std::move(definition));
    }
    for (auto& definition : make_solver_definitions()) {
        register_definition(std::move(definition));
    }
    for (auto& definition : make_constraint_definitions()) {
        register_definition(std::move(definition));
    }
    return result;
}

RigGraph make_lbs_graph() {
    RigGraph graph;
    graph.module.module_id = "orlrig.lbs";
    graph.resources = add_lbs_resources(graph.module);
    add_interface_input(graph.module, graph.resources.bind_positions,
        "bind_positions", orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        "vertex_count");
    add_interface_input(graph.module, graph.resources.weights,
        "weights", orlgraph::LogicalType::buffer(orlgraph::LogicalType::struct_type("Weight")),
        "weight_count");
    graph.module.add_output(orlgraph::InterfacePort{
        graph.resources.posed_positions, "posed_positions",
        orlgraph::PortDirection::Output,
        orlgraph::LogicalType::buffer(orlgraph::LogicalType::point()),
        orlgraph::Domain::buffer(), orlgraph::Shape::one("vertex_count"),
        true, std::nullopt, false, "posed_positions", {}, {}});

    register_rig_node_definitions(graph.registry);
    graph.module.add_node(orlgraph::NodeInstance{
        StableId{"capture_bind"}, StableId{"orlrig.deformer.lbs.capture_bind"},
        "capture_bind", {}, {}, orlgraph::InlinePolicy::Never});
    graph.module.add_node(orlgraph::NodeInstance{
        StableId{"deform"}, StableId{"orlrig.deformer.lbs.evaluate"},
        "deform", {}, {}, orlgraph::InlinePolicy::Never});

    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(graph.resources.bind_positions),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"bind_positions"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::graph_input(graph.resources.weights),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"weights"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            StableId{"capture_bind"}, StableId{"inverse_binds"}),
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"inverse_binds"})});
    graph.module.add_connection(orlgraph::Connection{
        orlgraph::Endpoint::node_port(
            StableId{"deform"}, StableId{"posed_positions"}),
        orlgraph::Endpoint::graph_output(graph.resources.posed_positions)});
    return graph;
}

} // namespace orlrig
