#include "runners.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include <glm/vec4.hpp>

namespace orlrig
{
namespace exec = ORL::exec;

namespace
{

RunnerStatus success() {
    return RunnerStatus{true, {}};
}

RunnerStatus failure(std::string message) {
    RunnerStatus result;
    result.errors.push_back(std::move(message));
    return result;
}

RunnerStatus failure(const std::vector<std::string>& errors) {
    RunnerStatus result;
    result.errors = errors;
    if (result.errors.empty()) {
        result.errors.emplace_back("ORL execution failed");
    }
    return result;
}

bool known_algorithm(std::string_view name) {
    return name == "closest_distance"
        || name == "closest_hierarchy"
        || name == "heat"
        || name == "geodesic";
}

bool fill_positions(exec::OrlBuffer& destination, const MeshData& mesh) {
    if (mesh.positions.empty() || !destination.resize(mesh.positions.size())) {
        return false;
    }
    auto* output = static_cast<double*>(destination.data());
    for (std::size_t index = 0; index < mesh.positions.size(); ++index) {
        const glm::vec4 world = mesh.model
            * glm::vec4{mesh.positions[index], 1.0f};
        output[index * 4 + 0] = world.x;
        output[index * 4 + 1] = world.y;
        output[index * 4 + 2] = world.z;
        output[index * 4 + 3] = 0.0;
    }
    return true;
}

bool fill_joints(exec::OrlBuffer& destination, const std::vector<Joint>& joints) {
    if (!destination.resize(joints.size())) {
        return false;
    }
    if (!joints.empty()) {
        std::memcpy(destination.data(), joints.data(), joints.size() * kJointStride);
    }
    return true;
}

bool fill_uints_as_int(exec::OrlBuffer& destination,
    const std::vector<std::uint32_t>& source)
{
    if (!destination.resize(source.size())) {
        return false;
    }
    auto* output = static_cast<std::int64_t*>(destination.data());
    for (std::size_t index = 0; index < source.size(); ++index) {
        output[index] = static_cast<std::int64_t>(source[index]);
    }
    return true;
}

bool fill_radii(exec::OrlBuffer& destination, const std::vector<Joint>& joints) {
    if (!destination.resize(joints.size())) {
        return false;
    }
    double accumulated = 0.0;
    int bone_count = 0;
    for (std::size_t index = 0; index < joints.size(); ++index) {
        if (joints[index].parent < 0) {
            continue;
        }
        const glm::vec3 child{
            joint_world_matrix(joints, static_cast<std::int64_t>(index))[3]};
        const glm::vec3 parent{
            joint_world_matrix(joints, joints[index].parent)[3]};
        accumulated += static_cast<double>(glm::length(child - parent));
        ++bone_count;
    }
    double radius = bone_count > 0
        ? accumulated / static_cast<double>(bone_count)
        : 1.0;
    radius = std::max(radius, 0.25) * 1.25;
    auto* output = static_cast<double*>(destination.data());
    for (std::size_t index = 0; index < joints.size(); ++index) {
        output[index] = radius;
    }
    return true;
}

bool fill_xform(exec::OrlBuffer& destination, const Controller& controller) {
    if (!destination.resize(1)) {
        return false;
    }
    pack_xform(controller, static_cast<double*>(destination.data()));
    return true;
}

RunnerStatus bind_error(const exec::OrlExecution& execution, const char* stage) {
    RunnerStatus result = failure(execution.errors());
    if (result.errors.empty()) {
        result.errors.emplace_back(std::string{"ORL "} + stage + " failed");
    }
    return result;
}

} // namespace

LbsRunner::LbsRunner(exec::Backend backend)
    : compute_backend(backend)
    , joints(kJointOrlType, kJointStride)
    , output(kPointOrlType, kPointStride)
{
}

RunnerStatus LbsRunner::ensure_programs(const std::string& type) {
    if (type != "lbs") {
        return failure("Unknown deformer type '" + type + "'");
    }
    if (compiled_type == type
        && capture_program.has_value() && capture_program->valid()
        && capture_execution.has_value() && capture_execution->valid()
        && deform_program.has_value() && deform_program->valid()
        && deform_execution.has_value() && deform_execution->valid()
        && capture_execution->backend() == compute_backend
        && deform_execution->backend() == compute_backend)
    {
        capture_execution->clear_bindings();
        deform_execution->clear_bindings();
        return success();
    }

    capture_program.reset();
    capture_execution.reset();
    deform_program.reset();
    deform_execution.reset();
    compiled_type.clear();

    const std::string source = "use deformer/" + type + ";\n";
    auto capture = exec::OrlProgram::Compile(source, {
        .entry_function = "deformer_" + type + "_capture_bind",
        .source_name = "orlrig_deformer_capture",
    });
    if (!capture.valid()) {
        return failure(capture.errors());
    }
    auto deform = exec::OrlProgram::Compile(source, {
        .entry_function = "deformer_" + type,
        .source_name = "orlrig_deformer",
    });
    if (!deform.valid()) {
        return failure(deform.errors());
    }
    auto capture_exec = exec::OrlExecution::Create(capture, compute_backend);
    if (!capture_exec.valid()) {
        return failure(capture_exec.errors());
    }
    auto deform_exec = exec::OrlExecution::Create(deform, compute_backend);
    if (!deform_exec.valid()) {
        return failure(deform_exec.errors());
    }

    capture_program = std::move(capture);
    capture_execution = std::move(capture_exec);
    deform_program = std::move(deform);
    deform_execution = std::move(deform_exec);
    compiled_type = type;
    return success();
}

RunnerStatus LbsRunner::bind_capture(const std::vector<Joint>& packed,
    DeformerData& deformer)
{
    if (!capture_execution->bind_buffer("joints", joints)
        || !capture_execution->bind_buffer("inverse_binds", deformer.inverse_binds)
        || !capture_execution->bind_int("joint_count",
            static_cast<std::int64_t>(packed.size())))
    {
        return bind_error(*capture_execution, "capture binding");
    }
    if (!capture_execution->evaluate(1).has_value()) {
        return failure(capture_execution->errors());
    }
    return success();
}

RunnerStatus LbsRunner::capture_bind(DeformerData& deformer,
    const MeshData& mesh,
    const std::vector<Joint>& packed)
{
    if (packed.empty() || mesh.positions.empty()) {
        return failure("Bind capture requires positions and joints");
    }
    if (const auto status = ensure_programs(deformer.type); !status) {
        return status;
    }
    if (!fill_positions(deformer.bind_positions, mesh)
        || !fill_joints(joints, packed)
        || !deformer.inverse_binds.resize(packed.size()))
    {
        return failure("Failed to pack bind pose");
    }
    deformer.bind_model = mesh.model;
    if (const auto status = bind_capture(packed, deformer); !status) {
        return status;
    }
    deformer.bound = true;
    return success();
}

RunnerStatus LbsRunner::bind_deform(const std::vector<Joint>& packed,
    DeformerData& deformer,
    WeightData& weights,
    std::int64_t vertex_count)
{
    const std::int64_t weight_count = std::max<std::int64_t>(1, weights.weight_cnt);
    const std::int64_t joint_count = static_cast<std::int64_t>(packed.size());
    for (const auto& parameter : deform_program->parameters()) {
        bool bound = true;
        if (parameter.name == "bind_positions") {
            bound = deform_execution->bind_buffer("bind_positions",
                deformer.bind_positions);
        }
        else if (parameter.name == "output_positions") {
            bound = deform_execution->bind_buffer("output_positions", output);
        }
        else if (parameter.name == "joints") {
            bound = deform_execution->bind_buffer("joints", joints);
        }
        else if (parameter.name == "inverse_binds") {
            bound = deform_execution->bind_buffer("inverse_binds",
                deformer.inverse_binds);
        }
        else if (parameter.name == "weights") {
            bound = deform_execution->bind_buffer("weights", weights.weights);
        }
        else if (parameter.name == "vertex_count") {
            bound = deform_execution->bind_int("vertex_count", vertex_count);
        }
        else if (parameter.name == "joint_count") {
            bound = deform_execution->bind_int("joint_count", joint_count);
        }
        else if (parameter.name == "weight_cnt") {
            bound = deform_execution->bind_int("weight_cnt", weight_count);
        }
        else {
            return failure("Unhandled deformer parameter '" + parameter.name + "'");
        }
        if (!bound) {
            return bind_error(*deform_execution, "deformer binding");
        }
    }
    return success();
}

RunnerStatus LbsRunner::evaluate(DeformerData& deformer,
    WeightData& weights,
    const std::vector<Joint>& packed,
    bool device_only)
{
    if (!deformer.bound || packed.empty() || deformer.bind_positions.count() == 0) {
        return failure("LBS evaluation requires a captured bind pose");
    }
    if (const auto status = ensure_programs(deformer.type); !status) {
        return status;
    }
    if (!fill_joints(joints, packed)
        || !output.resize(deformer.bind_positions.count()))
    {
        return failure("Failed to pack LBS buffers");
    }
    last_vertex_count = static_cast<std::int64_t>(deformer.bind_positions.count());
    if (const auto status = bind_deform(packed, deformer, weights, last_vertex_count);
        !status)
    {
        return status;
    }

    if (device_only && compute_backend == exec::Backend::Cuda) {
        if (!deform_execution->evaluate_device(
                static_cast<std::uint32_t>(last_vertex_count)))
        {
            return failure(deform_execution->errors());
        }
    } else if (!deform_execution->evaluate(
                   static_cast<std::uint32_t>(last_vertex_count)).has_value()) {
        return failure(deform_execution->errors());
    }
    return success();
}

RunnerStatus LbsRunner::readback() {
    if (compute_backend == exec::Backend::Cpu) {
        return success();
    }
    if (!deform_execution.has_value() || last_vertex_count <= 0
        || !deform_execution->evaluate(
                static_cast<std::uint32_t>(last_vertex_count)).has_value())
    {
        return deform_execution.has_value()
            ? failure(deform_execution->errors())
            : failure("LBS execution is not initialized");
    }
    return success();
}

std::optional<exec::DeviceBufferView> LbsRunner::output_device() {
    if (!deform_execution.has_value()) {
        return std::nullopt;
    }
    return deform_execution->device_buffer_view("output_positions");
}

AutoWeightRunner::AutoWeightRunner(std::string algorithm, exec::Backend backend)
    : algorithm_name(std::move(algorithm))
    , compute_backend(backend)
    , positions(kPointOrlType, kPointStride)
    , packed_joints(kJointOrlType, kJointStride)
    , offsets("int", sizeof(std::int64_t))
    , neighbors("int", sizeof(std::int64_t))
    , radii("float", sizeof(double))
    , scratch("float", sizeof(double))
{
}

bool AutoWeightRunner::set_algorithm(std::string_view name) {
    if (!known_algorithm(name)) {
        return false;
    }
    algorithm_name = std::string{name};
    program.reset();
    execution.reset();
    return true;
}

RunnerStatus AutoWeightRunner::ensure_program() {
    if (!known_algorithm(algorithm_name)) {
        return failure("Unknown auto-weight algorithm '" + algorithm_name + "'");
    }
    if (program.has_value() && program->valid()
        && execution.has_value() && execution->valid()
        && execution->backend() == compute_backend)
    {
        execution->clear_bindings();
        return success();
    }

    program.reset();
    execution.reset();
    const std::string source = "use auto_weight/" + algorithm_name + ";\n";
    auto compiled = exec::OrlProgram::Compile(source, {
        .entry_function = "auto_weight_" + algorithm_name,
        .source_name = "orlrig_auto_weight",
    });
    if (!compiled.valid()) {
        return failure(compiled.errors());
    }
    auto created = exec::OrlExecution::Create(compiled, compute_backend);
    if (!created.valid()) {
        return failure(created.errors());
    }
    program = std::move(compiled);
    execution = std::move(created);
    return success();
}

RunnerStatus AutoWeightRunner::run(const MeshData& mesh,
    const std::vector<Joint>& packed,
    const MeshCsrData* csr,
    WeightData& weights,
    double dropoff)
{
    if (packed.empty() || mesh.positions.empty()) {
        return failure("Auto-weight requires positions and joints");
    }
    if (const auto status = ensure_program(); !status) {
        return status;
    }
    if (!fill_positions(positions, mesh)
        || !fill_joints(packed_joints, packed)
        || !fill_radii(radii, packed))
    {
        return failure("Failed to pack auto-weight inputs");
    }

    bool needs_csr = false;
    bool needs_scratch = false;
    for (const auto& parameter : program->parameters()) {
        needs_csr = needs_csr
            || parameter.name == "offsets"
            || parameter.name == "neighbors";
        needs_scratch = needs_scratch || parameter.name == "scratch";
    }
    if (needs_csr && csr == nullptr) {
        return failure("Auto-weight algorithm requires mesh CSR data");
    }
    if (csr != nullptr
        && (!fill_uints_as_int(offsets, csr->offsets)
            || !fill_uints_as_int(neighbors, csr->neighbors)))
    {
        return failure("Failed to pack mesh CSR data");
    }

    std::int64_t weight_count = weights.weight_cnt;
    if (weight_count <= 0) {
        weight_count = static_cast<std::int64_t>(kDefaultWeightCount);
        weights.weight_cnt = weight_count;
    }
    if (needs_scratch
        && !scratch.resize(mesh.positions.size() * packed.size()))
    {
        return failure("Failed to resize auto-weight scratch data");
    }
    const std::size_t output_count = mesh.positions.size()
        * static_cast<std::size_t>(weight_count);
    if (!weights.weights.resize(output_count)) {
        return failure("Failed to resize weight buffer");
    }

    const std::int64_t vertex_count =
        static_cast<std::int64_t>(mesh.positions.size());
    const std::int64_t joint_count =
        static_cast<std::int64_t>(packed.size());
    for (const auto& parameter : program->parameters()) {
        bool bound = true;
        if (parameter.name == "positions") {
            bound = execution->bind_buffer("positions", positions);
        }
        else if (parameter.name == "joints") {
            bound = execution->bind_buffer("joints", packed_joints);
        }
        else if (parameter.name == "weights") {
            bound = execution->bind_buffer("weights", weights.weights);
        }
        else if (parameter.name == "offsets") {
            bound = execution->bind_buffer("offsets", offsets);
        }
        else if (parameter.name == "neighbors") {
            bound = execution->bind_buffer("neighbors", neighbors);
        }
        else if (parameter.name == "radii") {
            bound = execution->bind_buffer("radii", radii);
        }
        else if (parameter.name == "scratch") {
            bound = execution->bind_buffer("scratch", scratch);
        }
        else if (parameter.name == "vertex_count") {
            bound = execution->bind_int("vertex_count", vertex_count);
        }
        else if (parameter.name == "joint_count") {
            bound = execution->bind_int("joint_count", joint_count);
        }
        else if (parameter.name == "weight_cnt") {
            bound = execution->bind_int("weight_cnt", weight_count);
        }
        else if (parameter.name == "dropoff") {
            bound = execution->bind_float("dropoff", dropoff);
        }
        else {
            return failure("Unhandled auto-weight parameter '" + parameter.name + "'");
        }
        if (!bound) {
            return bind_error(*execution, "auto-weight binding");
        }
    }
    if (!execution->evaluate(static_cast<std::uint32_t>(mesh.positions.size()))
             .has_value())
    {
        return failure(execution->errors());
    }
    return success();
}

SolverRunner::SolverRunner(exec::Backend backend)
    : compute_backend(backend)
    , packed_joints(kJointOrlType, kJointStride)
    , target_xform(kControllerOrlType, kControllerXformStride)
    , pole_xform(kControllerOrlType, kControllerXformStride)
{
}

RunnerStatus SolverRunner::ensure_program() {
    if (program.has_value() && program->valid()
        && execution.has_value() && execution->valid()
        && execution->backend() == compute_backend)
    {
        execution->clear_bindings();
        return success();
    }

    program.reset();
    execution.reset();
    auto compiled = exec::OrlProgram::Compile("use solver/ik_two_bone;\n", {
        .entry_function = "solver_ik_two_bone",
        .source_name = "orlrig_solver_ik_two_bone",
    });
    if (!compiled.valid()) {
        return failure(compiled.errors());
    }
    auto created = exec::OrlExecution::Create(compiled, compute_backend);
    if (!created.valid()) {
        return failure(created.errors());
    }
    program = std::move(compiled);
    execution = std::move(created);
    return success();
}

RunnerStatus SolverRunner::evaluate_two_bone(std::vector<Joint>& joints,
    std::int64_t root,
    std::int64_t mid,
    std::int64_t end,
    const Controller& target,
    const Controller& pole)
{
    if (root < 0 || mid < 0 || end < 0
        || static_cast<std::size_t>(root) >= joints.size()
        || static_cast<std::size_t>(mid) >= joints.size()
        || static_cast<std::size_t>(end) >= joints.size())
    {
        return failure("Two-bone solver indices are out of range");
    }
    if (const auto status = ensure_program(); !status) {
        return status;
    }
    if (!fill_joints(packed_joints, joints)
        || !fill_xform(target_xform, target)
        || !fill_xform(pole_xform, pole))
    {
        return failure("Failed to pack solver inputs");
    }

    const auto joint_count = static_cast<std::int64_t>(joints.size());
    if (!execution->bind_buffer("joints", packed_joints)
        || !execution->bind_int("root", root)
        || !execution->bind_int("mid", mid)
        || !execution->bind_int("end", end)
        || !execution->bind_buffer("target", target_xform)
        || !execution->bind_buffer("pole", pole_xform)
        || !execution->bind_int("joint_count", joint_count))
    {
        return bind_error(*execution, "solver binding");
    }
    if (!execution->evaluate(1).has_value()) {
        return failure(execution->errors());
    }
    std::memcpy(joints.data(), packed_joints.data(), joints.size() * kJointStride);
    return success();
}

} // namespace orlrig
