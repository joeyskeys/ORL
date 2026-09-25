#include "runners.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <limits>
#include <utility>

#include <tbb/parallel_for.h>
#include <glm/vec4.hpp>

#include "orl_runtime_signature.h"

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

bool fill_xform(exec::OrlBuffer& destination, const Locator& locator) {
    if (!destination.resize(1)) {
        return false;
    }
    pack_xform(locator, static_cast<double*>(destination.data()));
    return true;
}

bool pack_solver_context(
    exec::OrlBuffer& storage,
    const exec::OrlBuffer& joints,
    const exec::OrlBuffer& locators,
    const exec::OrlBuffer& controllers,
    orlrig::SolverContext* context)
{
    if (context == nullptr) {
        return false;
    }
    constexpr std::size_t alignment = 16;
    const auto align_up = [alignment](std::size_t value) {
        const std::size_t remainder = value % alignment;
        return remainder == 0
            ? value
            : value > std::numeric_limits<std::size_t>::max()
                    - (alignment - remainder)
                ? std::numeric_limits<std::size_t>::max()
                : value + (alignment - remainder);
    };
    std::size_t offset = align_up(sizeof(orlrig::SolverContext));
    if (offset == std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    const std::size_t joints_offset = offset;
    if (joints.byte_size() > std::numeric_limits<std::size_t>::max() - offset) {
        return false;
    }
    offset += joints.byte_size();
    offset = align_up(offset);
    if (offset == std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    const std::size_t locators_offset = offset;
    if (locators.byte_size()
        > std::numeric_limits<std::size_t>::max() - offset)
    {
        return false;
    }
    offset += locators.byte_size();
    offset = align_up(offset);
    if (offset == std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    const std::size_t controllers_offset = offset;
    if (controllers.byte_size()
        > std::numeric_limits<std::size_t>::max() - offset)
    {
        return false;
    }
    offset += controllers.byte_size();

    if (!storage.resize(offset)) {
        return false;
    }
    *context = orlrig::SolverContext{
        static_cast<std::int64_t>(joints.count()),
        static_cast<std::int64_t>(controllers.count()),
        static_cast<std::int64_t>(locators.count()),
        static_cast<std::int64_t>(joints_offset),
        static_cast<std::int64_t>(controllers_offset),
        static_cast<std::int64_t>(locators_offset),
    };
    std::memcpy(storage.data(), context, sizeof(*context));
    const auto copy = [&storage](
        const exec::OrlBuffer& source, std::size_t destination_offset) {
        if (source.byte_size() != 0) {
            std::memcpy(
                static_cast<std::byte*>(storage.data()) + destination_offset,
                source.data(), source.byte_size());
        }
    };
    copy(joints, joints_offset);
    copy(locators, locators_offset);
    copy(controllers, controllers_offset);
    return true;
}

RunnerStatus bind_error(const exec::OrlExecution& execution, const char* stage) {
    RunnerStatus result = failure(execution.errors());
    if (result.errors.empty()) {
        result.errors.emplace_back(std::string{"ORL "} + stage + " failed");
    }
    return result;
}

RunnerStatus validate_joint_buffer(const exec::OrlBuffer& buffer,
    const char* stage)
{
    if (buffer.orl_type() != kJointOrlType
        || buffer.element_stride() != kJointStride)
    {
        return failure(
            std::string{"ORL "} + stage
            + " requires a Joint buffer with stride "
            + std::to_string(kJointStride));
    }
    if (buffer.count() == 0) {
        return failure(
            std::string{"ORL "} + stage + " requires at least one joint");
    }
    return success();
}

} // namespace

LbsRunner::LbsRunner(exec::Backend backend)
    : compute_backend(backend)
    , joints(kJointOrlType, kJointStride)
    , solver_locators(kLocatorOrlType, kLocatorStride)
    , solver_controllers(kMatrixOrlType, kMatrixStride)
    , solver_context_storage(kSolverContextOrlType, 1)
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

RunnerStatus LbsRunner::bind_capture(exec::OrlBuffer& packed,
    DeformerData& deformer)
{
    orlrig::SolverContext context;
    if (!pack_solver_context(
            solver_context_storage, packed, solver_locators,
            solver_controllers, &context)
        || !capture_execution->bind_solver_context(exec::PackedBufferView{
            solver_context_storage.data(),
            solver_context_storage.byte_size(),
            0,
            solver_context_storage.byte_size(),
            solver_context_storage.version()})
        || !capture_execution->set_solver_context(context)
        || !capture_execution->bind_buffer(
            "inverse_binds", deformer.inverse_binds))
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
    if (!fill_joints(joints, packed)) {
        return failure("Failed to pack bind pose joints");
    }
    return capture_bind(deformer, mesh, joints);
}

RunnerStatus LbsRunner::capture_bind(DeformerData& deformer,
    const MeshData& mesh,
    exec::OrlBuffer& packed)
{
    if (mesh.positions.empty()) {
        return failure("Bind capture requires positions and joints");
    }
    if (const auto status = validate_joint_buffer(packed, "Bind capture");
        !status)
    {
        return status;
    }
    if (const auto status = ensure_programs(deformer.type); !status) {
        return status;
    }
    if (!fill_positions(deformer.bind_positions, mesh)
        || !deformer.inverse_binds.resize(packed.count()))
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

RunnerStatus LbsRunner::bind_deform(exec::OrlBuffer& packed,
    DeformerData& deformer,
    WeightData& weights,
    std::int64_t vertex_count)
{
    const std::int64_t weight_count = std::max<std::int64_t>(1, weights.weight_cnt);
    orlrig::SolverContext context;
    if (!pack_solver_context(
            solver_context_storage, packed, solver_locators,
            solver_controllers, &context)
        || !deform_execution->bind_solver_context(exec::PackedBufferView{
            solver_context_storage.data(),
            solver_context_storage.byte_size(),
            0,
            solver_context_storage.byte_size(),
            solver_context_storage.version()})
        || !deform_execution->set_solver_context(context))
    {
        return bind_error(*deform_execution, "deformer context binding");
    }
    for (const auto& parameter : deform_program->parameters()) {
        if (parameter.name == orlcomp::kSolverContextParameterName) {
            continue;
        }
        bool bound = true;
        if (parameter.name == "bind_positions") {
            bound = deform_execution->bind_buffer("bind_positions",
                deformer.bind_positions);
        }
        else if (parameter.name == "output_positions") {
            bound = deform_execution->bind_buffer("output_positions", output);
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
    if (packed.empty() || !fill_joints(joints, packed)) {
        return failure("Failed to pack LBS joints");
    }
    return evaluate(deformer, weights, joints, device_only);
}

RunnerStatus LbsRunner::evaluate(DeformerData& deformer,
    WeightData& weights,
    exec::OrlBuffer& packed,
    bool device_only)
{
    if (!deformer.bound || deformer.bind_positions.count() == 0) {
        return failure("LBS evaluation requires a captured bind pose");
    }
    if (const auto status = validate_joint_buffer(packed, "LBS evaluation");
        !status)
    {
        return status;
    }
    if (const auto status = ensure_programs(deformer.type); !status) {
        return status;
    }
    if (!output.resize(deformer.bind_positions.count())) {
        return failure("Failed to resize LBS output buffer");
    }
    last_vertex_count = static_cast<std::int64_t>(deformer.bind_positions.count());
    if (const auto status = bind_deform(
            packed, deformer, weights, last_vertex_count);
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
    , target_xform(kLocatorOrlType, kLocatorStride)
    , pole_xform(kLocatorOrlType, kLocatorStride)
    , solver_locators(kLocatorOrlType, kLocatorStride)
    , solver_context_storage(kSolverContextOrlType, 1)
    , hierarchy_data(kHierarchyDataOrlType, kHierarchyDataStride)
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
    std::string view_error;
    if (!register_rig_handle_views(
            orlcomp::global_handle_view_registry(), &view_error))
    {
        return failure("Solver handle-view registration failed: "
            + view_error);
    }
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

RunnerStatus SolverRunner::set_hierarchy_plan(
    const HierarchyPlan& plan)
{
    if (plan.joint_count != plan.preorder_joints.size()
        || plan.subtree_begin.size() != plan.joint_count
        || plan.subtree_end.size() != plan.joint_count
        || plan.depth.size() != plan.joint_count)
    {
        return failure("Solver hierarchy plan has inconsistent sizes");
    }
    if (compiled_hierarchy_plan.has_value()
        && compiled_hierarchy_plan->topology_revision
            == plan.topology_revision
        && compiled_hierarchy_plan->ancestor_storage
            == plan.ancestor_storage)
    {
        return success();
    }
    compiled_hierarchy_plan = plan;
    const auto packed = pack_hierarchy_plan(plan);
    hierarchy_context = packed.context;
    if (!hierarchy_data.resize(packed.data.size())) {
        compiled_hierarchy_plan.reset();
        return failure("Failed to allocate solver hierarchy data");
    }
    for (std::size_t index = 0; index < packed.data.size(); ++index) {
        if (!hierarchy_data.write(index, packed.data[index])) {
            compiled_hierarchy_plan.reset();
            return failure("Failed to populate solver hierarchy data");
        }
    }
    return success();
}

RunnerStatus SolverRunner::evaluate_two_bone(std::vector<Joint>& joints,
    std::int64_t root,
    std::int64_t mid,
    std::int64_t end,
    const Locator& target,
    const Locator& pole)
{
    if (!fill_joints(packed_joints, joints)) {
        return failure("Failed to pack solver joints");
    }
    const auto status = evaluate_two_bone(
        packed_joints, root, mid, end, target, pole);
    if (!status) {
        return status;
    }
    if (!joints.empty()) {
        std::memcpy(
            joints.data(), packed_joints.data(), joints.size() * kJointStride);
    }
    return success();
}

RunnerStatus SolverRunner::evaluate_two_bone(exec::OrlBuffer& joint_buffer,
    std::int64_t root,
    std::int64_t mid,
    std::int64_t end,
    const Locator& target,
    const Locator& pole)
{
    if (const auto status = validate_joint_buffer(
            joint_buffer, "Two-bone solver"); !status)
    {
        return status;
    }
    if (root < 0 || mid < 0 || end < 0
        || static_cast<std::size_t>(root) >= joint_buffer.count()
        || static_cast<std::size_t>(mid) >= joint_buffer.count()
        || static_cast<std::size_t>(end) >= joint_buffer.count())
    {
        return failure("Two-bone solver indices are out of range");
    }
    const auto* packed = static_cast<const Joint*>(joint_buffer.data());
    if (packed == nullptr
        || packed[mid].parent != root
        || packed[end].parent != mid)
    {
        return failure(
            "Two-bone solver joints do not form a root-mid-end hierarchy");
    }
    if (const auto status = ensure_program(); !status) {
        return status;
    }
    if (!fill_xform(target_xform, target)
        || !fill_xform(pole_xform, pole)
        || !solver_locators.resize(2))
    {
        return failure("Failed to pack solver inputs");
    }
    std::memcpy(
        static_cast<std::byte*>(solver_locators.data()),
        target_xform.data(), kLocatorStride);
    std::memcpy(
        static_cast<std::byte*>(solver_locators.data()) + kLocatorStride,
        pole_xform.data(), kLocatorStride);

    handle_view_context.joints = {
        joint_buffer.data(), joint_buffer.count(), kJointStride, true};
    handle_view_context.locators = {
        solver_locators.data(), solver_locators.count(), kLocatorStride, false};
    handle_view_context.topology_revision =
        compiled_hierarchy_plan.has_value()
            && compiled_hierarchy_plan->topology_revision != 0
        ? compiled_hierarchy_plan->topology_revision : 1;
    const auto joint_type_id =
        orlcomp::HandleTypeIdFor(kJointHandleCanonical);
    const auto locator_type_id =
        orlcomp::HandleTypeIdFor(kLocatorHandleCanonical);
    if (!execution->bind_handle_view_context(handle_view_context)
        || !execution->bind_handle(
            "root", {joint_type_id, root})
        || !execution->bind_handle(
            "mid", {joint_type_id, mid})
        || !execution->bind_handle(
            "end", {joint_type_id, end})
        || !execution->bind_handle(
            "target", {locator_type_id, 0})
        || !execution->bind_handle(
            "pole", {locator_type_id, 1}))
    {
        return bind_error(*execution, "solver binding");
    }
    if (!execution->evaluate(1).has_value()) {
        return failure(execution->errors());
    }
    return success();
}

RunnerStatus SolverRunner::evaluate_two_bone(std::vector<Joint>& joints,
    std::int64_t root,
    std::int64_t mid,
    std::int64_t end,
    const Controller& target,
    const Controller& pole)
{
    Locator target_locator;
    target_locator.xform = target.xform;
    Locator pole_locator;
    pole_locator.xform = pole.xform;
    return evaluate_two_bone(joints, root, mid, end,
        target_locator, pole_locator);
}

RunnerStatus SolverRunner::evaluate_two_bone(
    ComponentStore& components,
    ComponentId root,
    ComponentId mid,
    ComponentId end,
    const Locator& target,
    const Locator& pole)
{
    if (!compiled_hierarchy_plan.has_value()) {
        return failure("Solver hierarchy plan is not compiled");
    }
    if (compiled_hierarchy_plan->joint_count
        != components.size(ComponentKind::Joint))
    {
        return failure("Solver hierarchy plan does not match the joint store");
    }
    if (!compiled_hierarchy_plan->is_direct_parent(root, mid)
        || !compiled_hierarchy_plan->is_direct_parent(mid, end))
    {
        return failure(
            "Two-bone solver chain does not match the compiled hierarchy");
    }

    const auto root_index = components.joint_index(root);
    const auto mid_index = components.joint_index(mid);
    const auto end_index = components.joint_index(end);
    auto packed = components.packed_joints();
    if (root_index < 0 || mid_index < 0 || end_index < 0
        || packed.size() != compiled_hierarchy_plan->joint_count)
    {
        return failure("Two-bone solver joints are unavailable");
    }

    const auto status = evaluate_two_bone(
        packed, root_index, mid_index, end_index, target, pole);
    if (!status) {
        return status;
    }

    auto* root_joint = components.joint(root);
    auto* mid_joint = components.joint(mid);
    if (root_joint == nullptr || mid_joint == nullptr) {
        return failure("Two-bone solver joints are unavailable");
    }
    *root_joint = packed[static_cast<std::size_t>(root_index)];
    *mid_joint = packed[static_cast<std::size_t>(mid_index)];
    return success();
}

RunnerStatus dispatch_cpu_levels(
    const EvaluationPlan& plan,
    const DynamicDispatchPlan& dispatch,
    const SolverRegionCallback& callback)
{
    if (!callback) {
        return failure("CPU solver dispatch callback is empty");
    }
    for (const auto& level : dispatch.levels) {
        std::vector<RunnerStatus> statuses(level.size());
        std::atomic<bool> failed{false};
        tbb::parallel_for(std::size_t{0}, level.size(),
            [&](std::size_t offset) {
                const std::size_t region_index = level[offset];
                if (region_index >= plan.regions.size()) {
                    statuses[offset] = failure(
                        "CPU solver dispatch contains an invalid region");
                    failed.store(true);
                    return;
                }
                statuses[offset] = callback(plan.regions[region_index]);
                if (!statuses[offset]) {
                    failed.store(true);
                }
            });
        if (failed.load()) {
            RunnerStatus result;
            for (const auto& status : statuses) {
                if (!status) {
                    result.errors.insert(result.errors.end(),
                        status.errors.begin(), status.errors.end());
                }
            }
            return result.errors.empty()
                ? failure("CPU solver region dispatch failed")
                : result;
        }
    }
    return success();
}

RunnerStatus compute_world_matrices_parallel(
    const HierarchyPlan& hierarchy,
    const ComponentStore& components,
    std::vector<glm::mat4>* world_matrices)
{
    if (world_matrices == nullptr) {
        return failure("World-matrix output is null");
    }
    if (hierarchy.preorder_joints.size() != hierarchy.joint_count
        || hierarchy.level_offsets.empty()
        || hierarchy.level_joints.size() != hierarchy.joint_count)
    {
        return failure("Hierarchy plan has incomplete level buckets");
    }
    world_matrices->assign(hierarchy.joint_count, glm::mat4{1.0f});
    for (std::size_t level = 0;
         level + 1 < hierarchy.level_offsets.size(); ++level)
    {
        const auto begin = hierarchy.level_offsets[level];
        const auto end = hierarchy.level_offsets[level + 1];
        tbb::parallel_for(begin, end,
            [&](std::uint32_t offset) {
                if (offset >= hierarchy.level_joints.size()) {
                    return;
                }
                const ComponentId id = hierarchy.level_joints[offset];
                const auto position = hierarchy.preorder_position(id);
                const auto* joint = components.joint(id);
                if (!position.has_value() || joint == nullptr) {
                    return;
                }
                glm::mat4 world = joint_local_matrix(*joint);
                const auto parent = hierarchy.parent_of(id);
                if (parent.has_value()) {
                    const auto parent_position =
                        hierarchy.preorder_position(*parent);
                    if (!parent_position.has_value()) {
                        return;
                    }
                    world = (*world_matrices)[*parent_position] * world;
                }
                (*world_matrices)[*position] = world;
            });
    }
    return success();
}

} // namespace orlrig
