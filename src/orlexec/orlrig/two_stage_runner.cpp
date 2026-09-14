#include "two_stage_runner.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace orlrig
{
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
        result.errors.emplace_back("ORL two-stage execution failed");
    }
    return result;
}

} // namespace

bool StageBindings::bind_buffer(std::string parameter,
    ORL::exec::OrlBuffer& buffer)
{
    if (parameter.empty()) {
        return false;
    }
    device_buffers_.erase(parameter);
    buffers_[std::move(parameter)] = &buffer;
    return true;
}

bool StageBindings::bind_device_buffer(std::string parameter,
    ORL::exec::DeviceBufferView buffer,
    std::size_t element_count)
{
    if (parameter.empty() || buffer.device_ptr == 0 || buffer.bytes == 0) {
        return false;
    }
    buffers_.erase(parameter);
    device_buffers_[std::move(parameter)] = DeviceBinding{
        buffer, element_count};
    return true;
}

bool StageBindings::bind_int(std::string parameter, std::int64_t value) {
    if (parameter.empty()) {
        return false;
    }
    floats_.erase(parameter);
    integers_[std::move(parameter)] = value;
    return true;
}

bool StageBindings::bind_float(std::string parameter, double value) {
    if (parameter.empty()) {
        return false;
    }
    integers_.erase(parameter);
    floats_[std::move(parameter)] = value;
    return true;
}

void StageBindings::clear() {
    buffers_.clear();
    device_buffers_.clear();
    integers_.clear();
    floats_.clear();
}

TwoStageRunner::TwoStageRunner(TwoStageProgram program,
    ORL::exec::Backend backend)
    : program_(std::move(program))
    , compute_backend_(backend)
{
}

bool TwoStageRunner::bind_varying(std::string name,
    ORL::exec::OrlBuffer& buffer,
    std::string solver_parameter,
    std::string deformer_parameter)
{
    if (name.empty()) {
        return false;
    }
    if (solver_parameter.empty()) {
        solver_parameter = name;
    }
    if (deformer_parameter.empty()) {
        deformer_parameter = name;
    }
    if (solver_parameter.empty() || deformer_parameter.empty()) {
        return false;
    }

    const std::string key = name;
    varyings_[key] = VaryingBinding{
        std::move(name),
        std::move(solver_parameter),
        std::move(deformer_parameter),
        &buffer,
    };
    solver_complete_ = false;
    solver_device_varyings_.clear();
    use_device_varyings_ = false;
    return true;
}

RunnerStatus TwoStageRunner::ensure_programs() {
    if (solver_program_.has_value() && solver_program_->valid()
        && solver_execution_.has_value() && solver_execution_->valid()
        && deformer_program_.has_value() && deformer_program_->valid()
        && deformer_execution_.has_value() && deformer_execution_->valid())
    {
        return success();
    }

    solver_program_.reset();
    solver_execution_.reset();
    deformer_program_.reset();
    deformer_execution_.reset();
    solver_complete_ = false;
    solver_device_varyings_.clear();
    use_device_varyings_ = false;

    auto solver = ORL::exec::OrlProgram::Compile(
        program_.solver.source, program_.solver.options);
    if (!solver.valid()) {
        return failure(solver.errors());
    }
    auto deformer = ORL::exec::OrlProgram::Compile(
        program_.deformer.source, program_.deformer.options);
    if (!deformer.valid()) {
        return failure(deformer.errors());
    }

    auto solver_execution =
        ORL::exec::OrlExecution::Create(solver, compute_backend_);
    if (!solver_execution.valid()) {
        return failure(solver_execution.errors());
    }
    auto deformer_execution =
        ORL::exec::OrlExecution::Create(deformer, compute_backend_);
    if (!deformer_execution.valid()) {
        return failure(deformer_execution.errors());
    }

    solver_program_ = std::move(solver);
    solver_execution_ = std::move(solver_execution);
    deformer_program_ = std::move(deformer);
    deformer_execution_ = std::move(deformer_execution);
    return success();
}

RunnerStatus TwoStageRunner::bind_error(
    const ORL::exec::OrlExecution& execution,
    const char* stage) const
{
    RunnerStatus result = failure(execution.errors());
    if (result.errors.empty()) {
        result.errors.emplace_back(
            std::string{"ORL "} + stage + " binding failed");
    }
    return result;
}

RunnerStatus TwoStageRunner::validate_host_buffer(
    const ORL::exec::OrlProgram& program,
    std::string_view parameter,
    const ORL::exec::OrlBuffer& buffer,
    const char* stage) const
{
    const auto found = std::find_if(
        program.parameters().begin(), program.parameters().end(),
        [parameter](const ORL::exec::ParameterDesc& desc) {
            return desc.name == parameter;
        });
    if (found == program.parameters().end()) {
        return failure(
            std::string{"ORL "} + stage + " parameter '" + std::string{parameter}
            + "' is not declared");
    }
    if (found->kind != ORL::exec::ParameterKind::Buffer) {
        return failure(
            std::string{"ORL "} + stage + " parameter '" + std::string{parameter}
            + "' is not a buffer");
    }
    if (buffer.orl_type() != found->orl_type
        || buffer.element_stride() != found->element_stride)
    {
        return failure(
            std::string{"ORL "} + stage + " buffer '" + std::string{parameter}
            + "' expects type '" + found->orl_type + "' with stride "
            + std::to_string(found->element_stride));
    }
    if (buffer.count() == 0) {
        return failure(
            std::string{"ORL "} + stage + " buffer '" + std::string{parameter}
            + "' has no active elements");
    }
    return success();
}

RunnerStatus TwoStageRunner::validate_device_buffer(
    const ORL::exec::OrlProgram& program,
    std::string_view parameter,
    ORL::exec::DeviceBufferView buffer,
    std::size_t element_count,
    const char* stage) const
{
    const auto found = std::find_if(
        program.parameters().begin(), program.parameters().end(),
        [parameter](const ORL::exec::ParameterDesc& desc) {
            return desc.name == parameter;
        });
    if (found == program.parameters().end()) {
        return failure(
            std::string{"ORL "} + stage + " parameter '" + std::string{parameter}
            + "' is not declared");
    }
    if (found->kind != ORL::exec::ParameterKind::Buffer) {
        return failure(
            std::string{"ORL "} + stage + " parameter '" + std::string{parameter}
            + "' is not a buffer");
    }
    if (buffer.device_ptr == 0 || buffer.bytes == 0) {
        return failure(
            std::string{"ORL "} + stage + " device buffer '"
            + std::string{parameter} + "' is empty");
    }
    const std::size_t count = std::max<std::size_t>(1, element_count);
    if (found->element_stride == 0
        || count > std::numeric_limits<std::size_t>::max()
            / found->element_stride)
    {
        return failure(
            std::string{"ORL "} + stage + " device buffer '" + std::string{parameter}
            + "' has an invalid required size");
    }
    const std::size_t required = count * found->element_stride;
    if (buffer.bytes < required) {
        return failure(
            std::string{"ORL "} + stage + " device buffer '"
            + std::string{parameter} + "' is too small; requires "
            + std::to_string(required) + " bytes");
    }
    return success();
}

RunnerStatus TwoStageRunner::bind_stage(
    ORL::exec::OrlExecution& execution,
    const ORL::exec::OrlProgram& program,
    const StageBindings& bindings,
    const char* stage)
{
    execution.clear_bindings();
    for (const auto& [parameter, buffer] : bindings.buffers_) {
        if (buffer == nullptr) {
            return failure(
                std::string{"ORL "} + stage + " buffer '" + parameter
                + "' is null");
        }
        if (const auto status = validate_host_buffer(
                program, parameter, *buffer, stage); !status)
        {
            return status;
        }
        if (!execution.bind_buffer(parameter, *buffer)) {
            return bind_error(execution, stage);
        }
    }
    for (const auto& [parameter, binding] : bindings.device_buffers_) {
        if (const auto status = validate_device_buffer(
                program, parameter, binding.view, binding.element_count, stage);
            !status)
        {
            return status;
        }
        if (!execution.bind_device_buffer(
                parameter, binding.view.device_ptr, binding.view.bytes))
        {
            return bind_error(execution, stage);
        }
    }
    for (const auto& [parameter, value] : bindings.integers_) {
        if (!execution.bind_int(parameter, value)) {
            return bind_error(execution, stage);
        }
    }
    for (const auto& [parameter, value] : bindings.floats_) {
        if (!execution.bind_float(parameter, value)) {
            return bind_error(execution, stage);
        }
    }
    return success();
}

RunnerStatus TwoStageRunner::bind_solver_stage() {
    if (!solver_execution_.has_value()) {
        return failure("Solver stage is not initialized");
    }
    if (const auto status = bind_stage(
            *solver_execution_, *solver_program_, solver_inputs_, "solver");
        !status)
    {
        return status;
    }
    for (const auto& [_, varying] : varyings_) {
        if (varying.buffer == nullptr) {
            return failure("Solver varying has no host buffer");
        }
        if (const auto status = validate_host_buffer(
                *solver_program_, varying.solver_parameter,
                *varying.buffer, "solver varying"); !status)
        {
            return status;
        }
        if (!solver_execution_->bind_buffer(
                varying.solver_parameter, *varying.buffer))
        {
            return bind_error(*solver_execution_, "solver");
        }
    }
    return success();
}

RunnerStatus TwoStageRunner::bind_deformer_stage() {
    if (!deformer_execution_.has_value()) {
        return failure("Deformer stage is not initialized");
    }
    if (const auto status = bind_stage(
            *deformer_execution_, *deformer_program_,
            deformer_inputs_, "deformer"); !status)
    {
        return status;
    }

    for (const auto& [name, varying] : varyings_) {
        if (varying.buffer == nullptr) {
            return failure("Varying '" + name + "' has no host buffer");
        }
        if (use_device_varyings_) {
            const auto device = solver_device_varyings_.find(name);
            if (device == solver_device_varyings_.end()) {
                return failure(
                    "Deformer varying '" + name
                    + "' has no solver device allocation");
            }
            if (const auto status = validate_host_buffer(
                    *deformer_program_, varying.deformer_parameter,
                    *varying.buffer, "deformer varying"); !status)
            {
                return status;
            }
            if (const auto status = validate_device_buffer(
                    *deformer_program_, varying.deformer_parameter,
                    device->second, varying.buffer->count(),
                    "deformer varying"); !status)
            {
                return status;
            }
            if (!deformer_execution_->bind_device_buffer(
                    varying.deformer_parameter,
                    device->second.device_ptr,
                    device->second.bytes))
            {
                return bind_error(*deformer_execution_, "deformer varying");
            }
        } else {
            if (const auto status = validate_host_buffer(
                    *deformer_program_, varying.deformer_parameter,
                    *varying.buffer, "deformer varying"); !status)
            {
                return status;
            }
            if (!deformer_execution_->bind_buffer(
                    varying.deformer_parameter, *varying.buffer))
            {
                return bind_error(*deformer_execution_, "deformer varying");
            }
        }
    }
    return success();
}

RunnerStatus TwoStageRunner::execute_solver(
    std::uint32_t element_count,
    bool device_only)
{
    solver_complete_ = false;
    solver_device_varyings_.clear();
    use_device_varyings_ = false;
    if (device_only && compute_backend_ != ORL::exec::Backend::Cuda) {
        return failure("Device-only two-stage execution requires CUDA");
    }
    if (const auto status = ensure_programs(); !status) {
        return status;
    }
    if (const auto status = bind_solver_stage(); !status) {
        return status;
    }

    if (device_only) {
        if (!solver_execution_->evaluate_device(element_count)) {
            return failure(solver_execution_->errors());
        }
    } else if (!solver_execution_->evaluate(element_count).has_value()) {
        return failure(solver_execution_->errors());
    }

    if (device_only) {
        for (const auto& [name, varying] : varyings_) {
            const auto device = solver_execution_->device_buffer_view(
                varying.solver_parameter);
            if (!device.has_value()) {
                return failure(solver_execution_->errors());
            }
            if (const auto status = validate_device_buffer(
                    *solver_program_, varying.solver_parameter, *device,
                    varying.buffer->count(), "solver varying"); !status)
            {
                return status;
            }
            solver_device_varyings_[name] = *device;
        }
        use_device_varyings_ = true;
    }
    solver_complete_ = true;
    return success();
}

RunnerStatus TwoStageRunner::execute_deformer(
    std::uint32_t element_count,
    bool device_only)
{
    if (!solver_complete_) {
        return failure("Deformer stage requires a completed solver stage");
    }
    if (device_only && compute_backend_ != ORL::exec::Backend::Cuda) {
        return failure("Device-only two-stage execution requires CUDA");
    }
    if (const auto status = ensure_programs(); !status) {
        return status;
    }
    if (const auto status = bind_deformer_stage(); !status) {
        return status;
    }

    if (device_only) {
        if (!deformer_execution_->evaluate_device(element_count)) {
            return failure(deformer_execution_->errors());
        }
    } else if (!deformer_execution_->evaluate(element_count).has_value()) {
        return failure(deformer_execution_->errors());
    }
    return success();
}

RunnerStatus TwoStageRunner::execute(
    std::uint32_t solver_element_count,
    std::uint32_t deformer_element_count,
    bool device_only)
{
    if (const auto status = execute_solver(
            solver_element_count, device_only); !status)
    {
        return status;
    }
    return execute_deformer(deformer_element_count, device_only);
}

std::optional<ORL::exec::DeviceBufferView> TwoStageRunner::device_buffer(
    RigStage stage,
    std::string_view parameter)
{
    if (stage == RigStage::Solver) {
        return solver_execution_.has_value()
            ? solver_execution_->device_buffer_view(parameter)
            : std::nullopt;
    }
    return deformer_execution_.has_value()
        ? deformer_execution_->device_buffer_view(parameter)
        : std::nullopt;
}

std::optional<ORL::exec::DeviceBufferView>
TwoStageRunner::varying_device_buffer(std::string_view name) {
    if (const auto found = solver_device_varyings_.find(std::string{name});
        found != solver_device_varyings_.end())
    {
        return found->second;
    }
    const auto varying = varyings_.find(std::string{name});
    if (varying == varyings_.end() || !solver_execution_.has_value()) {
        return std::nullopt;
    }
    return solver_execution_->device_buffer_view(
        varying->second.solver_parameter);
}

} // namespace orlrig
