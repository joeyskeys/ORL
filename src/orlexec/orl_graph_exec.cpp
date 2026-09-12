#include "orl_graph_exec.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace ORL::exec
{

namespace
{

void append_errors(std::vector<std::string>& destination,
    const std::vector<std::string>& source)
{
    destination.insert(destination.end(), source.begin(), source.end());
}

std::string graph_input_parameter_name(const orlgraph::StableId& id,
    const orlgraph::InterfacePort& input)
{
    const std::string_view source = input.name.empty() ? id.value : input.name;
    std::string result;
    result.reserve(source.size() + 1);
    for (const char character : source) {
        result.push_back(std::isalnum(static_cast<unsigned char>(character))
                || character == '_'
            ? character : '_');
    }
    if (result.empty() || std::isdigit(static_cast<unsigned char>(result.front()))) {
        result.insert(result.begin(), '_');
    }
    return result;
}

const ParameterDesc* find_parameter(const std::vector<ParameterDesc>& parameters,
    std::string_view name)
{
    for (const auto& parameter : parameters) {
        if (parameter.name == name) {
            return &parameter;
        }
    }
    return nullptr;
}

bool has_required_bytes(const GraphInputBinding& binding,
    const ParameterDesc& parameter)
{
    if (binding.bytes == 0 || parameter.element_stride == 0) {
        return false;
    }
    if (binding.element_count == 0) {
        return binding.bytes >= parameter.element_stride;
    }
    return binding.element_count
        <= binding.bytes / parameter.element_stride;
}

} // namespace

OrlGraphProgram OrlGraphProgram::Compile(const orlgraph::GraphModule& module,
    const orlgraph::NodeRegistry& registry,
    orlcomp::GraphLoweringOptions options)
{
    OrlGraphProgram result;
    const auto include_paths = options.include_paths;
    const auto lowered = orlcomp::OrlGraphLowerer{}.lower(
        module, registry, std::move(options));
    result.source_ = lowered.source;
    result.entry_function_ = lowered.entry_function;
    for (const auto& diagnostic : lowered.diagnostics) {
        result.errors_.push_back(diagnostic.code + ": " + diagnostic.message);
    }
    if (!lowered.ok) {
        return result;
    }

    result.program_ = OrlProgram::Compile(result.source_, {
        .entry_function = result.entry_function_,
        .source_name = "orl_graph_program",
        .include_paths = include_paths,
    });
    if (!result.program_->valid()) {
        append_errors(result.errors_, result.program_->errors());
        result.program_.reset();
    }
    return result;
}

bool OrlGraphProgram::valid() const {
    return program_.has_value() && program_->valid();
}

const std::string& OrlGraphProgram::source() const {
    return source_;
}

const std::string& OrlGraphProgram::entry_function() const {
    return entry_function_;
}

const std::vector<ParameterDesc>& OrlGraphProgram::parameters() const {
    static const std::vector<ParameterDesc> empty;
    return program_.has_value() ? program_->parameters() : empty;
}

const std::vector<std::string>& OrlGraphProgram::errors() const {
    return errors_;
}

OrlGraphExecution OrlGraphExecution::Create(const OrlGraphProgram& program,
    Backend backend)
{
    OrlGraphExecution result;
    if (!program.valid() || !program.program_.has_value()) {
        result.errors_ = program.errors_;
        if (result.errors_.empty()) {
            result.errors_.emplace_back("ORL graph program is invalid");
        }
        return result;
    }
    result.execution_ = OrlExecution::Create(*program.program_, backend);
    if (!result.execution_->valid()) {
        append_errors(result.errors_, result.execution_->errors());
        result.execution_.reset();
    } else {
        result.parameters_ = program.parameters();
    }
    return result;
}

bool OrlGraphExecution::valid() const {
    return execution_.has_value() && execution_->valid();
}

bool OrlGraphExecution::bind_buffer(std::string_view parameter, OrlBuffer& buffer) {
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!execution_->bind_buffer(parameter, buffer)) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

bool OrlGraphExecution::bind_device_buffer(std::string_view parameter,
    std::uint64_t device_ptr, std::size_t bytes)
{
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!execution_->bind_device_buffer(parameter, device_ptr, bytes)) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

bool OrlGraphExecution::bind_int(std::string_view parameter, std::int64_t value) {
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!execution_->bind_int(parameter, value)) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

bool OrlGraphExecution::bind_float(std::string_view parameter, double value) {
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!execution_->bind_float(parameter, value)) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

bool OrlGraphExecution::bind_graph_inputs(const orlgraph::GraphModule& module,
    const GraphInputResolver& resolver)
{
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!resolver) {
        errors_.emplace_back("ORL graph input resolver is empty");
        return false;
    }

    errors_.clear();
    execution_->clear_bindings();
    for (const auto& [id, input] : module.inputs()) {
        GraphInputBinding binding;
        std::string resolve_error;
        if (!resolver(input, binding, resolve_error)) {
            errors_.push_back(resolve_error.empty()
                ? "Unable to resolve graph input '" + id.value + "'"
                : std::move(resolve_error));
            execution_->clear_bindings();
            return false;
        }

        const std::string parameter_name = graph_input_parameter_name(id, input);
        const auto* parameter = find_parameter(parameters_, parameter_name);
        if (parameter == nullptr) {
            errors_.push_back("Graph input '" + id.value
                + "' has no runtime parameter '" + parameter_name + "'");
            execution_->clear_bindings();
            return false;
        }
        if (binding.kind != parameter->kind) {
            errors_.push_back("Graph input '" + id.value
                + "' resolved to an incompatible runtime binding kind");
            execution_->clear_bindings();
            return false;
        }

        bool bound = false;
        if (parameter->kind == ParameterKind::Buffer) {
            if (binding.buffer != nullptr && binding.device_ptr != 0) {
                errors_.push_back("Graph input '" + id.value
                    + "' supplied both host and device buffers");
                execution_->clear_bindings();
                return false;
            }
            if (binding.buffer != nullptr) {
                if (binding.buffer->orl_type() != parameter->orl_type
                    || binding.buffer->element_stride() != parameter->element_stride)
                {
                    errors_.push_back("Graph input '" + id.value
                        + "' has a buffer ABI incompatible with ORL type '"
                        + parameter->orl_type + "'");
                    execution_->clear_bindings();
                    return false;
                }
                if (binding.element_count != 0
                    && binding.buffer->count() < binding.element_count)
                {
                    errors_.push_back("Graph input '" + id.value
                        + "' has fewer elements than its declared binding");
                    execution_->clear_bindings();
                    return false;
                }
                bound = execution_->bind_buffer(parameter_name, *binding.buffer);
            } else if (binding.device_ptr != 0) {
                if (!has_required_bytes(binding, *parameter)) {
                    errors_.push_back("Graph input '" + id.value
                        + "' has insufficient device-buffer bytes for ORL type '"
                        + parameter->orl_type + "'");
                    execution_->clear_bindings();
                    return false;
                }
                bound = execution_->bind_device_buffer(parameter_name,
                    binding.device_ptr, binding.bytes);
            } else {
                errors_.push_back("Graph input '" + id.value
                    + "' has no host or device buffer");
                execution_->clear_bindings();
                return false;
            }
        } else if (parameter->kind == ParameterKind::Int64) {
            bound = execution_->bind_int(parameter_name, binding.int_value);
        } else if (parameter->kind == ParameterKind::Float64) {
            bound = execution_->bind_float(parameter_name, binding.float_value);
        }

        if (!bound) {
            errors_ = execution_->errors();
            if (errors_.empty()) {
                errors_.push_back("Failed to bind graph input '" + id.value + "'");
            }
            execution_->clear_bindings();
            return false;
        }
    }
    return true;
}

void OrlGraphExecution::clear_bindings() {
    if (execution_.has_value()) {
        execution_->clear_bindings();
    }
}

std::optional<std::int64_t> OrlGraphExecution::evaluate(std::uint32_t element_count) {
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return std::nullopt;
    }
    const auto result = execution_->evaluate(element_count);
    if (!result.has_value()) {
        errors_ = execution_->errors();
    }
    return result;
}

bool OrlGraphExecution::evaluate_device(std::uint32_t element_count) {
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!execution_->evaluate_device(element_count)) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

std::optional<DeviceBufferView> OrlGraphExecution::device_buffer_view(
    std::string_view parameter)
{
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return std::nullopt;
    }
    const auto result = execution_->device_buffer_view(parameter);
    if (!result.has_value()) {
        errors_ = execution_->errors();
    }
    return result;
}

std::optional<std::uint64_t> OrlGraphExecution::device_buffer_pointer(
    std::string_view parameter)
{
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return std::nullopt;
    }
    const auto result = execution_->device_buffer_pointer(parameter);
    if (!result.has_value()) {
        errors_ = execution_->errors();
    }
    return result;
}

bool OrlGraphExecution::synchronize() {
    if (!execution_.has_value()) {
        errors_.emplace_back("ORL graph execution is invalid");
        return false;
    }
    if (!execution_->synchronize()) {
        errors_ = execution_->errors();
        return false;
    }
    return true;
}

Backend OrlGraphExecution::backend() const {
    return execution_.has_value() ? execution_->backend() : Backend::Cpu;
}

const std::vector<std::string>& OrlGraphExecution::errors() const {
    return errors_;
}

const std::string& OrlGraphExecution::ir() const {
    static const std::string empty;
    return execution_.has_value() ? execution_->ir() : empty;
}

} // namespace ORL::exec
