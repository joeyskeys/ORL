#include "orl_graph_exec.hpp"

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

} // namespace

OrlGraphProgram OrlGraphProgram::Compile(const orlgraph::GraphModule& module,
    const orlgraph::NodeRegistry& registry,
    orlcomp::GraphLoweringOptions options)
{
    OrlGraphProgram result;
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
