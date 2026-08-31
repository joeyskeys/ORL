#include "orl_exec.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <utility>

#include "orl_codegen.h"
#include "orl_gpu.h"
#include "orl_jit.h"
#include "orl_parser.h"
#include "orl_runtime_signature.h"

#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>

namespace ORL::exec
{
namespace
{

std::size_t element_stride_for(std::string_view type_name) {
    if (type_name == "int" || type_name == "float") {
        return sizeof(std::int64_t);
    }
    if (type_name == "point" || type_name == "vector" || type_name == "normal"
        || type_name == "vec3" || type_name == "dvec3"
        || type_name == "vec4" || type_name == "dvec4" || type_name == "quat")
    {
        // LLVM's vector ABI rounds 3 doubles up to the same 32-byte slot as
        // a four-component vector; this matches the existing skinning ABI.
        return sizeof(double) * 4;
    }
    if (type_name == "matrix") {
        return sizeof(double) * 16;
    }
    if (type_name == "Joint") {
        // Matches orlviewer::kJointStride and the unpacked LLVM Joint ABI.
        return 128;
    }
    if (type_name == "Weight") {
        // Matches orlviewer::kWeightStride: float weight, int joint.
        return 16;
    }
    return 0;
}

void append_errors(std::vector<std::string>& destination,
    const std::vector<std::string>& source)
{
    destination.insert(destination.end(), source.begin(), source.end());
}

bool parse_source(const std::string& source, orlcomp::Parser& parser,
    std::vector<std::string>& errors)
{
    if (parser.Parse() && parser.Ast() != nullptr) {
        return true;
    }
    append_errors(errors, parser.Errors());
    if (errors.empty()) {
        errors.emplace_back("ORL parser did not produce an AST");
    }
    return false;
}

} // namespace

OrlBuffer::OrlBuffer(std::string orl_type, std::size_t element_stride)
    : orl_type_(std::move(orl_type))
    , element_stride_(element_stride)
{
}

bool OrlBuffer::reserve(std::size_t element_capacity) {
    if (element_stride_ == 0
        || element_capacity > std::numeric_limits<std::size_t>::max() / element_stride_)
    {
        return false;
    }
    const std::size_t bytes = element_capacity * element_stride_;
    if (bytes <= storage_.size()) {
        return true;
    }
    storage_.resize(bytes, std::byte{0});
    ++version_;
    return true;
}

bool OrlBuffer::resize(std::size_t element_count) {
    const std::size_t previous_count = count_;
    if (!reserve(element_count)) {
        return false;
    }
    if (element_count > previous_count) {
        const std::size_t begin = previous_count * element_stride_;
        const std::size_t bytes = (element_count - previous_count) * element_stride_;
        std::fill(storage_.begin() + static_cast<std::ptrdiff_t>(begin),
            storage_.begin() + static_cast<std::ptrdiff_t>(begin + bytes), std::byte{0});
    }
    if (count_ != element_count) {
        count_ = element_count;
        ++version_;
    }
    return true;
}

void OrlBuffer::clear() {
    if (count_ != 0) {
        count_ = 0;
        ++version_;
    }
}

void* OrlBuffer::data() {
    mark_modified();
    return storage_.data();
}

void OrlBuffer::mark_modified() {
    ++version_;
}

bool OrlBuffer::write(std::size_t index, const void* source, std::size_t bytes) {
    if (source == nullptr || bytes != element_stride_ || index >= count_) {
        return false;
    }
    std::memcpy(storage_.data() + index * element_stride_, source, bytes);
    ++version_;
    return true;
}

bool OrlBuffer::read(std::size_t index, void* destination, std::size_t bytes) const {
    if (destination == nullptr || bytes != element_stride_ || index >= count_) {
        return false;
    }
    std::memcpy(destination, storage_.data() + index * element_stride_, bytes);
    return true;
}

struct OrlProgram::Impl {
    std::string source;
    CompileOptions options;
    std::vector<ParameterDesc> parameters;
    std::vector<std::string> errors;
};

OrlProgram::OrlProgram(std::shared_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

OrlProgram OrlProgram::Compile(std::string source, CompileOptions options) {
    auto impl = std::make_shared<Impl>();
    impl->source = std::move(source);
    impl->options = std::move(options);

    orlcomp::Parser parser(impl->source);
    if (!parse_source(impl->source, parser, impl->errors)) {
        return OrlProgram(std::move(impl));
    }
    const auto signature =
        orlcomp::DescribeRuntimeFunction(*parser.Ast(), impl->options.entry_function);
    if (!signature.has_value()) {
        impl->errors.emplace_back("ORL entry function '" + impl->options.entry_function + "' was not found");
        return OrlProgram(std::move(impl));
    }
    if (signature->return_type != "int") {
        impl->errors.emplace_back("ORL runtime entry function must return int");
        return OrlProgram(std::move(impl));
    }

    for (const auto& parameter : signature->parameters) {
        ParameterDesc desc;
        desc.name = parameter.name;
        desc.orl_type = parameter.type_name;
        if (parameter.kind == orlcomp::OrlRuntimeParameterKind::Buffer) {
            desc.kind = ParameterKind::Buffer;
            desc.element_stride = element_stride_for(parameter.type_name);
            if (desc.element_stride == 0) {
                desc.kind = ParameterKind::Unsupported;
            }
        } else {
            desc.kind = parameter.kind == orlcomp::OrlRuntimeParameterKind::Int64
                ? ParameterKind::Int64
                : parameter.kind == orlcomp::OrlRuntimeParameterKind::Float64
                    ? ParameterKind::Float64
                    : ParameterKind::Unsupported;
        }
        impl->parameters.push_back(std::move(desc));
    }

    return OrlProgram(std::move(impl));
}

bool OrlProgram::valid() const {
    return impl_ != nullptr && impl_->errors.empty();
}

const std::string& OrlProgram::entry_function() const {
    return impl_->options.entry_function;
}

const std::vector<ParameterDesc>& OrlProgram::parameters() const {
    return impl_->parameters;
}

const std::vector<std::string>& OrlProgram::errors() const {
    return impl_->errors;
}

struct OrlExecution::Impl {
    struct DeviceBuffer {
        orlcomp::OrlGpuBuffer handle = 0;
        std::size_t capacity_bytes = 0;
        std::uint64_t uploaded_version = 0;
    };

    struct ExternalDeviceBuffer {
        std::uint64_t device_ptr = 0;
        std::size_t bytes = 0;
        orlcomp::OrlGpuBuffer handle = 0;
    };

    std::shared_ptr<OrlProgram::Impl> program;
    Backend backend = Backend::Cpu;
    std::unordered_map<std::string, OrlBuffer*> buffers;
    std::unordered_map<std::string, ExternalDeviceBuffer> device_bindings;
    std::unordered_map<std::string, std::int64_t> integers;
    std::unordered_map<std::string, double> floats;
    std::unordered_map<OrlBuffer*, DeviceBuffer> device_buffers;
    std::unique_ptr<orlcomp::OrlJitEngine> jit;
    std::unique_ptr<orlcomp::OrlGpuEngine> gpu;
    std::vector<std::string> errors;
    std::string ir;
    bool initialized = false;

    ~Impl() {
        release_device_bindings();
        if (gpu != nullptr) {
            for (const auto& [_, buffer] : device_buffers) {
                if (buffer.handle != 0) {
                    gpu->FreeBuffer(buffer.handle);
                }
            }
        }
    }

    void release_device_binding(const std::string& name) {
        const auto found = device_bindings.find(name);
        if (found == device_bindings.end()) {
            return;
        }
        if (gpu != nullptr && found->second.handle != 0) {
            gpu->FreeBuffer(found->second.handle);
        }
        device_bindings.erase(found);
    }

    void release_device_bindings() {
        if (gpu != nullptr) {
            for (const auto& [_, bound] : device_bindings) {
                if (bound.handle != 0) {
                    gpu->FreeBuffer(bound.handle);
                }
            }
        }
        device_bindings.clear();
    }

    const ParameterDesc* parameter(std::string_view name) const {
        const auto found = std::find_if(program->parameters.begin(), program->parameters.end(),
            [name](const ParameterDesc& parameter) { return parameter.name == name; });
        return found == program->parameters.end() ? nullptr : &*found;
    }

    bool validate_bindings(std::vector<void*>& ordered_buffers,
        std::vector<std::int64_t>& ordered_integers,
        std::vector<double>& ordered_floats)
    {
        errors.clear();
        for (const auto& parameter : program->parameters) {
            if (parameter.kind == ParameterKind::Unsupported) {
                errors.emplace_back("Unsupported runtime parameter type '" + parameter.orl_type
                    + "' for '" + parameter.name + "'");
                continue;
            }
            if (parameter.kind == ParameterKind::Buffer) {
                const auto device = device_bindings.find(parameter.name);
                if (device != device_bindings.end()) {
                    if (backend != Backend::Cuda) {
                        errors.emplace_back("Device buffer binding for '" + parameter.name
                            + "' requires the CUDA backend");
                        continue;
                    }
                    ordered_buffers.push_back(nullptr);
                    continue;
                }
                const auto bound = buffers.find(parameter.name);
                if (bound == buffers.end() || bound->second == nullptr) {
                    errors.emplace_back("Missing buffer binding for parameter '" + parameter.name + "'");
                    continue;
                }
                OrlBuffer& buffer = *bound->second;
                if (buffer.orl_type() != parameter.orl_type
                    || buffer.element_stride() != parameter.element_stride)
                {
                    errors.emplace_back("Buffer binding for '" + parameter.name
                        + "' does not match ORL type '" + parameter.orl_type + "'");
                    continue;
                }
                ordered_buffers.push_back(backend == Backend::Cpu ? buffer.data() : nullptr);
            } else if (parameter.kind == ParameterKind::Int64) {
                const auto bound = integers.find(parameter.name);
                if (bound == integers.end()) {
                    errors.emplace_back("Missing int binding for parameter '" + parameter.name + "'");
                    continue;
                }
                ordered_integers.push_back(bound->second);
            } else {
                const auto bound = floats.find(parameter.name);
                if (bound == floats.end()) {
                    errors.emplace_back("Missing float binding for parameter '" + parameter.name + "'");
                    continue;
                }
                ordered_floats.push_back(bound->second);
            }
        }
        return errors.empty();
    }

    bool ensure_device_buffer(OrlBuffer& buffer, orlcomp::OrlGpuBuffer* handle) {
        const std::size_t capacity_bytes = buffer.capacity() * buffer.element_stride();
        if (capacity_bytes == 0) {
            errors.emplace_back("CUDA buffer binding requires non-zero capacity");
            return false;
        }

        auto& device = device_buffers[&buffer];
        if (device.handle == 0 || device.capacity_bytes != capacity_bytes) {
            if (device.handle != 0 && !gpu->FreeBuffer(device.handle)) {
                append_errors(errors, gpu->Errors());
                return false;
            }
            const auto allocated = gpu->AllocateBuffer(capacity_bytes);
            if (!allocated.has_value()) {
                append_errors(errors, gpu->Errors());
                return false;
            }
            device.handle = *allocated;
            device.capacity_bytes = capacity_bytes;
            device.uploaded_version = 0;
        }
        if (buffer.byte_size() != 0 && device.uploaded_version != buffer.version()) {
            if (!gpu->UploadBuffer(device.handle, static_cast<const OrlBuffer&>(buffer).data(),
                    buffer.byte_size()))
            {
                append_errors(errors, gpu->Errors());
                return false;
            }
            device.uploaded_version = buffer.version();
        }
        *handle = device.handle;
        return true;
    }

    bool initialize() {
        orlcomp::Parser parser(program->source);
        if (!parse_source(program->source, parser, errors)) {
            return false;
        }

        const auto target = backend == Backend::Cpu
            ? orlcomp::OrlCodegenTarget::Host
            : orlcomp::OrlCodegenTarget::Cuda;
        orlcomp::LlvmIrCodegen codegen(program->options.source_name, target);
        if (!codegen.Generate(*parser.Ast())) {
            append_errors(errors, codegen.Errors());
            return false;
        }
        ir = codegen.DumpIR();

        if (backend == Backend::Cpu) {
            jit = std::make_unique<orlcomp::OrlJitEngine>(orlcomp::OrlJitTarget::Native);
            if (!jit->LoadModuleWithOptimization(codegen.ReleaseModule(), codegen.ReleaseContext())) {
                append_errors(errors, jit->Errors());
                return false;
            }
            initialized = true;
            return true;
        }

        gpu = std::make_unique<orlcomp::OrlGpuEngine>(orlcomp::OrlGpuBackend::Cuda);
        gpu->SetCudaEntryFunction(program->options.entry_function);
        if (!gpu->CompileModuleWithOptimization(codegen.ReleaseModule(), codegen.ReleaseContext())) {
            append_errors(errors, gpu->Errors());
            return false;
        }
        if (!gpu->LoadToDriver()) {
            append_errors(errors, gpu->Errors());
            return false;
        }
        initialized = true;
        return true;
    }
};

OrlExecution::OrlExecution(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

OrlExecution::~OrlExecution() = default;
OrlExecution::OrlExecution(OrlExecution&&) noexcept = default;
OrlExecution& OrlExecution::operator=(OrlExecution&&) noexcept = default;

OrlExecution OrlExecution::Create(const OrlProgram& program, Backend backend) {
    auto impl = std::make_unique<Impl>();
    impl->backend = backend;
    if (!program.valid()) {
        append_errors(impl->errors, program.errors());
        return OrlExecution(std::move(impl));
    }
    impl->program = program.impl_;
    impl->initialize();
    return OrlExecution(std::move(impl));
}

bool OrlExecution::bind_buffer(std::string_view parameter, OrlBuffer& buffer) {
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    const auto* desc = impl_->parameter(parameter);
    if (desc == nullptr || desc->kind != ParameterKind::Buffer) {
        impl_->errors.emplace_back("Parameter '" + std::string(parameter) + "' is not a buffer");
        return false;
    }
    impl_->buffers[std::string(parameter)] = &buffer;
    impl_->release_device_binding(std::string(parameter));
    return true;
}

bool OrlExecution::bind_device_buffer(std::string_view parameter, std::uint64_t device_ptr,
    std::size_t bytes)
{
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    if (impl_->backend != Backend::Cuda || impl_->gpu == nullptr) {
        impl_->errors.emplace_back("bind_device_buffer requires the CUDA backend");
        return false;
    }
    const auto* desc = impl_->parameter(parameter);
    if (desc == nullptr || desc->kind != ParameterKind::Buffer) {
        impl_->errors.emplace_back("Parameter '" + std::string(parameter) + "' is not a buffer");
        return false;
    }
    if (device_ptr == 0 || bytes == 0) {
        impl_->errors.emplace_back("Device buffer binding for '" + std::string(parameter)
            + "' requires a non-null CUDA pointer and non-zero size");
        return false;
    }

    const std::string name(parameter);
    impl_->buffers.erase(name);
    auto& bound = impl_->device_bindings[name];
    if (bound.handle != 0 && bound.device_ptr == device_ptr && bound.bytes == bytes) {
        return true;
    }
    if (bound.handle != 0 && !impl_->gpu->FreeBuffer(bound.handle)) {
        append_errors(impl_->errors, impl_->gpu->Errors());
        impl_->device_bindings.erase(name);
        return false;
    }

    const auto imported = impl_->gpu->ImportBuffer(device_ptr, bytes);
    if (!imported.has_value()) {
        append_errors(impl_->errors, impl_->gpu->Errors());
        impl_->device_bindings.erase(name);
        return false;
    }
    bound.device_ptr = device_ptr;
    bound.bytes = bytes;
    bound.handle = *imported;
    return true;
}

bool OrlExecution::bind_int(std::string_view parameter, std::int64_t value) {
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    const auto* desc = impl_->parameter(parameter);
    if (desc == nullptr || desc->kind != ParameterKind::Int64) {
        impl_->errors.emplace_back("Parameter '" + std::string(parameter) + "' is not an int");
        return false;
    }
    impl_->integers[std::string(parameter)] = value;
    return true;
}

bool OrlExecution::bind_float(std::string_view parameter, double value) {
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    const auto* desc = impl_->parameter(parameter);
    if (desc == nullptr || desc->kind != ParameterKind::Float64) {
        impl_->errors.emplace_back("Parameter '" + std::string(parameter) + "' is not a float");
        return false;
    }
    impl_->floats[std::string(parameter)] = value;
    return true;
}

void OrlExecution::clear_bindings() {
    impl_->release_device_bindings();
    impl_->buffers.clear();
    impl_->integers.clear();
    impl_->floats.clear();
}

bool OrlExecution::valid() const {
    return impl_ != nullptr && impl_->initialized;
}

std::optional<std::int64_t> OrlExecution::evaluate(std::uint32_t element_count) {
    if (!impl_->initialized) {
        if (impl_->errors.empty()) {
            impl_->errors.emplace_back("ORL execution was not initialized");
        }
        return std::nullopt;
    }
    std::vector<void*> host_buffers;
    std::vector<std::int64_t> integers;
    std::vector<double> floats;
    if (!impl_->validate_bindings(host_buffers, integers, floats)) {
        return std::nullopt;
    }

    if (impl_->backend == Backend::Cpu) {
        const std::string wrapper = "__orl_host_entry_" + impl_->program->options.entry_function;
        const auto result = impl_->jit->InvokeInt64WithRuntimeArgs(
            wrapper, host_buffers.data(), integers.data(), floats.data());
        if (!result.has_value()) {
            append_errors(impl_->errors, impl_->jit->Errors());
        }
        return result;
    }

    std::vector<orlcomp::OrlGpuKernelArgument> arguments;
    arguments.reserve(impl_->program->parameters.size());
    for (const auto& parameter : impl_->program->parameters) {
        if (parameter.kind == ParameterKind::Buffer) {
            orlcomp::OrlGpuBuffer handle = 0;
            const auto device = impl_->device_bindings.find(parameter.name);
            if (device != impl_->device_bindings.end()) {
                handle = device->second.handle;
            } else if (!impl_->ensure_device_buffer(*impl_->buffers.at(parameter.name), &handle)) {
                return std::nullopt;
            }
            orlcomp::OrlGpuKernelArgument argument;
            argument.is_buffer = true;
            argument.buffer = handle;
            argument.scalar_type = orlcomp::OrlGpuKernelParameterType::Buffer;
            arguments.push_back(std::move(argument));
        } else if (parameter.kind == ParameterKind::Int64) {
            orlcomp::OrlGpuKernelArgument argument;
            argument.scalar_type = orlcomp::OrlGpuKernelParameterType::Int64;
            const auto value = impl_->integers.at(parameter.name);
            argument.scalar_bytes.resize(sizeof(value));
            std::memcpy(argument.scalar_bytes.data(), &value, sizeof(value));
            arguments.push_back(std::move(argument));
        } else {
            orlcomp::OrlGpuKernelArgument argument;
            argument.scalar_type = orlcomp::OrlGpuKernelParameterType::Float64;
            const auto value = impl_->floats.at(parameter.name);
            argument.scalar_bytes.resize(sizeof(value));
            std::memcpy(argument.scalar_bytes.data(), &value, sizeof(value));
            arguments.push_back(std::move(argument));
        }
    }

    if (!impl_->gpu->SetupCudaKernelArguments(orlcomp::OrlGpuEngine::CudaEntryKernelName,
            std::move(arguments))
        || !impl_->gpu->LaunchCudaKernelForElements(element_count)
        || !impl_->gpu->Synchronize())
    {
        append_errors(impl_->errors, impl_->gpu->Errors());
        return std::nullopt;
    }

    for (const auto& [name, buffer] : impl_->buffers) {
        if (impl_->device_bindings.contains(name)) {
            continue;
        }
        const auto device = impl_->device_buffers.find(buffer);
        if (device == impl_->device_buffers.end() || buffer->byte_size() == 0) {
            continue;
        }
        if (!impl_->gpu->DownloadBuffer(device->second.handle,
                const_cast<void*>(static_cast<const OrlBuffer*>(buffer)->data()), buffer->byte_size()))
        {
            append_errors(impl_->errors, impl_->gpu->Errors());
            return std::nullopt;
        }
    }

    std::int32_t result = 0;
    if (!impl_->gpu->ReadCudaGlobalInt32(orlcomp::OrlGpuEngine::CudaResultSymbolName, &result)) {
        append_errors(impl_->errors, impl_->gpu->Errors());
        return std::nullopt;
    }
    return static_cast<std::int64_t>(result);
}

bool OrlExecution::synchronize() {
    if (impl_->backend == Backend::Cpu) {
        return true;
    }
    impl_->errors.clear();
    if (!impl_->gpu->Synchronize()) {
        append_errors(impl_->errors, impl_->gpu->Errors());
        return false;
    }
    return true;
}

Backend OrlExecution::backend() const {
    return impl_->backend;
}

const std::vector<std::string>& OrlExecution::errors() const {
    return impl_->errors;
}

const std::string& OrlExecution::ir() const {
    static const std::string empty;
    return impl_ != nullptr ? impl_->ir : empty;
}

} // namespace ORL::exec
