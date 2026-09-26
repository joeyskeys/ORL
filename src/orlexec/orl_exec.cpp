#include "orl_exec.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>

#include "orl_codegen.h"
#include "orl_analysis.h"
#include "orl_gpu.h"
#include "orl_jit.h"
#include "orl_parser.h"
#include "orl_runtime_signature.h"
#include "orlrig/abi.hpp"
#include "orlrig/handle_registry.hpp"

#include <llvm/Config/llvm-config.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#if __has_include(<llvm/TargetParser/Host.h>)
#include <llvm/TargetParser/Host.h>
#define ORL_HAS_LLVM_HOST_TRIPLE 1
#elif __has_include(<llvm/Support/Host.h>)
#include <llvm/Support/Host.h>
#define ORL_HAS_LLVM_HOST_TRIPLE 1
#endif

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
    if (type_name == orlrig::kJointOrlType) {
        return orlrig::kJointStride;
    }
    if (type_name == orlrig::kLocatorOrlType) {
        return orlrig::kLocatorStride;
    }
    if (type_name == orlrig::kWeightOrlType) {
        return orlrig::kWeightStride;
    }
    if (type_name == orlrig::kSolverContextOrlType) {
        return orlrig::kSolverContextStride;
    }
    if (type_name == orlrig::kHierarchyContextOrlType) {
        return orlrig::kHierarchyContextStride;
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

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start, Clock::time_point end = Clock::now()) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

const char* backend_label(Backend backend) {
    return backend == Backend::Cuda ? "CUDA" : "CPU";
}

void register_handle_view_symbols(orlcomp::OrlJitEngine& jit) {
    jit.RegisterRuntimeSymbol(
        "__orlrig_joint_read_i64",
        reinterpret_cast<void*>(&orlrig::__orlrig_joint_read_i64));
    jit.RegisterRuntimeSymbol(
        "__orlrig_joint_write_i64",
        reinterpret_cast<void*>(&orlrig::__orlrig_joint_write_i64));
    jit.RegisterRuntimeSymbol(
        "__orlrig_joint_read_vec4",
        reinterpret_cast<void*>(&orlrig::__orlrig_joint_read_vec4));
    jit.RegisterRuntimeSymbol(
        "__orlrig_joint_write_vec4",
        reinterpret_cast<void*>(&orlrig::__orlrig_joint_write_vec4));
    jit.RegisterRuntimeSymbol(
        "__orlrig_locator_read_matrix",
        reinterpret_cast<void*>(&orlrig::__orlrig_locator_read_matrix));
    jit.RegisterRuntimeSymbol(
        "__orlrig_locator_write_matrix",
        reinterpret_cast<void*>(&orlrig::__orlrig_locator_write_matrix));
    jit.RegisterRuntimeSymbol(
        "__orlrig_world_read_matrix",
        reinterpret_cast<void*>(&orlrig::__orlrig_world_read_matrix));
    jit.RegisterRuntimeSymbol(
        "__orlrig_world_write_matrix",
        reinterpret_cast<void*>(&orlrig::__orlrig_world_write_matrix));
}

std::string execution_cache_material(
    Backend backend, std::string_view source, std::string_view entry,
    std::string_view source_name,
    const std::vector<std::string>& include_paths)
{
    std::string material = "orl-binary-cache-v1\n";
    material += "backend=";
    material += backend == Backend::Cuda ? "cuda\n" : "cpu\n";
    material += "target=";
    material += backend == Backend::Cuda ? "cuda-sm-52\n" : "native-host\n";
    material += "runtime_abi=orl-runtime-signature-v2\n";
    material += "handle_abi=";
    material += orlcomp::kHandleAbiVersion;
    material += "\n";
    material += "handle_view_registry=";
    material += orlcomp::global_handle_view_registry().fingerprint();
    material += "\n";
    material += "llvm_version=";
    material += LLVM_VERSION_STRING;
    material += "\nhost_triple=";
#if defined(ORL_HAS_LLVM_HOST_TRIPLE)
    material += llvm::sys::getDefaultTargetTriple();
#else
    material += "unknown";
#endif
    material += "pointer_size=";
    material += std::to_string(sizeof(void*));
    material += "\nentry=";
    material += entry;
    material += "\nsource_name=";
    material += source_name;
    material += "\ninclude_paths=";
    for (const auto& include_path : include_paths) {
        material += include_path;
        material.push_back('\n');
    }
    material += "source_size=";
    material += std::to_string(source.size());
    material += "\nsource=";
    material += source;
    return material;
}

orlcomp::OrlGpuKernelParameterType gpu_parameter_type(ParameterKind kind) {
    switch (kind) {
    case ParameterKind::Buffer:
        return orlcomp::OrlGpuKernelParameterType::Buffer;
    case ParameterKind::Int64:
        return orlcomp::OrlGpuKernelParameterType::Int64;
    case ParameterKind::Float64:
        return orlcomp::OrlGpuKernelParameterType::Float64;
    case ParameterKind::Handle:
        return orlcomp::OrlGpuKernelParameterType::Handle;
    case ParameterKind::Unsupported:
        return orlcomp::OrlGpuKernelParameterType::Unsupported;
    }
    return orlcomp::OrlGpuKernelParameterType::Unsupported;
}

std::string fmt_ms(double ms) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", ms);
    return buf;
}

void print_jit(const std::string& entry, Backend backend, const std::string& source_name,
    double parse_ms, double codegen_ms, double compile_ms, double load_ms, bool ok)
{
    std::cout << "ORL JIT '" << entry << "' " << backend_label(backend)
        << " (" << source_name << "): parse " << fmt_ms(parse_ms) << "ms  codegen "
        << fmt_ms(codegen_ms) << "ms  ";
    if (backend == Backend::Cpu) {
        std::cout << "jit " << fmt_ms(compile_ms) << "ms";
    } else {
        std::cout << "ptx " << fmt_ms(compile_ms) << "ms  load " << fmt_ms(load_ms) << "ms";
    }
    const double total = parse_ms + codegen_ms + compile_ms + load_ms;
    std::cout << "  total " << fmt_ms(total) << "ms";
    if (!ok) {
        std::cout << "  FAILED";
    }
    std::cout << '\n';
}

struct KernelTiming {
    double upload_ms = 0;
    double kernel_ms = 0;
    double download_ms = 0;
    double total_ms = 0;
    std::size_t upload_calls = 0;
    std::size_t upload_bytes = 0;
};

struct ExecStats {
    std::uint64_t count = 0;
    double sum_ms = 0;
    double min_ms = 1.0e300;
    double max_ms = 0;
    Clock::time_point last_print{};
};

void print_kernel(const std::string& entry, Backend backend, std::uint32_t element_count,
    const KernelTiming& timing, const ExecStats& stats, bool detail)
{
    std::cout << "ORL exec '" << entry << "' " << backend_label(backend);
    if (detail) {
        std::cout << " elems=" << element_count
            << " kernel=" << fmt_ms(timing.kernel_ms) << "ms";
        if (backend == Backend::Cuda) {
            std::cout << " upload=" << fmt_ms(timing.upload_ms) << "ms"
                << " (" << timing.upload_calls << " calls "
                << timing.upload_bytes << " bytes)"
                << " download=" << fmt_ms(timing.download_ms) << "ms";
        }
        std::cout << " total=" << fmt_ms(timing.total_ms) << "ms\n";
        return;
    }
    const double avg = stats.count == 0 ? 0.0 : stats.sum_ms / static_cast<double>(stats.count);
    std::cout << " n=" << stats.count
        << " last=" << fmt_ms(timing.total_ms) << "ms";
    if (backend == Backend::Cuda) {
        std::cout << " (kernel " << fmt_ms(timing.kernel_ms)
            << " upload " << fmt_ms(timing.upload_ms)
            << " [" << timing.upload_calls << " calls "
            << timing.upload_bytes << " bytes]"
            << " download " << fmt_ms(timing.download_ms) << ")";
    }
    std::cout << " avg=" << fmt_ms(avg) << "ms min=" << fmt_ms(stats.min_ms)
        << "ms max=" << fmt_ms(stats.max_ms) << "ms\n";
}

void record_kernel(const std::string& entry, Backend backend, std::uint32_t element_count,
    const KernelTiming& timing, ExecStats& stats)
{
    ++stats.count;
    stats.sum_ms += timing.total_ms;
    if (timing.total_ms < stats.min_ms) {
        stats.min_ms = timing.total_ms;
    }
    if (timing.total_ms > stats.max_ms) {
        stats.max_ms = timing.total_ms;
    }
    const auto now = Clock::now();
    const bool first = stats.count <= 3;
    const bool periodic = stats.last_print.time_since_epoch().count() == 0
        || (now - stats.last_print) >= std::chrono::seconds(1);
    if (!first && !periodic) {
        return;
    }
    print_kernel(entry, backend, element_count, timing, stats, first);
    stats.last_print = now;
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
    bool handle_views = false;

    bool has_handle_parameters() const {
        return std::any_of(parameters.begin(), parameters.end(),
            [](const ParameterDesc& parameter) {
                return parameter.kind == ParameterKind::Handle;
            });
    }
};

OrlProgram::OrlProgram(std::shared_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

OrlProgram OrlProgram::Compile(std::string source, CompileOptions options) {
    auto impl = std::make_shared<Impl>();
    impl->source = std::move(source);
    impl->options = std::move(options);
    std::string view_error;
    if (!orlrig::register_rig_handle_views(
            orlcomp::global_handle_view_registry(), &view_error))
    {
        impl->errors.push_back(
            "ORL handle view registration failed: " + view_error);
        return OrlProgram(std::move(impl));
    }
    impl->handle_views = impl->source.find("Joint(")
            != std::string::npos
        || impl->source.find("Locator(") != std::string::npos
        || impl->source.find("WorldTransform(")
            != std::string::npos;

    orlcomp::Parser parser(
        impl->source, impl->options.source_name);
    for (const auto& include_path : impl->options.include_paths) {
        parser.AddIncludePath(include_path);
    }
    if (!parse_source(impl->source, parser, impl->errors)) {
        return OrlProgram(std::move(impl));
    }
    const auto analysis = orlcomp::SemanticAnalyzer{}.analyze(
        *parser.Ast(), impl->options.source_name,
        impl->options.handle_type_identities);
    if (!analysis.ok()) {
        for (const auto& diagnostic : analysis.diagnostics) {
            impl->errors.push_back(
                diagnostic.code + ": " + diagnostic.message);
        }
        return OrlProgram(std::move(impl));
    }
    impl->handle_views = impl->handle_views
        || std::any_of(analysis.functions.begin(),
            analysis.functions.end(),
            [](const orlcomp::FunctionSummary& function) {
                return !function.handle_view_effects.empty();
            });
    orlcomp::HandleTypeRegistry handle_registry;
    for (const auto& handle : analysis.handle_types) {
        std::string collision_error;
        if (!handle_registry.register_type(
                handle.canonical_name, &collision_error))
        {
            impl->errors.push_back(
                "ORL_RUNTIME_HANDLE_TYPE: " + collision_error);
        }
    }
    if (!impl->errors.empty()) {
        return OrlProgram(std::move(impl));
    }
    const auto signature =
        orlcomp::DescribeRuntimeFunction(
            *parser.Ast(), impl->options.entry_function,
            impl->options.handle_type_identities);
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
        } else if (parameter.kind
            == orlcomp::OrlRuntimeParameterKind::Handle) {
            desc.kind = ParameterKind::Handle;
            desc.canonical_type_name = parameter.canonical_type_name;
            desc.handle_type_id = parameter.handle_type_id;
        } else {
            if (parameter.type_name == "handle") {
                impl->errors.emplace_back(
                    "ORL_RUNTIME_HANDLE_UNSUPPORTED: universal handle "
                    "parameters are not supported by this ABI");
                continue;
            }
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

bool OrlProgram::has_handle_parameters() const {
    return impl_ != nullptr && std::any_of(
        impl_->parameters.begin(), impl_->parameters.end(),
        [](const ParameterDesc& parameter) {
            return parameter.kind == ParameterKind::Handle;
        });
}

bool OrlProgram::has_handle_views() const {
    return impl_ != nullptr && impl_->handle_views;
}

const std::vector<std::string>& OrlProgram::errors() const {
    return impl_->errors;
}

struct OrlExecution::Impl {
    struct DeviceBuffer {
        orlcomp::OrlGpuBuffer handle = 0;
        std::size_t capacity_bytes = 0;
        std::uint64_t uploaded_version = 0;
        const void* source_data = nullptr;
    };

    struct PackedBinding {
        std::size_t offset = 0;
        std::size_t bytes = 0;
    };

    struct PackedStorage {
        void* data = nullptr;
        std::size_t bytes = 0;
        std::uint64_t version = 0;
    };

    struct ExternalDeviceBuffer {
        std::uint64_t device_ptr = 0;
        std::size_t bytes = 0;
        orlcomp::OrlGpuBuffer handle = 0;
    };

    std::shared_ptr<OrlProgram::Impl> program;
    Backend backend = Backend::Cpu;
    orlcomp::OrlBinaryCacheOptions cache_options;
    std::string cache_key;
    std::unordered_map<std::string, OrlBuffer*> buffers;
    std::unordered_map<std::string, ExternalDeviceBuffer> device_bindings;
    std::unordered_map<std::string, PackedBinding> packed_bindings;
    std::unordered_map<std::string, std::int64_t> integers;
    std::unordered_map<std::string, double> floats;
    std::unordered_map<std::string, HandleValue> handles;
    std::unordered_map<OrlBuffer*, DeviceBuffer> device_buffers;
    std::optional<PackedStorage> packed_storage;
    DeviceBuffer packed_device;
    std::optional<OrlBuffer> solver_context;
    std::optional<OrlBuffer> hierarchy_context;
    std::unique_ptr<orlcomp::OrlJitEngine> jit;
    std::unique_ptr<orlcomp::OrlGpuEngine> gpu;
    orlrig::HandleViewContext* handle_view_context = nullptr;
    orlcomp::OrlGpuBuffer handle_context_device = 0;
    orlcomp::OrlGpuBuffer handle_joint_device = 0;
    orlcomp::OrlGpuBuffer handle_locator_device = 0;
    std::size_t handle_joint_bytes = 0;
    std::size_t handle_locator_bytes = 0;
    std::vector<std::string> errors;
    std::string ir;
    bool initialized = false;
    ExecStats exec_stats;

    ~Impl() {
        release_device_bindings();
        if (gpu != nullptr) {
            if (handle_context_device != 0) {
                gpu->FreeBuffer(handle_context_device);
            }
            if (handle_joint_device != 0) {
                gpu->FreeBuffer(handle_joint_device);
            }
            if (handle_locator_device != 0) {
                gpu->FreeBuffer(handle_locator_device);
            }
            for (const auto& [_, buffer] : device_buffers) {
                if (buffer.handle != 0) {
                    gpu->FreeBuffer(buffer.handle);
                }
            }
            if (packed_device.handle != 0) {
                gpu->FreeBuffer(packed_device.handle);
            }
        }
    }

    void release_packed_device() {
        if (gpu != nullptr && packed_device.handle != 0) {
            gpu->FreeBuffer(packed_device.handle);
        }
        packed_device = {};
    }

    void* solver_context_data() {
        if (packed_storage.has_value()) {
            return packed_storage->data;
        }
        return solver_context.has_value() ? solver_context->data() : nullptr;
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
        std::vector<double>& ordered_floats,
        std::vector<std::uint64_t>& ordered_handles)
    {
        errors.clear();
        for (const auto& parameter : program->parameters) {
            if (parameter.kind == ParameterKind::Unsupported) {
                errors.emplace_back("Unsupported runtime parameter type '" + parameter.orl_type
                    + "' for '" + parameter.name + "'");
                continue;
            }
            if (parameter.kind == ParameterKind::Buffer) {
                if (parameter.name == orlcomp::kSolverContextParameterName) {
                    if (!solver_context.has_value()) {
                        errors.emplace_back(
                            "Implicit SolverContext storage is unavailable");
                        continue;
                    }
                    if (solver_context->orl_type() != parameter.orl_type
                        || solver_context->element_stride()
                            != parameter.element_stride)
                    {
                        errors.emplace_back(
                            "Implicit SolverContext buffer ABI is incompatible");
                        continue;
                    }
                    ordered_buffers.push_back(
                        backend == Backend::Cpu
                            ? solver_context->data() : nullptr);
                    continue;
                }
                if (parameter.name == orlcomp::kHierarchyContextParameterName) {
                    if (!hierarchy_context.has_value()) {
                        errors.emplace_back(
                            "Implicit HierarchyContext storage is unavailable");
                        continue;
                    }
                    if (hierarchy_context->orl_type()
                            != parameter.orl_type
                        || hierarchy_context->element_stride()
                            != parameter.element_stride)
                    {
                        errors.emplace_back(
                            "Implicit HierarchyContext buffer ABI is incompatible");
                        continue;
                    }
                    ordered_buffers.push_back(
                        backend == Backend::Cpu
                            ? hierarchy_context->data() : nullptr);
                    continue;
                }
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
                const auto packed = packed_bindings.find(parameter.name);
                if (packed != packed_bindings.end()) {
                    if (backend != Backend::Cuda
                        || !packed_storage.has_value()
                        || packed_storage->data == nullptr
                        || packed->second.offset > packed_storage->bytes
                        || packed->second.bytes
                            > packed_storage->bytes - packed->second.offset)
                    {
                        errors.emplace_back(
                            "Packed buffer binding for '" + parameter.name
                            + "' is invalid for this backend");
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
            } else if (parameter.kind == ParameterKind::Handle) {
                const auto bound = handles.find(parameter.name);
                if (bound == handles.end()) {
                    errors.emplace_back(
                        "Missing handle binding for parameter '"
                        + parameter.name + "'");
                    continue;
                }
                if (!orlcomp::IsValidHandleValue(bound->second)) {
                    errors.emplace_back(
                        "Invalid handle binding for parameter '"
                        + parameter.name + "'");
                    continue;
                }
                if (bound->second.type_id != parameter.handle_type_id) {
                    errors.emplace_back(
                        "Handle type mismatch for parameter '"
                        + parameter.name + "': expected "
                        + parameter.canonical_type_name);
                    continue;
                }
                ordered_handles.push_back(bound->second.type_id);
                ordered_handles.push_back(
                    static_cast<std::uint64_t>(bound->second.slot));
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

    bool ensure_device_buffer(OrlBuffer& buffer,
        orlcomp::OrlGpuBuffer* handle,
        std::size_t* upload_calls,
        std::size_t* upload_bytes) {
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
            if (upload_calls != nullptr) {
                ++*upload_calls;
            }
            if (upload_bytes != nullptr) {
                *upload_bytes += buffer.byte_size();
            }
            device.uploaded_version = buffer.version();
        }
        *handle = device.handle;
        return true;
    }

    bool ensure_packed_device_buffer(
        orlcomp::OrlGpuBuffer* handle,
        std::size_t* upload_calls,
        std::size_t* upload_bytes)
    {
        if (!packed_storage.has_value()
            || packed_storage->data == nullptr
            || packed_storage->bytes == 0)
        {
            errors.emplace_back("Packed CUDA buffer storage is unavailable");
            return false;
        }

        if (packed_device.handle == 0
            || packed_device.capacity_bytes != packed_storage->bytes
            || packed_device.source_data != packed_storage->data)
        {
            release_packed_device();
            const auto allocated = gpu->AllocateBuffer(packed_storage->bytes);
            if (!allocated.has_value()) {
                append_errors(errors, gpu->Errors());
                return false;
            }
            packed_device.handle = *allocated;
            packed_device.capacity_bytes = packed_storage->bytes;
            packed_device.uploaded_version = 0;
            packed_device.source_data = packed_storage->data;
        }

        if (packed_device.uploaded_version != packed_storage->version) {
            if (!gpu->UploadBuffer(
                    packed_device.handle,
                    packed_storage->data,
                    packed_storage->bytes))
            {
                append_errors(errors, gpu->Errors());
                return false;
            }
            if (upload_calls != nullptr) {
                ++*upload_calls;
            }
            if (upload_bytes != nullptr) {
                *upload_bytes += packed_storage->bytes;
            }
            packed_device.uploaded_version = packed_storage->version;
        }
        *handle = packed_device.handle;
        return true;
    }

    bool ensure_handle_context_device(
        std::size_t* upload_calls, std::size_t* upload_bytes)
    {
        if (handle_view_context == nullptr) {
            errors.emplace_back(
                "CUDA view access requires a bound handle context");
            return false;
        }
        const auto upload_arena = [&](const orlrig::HandleStorage& storage,
            orlcomp::OrlGpuBuffer* device_handle,
            std::size_t* device_bytes) {
            if (storage.data == nullptr || storage.count == 0
                || storage.stride == 0)
            {
                *device_handle = 0;
                *device_bytes = 0;
                return true;
            }
            if (storage.count > std::numeric_limits<std::size_t>::max()
                    / storage.stride)
            {
                errors.emplace_back("Handle arena size overflows");
                return false;
            }
            const std::size_t bytes = storage.count * storage.stride;
            if (*device_handle == 0 || *device_bytes != bytes) {
                if (*device_handle != 0) {
                    gpu->FreeBuffer(*device_handle);
                }
                const auto allocated = gpu->AllocateBuffer(bytes);
                if (!allocated.has_value()) {
                    append_errors(errors, gpu->Errors());
                    return false;
                }
                *device_handle = *allocated;
                *device_bytes = bytes;
            }
            if (!gpu->UploadBuffer(
                    *device_handle, storage.data, bytes))
            {
                append_errors(errors, gpu->Errors());
                return false;
            }
            if (upload_calls != nullptr) {
                ++*upload_calls;
            }
            if (upload_bytes != nullptr) {
                *upload_bytes += bytes;
            }
            return true;
        };
        if (!upload_arena(handle_view_context->joints,
                &handle_joint_device, &handle_joint_bytes)
            || !upload_arena(handle_view_context->locators,
                &handle_locator_device, &handle_locator_bytes))
        {
            return false;
        }
        orlcomp::HandleDeviceContext device_context;
        const auto joint_pointer = handle_joint_device == 0
            ? std::optional<std::uint64_t>{}
            : gpu->DeviceBufferPointer(handle_joint_device);
        const auto locator_pointer = handle_locator_device == 0
            ? std::optional<std::uint64_t>{}
            : gpu->DeviceBufferPointer(handle_locator_device);
        if (handle_joint_device != 0 && !joint_pointer.has_value()) {
            append_errors(errors, gpu->Errors());
            return false;
        }
        if (handle_locator_device != 0 && !locator_pointer.has_value()) {
            append_errors(errors, gpu->Errors());
            return false;
        }
        device_context.joint_arena =
            joint_pointer.value_or(0);
        device_context.locator_arena =
            locator_pointer.value_or(0);
        device_context.joint_count = handle_view_context->joints.count;
        device_context.locator_count = handle_view_context->locators.count;
        device_context.joint_stride = handle_view_context->joints.stride;
        device_context.locator_stride = handle_view_context->locators.stride;
        device_context.topology_revision =
            handle_view_context->topology_revision;
        device_context.flags =
            (handle_view_context->joints.writable ? 1ull : 0ull)
            | (handle_view_context->locators.writable ? 2ull : 0ull);
        if (handle_context_device == 0) {
            const auto allocated = gpu->AllocateBuffer(
                sizeof(device_context));
            if (!allocated.has_value()) {
                append_errors(errors, gpu->Errors());
                return false;
            }
            handle_context_device = *allocated;
        }
        if (!gpu->UploadBuffer(handle_context_device,
                &device_context, sizeof(device_context)))
        {
            append_errors(errors, gpu->Errors());
            return false;
        }
        if (upload_calls != nullptr) {
            ++*upload_calls;
        }
        if (upload_bytes != nullptr) {
            *upload_bytes += sizeof(device_context);
        }
        return true;
    }

    void configure_cached_gpu_parameters() {
        if (gpu == nullptr) {
            return;
        }
        std::vector<orlcomp::OrlGpuKernelParameter> parameters;
        parameters.reserve(program->parameters.size());
        for (const auto& parameter : program->parameters) {
            orlcomp::OrlGpuKernelParameter reflected{
                parameter.name, gpu_parameter_type(parameter.kind)};
            if (parameter.kind == ParameterKind::Handle) {
                reflected.byte_size = sizeof(HandleValue);
                reflected.alignment = alignof(HandleValue);
                reflected.lane_count = 2;
                reflected.lane_order = "type_id,slot";
            }
            parameters.push_back(std::move(reflected));
        }
        if (program->handle_views) {
            parameters.push_back({
                std::string{orlcomp::kHandleContextParameterName},
                orlcomp::OrlGpuKernelParameterType::Buffer});
        }
        gpu->SetCudaEntryParameters(std::move(parameters));
    }

    bool try_load_cached_cpu() {
        if (backend != Backend::Cpu
            || cache_key.empty()
            || cache_options.directory.empty()
            || cache_options.force_recompile)
        {
            return false;
        }

        orlcomp::OrlBinaryCache cache(cache_options);
        std::vector<std::uint8_t> object;
        if (!cache.load(orlcomp::OrlBinaryKind::CpuObject, cache_key, object)) {
            return false;
        }

        jit = std::make_unique<orlcomp::OrlJitEngine>(
            orlcomp::OrlJitTarget::Native, cache_options);
        register_handle_view_symbols(*jit);
        if (!jit->LoadObject(object)) {
            jit.reset();
            return false;
        }

        std::cout << "ORL binary cache hit '"
            << program->options.entry_function
            << "' CPU (" << program->options.source_name << ")\n";
        initialized = true;
        return true;
    }

    bool try_load_cached_gpu() {
        if (backend != Backend::Cuda
            || cache_key.empty()
            || cache_options.directory.empty()
            || cache_options.force_recompile)
        {
            return false;
        }

        orlcomp::OrlBinaryCache cache(cache_options);
        for (const auto kind : {
                 orlcomp::OrlBinaryKind::CudaCubin,
                 orlcomp::OrlBinaryKind::CudaPtx})
        {
            std::vector<std::uint8_t> binary;
            if (!cache.load(kind, cache_key, binary)) {
                continue;
            }

            gpu = std::make_unique<orlcomp::OrlGpuEngine>(
                orlcomp::OrlGpuBackend::Cuda);
            gpu->SetCudaEntryFunction(program->options.entry_function);
            configure_cached_gpu_parameters();
            if (kind == orlcomp::OrlBinaryKind::CudaCubin) {
                gpu->SetDeviceBinary(std::move(binary));
            } else {
                gpu->SetDeviceCode(std::string(
                    reinterpret_cast<const char*>(binary.data()),
                    binary.size()));
            }
            if (gpu->LoadToDriver()) {
                if (!gpu->DeviceBinary().empty()) {
                    cache.save(
                        orlcomp::OrlBinaryKind::CudaCubin,
                        cache_key,
                        std::span<const std::uint8_t>(
                            gpu->DeviceBinary().data(),
                            gpu->DeviceBinary().size()));
                }
                std::cout << "ORL binary cache hit '"
                    << program->options.entry_function
                    << "' CUDA (" << program->options.source_name << ")\n";
                initialized = true;
                return true;
            }
            gpu.reset();
        }
        return false;
    }

    bool initialize() {
        const auto t0 = Clock::now();
        if (try_load_cached_cpu() || try_load_cached_gpu()) {
            return true;
        }
        orlcomp::Parser parser(
            program->source, program->options.source_name);
        for (const auto& include_path : program->options.include_paths) {
            parser.AddIncludePath(include_path);
        }
        if (!parse_source(program->source, parser, errors)) {
            print_jit(program->options.entry_function, backend, program->options.source_name,
                elapsed_ms(t0), 0, 0, 0, false);
            return false;
        }
        const auto t1 = Clock::now();

        const auto target = backend == Backend::Cpu
            ? orlcomp::OrlCodegenTarget::Host
            : orlcomp::OrlCodegenTarget::Cuda;
        orlcomp::LlvmIrCodegen codegen(program->options.source_name, target);
        codegen.SetHandleTypeIdentities(
            program->options.handle_type_identities);
        if (!codegen.Generate(*parser.Ast())) {
            append_errors(errors, codegen.Errors());
            print_jit(program->options.entry_function, backend, program->options.source_name,
                elapsed_ms(t0, t1), elapsed_ms(t1), 0, 0, false);
            return false;
        }
        ir = codegen.DumpIR();
        const auto t2 = Clock::now();

        if (backend == Backend::Cpu) {
            jit = std::make_unique<orlcomp::OrlJitEngine>(
                orlcomp::OrlJitTarget::Native, cache_options);
            register_handle_view_symbols(*jit);
            if (!jit->LoadModuleWithOptimization(
                    codegen.ReleaseModule(), codegen.ReleaseContext(),
                    orlcomp::OrlOptimizationLevel::O2, cache_key))
            {
                append_errors(errors, jit->Errors());
                print_jit(program->options.entry_function, backend, program->options.source_name,
                    elapsed_ms(t0, t1), elapsed_ms(t1, t2), elapsed_ms(t2), 0, false);
                return false;
            }
            print_jit(program->options.entry_function, backend, program->options.source_name,
                elapsed_ms(t0, t1), elapsed_ms(t1, t2), elapsed_ms(t2), 0, true);
            initialized = true;
            return true;
        }

        gpu = std::make_unique<orlcomp::OrlGpuEngine>(orlcomp::OrlGpuBackend::Cuda);
        gpu->SetCudaEntryFunction(program->options.entry_function);
        if (!gpu->CompileModuleWithOptimization(codegen.ReleaseModule(), codegen.ReleaseContext())) {
            append_errors(errors, gpu->Errors());
            print_jit(program->options.entry_function, backend, program->options.source_name,
                elapsed_ms(t0, t1), elapsed_ms(t1, t2), elapsed_ms(t2), 0, false);
            return false;
        }
        const auto t3 = Clock::now();
        if (!gpu->LoadToDriver()) {
            append_errors(errors, gpu->Errors());
            print_jit(program->options.entry_function, backend, program->options.source_name,
                elapsed_ms(t0, t1), elapsed_ms(t1, t2), elapsed_ms(t2, t3), elapsed_ms(t3), false);
            return false;
        }
        if (!gpu->DeviceBinary().empty() && !cache_key.empty()) {
            orlcomp::OrlBinaryCache cache(cache_options);
            cache.save(
                orlcomp::OrlBinaryKind::CudaCubin,
                cache_key,
                std::span<const std::uint8_t>(
                    gpu->DeviceBinary().data(),
                    gpu->DeviceBinary().size()));
        } else if (!gpu->DeviceCode().empty() && !cache_key.empty()) {
            orlcomp::OrlBinaryCache cache(cache_options);
            const auto& ptx = gpu->DeviceCode();
            cache.save(
                orlcomp::OrlBinaryKind::CudaPtx,
                cache_key,
                std::span<const std::uint8_t>(
                    reinterpret_cast<const std::uint8_t*>(ptx.data()),
                    ptx.size()));
        }
        print_jit(program->options.entry_function, backend, program->options.source_name,
            elapsed_ms(t0, t1), elapsed_ms(t1, t2), elapsed_ms(t2, t3), elapsed_ms(t3), true);
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

OrlExecution OrlExecution::Create(
    const OrlProgram& program, Backend backend,
    orlcomp::OrlBinaryCacheOptions cache) {
    auto impl = std::make_unique<Impl>();
    impl->backend = backend;
    impl->cache_options = std::move(cache);
    std::string view_error;
    if (!orlrig::register_rig_handle_views(
            orlcomp::global_handle_view_registry(), &view_error))
    {
        impl->errors.push_back(
            "ORL handle view registration failed: " + view_error);
        return OrlExecution(std::move(impl));
    }
    OrlProgram active_program = program;
    if (!active_program.valid() && active_program.impl_ != nullptr) {
        active_program = OrlProgram::Compile(
            active_program.impl_->source,
            active_program.impl_->options);
    }
    if (!active_program.valid()) {
        append_errors(impl->errors, active_program.errors());
        return OrlExecution(std::move(impl));
    }
    impl->program = active_program.impl_;
    impl->cache_key = orlcomp::make_binary_cache_key(
        execution_cache_material(
            backend,
            impl->program->source,
            impl->program->options.entry_function,
            impl->program->options.source_name,
            impl->program->options.include_paths));
    const auto context_parameter = std::find_if(
        impl->program->parameters.begin(),
        impl->program->parameters.end(),
        [](const ParameterDesc& parameter) {
            return parameter.name == orlcomp::kSolverContextParameterName;
        });
    if (context_parameter != impl->program->parameters.end()) {
        impl->solver_context.emplace(
            orlrig::kSolverContextOrlType,
            orlrig::kSolverContextStride);
        orlrig::SolverContext context{};
        if (!impl->solver_context->resize(1)
            || !impl->solver_context->write(0, context))
        {
            impl->errors.emplace_back(
                "Failed to initialize implicit SolverContext storage");
            return OrlExecution(std::move(impl));
        }
    }
    const auto hierarchy_parameter = std::find_if(
        impl->program->parameters.begin(),
        impl->program->parameters.end(),
        [](const ParameterDesc& parameter) {
            return parameter.name
                == orlcomp::kHierarchyContextParameterName;
        });
    if (hierarchy_parameter != impl->program->parameters.end()) {
        impl->hierarchy_context.emplace(
            orlrig::kHierarchyContextOrlType,
            orlrig::kHierarchyContextStride);
        const orlrig::HierarchyContext context{};
        if (!impl->hierarchy_context->resize(1)
            || !impl->hierarchy_context->write(0, context))
        {
            impl->errors.emplace_back(
                "Failed to initialize implicit HierarchyContext storage");
            return OrlExecution(std::move(impl));
        }
    }
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
    const std::string name(parameter);
    impl_->buffers[name] = &buffer;
    impl_->packed_bindings.erase(name);
    impl_->release_device_binding(name);
    return true;
}

bool OrlExecution::bind_packed_buffer(
    std::string_view parameter, const PackedBufferView& view)
{
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    if (impl_->backend != Backend::Cuda
        && parameter != orlcomp::kSolverContextParameterName)
    {
        impl_->errors.emplace_back(
            "bind_packed_buffer requires the CUDA backend except for "
            "the implicit SolverContext");
        return false;
    }
    const auto* desc = impl_->parameter(parameter);
    if (desc == nullptr || desc->kind != ParameterKind::Buffer) {
        impl_->errors.emplace_back(
            "Parameter '" + std::string(parameter)
            + "' is not a buffer");
        return false;
    }
    if (view.data == nullptr || view.storage_bytes == 0
        || view.bytes == 0
        || view.offset > view.storage_bytes
        || view.bytes > view.storage_bytes - view.offset)
    {
        impl_->errors.emplace_back(
            "Packed buffer binding for '" + std::string(parameter)
            + "' has an invalid range");
        return false;
    }

    const std::string name(parameter);
    if (impl_->packed_storage.has_value()
        && (impl_->packed_storage->data != view.data
            || impl_->packed_storage->bytes != view.storage_bytes))
    {
        impl_->errors.emplace_back(
            "All packed buffer bindings must share one storage allocation");
        return false;
    }
    if (!impl_->packed_storage.has_value()) {
        impl_->packed_storage = Impl::PackedStorage{
            view.data, view.storage_bytes, view.version};
    } else {
        impl_->packed_storage->version = view.version;
    }

    impl_->buffers.erase(name);
    impl_->release_device_binding(name);
    impl_->packed_bindings[name] = Impl::PackedBinding{
        view.offset, view.bytes};
    return true;
}

bool OrlExecution::bind_solver_context(
    const PackedBufferView& view)
{
    if (impl_ == nullptr || !impl_->initialized) {
        if (impl_ != nullptr) {
            impl_->errors.clear();
            impl_->errors.emplace_back(
                "ORL execution was not initialized");
        }
        return false;
    }
    if (impl_->parameter(orlcomp::kSolverContextParameterName) == nullptr) {
        return true;
    }
    return bind_packed_buffer(
        orlcomp::kSolverContextParameterName, view);
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
    impl_->packed_bindings.erase(name);
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

bool OrlExecution::bind_handle(
    std::string_view parameter, HandleValue value)
{
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    const auto* desc = impl_->parameter(parameter);
    if (desc == nullptr || desc->kind != ParameterKind::Handle) {
        impl_->errors.emplace_back(
            "Parameter '" + std::string(parameter)
            + "' is not a handle");
        return false;
    }
    if (!orlcomp::IsValidHandleValue(value)) {
        impl_->errors.emplace_back(
            "Handle parameter '" + std::string(parameter)
            + "' received an invalid token");
        return false;
    }
    if (value.type_id != desc->handle_type_id) {
        impl_->errors.emplace_back(
            "Handle parameter '" + std::string(parameter)
            + "' expects nominal type '" + desc->canonical_type_name + "'");
        return false;
    }
    impl_->handles[std::string(parameter)] = value;
    return true;
}

bool OrlExecution::bind_handle_view_context(
    orlrig::HandleViewContext& context)
{
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    if (!context.topology_revision) {
        impl_->errors.emplace_back(
            "Handle view context requires a topology revision");
        return false;
    }
    impl_->handle_view_context = &context;
    return true;
}

bool OrlExecution::set_solver_context(
    std::int64_t joint_count, std::int64_t controller_count)
{
    return set_solver_context(orlrig::SolverContext{
        joint_count, controller_count, 0, 0, 0, 0});
}

bool OrlExecution::set_solver_context(
    const orlrig::SolverContext& context)
{
    impl_->errors.clear();
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    if (impl_->packed_storage.has_value()) {
        if (impl_->packed_storage->data == nullptr
            || impl_->packed_storage->bytes < sizeof(orlrig::SolverContext))
        {
            impl_->errors.emplace_back(
                "Packed SolverContext storage is too small");
            return false;
        }
        std::memcpy(
            impl_->packed_storage->data,
            &context,
            sizeof(context));
        ++impl_->packed_storage->version;
        return true;
    }
    if (!impl_->solver_context.has_value()) {
        return true;
    }
    if (!impl_->solver_context->write(0, context)) {
        impl_->errors.emplace_back(
            "Failed to update implicit SolverContext storage");
        return false;
    }
    return true;
}

bool OrlExecution::set_hierarchy_context(
    const orlrig::HierarchyContext& context)
{
    impl_->errors.clear();
    if (!impl_->hierarchy_context.has_value()) {
        return true;
    }
    if (!impl_->initialized) {
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    if (std::memcmp(
            static_cast<const OrlBuffer&>(*impl_->hierarchy_context).data(),
            &context,
            sizeof(context)) == 0)
    {
        return true;
    }
    if (!impl_->hierarchy_context->write(0, context)) {
        impl_->errors.emplace_back(
            "Failed to update implicit HierarchyContext storage");
        return false;
    }
    return true;
}

bool OrlExecution::bind_hierarchy_data(OrlBuffer& buffer)
{
    if (impl_ == nullptr) {
        return false;
    }
    if (!impl_->initialized) {
        impl_->errors.clear();
        impl_->errors.emplace_back("ORL execution was not initialized");
        return false;
    }
    const auto* parameter =
        impl_->parameter(orlcomp::kHierarchyDataParameterName);
    if (parameter == nullptr) {
        impl_->errors.clear();
        return true;
    }
    if (buffer.orl_type() != parameter->orl_type
        || buffer.element_stride() != parameter->element_stride)
    {
        impl_->errors.clear();
        impl_->errors.emplace_back(
            "Hierarchy data buffer ABI is incompatible");
        return false;
    }
    impl_->errors.clear();
    impl_->buffers[std::string(orlcomp::kHierarchyDataParameterName)] =
        &buffer;
    impl_->packed_bindings.erase(
        std::string(orlcomp::kHierarchyDataParameterName));
    impl_->release_device_binding(
        std::string(orlcomp::kHierarchyDataParameterName));
    return true;
}

void OrlExecution::clear_bindings() {
    impl_->release_device_bindings();
    impl_->buffers.clear();
    impl_->packed_bindings.clear();
    impl_->packed_storage.reset();
    impl_->integers.clear();
    impl_->floats.clear();
    impl_->handles.clear();
}

bool OrlExecution::valid() const {
    return impl_ != nullptr && impl_->initialized;
}

std::optional<std::int64_t> OrlExecution::evaluate_impl(std::uint32_t element_count,
    bool host_readback) {
    if (!impl_->initialized) {
        if (impl_->errors.empty()) {
            impl_->errors.emplace_back("ORL execution was not initialized");
        }
        return std::nullopt;
    }
    std::optional<orlrig::ScopedHandleViewContext> active_view_context;
    if (impl_->handle_view_context != nullptr) {
        active_view_context.emplace(impl_->handle_view_context);
    }
    std::vector<void*> host_buffers;
    std::vector<std::int64_t> integers;
    std::vector<double> floats;
    std::vector<std::uint64_t> handles;
    if (!impl_->validate_bindings(
            host_buffers, integers, floats, handles)) {
        return std::nullopt;
    }

    const auto& entry = impl_->program->options.entry_function;
    KernelTiming timing;
    const auto t0 = Clock::now();

    if (impl_->backend == Backend::Cpu) {
        const std::string wrapper = "__orl_host_entry_" + entry;
        std::optional<std::int64_t> result;
        const bool has_solver_context = impl_->solver_context.has_value();
        void* solver_context_data = impl_->solver_context_data();
        const bool has_hierarchy_context =
            impl_->hierarchy_context.has_value();
        const bool has_handles = std::any_of(
            impl_->program->parameters.begin(),
            impl_->program->parameters.end(),
            [](const ParameterDesc& parameter) {
                return parameter.kind == ParameterKind::Handle;
            });
        const auto* handle_lanes = has_handles ? handles.data() : nullptr;
        if (has_handles && has_solver_context && has_hierarchy_context) {
            result = impl_->jit
                ->InvokeInt64WithRuntimeArgsAndHandlesAndContexts(
                    wrapper, host_buffers.data(), integers.data(),
                    floats.data(), handle_lanes,
                    solver_context_data,
                    const_cast<void*>(static_cast<const OrlBuffer&>(
                        *impl_->hierarchy_context).data()),
                    const_cast<void*>(static_cast<const OrlBuffer&>(
                        *impl_->buffers.at(
                            std::string(orlcomp::kHierarchyDataParameterName)))
                        .data()));
        } else if (has_handles && has_solver_context) {
            result = impl_->jit
                ->InvokeInt64WithRuntimeArgsAndHandlesAndContext(
                    wrapper, host_buffers.data(), integers.data(),
                    floats.data(), handle_lanes, solver_context_data);
        } else if (has_handles && has_hierarchy_context) {
            result = impl_->jit
                ->InvokeInt64WithRuntimeArgsAndHandlesAndHierarchyContext(
                    wrapper, host_buffers.data(), integers.data(),
                    floats.data(), handle_lanes,
                    const_cast<void*>(static_cast<const OrlBuffer&>(
                        *impl_->hierarchy_context).data()),
                    const_cast<void*>(static_cast<const OrlBuffer&>(
                        *impl_->buffers.at(
                            std::string(orlcomp::kHierarchyDataParameterName)))
                        .data()));
        } else if (has_handles) {
            result = impl_->jit
                ->InvokeInt64WithRuntimeArgsAndHandles(
                    wrapper, host_buffers.data(), integers.data(),
                    floats.data(), handle_lanes);
        } else if (has_solver_context && has_hierarchy_context) {
            result = impl_->jit->InvokeInt64WithRuntimeArgsAndContexts(
                wrapper, host_buffers.data(), integers.data(), floats.data(),
                solver_context_data,
                const_cast<void*>(static_cast<const OrlBuffer&>(
                    *impl_->hierarchy_context).data()),
                const_cast<void*>(static_cast<const OrlBuffer&>(
                    *impl_->buffers.at(
                        std::string(orlcomp::kHierarchyDataParameterName)))
                    .data()));
        } else if (has_solver_context) {
            result = impl_->jit->InvokeInt64WithRuntimeArgsAndContext(
                wrapper, host_buffers.data(), integers.data(), floats.data(),
                solver_context_data);
        } else if (has_hierarchy_context) {
            result =
                impl_->jit->InvokeInt64WithRuntimeArgsAndHierarchyContext(
                    wrapper, host_buffers.data(), integers.data(), floats.data(),
                    const_cast<void*>(static_cast<const OrlBuffer&>(
                        *impl_->hierarchy_context).data()),
                    const_cast<void*>(static_cast<const OrlBuffer&>(
                        *impl_->buffers.at(
                            std::string(orlcomp::kHierarchyDataParameterName)))
                        .data()));
        } else {
            result = impl_->jit->InvokeInt64WithRuntimeArgs(
                wrapper, host_buffers.data(), integers.data(), floats.data());
        }
        timing.kernel_ms = elapsed_ms(t0);
        timing.total_ms = timing.kernel_ms;
        if (!result.has_value()) {
            append_errors(impl_->errors, impl_->jit->Errors());
            return result;
        }
        record_kernel(entry, impl_->backend, element_count, timing, impl_->exec_stats);
        return result;
    }

    const auto t_upload = Clock::now();
    std::size_t upload_calls = 0;
    std::size_t upload_bytes = 0;
    std::vector<orlcomp::OrlGpuKernelArgument> arguments;
    arguments.reserve(impl_->program->parameters.size());
    for (const auto& parameter : impl_->program->parameters) {
        if (parameter.kind == ParameterKind::Buffer) {
            orlcomp::OrlGpuBuffer handle = 0;
            std::size_t buffer_offset = 0;
            if (parameter.name == orlcomp::kSolverContextParameterName) {
                const bool packed_context =
                    impl_->packed_storage.has_value();
                const bool ready = packed_context
                    ? impl_->ensure_packed_device_buffer(
                        &handle, &upload_calls, &upload_bytes)
                    : impl_->solver_context.has_value()
                        && impl_->ensure_device_buffer(
                            *impl_->solver_context,
                            &handle,
                            &upload_calls,
                            &upload_bytes);
                if (!ready)
                {
                    return std::nullopt;
                }
            } else if (parameter.name
                == orlcomp::kHierarchyContextParameterName)
            {
                if (!impl_->hierarchy_context.has_value()
                    || !impl_->ensure_device_buffer(
                        *impl_->hierarchy_context,
                        &handle,
                        &upload_calls,
                        &upload_bytes))
                {
                    return std::nullopt;
                }
            } else {
                const auto device =
                    impl_->device_bindings.find(parameter.name);
                if (device != impl_->device_bindings.end()) {
                    handle = device->second.handle;
                } else {
                    const auto packed =
                        impl_->packed_bindings.find(parameter.name);
                    if (packed != impl_->packed_bindings.end()) {
                        if (!impl_->ensure_packed_device_buffer(
                                &handle,
                                &upload_calls,
                                &upload_bytes))
                        {
                            return std::nullopt;
                        }
                        buffer_offset = packed->second.offset;
                    } else if (!impl_->ensure_device_buffer(
                            *impl_->buffers.at(parameter.name),
                            &handle,
                            &upload_calls,
                            &upload_bytes))
                    {
                        return std::nullopt;
                    }
                }
                if (handle == 0)
                {
                    return std::nullopt;
                }
            }
            orlcomp::OrlGpuKernelArgument argument;
            argument.is_buffer = true;
            argument.buffer = handle;
            argument.buffer_offset = buffer_offset;
            argument.scalar_type = orlcomp::OrlGpuKernelParameterType::Buffer;
            arguments.push_back(std::move(argument));
        } else if (parameter.kind == ParameterKind::Int64) {
            orlcomp::OrlGpuKernelArgument argument;
            argument.scalar_type = orlcomp::OrlGpuKernelParameterType::Int64;
            const auto value = impl_->integers.at(parameter.name);
            argument.scalar_bytes.resize(sizeof(value));
            std::memcpy(argument.scalar_bytes.data(), &value, sizeof(value));
            arguments.push_back(std::move(argument));
        } else if (parameter.kind == ParameterKind::Handle) {
            orlcomp::OrlGpuKernelArgument argument;
            argument.scalar_type = orlcomp::OrlGpuKernelParameterType::Handle;
            argument.scalar_bytes.resize(
                sizeof(std::uint64_t) * 2);
            argument.scalar_alignment = alignof(HandleValue);
            argument.lane_count = 2;
            argument.lane_order = "type_id,slot";
            const auto found = impl_->handles.find(parameter.name);
            if (found == impl_->handles.end()) {
                impl_->errors.emplace_back(
                    "Missing handle binding for parameter '"
                    + parameter.name + "'");
                return std::nullopt;
            }
            std::memcpy(argument.scalar_bytes.data(),
                &found->second.type_id, sizeof(std::uint64_t));
            const auto slot = static_cast<std::uint64_t>(
                found->second.slot);
            std::memcpy(argument.scalar_bytes.data()
                    + sizeof(std::uint64_t),
                &slot, sizeof(std::uint64_t));
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
    if (impl_->program->handle_views) {
        if (!impl_->ensure_handle_context_device(
                &upload_calls, &upload_bytes))
        {
            return std::nullopt;
        }
        orlcomp::OrlGpuKernelArgument argument;
        argument.is_buffer = true;
        argument.buffer = impl_->handle_context_device;
        argument.scalar_type =
            orlcomp::OrlGpuKernelParameterType::Buffer;
        arguments.push_back(std::move(argument));
    }

    if (!impl_->gpu->SetupCudaKernelArguments(orlcomp::OrlGpuEngine::CudaEntryKernelName,
            std::move(arguments)))
    {
        append_errors(impl_->errors, impl_->gpu->Errors());
        return std::nullopt;
    }
    timing.upload_calls = upload_calls;
    timing.upload_bytes = upload_bytes;
    timing.upload_ms = elapsed_ms(t_upload);

    const auto t_kernel = Clock::now();
    if (!impl_->gpu->LaunchCudaKernelForElements(element_count)
        || !impl_->gpu->Synchronize())
    {
        append_errors(impl_->errors, impl_->gpu->Errors());
        return std::nullopt;
    }
    timing.kernel_ms = elapsed_ms(t_kernel);

    const auto t_download = Clock::now();
    std::int32_t result = 0;
    if (host_readback) {
        for (const auto& [name, buffer] : impl_->buffers) {
            if (impl_->device_bindings.contains(name)) {
                continue;
            }
            const auto device = impl_->device_buffers.find(buffer);
            if (device == impl_->device_buffers.end() || buffer->byte_size() == 0) {
                continue;
            }
            if (!impl_->gpu->DownloadBuffer(device->second.handle,
                    const_cast<void*>(static_cast<const OrlBuffer*>(buffer)->data()),
                    buffer->byte_size()))
            {
                append_errors(impl_->errors, impl_->gpu->Errors());
                return std::nullopt;
            }
        }
        if (impl_->handle_view_context != nullptr) {
            const auto download_handle_arena =
                [&](const orlrig::HandleStorage& storage,
                    orlcomp::OrlGpuBuffer device,
                    std::size_t bytes) -> bool {
                if (!storage.writable || device == 0 || bytes == 0) {
                    return true;
                }
                return impl_->gpu->DownloadBuffer(
                    device, storage.data, bytes);
            };
            if (!download_handle_arena(
                    impl_->handle_view_context->joints,
                    impl_->handle_joint_device,
                    impl_->handle_joint_bytes)
                || !download_handle_arena(
                    impl_->handle_view_context->locators,
                    impl_->handle_locator_device,
                    impl_->handle_locator_bytes))
            {
                append_errors(impl_->errors, impl_->gpu->Errors());
                return std::nullopt;
            }
        }

        if (!impl_->gpu->ReadCudaGlobalInt32(
                orlcomp::OrlGpuEngine::CudaResultSymbolName, &result))
        {
            append_errors(impl_->errors, impl_->gpu->Errors());
            return std::nullopt;
        }
    }
    timing.download_ms = elapsed_ms(t_download);
    timing.total_ms = elapsed_ms(t0);
    record_kernel(entry, impl_->backend, element_count, timing, impl_->exec_stats);
    return static_cast<std::int64_t>(result);
}

std::optional<std::int64_t> OrlExecution::evaluate(std::uint32_t element_count) {
    const auto result = evaluate_impl(element_count, true);
    if (impl_ != nullptr) {
        impl_->handles.clear();
    }
    return result;
}

bool OrlExecution::evaluate_device(std::uint32_t element_count) {
    if (impl_ == nullptr || impl_->backend != Backend::Cuda) {
        if (impl_ != nullptr) {
            impl_->errors.clear();
            impl_->errors.emplace_back("evaluate_device requires the CUDA backend");
        }
        return false;
    }
    const auto result = evaluate_impl(element_count, false).has_value();
    impl_->handles.clear();
    return result;
}

std::optional<DeviceBufferView> OrlExecution::device_buffer_view(
    std::string_view parameter)
{
    if (impl_ == nullptr || !impl_->initialized
        || impl_->backend != Backend::Cuda || impl_->gpu == nullptr)
    {
        if (impl_ == nullptr) {
            return std::nullopt;
        }
        impl_->errors.clear();
        impl_->errors.emplace_back("device_buffer_view requires initialized CUDA execution");
        return std::nullopt;
    }

    const std::string name(parameter);
    if (const auto external = impl_->device_bindings.find(name);
        external != impl_->device_bindings.end())
    {
        return DeviceBufferView{external->second.device_ptr, external->second.bytes};
    }

    if (const auto packed = impl_->packed_bindings.find(name);
        packed != impl_->packed_bindings.end())
    {
        if (impl_->packed_device.handle == 0) {
            impl_->errors.clear();
            impl_->errors.emplace_back(
                "Packed device buffer for parameter '" + name
                + "' has not been allocated");
            return std::nullopt;
        }
        const auto view =
            impl_->gpu->DeviceBufferView(impl_->packed_device.handle);
        if (!view.has_value()) {
            append_errors(impl_->errors, impl_->gpu->Errors());
            return std::nullopt;
        }
        return DeviceBufferView{
            view->device_ptr + packed->second.offset,
            packed->second.bytes};
    }

    if (name == orlcomp::kSolverContextParameterName
        && impl_->packed_storage.has_value()
        && impl_->packed_device.handle != 0)
    {
        const auto view =
            impl_->gpu->DeviceBufferView(impl_->packed_device.handle);
        if (!view.has_value()) {
            append_errors(impl_->errors, impl_->gpu->Errors());
            return std::nullopt;
        }
        return DeviceBufferView{
            view->device_ptr, sizeof(orlrig::SolverContext)};
    }

    if (name == orlcomp::kHierarchyContextParameterName
        && impl_->hierarchy_context.has_value())
    {
        const auto device = impl_->device_buffers.find(
            &*impl_->hierarchy_context);
        if (device == impl_->device_buffers.end()) {
            impl_->errors.clear();
            impl_->errors.emplace_back(
                "Device buffer for hierarchy context has not been allocated");
            return std::nullopt;
        }
        const auto view =
            impl_->gpu->DeviceBufferView(device->second.handle);
        if (!view.has_value()) {
            append_errors(impl_->errors, impl_->gpu->Errors());
            return std::nullopt;
        }
        return DeviceBufferView{
            view->device_ptr, sizeof(orlrig::HierarchyContext)};
    }

    const auto host = impl_->buffers.find(name);
    if (host == impl_->buffers.end()) {
        impl_->errors.clear();
        impl_->errors.emplace_back("No buffer binding for parameter '" + name + "'");
        return std::nullopt;
    }
    const auto device = impl_->device_buffers.find(host->second);
    if (device == impl_->device_buffers.end()) {
        impl_->errors.clear();
        impl_->errors.emplace_back("Device buffer for parameter '" + name
            + "' has not been allocated");
        return std::nullopt;
    }
    const auto view = impl_->gpu->DeviceBufferView(device->second.handle);
    if (!view.has_value()) {
        append_errors(impl_->errors, impl_->gpu->Errors());
        return std::nullopt;
    }
    return DeviceBufferView{view->device_ptr, view->bytes};
}

std::optional<std::uint64_t> OrlExecution::device_buffer_pointer(
    std::string_view parameter)
{
    const auto view = device_buffer_view(parameter);
    return view.has_value()
        ? std::optional<std::uint64_t>(view->device_ptr)
        : std::nullopt;
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
