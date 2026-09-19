#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "orl_cache.h"
#include "orlrig/abi.hpp"

namespace ORL::exec
{

enum class Backend {
    Cpu,
    Cuda,
};

enum class ParameterKind {
    Buffer,
    Int64,
    Float64,
    Unsupported,
};

struct ParameterDesc {
    std::string name;
    std::string orl_type;
    ParameterKind kind = ParameterKind::Unsupported;
    std::size_t element_stride = 0;
};

struct DeviceBufferView {
    std::uint64_t device_ptr = 0;
    std::size_t bytes = 0;
};

// A view into one mutable host allocation shared by multiple buffer
// parameters. The execution uploads the complete storage allocation once and
// passes each parameter's byte range as a device pointer offset.
struct PackedBufferView {
    void* data = nullptr;
    std::size_t storage_bytes = 0;
    std::size_t offset = 0;
    std::size_t bytes = 0;
    std::uint64_t version = 0;
};

// Owning, growable host storage for one ORL buffer parameter. Applications
// control capacity and element count; ORL only receives data() and count.
class OrlBuffer {
public:
    OrlBuffer(std::string orl_type, std::size_t element_stride);

    const std::string& orl_type() const { return orl_type_; }
    std::size_t element_stride() const { return element_stride_; }
    std::size_t count() const { return count_; }
    std::size_t capacity() const {
        return element_stride_ == 0 ? 0 : storage_.size() / element_stride_;
    }
    std::size_t byte_size() const { return count_ * element_stride_; }
    std::uint64_t version() const { return version_; }

    bool reserve(std::size_t element_capacity);
    bool resize(std::size_t element_count);
    void clear();

    const void* data() const { return storage_.data(); }
    void* data();
    void mark_modified();

    bool write(std::size_t index, const void* source, std::size_t bytes);
    bool read(std::size_t index, void* destination, std::size_t bytes) const;

    template <typename T>
    bool write(std::size_t index, const T& value) {
        return write(index, &value, sizeof(T));
    }

    template <typename T>
    bool read(std::size_t index, T* value) const {
        return read(index, value, sizeof(T));
    }

private:
    std::string orl_type_;
    std::size_t element_stride_ = 0;
    std::size_t count_ = 0;
    std::uint64_t version_ = 1;
    std::vector<std::byte> storage_;
};

struct CompileOptions {
    std::string entry_function = "compute";
    std::string source_name = "orl_runtime_program";
    // Additional directories searched by `use module;` during parsing.
    std::vector<std::string> include_paths;
};

// Parsed, application-facing description of an ORL entry function. The source
// is retained so an Execution can compile it independently for CPU or CUDA.
class OrlProgram {
public:
    // Always returns a program object so applications can retrieve parse and
    // compile diagnostics through errors() when valid() is false.
    static OrlProgram Compile(std::string source,
        CompileOptions options = {});

    bool valid() const;
    const std::string& entry_function() const;
    const std::vector<ParameterDesc>& parameters() const;
    const std::vector<std::string>& errors() const;

private:
    struct Impl;
    explicit OrlProgram(std::shared_ptr<Impl> impl);

    std::shared_ptr<Impl> impl_;
    friend class OrlExecution;
};

// A backend-specific execution context. Bindings are named after the ORL
// entry parameters, so resizing a buffer only requires rebinding/evaluating,
// never recompiling the ORL source.
class OrlExecution {
public:
    // Creation retains diagnostics on the returned object when a backend is
    // unavailable, avoiding an error-losing optional construction path.
    static OrlExecution Create(const OrlProgram& program,
        Backend backend = Backend::Cpu,
        orlcomp::OrlBinaryCacheOptions cache = {});
    ~OrlExecution();

    OrlExecution(const OrlExecution&) = delete;
    OrlExecution& operator=(const OrlExecution&) = delete;
    OrlExecution(OrlExecution&&) noexcept;
    OrlExecution& operator=(OrlExecution&&) noexcept;

    bool bind_buffer(std::string_view parameter, OrlBuffer& buffer);
    bool bind_packed_buffer(std::string_view parameter,
        const PackedBufferView& view);
    // Bind an existing CUDA device pointer (CUdeviceptr) for a buffer parameter.
    // CUDA backend only; the pointer is not allocated or freed by ORL.
    bool bind_device_buffer(std::string_view parameter, std::uint64_t device_ptr,
        std::size_t bytes);
    bool bind_int(std::string_view parameter, std::int64_t value);
    bool bind_float(std::string_view parameter, double value);
    // Updates the implicit solver_context global when the compiled program
    // uses it. This is a no-op for ordinary programs.
    bool set_solver_context(
        std::int64_t joint_count, std::int64_t controller_count);
    bool set_solver_context(const orlrig::SolverContext& context);
    // Binds the packed arena used by the implicit solver_context global. CPU
    // execution uses the host allocation directly; CUDA uploads the same
    // allocation once and passes offsets from SolverContext.
    bool bind_solver_context(const PackedBufferView& view);
    // Updates the implicit hierarchy_context global when the compiled
    // program uses it. This is a no-op for ordinary programs.
    bool set_hierarchy_context(const orlrig::HierarchyContext& context);
    // Binds the static hierarchy_data buffer. This is a no-op for ordinary
    // programs that do not reference the implicit hierarchy globals.
    bool bind_hierarchy_data(OrlBuffer& buffer);
    void clear_bindings();

    bool valid() const;
    // For CUDA, element_count selects the launch size. CPU execution ignores
    // it and relies on the entry function's bound scalar parameter.
    std::optional<std::int64_t> evaluate(std::uint32_t element_count = 1);
    // Launch a CUDA entry without copying any bound buffers or the return value
    // back to the host. Host-bound buffers are still uploaded when modified.
    bool evaluate_device(std::uint32_t element_count = 1);
    std::optional<DeviceBufferView> device_buffer_view(std::string_view parameter);
    std::optional<std::uint64_t> device_buffer_pointer(std::string_view parameter);
    bool synchronize();

    Backend backend() const;
    const std::vector<std::string>& errors() const;
    const std::string& ir() const;

private:
    struct Impl;
    explicit OrlExecution(std::unique_ptr<Impl> impl);
    std::optional<std::int64_t> evaluate_impl(std::uint32_t element_count,
        bool host_readback);

    std::unique_ptr<Impl> impl_;
};

} // namespace ORL::exec
