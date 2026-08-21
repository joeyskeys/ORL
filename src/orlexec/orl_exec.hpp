#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

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
        Backend backend = Backend::Cpu);
    ~OrlExecution();

    OrlExecution(const OrlExecution&) = delete;
    OrlExecution& operator=(const OrlExecution&) = delete;
    OrlExecution(OrlExecution&&) noexcept;
    OrlExecution& operator=(OrlExecution&&) noexcept;

    bool bind_buffer(std::string_view parameter, OrlBuffer& buffer);
    bool bind_int(std::string_view parameter, std::int64_t value);
    bool bind_float(std::string_view parameter, double value);
    void clear_bindings();

    bool valid() const;
    // For CUDA, element_count selects the launch size. CPU execution ignores
    // it and relies on the entry function's bound scalar parameter.
    std::optional<std::int64_t> evaluate(std::uint32_t element_count = 1);
    bool synchronize();

    Backend backend() const;
    const std::vector<std::string>& errors() const;

private:
    struct Impl;
    explicit OrlExecution(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> impl_;
};

} // namespace ORL::exec
