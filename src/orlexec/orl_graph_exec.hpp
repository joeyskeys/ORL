#pragma once

#include "orl_exec.hpp"

#include "orl_graph_lowering.h"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ORL::exec
{

struct GraphInputBinding {
    ParameterKind kind = ParameterKind::Unsupported;
    OrlBuffer* buffer = nullptr;
    std::uint64_t device_ptr = 0;
    std::size_t bytes = 0;
    std::size_t element_count = 0;
    std::int64_t int_value = 0;
    double float_value = 0.0;
};

using GraphInputResolver = std::function<bool(
    const orlgraph::InterfacePort&, GraphInputBinding&, std::string&)>;

class OrlGraphProgram {
public:
    static OrlGraphProgram Compile(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry,
        orlcomp::GraphLoweringOptions options = {});

    bool valid() const;
    const std::string& source() const;
    const std::string& entry_function() const;
    const std::vector<ParameterDesc>& parameters() const;
    const std::vector<std::string>& errors() const;

private:
    OrlGraphProgram() = default;

    std::optional<OrlProgram> program_;
    std::string source_;
    std::string entry_function_;
    std::vector<std::string> errors_;

    friend class OrlGraphExecution;
};

class OrlGraphExecution {
public:
    static OrlGraphExecution Create(const OrlGraphProgram& program,
        Backend backend = Backend::Cpu);

    bool valid() const;
    bool bind_buffer(std::string_view parameter, OrlBuffer& buffer);
    bool bind_device_buffer(std::string_view parameter,
        std::uint64_t device_ptr, std::size_t bytes);
    bool bind_int(std::string_view parameter, std::int64_t value);
    bool bind_float(std::string_view parameter, double value);
    bool bind_graph_inputs(const orlgraph::GraphModule& module,
        const GraphInputResolver& resolver);
    void clear_bindings();

    std::optional<std::int64_t> evaluate(std::uint32_t element_count = 1);
    bool evaluate_device(std::uint32_t element_count = 1);
    std::optional<DeviceBufferView> device_buffer_view(std::string_view parameter);
    std::optional<std::uint64_t> device_buffer_pointer(std::string_view parameter);
    bool synchronize();

    Backend backend() const;
    const std::vector<std::string>& errors() const;
    const std::string& ir() const;

private:
    OrlGraphExecution() = default;

    std::optional<OrlExecution> execution_;
    std::vector<ParameterDesc> parameters_;
    std::vector<std::string> errors_;
};

} // namespace ORL::exec
