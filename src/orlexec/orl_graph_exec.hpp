#pragma once

#include "orl_exec.hpp"

#include "orl_graph_lowering.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
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
    std::optional<PackedBufferView> packed;
    std::uint64_t device_ptr = 0;
    std::size_t bytes = 0;
    std::size_t element_count = 0;
    std::int64_t int_value = 0;
    double float_value = 0.0;
};

using GraphInputResolver = std::function<bool(
    const orlgraph::InterfacePort&, GraphInputBinding&, std::string&)>;

struct GraphOutputDescriptor {
    orlgraph::StableId id;
    std::string name;
    orlgraph::LogicalType type;
    orlgraph::Domain domain = orlgraph::Domain::constant();
    orlgraph::Shape shape = orlgraph::Shape::scalar();
    std::string binding;
    std::string semantic;
    std::string coordinate_space;
    ParameterKind kind = ParameterKind::Unsupported;
    std::string source_parameter;
    bool returned = false;
};

struct GraphOutputValue {
    GraphOutputDescriptor descriptor;
    std::optional<std::int64_t> int_value;
    std::optional<double> float_value;
    OrlBuffer* buffer = nullptr;
    std::optional<DeviceBufferView> device_view;
};

struct GraphEvaluationResult {
    bool ok = false;
    std::optional<std::int64_t> status;
    std::vector<GraphOutputValue> outputs;
    std::vector<std::string> errors;

    const GraphOutputValue* output(const orlgraph::StableId& id) const;
};

class OrlGraphProgram {
public:
    OrlGraphProgram() = default;

    static OrlGraphProgram Compile(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry,
        orlcomp::GraphLoweringOptions options = {});

    bool valid() const;
    const std::string& source() const;
    const std::string& entry_function() const;
    std::size_t scene_revision() const { return scene_revision_; }
    bool scene_revision_matches(std::size_t revision) const {
        return scene_revision_ == revision;
    }
    const std::vector<ParameterDesc>& parameters() const;
    const std::vector<GraphOutputDescriptor>& outputs() const {
        return outputs_;
    }
    const std::vector<std::string>& errors() const;

private:
    std::optional<OrlProgram> program_;
    std::string source_;
    std::string entry_function_;
    std::size_t scene_revision_ = 0;
    std::vector<GraphOutputDescriptor> outputs_;
    std::vector<std::string> errors_;

    friend class OrlGraphExecution;
};

class OrlGraphExecution {
public:
    OrlGraphExecution() = default;

    static OrlGraphExecution Create(const OrlGraphProgram& program,
        Backend backend = Backend::Cpu);

    bool valid() const;
    bool bind_buffer(std::string_view parameter, OrlBuffer& buffer);
    bool bind_device_buffer(std::string_view parameter,
        std::uint64_t device_ptr, std::size_t bytes);
    bool bind_int(std::string_view parameter, std::int64_t value);
    bool bind_float(std::string_view parameter, double value);
    bool set_solver_context(
        std::int64_t joint_count, std::int64_t controller_count);
    bool bind_graph_inputs(const orlgraph::GraphModule& module,
        const GraphInputResolver& resolver);
    void clear_bindings();

    std::optional<std::int64_t> evaluate(std::uint32_t element_count = 1);
    GraphEvaluationResult evaluate_result(std::uint32_t element_count = 1);
    bool evaluate_device(std::uint32_t element_count = 1);
    std::optional<DeviceBufferView> device_buffer_view(std::string_view parameter);
    std::optional<std::uint64_t> device_buffer_pointer(std::string_view parameter);
    bool synchronize();

    Backend backend() const;
    const std::vector<std::string>& errors() const;
    const std::string& ir() const;
    const GraphOutputValue* output(const orlgraph::StableId& id) const;

private:
    std::optional<OrlExecution> execution_;
    std::vector<ParameterDesc> parameters_;
    std::vector<GraphOutputDescriptor> outputs_;
    std::map<std::string, OrlBuffer*> host_buffers_;
    std::map<std::string, DeviceBufferView> device_buffers_;
    std::map<std::string, std::int64_t> host_ints_;
    std::map<std::string, double> host_floats_;
    std::optional<std::int64_t> last_result_;
    std::vector<GraphOutputValue> last_outputs_;
    std::vector<std::string> errors_;
};

} // namespace ORL::exec
