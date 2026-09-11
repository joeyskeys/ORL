#pragma once

#include "graph_ir.hpp"
#include "graph_schedule.hpp"

#include <string>
#include <vector>

namespace orlgraph
{

struct ReflectedPort {
    StableId node;
    StableId id;
    std::string name;
    PortDirection direction = PortDirection::Input;
    LogicalType type;
    Domain domain = Domain::constant();
    Shape shape = Shape::scalar();
};

struct ReflectedNode {
    StableId id;
    StableId definition;
    std::string name;
    std::vector<ReflectedPort> ports;
    std::vector<ResourceEffect> effects;
    Provenance provenance;
};

struct GraphReflection {
    std::string module_id;
    Version version;
    std::vector<InterfacePort> inputs;
    std::vector<InterfacePort> outputs;
    std::vector<Resource> resources;
    std::vector<ReflectedNode> nodes;
    ScheduleResult schedule;
};

GraphReflection reflect(const GraphModule& module,
    const NodeRegistry& registry);

} // namespace orlgraph
