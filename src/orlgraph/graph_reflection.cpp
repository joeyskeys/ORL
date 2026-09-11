#include "graph_reflection.hpp"

#include <utility>

namespace orlgraph
{

GraphReflection reflect(const GraphModule& module,
    const NodeRegistry& registry)
{
    GraphReflection reflection;
    reflection.module_id = module.module_id;
    reflection.version = module.version;
    for (const auto& [_, input] : module.inputs()) {
        reflection.inputs.push_back(input);
    }
    for (const auto& [_, output] : module.outputs()) {
        reflection.outputs.push_back(output);
    }
    for (const auto& [_, resource] : module.resources()) {
        reflection.resources.push_back(resource);
    }
    for (const auto& [id, node] : module.nodes()) {
        ReflectedNode reflected;
        reflected.id = id;
        reflected.definition = node.definition;
        reflected.name = node.name;
        reflected.provenance = node.provenance;
        if (const auto* definition = registry.find(node.definition)) {
            reflected.effects = definition->effects;
            for (const auto& port : definition->inputs) {
                reflected.ports.push_back(ReflectedPort{
                    id, port.id, port.name, port.direction,
                    port.type, port.domain, port.shape,
                });
            }
            for (const auto& port : definition->outputs) {
                reflected.ports.push_back(ReflectedPort{
                    id, port.id, port.name, port.direction,
                    port.type, port.domain, port.shape,
                });
            }
        }
        reflection.nodes.push_back(std::move(reflected));
    }
    reflection.schedule = topological_schedule(module);
    return reflection;
}

} // namespace orlgraph
