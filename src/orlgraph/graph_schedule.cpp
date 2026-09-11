#include "graph_schedule.hpp"

#include <map>
#include <set>

namespace orlgraph
{

ScheduleResult topological_schedule(const GraphModule& module) {
    ScheduleResult result;
    std::map<StableId, std::set<StableId>> outgoing;
    std::map<StableId, std::size_t> incoming;

    for (const auto& [id, _] : module.nodes()) {
        outgoing.emplace(id, std::set<StableId>{});
        incoming.emplace(id, 0);
    }

    for (const auto& connection : module.connections()) {
        if (connection.feedback
            || connection.source.kind != EndpointKind::NodePort
            || connection.destination.kind != EndpointKind::NodePort)
        {
            continue;
        }
        const auto source = module.node(connection.source.owner);
        const auto destination = module.node(connection.destination.owner);
        if (source == nullptr || destination == nullptr) {
            continue;
        }
        if (outgoing[connection.source.owner].insert(connection.destination.owner).second) {
            ++incoming[connection.destination.owner];
        }
    }

    std::set<StableId> ready;
    for (const auto& [id, count] : incoming) {
        if (count == 0) {
            ready.insert(id);
        }
    }

    while (!ready.empty()) {
        const StableId current = *ready.begin();
        ready.erase(ready.begin());
        result.order.push_back(current);
        for (const StableId& dependent : outgoing[current]) {
            auto& count = incoming[dependent];
            if (--count == 0) {
                ready.insert(dependent);
            }
        }
    }

    if (result.order.size() != module.nodes().size()) {
        result.ok = false;
        result.errors.emplace_back("Graph contains a dependency cycle");
        result.order.clear();
        return result;
    }

    result.ok = true;
    return result;
}

} // namespace orlgraph
