#include "evaluation.hpp"

#include <algorithm>
#include <functional>
#include <iterator>
#include <utility>

namespace orlrig
{

namespace
{

template <typename T>
void append_unique(std::vector<T>* values, T value) {
    if (std::find(values->begin(), values->end(), value) == values->end()) {
        values->push_back(std::move(value));
    }
}

void append_unique_stable_ids(std::vector<orlgraph::StableId>* values,
    const std::vector<orlgraph::StableId>& additions)
{
    for (const auto& id : additions) {
        append_unique(values, id);
    }
}

const std::vector<orlgraph::StableId>& empty_regions() {
    static const std::vector<orlgraph::StableId> result;
    return result;
}

enum class ExpectedKind {
    Joint,
    Controller,
    Locator,
};

bool is_kind(const ComponentStore& components, ComponentId id,
    ExpectedKind expected)
{
    const Component* component = components.find(id);
    if (component == nullptr) {
        return false;
    }
    switch (expected) {
    case ExpectedKind::Joint:
        return component->kind == ComponentKind::Joint;
    case ExpectedKind::Controller:
        return component->kind == ComponentKind::Controller;
    case ExpectedKind::Locator:
        return component->kind == ComponentKind::Locator;
    }
    return false;
}

void resolve_explicit_components(
    const ComponentStore& components,
    const std::vector<orlgraph::StableId>& selectors,
    ExpectedKind expected,
    std::vector<ComponentId>* output,
    bool* global)
{
    for (const auto& selector : selectors) {
        const auto* component = components.find(selector.value);
        if (component == nullptr
            || !is_kind(components, component->id, expected))
        {
            *global = true;
            continue;
        }
        append_unique(output, component->id);
    }
}

std::optional<ComponentId> constant_component(
    const orlgraph::NodeInstance& node,
    const orlgraph::NodeDefinition& definition,
    const ComponentStore& components,
    ExpectedKind expected)
{
    // The input adapter nodes use a "name" parameter. Keeping this resolver
    // deliberately narrow makes unresolved symbolic selectors conservative.
    if (definition.qualified_name != "orlrig.input.find_joint"
        && definition.qualified_name != "orlrig.input.find_controller"
        && definition.qualified_name != "orlrig.input.find_locator")
    {
        return std::nullopt;
    }
    const auto parameter = node.parameter_values.find("name");
    if (parameter == node.parameter_values.end()) {
        return std::nullopt;
    }
    const auto* name = std::get_if<std::string>(&parameter->second.value);
    if (name == nullptr) {
        return std::nullopt;
    }
    const Component* component = components.find(*name);
    if (component == nullptr) {
        return std::nullopt;
    }
    const ComponentId id = component->id;
    return is_kind(components, id, expected)
        ? std::optional<ComponentId>{id} : std::nullopt;
}

enum class ConnectedKind {
    Absent,
    Joint,
    Controller,
    Locator,
};

struct ConnectedElement {
    ConnectedKind kind = ConnectedKind::Absent;
    ComponentId id;
};

ConnectedElement connected_element(
    const orlgraph::GraphModule& graph,
    const orlgraph::NodeRegistry& registry,
    const ComponentStore& components,
    const orlgraph::StableId& node_id,
    std::string_view port_name)
{
    for (const auto& connection : graph.connections()) {
        if (connection.destination.kind != orlgraph::EndpointKind::NodePort
            || connection.destination.owner != node_id
            || connection.destination.port.value != port_name)
        {
            continue;
        }
        ConnectedElement absent;
        if (connection.source.kind != orlgraph::EndpointKind::NodePort) {
            return absent;
        }
        const auto* source = graph.node(connection.source.owner);
        if (source == nullptr) {
            return absent;
        }
        const auto* definition = registry.find(source->definition);
        if (definition == nullptr) {
            return absent;
        }
        ConnectedElement found;
        if (const auto joint = constant_component(
                *source, *definition, components, ExpectedKind::Joint))
        {
            found.kind = ConnectedKind::Joint;
            found.id = *joint;
            return found;
        }
        if (const auto controller = constant_component(
                *source, *definition, components, ExpectedKind::Controller))
        {
            found.kind = ConnectedKind::Controller;
            found.id = *controller;
            return found;
        }
        if (const auto locator = constant_component(
                *source, *definition, components, ExpectedKind::Locator))
        {
            found.kind = ConnectedKind::Locator;
            found.id = *locator;
            return found;
        }
        return absent;
    }
    return {};
}

struct PortFlags {
    bool joint = false;
    bool controller = false;
    bool locator = false;
};

struct PortEntry {
    std::string name;
    PortFlags flags;
};

void note_port(std::vector<PortEntry>* ports, const std::vector<std::string>& names,
    void (*mark)(PortFlags*))
{
    for (const auto& name : names) {
        const auto found = std::find_if(ports->begin(), ports->end(),
            [&](const PortEntry& entry) { return entry.name == name; });
        PortEntry* entry = nullptr;
        if (found == ports->end()) {
            ports->push_back(PortEntry{name, {}});
            entry = &ports->back();
        } else {
            entry = &*found;
        }
        mark(&entry->flags);
    }
}

void resolve_element_ports(
    const orlgraph::GraphModule& graph,
    const orlgraph::NodeRegistry& registry,
    const ComponentStore& components,
    const orlgraph::StableId& node_id,
    const orlgraph::PartialEvaluationFootprint& footprint,
    bool writing,
    SolverRegion* region,
    bool* global)
{
    std::vector<PortEntry> ports;
    if (writing) {
        note_port(&ports, footprint.write_joint_ports,
            [](PortFlags* flags) { flags->joint = true; });
        note_port(&ports, footprint.write_controller_ports,
            [](PortFlags* flags) { flags->controller = true; });
        note_port(&ports, footprint.write_locator_ports,
            [](PortFlags* flags) { flags->locator = true; });
    } else {
        note_port(&ports, footprint.read_joint_ports,
            [](PortFlags* flags) { flags->joint = true; });
        note_port(&ports, footprint.read_controller_ports,
            [](PortFlags* flags) { flags->controller = true; });
        note_port(&ports, footprint.read_locator_ports,
            [](PortFlags* flags) { flags->locator = true; });
    }
    for (const auto& entry : ports) {
        const auto connected = connected_element(
            graph, registry, components, node_id, entry.name);
        const bool allowed = connected.kind == ConnectedKind::Joint
                ? entry.flags.joint
            : connected.kind == ConnectedKind::Controller
                ? entry.flags.controller
            : connected.kind == ConnectedKind::Locator
                ? entry.flags.locator
            : false;
        if (!allowed) {
            *global = true;
            continue;
        }
        if (writing) {
            if (connected.kind == ConnectedKind::Joint) {
                append_unique(&region->write_joints, connected.id);
            } else if (connected.kind == ConnectedKind::Controller) {
                append_unique(&region->write_controllers, connected.id);
            } else {
                append_unique(&region->write_locators, connected.id);
            }
        } else if (connected.kind == ConnectedKind::Joint) {
            append_unique(&region->read_joints, connected.id);
        } else if (connected.kind == ConnectedKind::Controller) {
            append_unique(&region->read_controllers, connected.id);
        } else {
            append_unique(&region->read_locators, connected.id);
        }
    }
}

bool is_ancestor(const HierarchyPlan& hierarchy,
    ComponentId ancestor, ComponentId joint)
{
    auto current = hierarchy.parent_of(joint);
    while (current.has_value()) {
        if (*current == ancestor) {
            return true;
        }
        current = hierarchy.parent_of(*current);
    }
    return false;
}

bool reaches(const std::vector<std::vector<std::size_t>>& precedes,
    std::size_t from, std::size_t to)
{
    std::vector<bool> seen(precedes.size(), false);
    std::vector<std::size_t> pending{from};
    while (!pending.empty()) {
        const std::size_t current = pending.back();
        pending.pop_back();
        if (current == to) {
            return true;
        }
        if (current >= seen.size() || seen[current]) {
            continue;
        }
        seen[current] = true;
        for (const std::size_t next : precedes[current]) {
            pending.push_back(next);
        }
    }
    return false;
}

std::string component_label(const ComponentStore& components, ComponentId id)
{
    const Component* component = components.find(id);
    if (component == nullptr || component->name.empty()) {
        return std::to_string(id.value);
    }
    return component->name;
}

void append_ancestors(const HierarchyPlan& hierarchy,
    ComponentId joint, std::vector<ComponentId>* output)
{
    const auto position = hierarchy.preorder_position(joint);
    if (!position.has_value()) {
        return;
    }
    if (hierarchy.ancestor_storage == AncestorStorageMode::Flattened
        && position.value() + 1 < hierarchy.ancestor_offsets.size())
    {
        const auto begin = hierarchy.ancestor_offsets[*position];
        const auto end = hierarchy.ancestor_offsets[*position + 1];
        for (std::size_t i = begin; i < end; ++i) {
            append_unique(output, hierarchy.ancestor_joints[i]);
        }
        return;
    }
    auto current = hierarchy.parent_of(joint);
    while (current.has_value()) {
        append_unique(output, *current);
        current = hierarchy.parent_of(*current);
    }
}

void append_descendants(const HierarchyPlan& hierarchy,
    ComponentId joint, std::vector<ComponentId>* output)
{
    const auto position = hierarchy.preorder_position(joint);
    if (!position.has_value() || *position >= hierarchy.subtree_end.size()) {
        return;
    }
    for (std::size_t i = hierarchy.subtree_begin[*position];
         i < hierarchy.subtree_end[*position]; ++i)
    {
        append_unique(output, hierarchy.preorder_joints[i]);
    }
}

bool intersects(const std::vector<ComponentId>& left,
    const std::vector<ComponentId>& right)
{
    for (const ComponentId id : left) {
        if (std::find(right.begin(), right.end(), id) != right.end()) {
            return true;
        }
    }
    return false;
}

bool resource_intersects(const SolverRegion& left,
    const SolverRegion& right)
{
    for (const auto& id : left.write_resources) {
        if (std::find(right.read_resources.begin(), right.read_resources.end(),
                id) != right.read_resources.end()
            || std::find(right.write_resources.begin(),
                right.write_resources.end(), id) != right.write_resources.end())
        {
            return true;
        }
    }
    for (const auto& id : right.write_resources) {
        if (std::find(left.read_resources.begin(), left.read_resources.end(),
                id) != left.read_resources.end())
        {
            return true;
        }
    }
    return false;
}

void build_packed_indices(const ComponentStore& components,
    const std::vector<ComponentId>& ids,
    std::vector<std::int64_t>* result,
    std::vector<std::string>* errors,
    std::string_view label)
{
    for (const ComponentId id : ids) {
        const std::int64_t index = components.joint_index(id);
        if (index < 0) {
            errors->push_back(
                std::string{label} + " contains a non-joint component");
            continue;
        }
        result->push_back(index);
    }
}

} // namespace

const SolverRegion* EvaluationPlan::region(
    const orlgraph::StableId& node_id) const
{
    const auto found = std::find_if(regions.begin(), regions.end(),
        [&](const SolverRegion& value) { return value.node_id == node_id; });
    return found == regions.end() ? nullptr : &*found;
}

const std::vector<orlgraph::StableId>&
EvaluationPlan::regions_for_controller(ComponentId id) const
{
    const auto found = controller_regions.find(id.value);
    return found == controller_regions.end() ? empty_regions() : found->second;
}

const std::vector<orlgraph::StableId>&
EvaluationPlan::regions_for_locator(ComponentId id) const
{
    const auto found = locator_regions.find(id.value);
    return found == locator_regions.end() ? empty_regions() : found->second;
}

EvaluationPlanCompileResult compile_evaluation_plan(
    const orlgraph::GraphModule& graph,
    const orlgraph::NodeRegistry& registry,
    const ComponentStore& components,
    const HierarchyPlan& hierarchy)
{
    EvaluationPlanCompileResult result;
    EvaluationPlan plan;
    plan.topology_revision = hierarchy.topology_revision;

    const auto schedule = orlgraph::topological_schedule(graph);
    if (!schedule.ok) {
        result.errors = schedule.errors;
        return result;
    }
    plan.order = schedule.order;

    for (const auto& node_id : schedule.order) {
        const auto* node = graph.node(node_id);
        if (node == nullptr) {
            result.errors.push_back("Scheduled node is missing: " + node_id.value);
            continue;
        }
        const auto* definition = registry.find(node->definition);
        if (definition == nullptr) {
            result.errors.push_back("Node definition is missing: "
                + node->definition.value);
            continue;
        }
        const bool solver_node = definition->operation == "solver"
            || definition->operation == "constraint"
            || definition->partial_footprint.has_value()
            || (definition->implementation.kind
                    == orlgraph::ImplementationKind::OrlFunction
                && definition->operation != "identity");
        if (!solver_node) {
            continue;
        }

        SolverRegion region;
        region.node_id = node_id;
        region.definition_id = definition->id;
        region.stateful = definition->stateful;
        if (definition->partial_footprint.has_value()) {
            const auto& footprint = *definition->partial_footprint;
            region.declared = footprint.declared;
            region.global = footprint.global;
            region.stateful = footprint.stateful || definition->stateful;
            region.supports_sparse_dispatch =
                footprint.supports_sparse_dispatch;
            region.propagation = footprint.propagation;
            resolve_explicit_components(components, footprint.read_joints,
                ExpectedKind::Joint, &region.read_joints, &region.global);
            resolve_explicit_components(components, footprint.write_joints,
                ExpectedKind::Joint, &region.write_joints, &region.global);
            resolve_explicit_components(components,
                footprint.read_controllers, ExpectedKind::Controller,
                &region.read_controllers, &region.global);
            resolve_explicit_components(components, footprint.read_locators,
                ExpectedKind::Locator, &region.read_locators, &region.global);
            resolve_explicit_components(components,
                footprint.write_controllers, ExpectedKind::Controller,
                &region.write_controllers, &region.global);
            resolve_explicit_components(components, footprint.write_locators,
                ExpectedKind::Locator, &region.write_locators, &region.global);
            append_unique_stable_ids(&region.read_resources,
                footprint.read_resources);
            append_unique_stable_ids(&region.write_resources,
                footprint.write_resources);

            resolve_element_ports(graph, registry, components, node_id,
                footprint, false, &region, &region.global);
            resolve_element_ports(graph, registry, components, node_id,
                footprint, true, &region, &region.global);
        } else {
            // Missing metadata is deliberately opaque. It can never silently
            // become a sparse region.
            region.global = true;
        }

        for (const auto& effect : definition->effects) {
            if (effect.access == orlgraph::AccessMode::Read
                || effect.access == orlgraph::AccessMode::ReadWrite)
            {
                append_unique(&region.read_resources, effect.resource);
            }
            if (effect.access == orlgraph::AccessMode::Write
                || effect.access == orlgraph::AccessMode::ReadWrite)
            {
                append_unique(&region.write_resources, effect.resource);
            }
        }
        if (region.stateful) {
            region.global = true;
            plan.stateful_evaluation_required = true;
        }
        if (definition->operation == "constraint") {
            region.propagation = region.write_joints.empty()
                ? orlgraph::PartialPropagation::None
                : orlgraph::PartialPropagation::Descendants;
        }
        const std::vector<ComponentId> initial_read_joints =
            region.read_joints;
        for (const ComponentId id : initial_read_joints) {
            append_ancestors(hierarchy, id, &region.read_joints);
        }
        for (const ComponentId id : region.write_joints) {
            append_unique(&region.affected_joints, id);
            if (region.propagation == orlgraph::PartialPropagation::Descendants
                || region.propagation
                    == orlgraph::PartialPropagation::AncestorsAndDescendants
                || region.propagation == orlgraph::PartialPropagation::Full)
            {
                append_descendants(hierarchy, id, &region.affected_joints);
            }
        }
        if (region.propagation == orlgraph::PartialPropagation::Ancestors
            || region.propagation
                == orlgraph::PartialPropagation::AncestorsAndDescendants)
        {
            for (const ComponentId id : region.write_joints) {
                append_ancestors(hierarchy, id, &region.affected_joints);
            }
        }
        if (region.propagation == orlgraph::PartialPropagation::Full) {
            region.global = true;
        }
        if (region.global) {
            region.affected_joints = hierarchy.preorder_joints;
            plan.full_evaluation_required = true;
        }

        build_packed_indices(components, region.read_joints,
            &region.read_packed_indices, &result.errors, "read joints");
        build_packed_indices(components, region.write_joints,
            &region.write_packed_indices, &result.errors, "write joints");
        build_packed_indices(components, region.affected_joints,
            &region.affected_packed_indices, &result.errors,
            "affected joints");
        if (!region.declared && !region.global) {
            region.global = true;
            plan.full_evaluation_required = true;
        }

        for (const ComponentId id : region.read_controllers) {
            plan.controller_regions[id.value].push_back(region.node_id);
        }
        for (const ComponentId id : region.read_locators) {
            plan.locator_regions[id.value].push_back(region.node_id);
        }
        for (const ComponentId id : region.read_joints) {
            plan.joint_readers[id.value].push_back(region.node_id);
        }
        for (const ComponentId id : region.write_joints) {
            plan.joint_writers[id.value].push_back(region.node_id);
        }
        plan.regions.push_back(std::move(region));
    }

    std::map<orlgraph::StableId, std::size_t> region_indices;
    for (std::size_t index = 0; index < plan.regions.size(); ++index) {
        region_indices.emplace(plan.regions[index].node_id, index);
    }
    for (const auto& connection : graph.connections()) {
        if (connection.source.kind != orlgraph::EndpointKind::NodePort
            || connection.destination.kind
                != orlgraph::EndpointKind::NodePort)
        {
            continue;
        }
        const auto source = region_indices.find(connection.source.owner);
        const auto destination =
            region_indices.find(connection.destination.owner);
        if (source == region_indices.end()
            || destination == region_indices.end()
            || source->second == destination->second)
        {
            continue;
        }
        append_unique(&plan.regions[destination->second].dependencies,
            source->second);
    }

    std::hash<std::string> hash;
    plan.dependency_revision = hash(graph.module_id)
        ^ (static_cast<std::uint64_t>(plan.regions.size()) << 32);
    for (const auto& region : plan.regions) {
        plan.dependency_revision ^= hash(region.node_id.value)
            + 0x9e3779b97f4a7c15ULL
            + (plan.dependency_revision << 6)
            + (plan.dependency_revision >> 2);
    }
    for (const auto& connection : graph.connections()) {
        plan.dependency_revision ^= hash(
                connection.source.owner.value
                + connection.source.port.value
                + connection.destination.owner.value
                + connection.destination.port.value)
            + 0x9e3779b97f4a7c15ULL
            + (plan.dependency_revision << 6)
            + (plan.dependency_revision >> 2);
    }

    const auto add_element_edges = [](
        const std::map<std::uint64_t, std::vector<std::size_t>>& writers,
        const std::map<std::uint64_t, std::vector<std::size_t>>& readers,
        std::vector<SolverRegion>* regions) {
        for (const auto& [id, writer_list] : writers) {
            const auto found = readers.find(id);
            if (found == readers.end()) {
                continue;
            }
            for (const std::size_t reader : found->second) {
                for (const std::size_t writer : writer_list) {
                    if (reader == writer) {
                        continue;
                    }
                    append_unique(&(*regions)[reader].dependencies, writer);
                }
            }
        }
    };
    std::map<std::uint64_t, std::vector<std::size_t>> joint_writer_regions;
    std::map<std::uint64_t, std::vector<std::size_t>> joint_reader_regions;
    std::map<std::uint64_t, std::vector<std::size_t>> controller_writer_regions;
    std::map<std::uint64_t, std::vector<std::size_t>> controller_reader_regions;
    std::map<std::uint64_t, std::vector<std::size_t>> locator_writer_regions;
    std::map<std::uint64_t, std::vector<std::size_t>> locator_reader_regions;
    const auto note_regions = [](
        const std::vector<ComponentId>& ids,
        std::size_t region_index,
        std::map<std::uint64_t, std::vector<std::size_t>>* output) {
        for (const ComponentId id : ids) {
            append_unique(&(*output)[id.value], region_index);
        }
    };
    for (std::size_t index = 0; index < plan.regions.size(); ++index) {
        const auto& region = plan.regions[index];
        note_regions(region.write_joints, index, &joint_writer_regions);
        note_regions(region.read_joints, index, &joint_reader_regions);
        note_regions(region.write_controllers, index,
            &controller_writer_regions);
        note_regions(region.read_controllers, index,
            &controller_reader_regions);
        note_regions(region.write_locators, index, &locator_writer_regions);
        note_regions(region.read_locators, index, &locator_reader_regions);
    }
    add_element_edges(joint_writer_regions, joint_reader_regions,
        &plan.regions);
    add_element_edges(controller_writer_regions, controller_reader_regions,
        &plan.regions);
    add_element_edges(locator_writer_regions, locator_reader_regions,
        &plan.regions);
    for (std::size_t left = 0; left < plan.regions.size(); ++left) {
        for (std::size_t right = 0; right < plan.regions.size(); ++right) {
            if (left == right) {
                continue;
            }
            for (const ComponentId parent : plan.regions[left].write_joints) {
                for (const ComponentId child : plan.regions[right].write_joints) {
                    if (is_ancestor(hierarchy, parent, child)) {
                        append_unique(&plan.regions[right].dependencies, left);
                    }
                }
            }
        }
    }

    std::vector<std::vector<std::size_t>> precedes(plan.regions.size());
    for (std::size_t index = 0; index < plan.regions.size(); ++index) {
        for (const std::size_t dependency : plan.regions[index].dependencies) {
            if (dependency < precedes.size() && dependency != index) {
                append_unique(&precedes[dependency], index);
            }
        }
    }
    const auto report_competing = [&](
        std::string_view kind,
        const std::map<std::uint64_t, std::vector<std::size_t>>& writers) {
        for (const auto& [id, writer_list] : writers) {
            if (writer_list.size() < 2) {
                continue;
            }
            for (std::size_t first = 0; first < writer_list.size(); ++first) {
                for (std::size_t second = first + 1;
                     second < writer_list.size(); ++second)
                {
                    const std::size_t left = writer_list[first];
                    const std::size_t right = writer_list[second];
                    const bool forward = reaches(precedes, left, right);
                    const bool backward = reaches(precedes, right, left);
                    if (forward != backward) {
                        continue;
                    }
                    result.errors.push_back(
                        "Competing writers of " + std::string{kind} + " '"
                        + component_label(components, ComponentId{id})
                        + "': " + plan.regions[left].node_id.value
                        + ", " + plan.regions[right].node_id.value);
                }
            }
        }
    };
    report_competing("joint", joint_writer_regions);
    report_competing("controller", controller_writer_regions);
    report_competing("locator", locator_writer_regions);
    for (std::size_t left = 0; left < plan.regions.size(); ++left) {
        if (!plan.regions[left].global) {
            continue;
        }
        for (std::size_t right = 0; right < plan.regions.size(); ++right) {
            if (left == right || plan.regions[right].global) {
                continue;
            }
            if (reaches(precedes, left, right)
                || reaches(precedes, right, left))
            {
                continue;
            }
            result.errors.push_back(
                "Unresolved order between '"
                + plan.regions[left].node_id.value + "' and '"
                + plan.regions[right].node_id.value + "'");
        }
    }

    std::vector<std::size_t> indegree(plan.regions.size(), 0);
    for (std::size_t index = 0; index < plan.regions.size(); ++index) {
        indegree[index] = plan.regions[index].dependencies.size();
    }
    std::vector<bool> placed(plan.regions.size(), false);
    std::size_t placed_count = 0;
    while (placed_count < plan.regions.size()) {
        std::vector<orlgraph::StableId> level;
        std::vector<std::size_t> level_indices;
        for (std::size_t index = 0; index < plan.regions.size(); ++index) {
            if (!placed[index] && indegree[index] == 0) {
                level.push_back(plan.regions[index].node_id);
                level_indices.push_back(index);
            }
        }
        if (level.empty()) {
            std::string message = "Evaluation dependency cycle involving";
            for (std::size_t index = 0; index < plan.regions.size(); ++index) {
                if (!placed[index]) {
                    message += " '" + plan.regions[index].node_id.value + "'";
                }
            }
            result.errors.push_back(std::move(message));
            break;
        }
        for (const std::size_t index : level_indices) {
            placed[index] = true;
            ++placed_count;
            for (const std::size_t dependent : precedes[index]) {
                if (indegree[dependent] > 0) {
                    --indegree[dependent];
                }
            }
        }
        plan.batches.push_back(std::move(level));
    }

    if (!result.errors.empty()) {
        return result;
    }
    result.plan = std::move(plan);
    return result;
}

DynamicDispatchPlan build_dynamic_dispatch_plan(
    const EvaluationPlan& plan, const DirtyInputs& dirty,
    double full_evaluation_threshold)
{
    DynamicDispatchPlan result;
    if (dirty.empty() && !plan.stateful_evaluation_required) {
        return result;
    }
    std::vector<bool> selected(plan.regions.size(), false);
    if (dirty.empty() || dirty.full_evaluation
        || plan.full_evaluation_required
        || plan.stateful_evaluation_required)
    {
        result.full_evaluation = true;
        for (std::size_t index = 0; index < plan.regions.size(); ++index) {
            selected[index] = true;
        }
    } else {
        for (std::size_t index = 0; index < plan.regions.size(); ++index) {
            const auto& region = plan.regions[index];
            bool is_dirty = region.global;
            if (!dirty.joints.empty()) {
                is_dirty = is_dirty
                    || intersects(region.affected_joints, dirty.joints)
                    || intersects(region.read_joints, dirty.joints);
            }
            if (!dirty.controllers.empty()) {
                is_dirty = is_dirty
                    || intersects(region.read_controllers,
                        dirty.controllers);
            }
            if (!dirty.locators.empty()) {
                is_dirty = is_dirty
                    || intersects(region.read_locators, dirty.locators);
            }
            for (const std::size_t dependency : region.dependencies) {
                if (dependency < selected.size() && selected[dependency]) {
                    is_dirty = true;
                    break;
                }
            }
            if (!is_dirty) {
                for (std::size_t previous = 0; previous < index; ++previous) {
                    if (!selected[previous]) {
                        continue;
                    }
                    const auto& prior = plan.regions[previous];
                    if (intersects(prior.affected_joints,
                            region.read_joints)
                        || intersects(prior.affected_joints,
                            region.affected_joints)
                        || resource_intersects(prior, region))
                    {
                        is_dirty = true;
                        break;
                    }
                }
            }
            selected[index] = is_dirty;
        }
        const std::size_t selected_count = static_cast<std::size_t>(
            std::count(selected.begin(), selected.end(), true));
        const double density = plan.regions.empty()
            ? 0.0
            : static_cast<double>(selected_count)
                / static_cast<double>(plan.regions.size());
        if (selected_count != 0
            && density > std::clamp(full_evaluation_threshold, 0.0, 1.0))
        {
            result.full_evaluation = true;
            std::fill(selected.begin(), selected.end(), true);
        }
    }

    for (const auto& batch : plan.batches) {
        std::vector<std::size_t> level;
        for (const auto& node_id : batch) {
            const auto found = std::find_if(plan.regions.begin(),
                plan.regions.end(),
                [&](const SolverRegion& region) {
                    return region.node_id == node_id;
                });
            if (found == plan.regions.end()) {
                continue;
            }
            const auto index = static_cast<std::size_t>(
                std::distance(plan.regions.begin(), found));
            if (index < selected.size() && selected[index])
            {
                level.push_back(index);
            }
        }
        if (!level.empty()) {
            result.levels.push_back(std::move(level));
        }
    }
    for (const auto& level : result.levels) {
        const auto first = static_cast<std::uint32_t>(
            result.region_indices.size());
        for (const auto index : level) {
            result.region_indices.push_back(
                static_cast<std::uint32_t>(index));
        }
        result.cuda_ranges.push_back({
            first,
            static_cast<std::uint32_t>(level.size()),
        });
    }
    return result;
}

DynamicDispatchPlan build_cuda_dispatch_plan(
    const EvaluationPlan& plan, const DirtyInputs& dirty,
    double full_evaluation_threshold)
{
    auto result = build_dynamic_dispatch_plan(
        plan, dirty, full_evaluation_threshold);
    if (result.region_indices.empty()) {
        return result;
    }
    const bool sparse_unsupported = std::any_of(
        result.region_indices.begin(), result.region_indices.end(),
        [&](std::uint32_t index) {
            return index >= plan.regions.size()
                || !plan.regions[index].supports_sparse_dispatch;
        });
    if (!sparse_unsupported) {
        return result;
    }

    result.full_evaluation = true;
    result.levels.clear();
    result.region_indices.clear();
    result.cuda_ranges.clear();
    for (const auto& batch : plan.batches) {
        std::vector<std::size_t> level;
        for (const auto& node_id : batch) {
            const auto found = std::find_if(plan.regions.begin(),
                plan.regions.end(),
                [&](const SolverRegion& region) {
                    return region.node_id == node_id;
                });
            if (found != plan.regions.end()) {
                level.push_back(static_cast<std::size_t>(
                    std::distance(plan.regions.begin(), found)));
            }
        }
        if (!level.empty()) {
            const auto first = static_cast<std::uint32_t>(
                result.region_indices.size());
            for (const auto index : level) {
                result.region_indices.push_back(
                    static_cast<std::uint32_t>(index));
            }
            result.levels.push_back(std::move(level));
            result.cuda_ranges.push_back({
                first,
                static_cast<std::uint32_t>(
                    result.levels.back().size()),
            });
        }
    }
    return result;
}

SolverDispatchRuntimeData pack_dynamic_dispatch_plan(
    const DynamicDispatchPlan& dispatch)
{
    SolverDispatchRuntimeData result;
    result.context.level_count =
        static_cast<std::int64_t>(dispatch.levels.size());
    result.context.range_count =
        static_cast<std::int64_t>(dispatch.cuda_ranges.size());
    result.context.index_count =
        static_cast<std::int64_t>(dispatch.region_indices.size());
    result.context.range_offset = 0;
    result.context.index_offset =
        static_cast<std::int64_t>(dispatch.cuda_ranges.size() * 2);
    result.context.full_evaluation = dispatch.full_evaluation ? 1 : 0;
    result.data.reserve(dispatch.cuda_ranges.size() * 2
        + dispatch.region_indices.size());
    for (const auto& range : dispatch.cuda_ranges) {
        result.data.push_back(static_cast<std::int64_t>(range.first));
        result.data.push_back(static_cast<std::int64_t>(range.count));
    }
    for (const std::uint32_t index : dispatch.region_indices) {
        result.data.push_back(static_cast<std::int64_t>(index));
    }
    if (result.data.empty()) {
        result.data.push_back(0);
    }
    return result;
}

} // namespace orlrig
