#include "node_ops.hpp"

#if ORL_USE_QT6

#include <algorithm>
#include <cctype>
#include <map>
#include <set>
#include <string_view>
#include <utility>
#include <vector>

#include <QAction>
#include <QLineEdit>
#include <QMenu>
#include <QString>
#include <QWidgetAction>

namespace ORL::node_graph
{

namespace
{

struct MenuEntry {
    QAction* action = nullptr;
    QMenu* category = nullptr;
    QString search_text;
};

bool set_error(std::string* error, std::string message) {
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

std::string short_name(std::string_view qualified_name) {
    const auto separator = qualified_name.rfind('.');
    return separator == std::string_view::npos
        ? std::string{qualified_name}
        : std::string{qualified_name.substr(separator + 1)};
}

QString display_name(std::string_view qualified_name) {
    std::string value = short_name(qualified_name);
    for (char& character : value) {
        if (character == '_' || character == '-') {
            character = ' ';
        }
    }
    if (!value.empty()) {
        value.front() = static_cast<char>(
            std::toupper(static_cast<unsigned char>(value.front())));
    }
    return QString::fromStdString(value);
}

QString category_name(std::string_view qualified_name) {
    if (qualified_name == "orlrig.stage.computed_joints") {
        return QStringLiteral("Input");
    }
    if (qualified_name.find(".stage.") != std::string_view::npos) {
        return QStringLiteral("Stage");
    }
    if (qualified_name.find(".deformer.") != std::string_view::npos) {
        return QStringLiteral("Deformer");
    }
    if (qualified_name.find(".auto_weight.") != std::string_view::npos) {
        return QStringLiteral("Auto Weight");
    }
    if (qualified_name.find(".solver.") != std::string_view::npos) {
        return QStringLiteral("Solver");
    }
    if (qualified_name.find(".constraint.") != std::string_view::npos) {
        return QStringLiteral("Constraint");
    }
    if (qualified_name.find(".input") != std::string_view::npos) {
        return QStringLiteral("Input");
    }
    if (qualified_name.find(".output") != std::string_view::npos) {
        return QStringLiteral("Output");
    }
    return QStringLiteral("Utilities");
}

} // namespace

void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    orlgraph::GraphStage stage, const QPoint& global_position,
    CreateNodeCallback callback)
{
    show_create_menu(parent, registry, stage, {}, global_position,
        std::move(callback), {});
}

void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    orlgraph::GraphStage stage,
    const std::vector<GraphInputMenuEntry>& graph_inputs,
    const QPoint& global_position, CreateNodeCallback node_callback,
    CreateGraphInputCallback graph_input_callback)
{
    QMenu menu(parent);
    menu.setTitle(QStringLiteral("Add"));

    auto* title = menu.addAction(QStringLiteral("Add"));
    title->setEnabled(false);

    auto* search = new QLineEdit(&menu);
    search->setPlaceholderText(QStringLiteral("Search..."));
    search->setClearButtonEnabled(true);
    search->setMinimumWidth(180);

    auto* search_action = new QWidgetAction(&menu);
    search_action->setDefaultWidget(search);
    menu.addAction(search_action);
    menu.addSeparator();

    std::map<std::string, QMenu*> categories;
    std::vector<MenuEntry> entries;
    const auto definitions = registry.definitions_for(stage);
    const bool has_registry_input = std::any_of(
        definitions.begin(), definitions.end(),
        [](const auto* definition) {
            return category_name(definition->qualified_name)
                == QStringLiteral("Input");
        });
    QMenu* input_category = nullptr;
    if (has_registry_input || !graph_inputs.empty()) {
        input_category = menu.addMenu(QStringLiteral("Input"));
        categories.emplace("Input", input_category);
    }

    for (const auto* definition : definitions) {
        const QString category = category_name(definition->qualified_name);
        const std::string category_key = category.toStdString();
        auto category_it = categories.find(category_key);
        if (category_it == categories.end()) {
            auto* category_menu = menu.addMenu(category);
            category_it = categories.emplace(category_key, category_menu).first;
        }

        const QString label = display_name(definition->qualified_name);
        auto* action = category_it->second->addAction(label);
        const QString search_text = category + QStringLiteral(" ") + label;
        const orlgraph::StableId definition_id = definition->id;
        QObject::connect(action, &QAction::triggered, &menu,
            [node_callback, definition_id] {
                if (node_callback) {
                    node_callback(definition_id);
                }
            });
        entries.push_back(MenuEntry{action, category_it->second, search_text});
    }

    if (input_category != nullptr) {
        for (const auto& input : graph_inputs) {
            auto* action = input_category->addAction(
                QString::fromStdString(input.label));
            const auto input_id = input.id;
            const QString search_text = QStringLiteral("Input ")
                + QString::fromStdString(input.label);
            QObject::connect(action, &QAction::triggered, &menu,
                [graph_input_callback, input_id] {
                    if (graph_input_callback) {
                        graph_input_callback(input_id);
                    }
                });
            entries.push_back(MenuEntry{
                action, input_category, search_text});
        }

    }

    if (entries.empty()) {
        auto* empty = menu.addAction(QStringLiteral("No registered nodes"));
        empty->setEnabled(false);
    }

    QObject::connect(search, &QLineEdit::textChanged, &menu,
        [&entries](const QString& text) {
            const QString query = text.trimmed();
            std::map<QMenu*, bool> category_visible;
            for (const auto& entry : entries) {
                const bool visible = query.isEmpty()
                    || entry.search_text.contains(query, Qt::CaseInsensitive);
                entry.action->setVisible(visible);
                category_visible[entry.category] =
                    category_visible[entry.category] || visible;
            }
            for (const auto& [category, visible] : category_visible) {
                category->menuAction()->setVisible(visible);
            }
        });

    menu.setActiveAction(search_action);
    search->setFocus();
    menu.exec(global_position);
}

bool create_node(orlgraph::GraphModule& graph, const orlgraph::NodeRegistry& registry,
    orlgraph::GraphStage stage, const orlgraph::StableId& definition_id,
    orlgraph::StableId* created_id,
    std::string* error)
{
    const auto* definition = registry.find(definition_id);
    if (definition == nullptr) {
        return set_error(error, "Unknown node definition: " + definition_id.value);
    }
    if (!registry.is_available(definition_id, stage)) {
        return set_error(error, "Node definition '" + definition_id.value
            + "' is not available in the selected graph stage");
    }

    const std::string base_name = short_name(definition->qualified_name);
    if (base_name.empty()) {
        return set_error(error, "Node definition has no usable instance name: "
            + definition->qualified_name);
    }

    std::string instance_id = base_name;
    for (std::size_t suffix = 1;
         graph.node(orlgraph::StableId{instance_id}) != nullptr; ++suffix)
    {
        instance_id = base_name + "_" + std::to_string(suffix);
    }

    orlgraph::NodeInstance instance;
    instance.id = orlgraph::StableId{instance_id};
    instance.definition = definition->id;
    instance.name = instance_id;
    if (!graph.add_node(std::move(instance), error)) {
        return false;
    }

    if (created_id != nullptr) {
        *created_id = orlgraph::StableId{std::move(instance_id)};
    }
    return true;
}

bool delete_node(orlgraph::GraphModule& graph, const orlgraph::StableId& node_id,
    std::string* error)
{
    if (graph.node(node_id) == nullptr) {
        return set_error(error, "Unknown graph node: " + node_id.value);
    }

    auto& connections = graph.mutable_connections();
    connections.erase(std::remove_if(connections.begin(), connections.end(),
        [&node_id](const orlgraph::Connection& connection) {
            const auto references_node =
                [&node_id](const orlgraph::Endpoint& endpoint) {
                    return endpoint.kind == orlgraph::EndpointKind::NodePort
                        && endpoint.owner == node_id;
                };
            return references_node(connection.source)
                || references_node(connection.destination);
        }), connections.end());

    // Parameter mappings are another way for a node to reference a node
    // output, so remove mappings that would otherwise dangle after deletion.
    for (auto& [_, instance] : graph.mutable_nodes()) {
        instance.parameter_mappings.erase(
            std::remove_if(instance.parameter_mappings.begin(),
                instance.parameter_mappings.end(),
                [&node_id](const orlgraph::ParameterMapping& mapping) {
                    return mapping.source.kind == orlgraph::EndpointKind::NodePort
                        && mapping.source.owner == node_id;
                }),
            instance.parameter_mappings.end());
    }

    graph.mutable_nodes().erase(node_id);
    return true;
}

bool create_frame(const std::vector<FrameMemberBounds>& members,
    const std::vector<std::string>& existing_ids, FrameDesc* created,
    std::string* error)
{
    if (members.empty()) {
        return set_error(error, "Select at least one node to create a frame");
    }

    std::string instance_id = "frame";
    for (std::size_t suffix = 1;; ++suffix) {
        const bool taken = std::find(existing_ids.begin(), existing_ids.end(),
            instance_id) != existing_ids.end();
        if (!taken) {
            break;
        }
        instance_id = "frame_" + std::to_string(suffix);
    }

    double min_x = members.front().x;
    double min_y = members.front().y;
    double max_x = members.front().x + members.front().width;
    double max_y = members.front().y + members.front().height;
    FrameDesc result;
    result.id = instance_id;
    result.title = "Frame";
    result.members.reserve(members.size());
    for (const auto& member : members) {
        min_x = std::min(min_x, member.x);
        min_y = std::min(min_y, member.y);
        max_x = std::max(max_x, member.x + member.width);
        max_y = std::max(max_y, member.y + member.height);
        result.members.push_back(member.id);
    }
    result.x = min_x - kFramePadding;
    result.y = min_y - kFrameHeader - kFramePadding;
    result.width = (max_x + kFramePadding) - result.x;
    result.height = (max_y + kFramePadding) - result.y;
    if (created != nullptr) {
        *created = std::move(result);
    }
    return true;
}

std::string editor_id_for(CopiedNodeKind kind, const std::string& owner_id)
{
    if (kind == CopiedNodeKind::GraphInput) {
        return "__graph_input__:" + owner_id;
    }
    if (kind == CopiedNodeKind::GraphOutput) {
        return "__graph_output__:" + owner_id;
    }
    return owner_id;
}

bool endpoint_in_copy(const orlgraph::Endpoint& endpoint,
    const std::vector<CopiedNode>& nodes)
{
    for (const auto& node : nodes) {
        if (node.owner_id != endpoint.owner.value) {
            continue;
        }
        if (endpoint.kind == orlgraph::EndpointKind::NodePort
            && node.kind == CopiedNodeKind::Definition)
        {
            return true;
        }
        if (endpoint.kind == orlgraph::EndpointKind::GraphInput
            && node.kind == CopiedNodeKind::GraphInput)
        {
            return true;
        }
        if (endpoint.kind == orlgraph::EndpointKind::GraphOutput
            && node.kind == CopiedNodeKind::GraphOutput)
        {
            return true;
        }
    }
    return false;
}

using OwnerKey = std::pair<orlgraph::EndpointKind, std::string>;

OwnerKey owner_key(CopiedNodeKind kind, const std::string& owner_id)
{
    if (kind == CopiedNodeKind::GraphInput) {
        return {orlgraph::EndpointKind::GraphInput, owner_id};
    }
    if (kind == CopiedNodeKind::GraphOutput) {
        return {orlgraph::EndpointKind::GraphOutput, owner_id};
    }
    return {orlgraph::EndpointKind::NodePort, owner_id};
}

bool id_taken(const orlgraph::GraphModule& graph,
    const std::set<std::string>& reserved, const std::string& id)
{
    if (reserved.contains(id)) {
        return true;
    }
    const orlgraph::StableId stable{id};
    return graph.node(stable) != nullptr
        || graph.input(stable) != nullptr
        || graph.output(stable) != nullptr;
}

std::string unique_copy_id(const orlgraph::GraphModule& graph,
    const std::set<std::string>& reserved, const std::string& base)
{
    std::string candidate = base.empty() ? "node" : base;
    for (std::size_t suffix = 1;; ++suffix) {
        if (!id_taken(graph, reserved, candidate)) {
            return candidate;
        }
        candidate = (base.empty() ? "node" : base) + "_" + std::to_string(suffix);
    }
}

std::string unique_label(const std::string& base,
    std::vector<std::string>& existing, const std::string& suffix_text)
{
    const std::string stem = base.empty() ? "Frame" : base;
    std::string candidate = stem + suffix_text;
    if (std::find(existing.begin(), existing.end(), candidate) == existing.end()) {
        existing.push_back(candidate);
        return candidate;
    }
    for (std::size_t index = 2;; ++index) {
        candidate = stem + suffix_text + " " + std::to_string(index);
        if (std::find(existing.begin(), existing.end(), candidate) == existing.end()) {
            existing.push_back(candidate);
            return candidate;
        }
    }
}

bool copy_nodes(const orlgraph::GraphModule& graph,
    const std::vector<CopyNodeRef>& nodes,
    const std::vector<CopiedFrame>& frames,
    NodeClipboard* clipboard, std::string* error)
{
    if (clipboard == nullptr) {
        return set_error(error, "Node clipboard destination is null");
    }
    NodeClipboard copied;
    copied.nodes.reserve(nodes.size());
    for (const auto& ref : nodes) {
        CopiedNode node;
        node.kind = ref.kind;
        node.editor_id = ref.editor_id;
        node.owner_id = ref.owner_id;
        node.x = ref.x;
        node.y = ref.y;
        if (ref.kind == CopiedNodeKind::Definition) {
            const auto* instance = graph.node(orlgraph::StableId{ref.owner_id});
            if (instance == nullptr) {
                continue;
            }
            node.instance = *instance;
        } else if (ref.kind == CopiedNodeKind::GraphInput) {
            const auto* input = graph.input(orlgraph::StableId{ref.owner_id});
            if (input == nullptr) {
                continue;
            }
            node.interface_port = *input;
        } else {
            const auto* output = graph.output(orlgraph::StableId{ref.owner_id});
            if (output == nullptr) {
                continue;
            }
            node.interface_port = *output;
        }
        copied.nodes.push_back(std::move(node));
    }
    if (copied.nodes.empty()) {
        return set_error(error, "Nothing to copy");
    }
    for (const auto& connection : graph.connections()) {
        if (endpoint_in_copy(connection.source, copied.nodes)
            && endpoint_in_copy(connection.destination, copied.nodes))
        {
            copied.connections.push_back(connection);
        }
    }
    std::set<std::string> copied_editors;
    for (const auto& node : copied.nodes) {
        copied_editors.insert(node.editor_id);
    }
    for (const auto& frame : frames) {
        CopiedFrame copied_frame = frame;
        copied_frame.members.erase(std::remove_if(
            copied_frame.members.begin(), copied_frame.members.end(),
            [&copied_editors](const std::string& member) {
                return !copied_editors.contains(member);
            }), copied_frame.members.end());
        if (!copied_frame.members.empty()) {
            copied.frames.push_back(std::move(copied_frame));
        }
    }
    *clipboard = std::move(copied);
    return true;
}

bool paste_nodes(orlgraph::GraphModule& graph,
    const orlgraph::NodeRegistry& registry, orlgraph::GraphStage stage,
    const NodeClipboard& clipboard, double offset_x, double offset_y,
    const std::vector<std::string>& existing_frame_ids,
    const std::vector<std::string>& existing_frame_titles,
    PasteResult* result, std::string* error)
{
    if (clipboard.nodes.empty()) {
        return set_error(error, "Nothing to paste");
    }

    std::set<std::string> reserved;
    std::map<OwnerKey, std::string> new_owners;
    std::map<std::string, std::string> new_editors;
    for (const auto& node : clipboard.nodes) {
        std::string base = node.owner_id;
        if (node.kind == CopiedNodeKind::Definition) {
            const auto* definition = registry.find(node.instance.definition);
            if (definition == nullptr
                || !registry.is_available(node.instance.definition, stage))
            {
                continue;
            }
            base = short_name(definition->qualified_name);
        }
        const std::string created = unique_copy_id(graph, reserved, base);
        reserved.insert(created);
        new_owners.emplace(owner_key(node.kind, node.owner_id), created);
        new_editors.emplace(node.editor_id, editor_id_for(node.kind, created));
    }
    if (new_owners.empty()) {
        return set_error(error, "Nothing to paste");
    }

    PasteResult pasted;
    for (const auto& node : clipboard.nodes) {
        const auto created = new_owners.find(owner_key(node.kind, node.owner_id));
        if (created == new_owners.end()) {
            continue;
        }
        const std::string& new_id = created->second;
        if (node.kind == CopiedNodeKind::Definition) {
            orlgraph::NodeInstance instance = node.instance;
            instance.id = orlgraph::StableId{new_id};
            instance.name = new_id;
            for (auto& mapping : instance.parameter_mappings) {
                const auto remapped = new_owners.find({
                    mapping.source.kind, mapping.source.owner.value});
                if (remapped != new_owners.end()) {
                    mapping.source.owner = orlgraph::StableId{remapped->second};
                }
            }
            if (!graph.add_node(std::move(instance), error)) {
                return false;
            }
        } else {
            orlgraph::InterfacePort port = node.interface_port;
            port.id = orlgraph::StableId{new_id};
            if (port.name.empty()) {
                port.name = new_id;
            } else {
                port.name += "_copy";
            }
            const bool added = node.kind == CopiedNodeKind::GraphInput
                ? graph.add_input(std::move(port), error)
                : graph.add_output(std::move(port), error);
            if (!added) {
                return false;
            }
        }
        pasted.nodes.push_back(PastedNode{
            new_editors.at(node.editor_id),
            node.x + offset_x,
            node.y + offset_y,
        });
    }

    for (const auto& connection : clipboard.connections) {
        const auto source = new_owners.find({
            connection.source.kind, connection.source.owner.value});
        const auto destination = new_owners.find({
            connection.destination.kind, connection.destination.owner.value});
        if (source == new_owners.end() || destination == new_owners.end()) {
            continue;
        }
        orlgraph::Connection copy = connection;
        copy.source.owner = orlgraph::StableId{source->second};
        copy.destination.owner = orlgraph::StableId{destination->second};
        if (!graph.add_connection(std::move(copy), error)) {
            return false;
        }
    }

    std::vector<std::string> frame_ids = existing_frame_ids;
    std::vector<std::string> frame_titles = existing_frame_titles;
    for (const auto& frame : clipboard.frames) {
        PastedFrame pasted_frame;
        pasted_frame.id = "frame";
        for (std::size_t suffix = 1;; ++suffix) {
            if (std::find(frame_ids.begin(), frame_ids.end(), pasted_frame.id)
                == frame_ids.end())
            {
                break;
            }
            pasted_frame.id = "frame_" + std::to_string(suffix);
        }
        frame_ids.push_back(pasted_frame.id);
        pasted_frame.title = unique_label(frame.title, frame_titles, " copy");
        pasted_frame.x = frame.x + offset_x;
        pasted_frame.y = frame.y + offset_y;
        pasted_frame.width = frame.width;
        pasted_frame.height = frame.height;
        pasted_frame.collapsed = frame.collapsed;
        for (const auto& member : frame.members) {
            const auto remapped = new_editors.find(member);
            if (remapped != new_editors.end()) {
                pasted_frame.members.push_back(remapped->second);
            }
        }
        if (pasted_frame.members.empty()) {
            continue;
        }
        pasted.frames.push_back(std::move(pasted_frame));
    }
    if (result != nullptr) {
        *result = std::move(pasted);
    }
    return true;
}

} // namespace ORL::node_graph

#endif
