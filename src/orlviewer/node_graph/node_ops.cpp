#include "node_ops.hpp"

#if ORL_USE_QT6

#include <algorithm>
#include <cctype>
#include <map>
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

} // namespace ORL::node_graph

#endif
