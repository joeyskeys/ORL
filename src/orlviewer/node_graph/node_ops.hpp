#pragma once

#if ORL_USE_QT6

#include <functional>
#include <string>
#include <vector>

#include <QPoint>

#include "orlgraph/graph_ir.hpp"

class QWidget;

namespace ORL::node_graph
{

using CreateNodeCallback = std::function<void(const orlgraph::StableId&)>;
using CreateGraphInputCallback = std::function<void(const orlgraph::StableId&)>;
using CreateInputCallback = std::function<void(const orlgraph::StableId&)>;

struct GraphInputMenuEntry {
    orlgraph::StableId id;
    std::string label;
};

struct SceneInputMenuEntry {
    orlgraph::StableId id;
    std::string label;
};

// Show the searchable, category-based node creation menu at a global screen
// position. The callback receives the selected definition ID.
void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    const QPoint& global_position, CreateNodeCallback callback);
void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    const std::vector<SceneInputMenuEntry>& scene_inputs,
    const QPoint& global_position, CreateNodeCallback node_callback,
    CreateInputCallback input_callback);
void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    const std::vector<GraphInputMenuEntry>& graph_inputs,
    const std::vector<SceneInputMenuEntry>& scene_inputs,
    const QPoint& global_position, CreateNodeCallback node_callback,
    CreateGraphInputCallback graph_input_callback,
    CreateInputCallback input_callback);

// Add a uniquely named instance of a registered definition to a graph.
bool create_node(orlgraph::GraphModule& graph, const orlgraph::NodeRegistry& registry,
    const orlgraph::StableId& definition_id, orlgraph::StableId* created_id = nullptr,
    std::string* error = nullptr);

// Remove a node instance and any graph data that references it.
bool delete_node(orlgraph::GraphModule& graph, const orlgraph::StableId& node_id,
    std::string* error = nullptr);

} // namespace ORL::node_graph

#endif
