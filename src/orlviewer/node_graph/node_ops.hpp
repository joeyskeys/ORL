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

struct GraphInputMenuEntry {
    orlgraph::StableId id;
    std::string label;
};

// Show the searchable, category-based node creation menu at a global screen
// position. The callback receives the selected definition ID.
void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    orlgraph::GraphStage stage, const QPoint& global_position,
    CreateNodeCallback callback);
void show_create_menu(QWidget* parent, const orlgraph::NodeRegistry& registry,
    orlgraph::GraphStage stage,
    const std::vector<GraphInputMenuEntry>& graph_inputs,
    const QPoint& global_position, CreateNodeCallback node_callback,
    CreateGraphInputCallback graph_input_callback);

// Add a uniquely named instance of a registered definition to a graph.
bool create_node(orlgraph::GraphModule& graph, const orlgraph::NodeRegistry& registry,
    orlgraph::GraphStage stage, const orlgraph::StableId& definition_id,
    orlgraph::StableId* created_id = nullptr,
    std::string* error = nullptr);

// Remove a node instance and any graph data that references it.
bool delete_node(orlgraph::GraphModule& graph, const orlgraph::StableId& node_id,
    std::string* error = nullptr);

inline constexpr double kFramePadding = 20.0;
inline constexpr double kFrameHeader = 32.0;
inline constexpr double kFrameCollapsedHeight = 36.0;
inline constexpr double kFrameCollapsedMinWidth = 160.0;

struct FrameMemberBounds {
    std::string id;
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

struct FrameDesc {
    std::string id;
    std::string title;
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
    bool collapsed = false;
    std::vector<std::string> members;
};

// Build an editor-only frame around the given members. Does not modify
// GraphModule; the caller stores the result in the node-graph layout.
bool create_frame(const std::vector<FrameMemberBounds>& members,
    const std::vector<std::string>& existing_ids, FrameDesc* created,
    std::string* error = nullptr);

} // namespace ORL::node_graph

#endif
