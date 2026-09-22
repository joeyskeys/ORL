#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "component_manager.hpp"
#include "scene_graph_context.hpp"

namespace ORL
{

// Viewer-only node-graph layout. Not part of GraphModule / orlgraph.
struct NodeGraphFrameLayout {
    std::string id;
    std::string title;
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
    bool collapsed = false;
    std::vector<std::string> members;
};

struct NodeGraphStageLayout {
    double pan_x = 48.0;
    double pan_y = 48.0;
    double zoom = 1.0;
    std::unordered_map<std::string, std::array<double, 2>> nodes;
    std::vector<NodeGraphFrameLayout> frames;
};

struct NodeGraphLayout {
    NodeGraphStageLayout solver;
    NodeGraphStageLayout deformer;
};

struct ProjectIoResult {
    bool ok = false;
    bool evaluate_orl = true;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    NodeGraphLayout node_graph_layout;
};

ProjectIoResult save_project_json(
    const std::string& path,
    SceneGraphContext& graph_context,
    const ComponentManager& components,
    ComponentId weight_id,
    ComponentId deformer_id,
    bool evaluate_orl,
    const NodeGraphLayout* node_graph_layout = nullptr);

ProjectIoResult load_project_json(
    const std::string& path,
    SceneGraphContext& graph_context,
    ComponentManager& components,
    ComponentId weight_id,
    ComponentId deformer_id);

} // namespace ORL
