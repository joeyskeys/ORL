#pragma once

#include <string>
#include <vector>

#include "component_manager.hpp"
#include "scene_graph_context.hpp"

namespace ORL
{

struct ProjectIoResult {
    bool ok = false;
    bool evaluate_orl = true;
    std::vector<std::string> errors;
};

ProjectIoResult save_project_json(
    const std::string& path,
    SceneGraphContext& graph_context,
    const ComponentManager& components,
    ComponentId weight_id,
    ComponentId deformer_id,
    bool evaluate_orl);

ProjectIoResult load_project_json(
    const std::string& path,
    SceneGraphContext& graph_context,
    ComponentManager& components,
    ComponentId weight_id,
    ComponentId deformer_id);

} // namespace ORL
