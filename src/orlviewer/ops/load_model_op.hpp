#pragma once

#include <filesystem>
#include <string>

#include <GLFW/glfw3.h>

#include "asset_mgr/scene.h"
#include "vk_ins/context.hpp"
#include "vp_operation.hpp"

namespace ORL
{

class LoadModelOp : public VpOperation<LoadModelOp> {
public:
    LoadModelOp(vkkk::Scene& scene, vkkk::Context& context, GLFWwindow* window);

    void on_eval(const InputEvent& event);

private:
    std::filesystem::path open_obj_dialog() const;
    std::string unique_object_name(std::string base) const;

    vkkk::Scene& scene;
    vkkk::Context& context;
    GLFWwindow* window = nullptr;
    bool busy = false;
};

} // namespace ORL
