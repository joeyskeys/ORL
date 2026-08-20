#include "load_model_op.hpp"

#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "asset_mgr/drawable_mgr.h"
#include "vk_ins/types.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <windows.h>
#include <commdlg.h>
#endif

namespace ORL
{

LoadModelOp::LoadModelOp(vkkk::Scene& scene, vkkk::Context& context, GLFWwindow* window)
    : scene(scene)
    , context(context)
    , window(window)
{
}

void LoadModelOp::on_eval(const InputEvent&) {
    if (busy) {
        return;
    }
    busy = true;
    struct BusyGuard {
        bool& flag;
        ~BusyGuard() { flag = false; }
    } guard{busy};

    const auto path = open_obj_dialog();
    if (path.empty()) {
        return;
    }
    if (scene.drawable_mgr == nullptr) {
        std::cerr << "LoadModelOp: scene has no DrawableMgr\n";
        return;
    }

    std::unordered_set<std::string> before;
    for (const auto& name : scene.drawable_mgr->mesh_names()) {
        before.insert(name);
    }

    const std::string base_name = path.stem().string();
    try {
        scene.drawable_mgr->load_file(path, base_name, {vkkk::VERTEX, vkkk::NORMAL});
    }
    catch (const std::exception& error) {
        std::cerr << "LoadModelOp: " << error.what() << '\n';
        return;
    }

    std::vector<std::string> added;
    for (const auto& name : scene.drawable_mgr->mesh_names()) {
        if (!before.contains(name)) {
            added.push_back(name);
        }
    }
    if (added.empty()) {
        std::cerr << "LoadModelOp: no mesh loaded from '" << path.string() << "'\n";
        return;
    }

    context.wait_idle();
    scene.drawable_mgr->sync_to_gpu(&context);

    for (const auto& mesh_name : added) {
        const std::string object_name = unique_object_name(mesh_name);
        if (!scene.add_object(object_name, mesh_name)) {
            std::cerr << "LoadModelOp: failed to add scene object '" << object_name << "'\n";
            continue;
        }
        std::cout << "Loaded model '" << object_name << "' from " << path.string() << '\n';
    }
}

std::string LoadModelOp::unique_object_name(std::string base) const {
    if (base.empty()) {
        base = "model";
    }
    if (scene.find_object(base) == nullptr) {
        return base;
    }
    for (int index = 1;; ++index) {
        std::string candidate = base + "_" + std::to_string(index);
        if (scene.find_object(candidate) == nullptr) {
            return candidate;
        }
    }
}

std::filesystem::path LoadModelOp::open_obj_dialog() const {
#ifdef _WIN32
    wchar_t file_buffer[32768] = {};
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = window != nullptr ? glfwGetWin32Window(window) : nullptr;
    ofn.lpstrFilter = L"Wavefront OBJ (*.obj)\0*.obj\0";
    ofn.lpstrFile = file_buffer;
    ofn.nMaxFile = static_cast<DWORD>(sizeof(file_buffer) / sizeof(file_buffer[0]));
    ofn.lpstrTitle = L"Open Model";
    ofn.lpstrDefExt = L"obj";
    ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameW(&ofn) != TRUE) {
        return {};
    }
    return std::filesystem::path(file_buffer);
#else
    std::cerr << "LoadModelOp: file dialog is only implemented on Windows\n";
    return {};
#endif
}

} // namespace ORL
