#include "load_model_op.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/mat3x3.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>

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
namespace
{

glm::mat3 frame_linear_map(const Frame& from, const Frame& to) {
    const Mat4f m = Frame::get_base_change_matrix(from, to);
    return {
        glm::vec3{m[0][0], m[0][1], m[0][2]},
        glm::vec3{m[1][0], m[1][1], m[1][2]},
        glm::vec3{m[2][0], m[2][1], m[2][2]},
    };
}

// OBJ/Assimp keep CCW faces in a right-handed frame. A handedness-changing
// Frame map (frame_gl -> frame_dx is a Z mirror) reverses winding; swap the
// last two indices so the mesh stays CCW from the outside in world space.
void transform_mesh_to_frame(vkkk::Mesh& mesh, const Frame& from, const Frame& to) {
    if (from.flag == to.flag || mesh.vbuf == nullptr) {
        return;
    }

    const glm::mat3 linear = frame_linear_map(from, to);
    const glm::mat3 normal_mat = glm::transpose(glm::inverse(linear));
    const bool flip_winding = glm::determinant(linear) < 0.0f;

    int vertex_offset = -1;
    int normal_offset = -1;
    int packed_offset = 0;
    for (const auto comp : mesh.comps) {
        if (comp == vkkk::VERTEX) {
            vertex_offset = packed_offset;
        }
        else if (comp == vkkk::NORMAL) {
            normal_offset = packed_offset;
        }
        packed_offset += static_cast<int>(vkkk::comp_sizes[comp]);
    }

    for (std::uint32_t v = 0; v < mesh.vcnt; ++v) {
        float* vertex = mesh.vbuf + v * mesh.comp_size;
        if (vertex_offset >= 0) {
            const glm::vec3 p = linear * glm::vec3{
                vertex[vertex_offset], vertex[vertex_offset + 1], vertex[vertex_offset + 2]};
            vertex[vertex_offset] = p.x;
            vertex[vertex_offset + 1] = p.y;
            vertex[vertex_offset + 2] = p.z;
        }
        if (normal_offset >= 0) {
            glm::vec3 n{
                vertex[normal_offset], vertex[normal_offset + 1], vertex[normal_offset + 2]};
            n = normal_mat * n;
            if (glm::dot(n, n) > 0.0f) {
                n = glm::normalize(n);
            }
            vertex[normal_offset] = n.x;
            vertex[normal_offset + 1] = n.y;
            vertex[normal_offset + 2] = n.z;
        }
    }

    if (flip_winding && mesh.ibuf != nullptr) {
        for (std::uint32_t i = 0; i < mesh.icnt; ++i) {
            std::swap(mesh.ibuf[i * 3 + 1], mesh.ibuf[i * 3 + 2]);
        }
    }
}

} // namespace

LoadModelOp::LoadModelOp(vkkk::Scene& scene, vkkk::Context& context, GLFWwindow* window,
    Frame world_frame, Frame file_frame)
    : scene(scene)
    , context(context)
    , window(window)
    , world_frame(world_frame)
    , file_frame(file_frame)
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

    for (const auto& mesh_name : added) {
        if (auto* mesh = scene.drawable_mgr->find_mesh(mesh_name)) {
            transform_mesh_to_frame(*mesh, file_frame, world_frame);
        }
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
