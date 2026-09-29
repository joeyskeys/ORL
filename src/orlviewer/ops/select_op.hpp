#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "comps/joint.hpp"
#include "gui/input.hpp"
#include "gui/window_backend.hpp"
#include "ops/create_joint_op.hpp"
#include "selection.hpp"
#include "vp/joint_picking_feature.hpp"
#include "vp/mesh_picking_feature.hpp"
#include "vp/rig_picking_feature.hpp"
#include "vp_operation.hpp"

namespace ORL
{

class SelectOp : public VpOperation<SelectOp> {
public:
    SelectOp(Selection& selection, ComponentManager& components, vkkk::Scene& scene,
        const vkkk::Camera& camera, vkkk::WindowBackend* window,
        const CreateJointOp& create_joint)
        : selection(selection)
        , components(components)
        , scene(scene)
        , camera(camera)
        , window(window)
        , create_joint(create_joint)
    {
    }

    void set_gpu_picking(JointPickingFeature& gpu_picking) {
        this->gpu_picking = &gpu_picking;
        gpu_picking.set_hit_callback([this](
            std::uint64_t request_id, const std::vector<GpuPickHit>& hits, bool overflow) {
            apply_gpu_joint_hits(request_id, hits, overflow);
        });
    }

    void set_mesh_picking(MeshPickingFeature& mesh_picking) {
        this->mesh_picking = &mesh_picking;
        mesh_picking.set_hit_callback([this](
            std::uint64_t request_id, const std::vector<GpuPickHit>& hits, bool overflow) {
            apply_gpu_mesh_hits(request_id, hits, overflow);
        });
    }

    void set_rig_picking(RigPickingFeature& rig_picking) {
        this->rig_picking = &rig_picking;
        rig_picking.set_hit_callback([this](
            std::uint64_t request_id, const std::vector<GpuPickHit>& hits, bool overflow) {
            apply_gpu_rig_hits(request_id, hits, overflow);
        });
    }

    void invalidate() {
        pending_hits.clear();
        awaiting_joint = false;
        awaiting_mesh = false;
        awaiting_rig = false;
        expected_joint_request = 0;
        expected_mesh_request = 0;
        expected_rig_request = 0;
        cycle_valid_ = false;
    }

    bool last_pick_overflow() const { return last_pick_overflow_; }

    void on_eval(const InputEvent& event) {
        if (create_joint.active()) {
            return;
        }
        if (event.kind != InputEvent::Kind::MouseButton
            || event.button != vkkk::MouseButton::Left
            || event.action != vkkk::InputAction::Press)
        {
            return;
        }

        fallback_x = event.x;
        fallback_y = event.y;
        pending_action = (event.mods & vkkk::input_mod::shift) != 0
            ? SelectionAction::Add : SelectionAction::Replace;
        click_x_ = pixel_coordinate(event.x);
        click_y_ = pixel_coordinate(event.y);
        pending_hits.clear();
        pending_overflow = false;
        last_pick_overflow_ = false;
        pending_mask = selection.mask();
        pending_mask_revision = selection.mask_revision();
        pending_view_ = camera.ubo_data.view;
        pending_proj_ = camera.ubo_data.proj;
        ++transaction_id;
        const auto pick_event = vkkk::mouse_button_event(
            vkkk::MouseButton::Left, vkkk::InputAction::Press,
            event.x, event.y, event.mods);
        expected_joint_request = gpu_picking != nullptr
            ? gpu_picking->request(pick_event) : 0;
        expected_mesh_request = mesh_picking != nullptr
            ? mesh_picking->request(pick_event) : 0;
        expected_rig_request = rig_picking != nullptr
            ? rig_picking->request(pick_event) : 0;
        awaiting_joint = expected_joint_request != 0;
        awaiting_mesh = expected_mesh_request != 0;
        awaiting_rig = expected_rig_request != 0;
        if (awaiting_joint || awaiting_mesh || awaiting_rig) {
            return;
        }
        pick_cpu(event.x, event.y);
    }

private:
    struct Hit {
        SelectionRef ref;
        float depth = 0.0f;
    };

    static constexpr float kPickPixels = 16.0f;

    void apply_gpu_joint_hits(std::uint64_t request_id,
        const std::vector<GpuPickHit>& hits, bool overflow) {
        if (request_id != expected_joint_request) {
            return;
        }
        awaiting_joint = false;
        expected_joint_request = 0;
        pending_overflow = pending_overflow || overflow;
        for (const auto& hit : hits) {
            if (gpu_picking == nullptr) {
                continue;
            }
            const auto id = gpu_picking->resolve_joint_index(hit.id);
            if (!id || components.joint(id) == nullptr) {
                continue;
            }
            append_hit(SelectionRef::joint(id), hit.depth);
        }
        finish_gpu_pick();
    }

    void apply_gpu_mesh_hits(std::uint64_t request_id,
        const std::vector<GpuPickHit>& hits, bool overflow) {
        if (request_id != expected_mesh_request) {
            return;
        }
        awaiting_mesh = false;
        expected_mesh_request = 0;
        pending_overflow = pending_overflow || overflow;
        if (mesh_picking != nullptr) {
            for (const auto& hit : hits) {
                const auto& name = mesh_picking->object_name(hit.id);
                if (name.empty() || scene.find_object(name) == nullptr) {
                    continue;
                }
                append_hit(SelectionRef::scene_object(name), hit.depth);
            }
        }
        finish_gpu_pick();
    }

    void apply_gpu_rig_hits(std::uint64_t request_id,
        const std::vector<GpuPickHit>& hits, bool overflow) {
        if (request_id != expected_rig_request) {
            return;
        }
        awaiting_rig = false;
        expected_rig_request = 0;
        pending_overflow = pending_overflow || overflow;
        if (rig_picking != nullptr) {
            for (const auto& hit : hits) {
                SelectionRef ref;
                if (rig_picking->resolve(hit.id, ref)) {
                    append_hit(std::move(ref), hit.depth);
                }
            }
        }
        finish_gpu_pick();
    }

    void finish_gpu_pick() {
        if (awaiting_joint || awaiting_mesh || awaiting_rig) {
            return;
        }
        if (selection.mask_revision() != pending_mask_revision) {
            pending_hits.clear();
            cycle_valid_ = false;
            return;
        }
        if (!matrix_equal(camera.ubo_data.view, pending_view_)
            || !matrix_equal(camera.ubo_data.proj, pending_proj_))
        {
            pending_hits.clear();
            cycle_valid_ = false;
            return;
        }
        if (pending_overflow) {
            last_pick_overflow_ = true;
            std::cerr << "Selection GPU A-buffer overflow; click ignored\n";
            pending_hits.clear();
            cycle_valid_ = false;
            return;
        }
        if (!pending_hits.empty()) {
            apply_hits(pending_hits);
        } else {
            pick_cpu(fallback_x, fallback_y);
        }
    }

    bool pick_cpu_controllers(double cursor_x, double cursor_y) {
        SelectionRef chosen;
        if (!closest_controller(cursor_x, cursor_y, chosen)) {
            return false;
        }
        apply_hits({Hit{chosen, depth(chosen)}});
        return true;
    }

    bool pick_cpu_locators(double cursor_x, double cursor_y) {
        SelectionRef chosen;
        if (!closest_locator(cursor_x, cursor_y, chosen)) {
            return false;
        }
        apply_hits({Hit{chosen, depth(chosen)}});
        return true;
    }

    static std::int64_t pixel_coordinate(double value) {
        return static_cast<std::int64_t>(std::floor(value));
    }

    bool same_cycle_pixel() const {
        return cycle_valid_
            && click_x_ == cycle_x_
            && click_y_ == cycle_y_
            && cycle_mask_revision_ == selection.mask_revision()
            && matrix_equal(camera.ubo_data.view, cycle_view_)
            && matrix_equal(camera.ubo_data.proj, cycle_proj_);
    }

    void append_hit(SelectionRef ref, float hit_depth) {
        if (!ref) {
            return;
        }
        pending_hits.push_back(Hit{std::move(ref), hit_depth});
    }

    static bool matrix_equal(const glm::mat4& first, const glm::mat4& second) {
        for (int column = 0; column < 4; ++column) {
            for (int row = 0; row < 4; ++row) {
                if (std::abs(first[column][row] - second[column][row]) > 1.0e-5f) {
                    return false;
                }
            }
        }
        return true;
    }

    void apply_hits(std::vector<Hit> hits) {
        std::stable_sort(hits.begin(), hits.end(),
            [](const Hit& first, const Hit& second) {
                if (!std::isfinite(first.depth)) {
                    return false;
                }
                if (!std::isfinite(second.depth)) {
                    return true;
                }
                if (first.depth != second.depth) {
                    return first.depth < second.depth;
                }
                if (first.ref.kind != second.ref.kind) {
                    return static_cast<int>(first.ref.kind)
                        < static_cast<int>(second.ref.kind);
                }
                switch (first.ref.kind) {
                case SelectionRef::Kind::SceneObject:
                    return first.ref.object_name < second.ref.object_name;
                case SelectionRef::Kind::Vector:
                    return first.ref.vector_id < second.ref.vector_id;
                case SelectionRef::Kind::Joint:
                case SelectionRef::Kind::Controller:
                case SelectionRef::Kind::Locator:
                    return first.ref.component.value < second.ref.component.value;
                case SelectionRef::Kind::None:
                    return false;
                }
                return false;
            });
        std::vector<Hit> unique_hits;
        unique_hits.reserve(hits.size());
        for (auto& hit : hits) {
            const bool duplicate = std::any_of(unique_hits.begin(),
                unique_hits.end(), [&hit](const Hit& existing) {
                    return selection_refs_equal(existing.ref, hit.ref);
                });
            if (!duplicate) {
                unique_hits.push_back(std::move(hit));
            }
        }

        std::vector<Hit> allowed;
        allowed.reserve(unique_hits.size());
        for (const auto& hit : unique_hits) {
            if (selection_mask_allows(pending_mask, hit.ref.kind)) {
                allowed.push_back(hit);
            }
        }
        if (allowed.empty()) {
            return;
        }

        const bool repeat = same_cycle_pixel();
        std::size_t anchor = allowed.size();
        if (repeat) {
            if (const auto* focus = selection.focus(); focus != nullptr) {
                for (std::size_t index = 0; index < allowed.size(); ++index) {
                    if (selection_refs_equal(*focus, allowed[index].ref)) {
                        anchor = index;
                        break;
                    }
                }
            }
            if (anchor == allowed.size()) {
                for (const auto& selected : selection.ordered()) {
                    for (std::size_t index = 0; index < allowed.size(); ++index) {
                        if (selection_refs_equal(selected, allowed[index].ref)) {
                            anchor = index;
                        }
                    }
                }
            }
        }

        if (pending_action == SelectionAction::Replace) {
            const std::size_t index = anchor == allowed.size()
                ? 0 : (anchor + 1) % allowed.size();
            selection.apply(SelectionAction::Replace, allowed[index].ref,
                SelectionChangeReason::Pick);
        } else {
            const std::size_t start = anchor == allowed.size()
                ? 0 : (anchor + 1) % allowed.size();
            for (std::size_t offset = 0; offset < allowed.size(); ++offset) {
                const std::size_t index = (start + offset) % allowed.size();
                if (selection.contains(allowed[index].ref)) {
                    continue;
                }
                selection.apply(SelectionAction::Add, allowed[index].ref,
                    SelectionChangeReason::Pick);
                break;
            }
        }
        cycle_valid_ = true;
        cycle_x_ = click_x_;
        cycle_y_ = click_y_;
        cycle_mask_revision_ = selection.mask_revision();
        cycle_view_ = camera.ubo_data.view;
        cycle_proj_ = camera.ubo_data.proj;
    }

    void pick_cpu(double cursor_x, double cursor_y) {
        if (window == nullptr) {
            if (pending_action == SelectionAction::Replace) {
                selection.clear(SelectionChangeReason::Pick);
            }
            cycle_valid_ = false;
            return;
        }
        std::vector<Hit> hits;
        collect_cpu_hits(cursor_x, cursor_y, hits);
        if (hits.empty()) {
            if (pending_action == SelectionAction::Replace) {
                selection.clear(SelectionChangeReason::Pick);
            }
            cycle_valid_ = false;
            return;
        }
        apply_hits(std::move(hits));
    }

    void collect_cpu_hits(double cursor_x, double cursor_y,
        std::vector<Hit>& hits) {
        if (window == nullptr) {
            return;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            return;
        }
        const float max_distance = kPickPixels * kPickPixels;
        const auto near_cursor = [&](const glm::vec3& world,
                                      float& best_distance) {
            glm::vec2 pixel{};
            if (!project(world, width, height, pixel)) {
                return false;
            }
            const float dx = pixel.x - static_cast<float>(cursor_x);
            const float dy = pixel.y - static_cast<float>(cursor_y);
            const float distance = dx * dx + dy * dy;
            if (distance >= max_distance || distance >= best_distance) {
                return false;
            }
            best_distance = distance;
            return true;
        };

        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Controller
                || components.controller(meta.id) == nullptr)
            {
                return;
            }
            const auto world = components.controller_world_xform(meta.id);
            const auto points = orlviewer::controller_shape_points(
                components.controller_shape(meta.id));
            float best_distance = max_distance;
            bool found = false;
            for (const auto& local : points) {
                found = near_cursor(
                    glm::vec3{world * glm::vec4{local, 1.0f}},
                    best_distance) || found;
            }
            if (found) {
                const auto ref = SelectionRef::controller(meta.id);
                hits.push_back(Hit{ref, depth(ref)});
            }
        });

        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Locator) {
                return;
            }
            const auto* locator = components.locator(meta.id);
            if (locator == nullptr) {
                return;
            }
            float best_distance = max_distance;
            if (near_cursor(glm::vec3{locator->xform[3]}, best_distance)) {
                const auto ref = SelectionRef::locator(meta.id);
                hits.push_back(Hit{ref, depth(ref)});
            }
        });

        const auto packed = components.packed_joints();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Joint) {
                return;
            }
            const auto index = components.joint_index(meta.id);
            if (index < 0 || static_cast<std::size_t>(index) >= packed.size()) {
                return;
            }
            const glm::vec3 world{
                orlviewer::joint_world_matrix(packed, index)[3]};
            float best_distance = max_distance;
            if (near_cursor(world, best_distance)) {
                const auto ref = SelectionRef::joint(meta.id);
                hits.push_back(Hit{ref, depth(ref)});
            }
        });

        // The CPU path cannot rasterize a mesh without duplicating the GPU
        // geometry path. Use object origins as a conservative degraded
        // provider so meshes remain selectable when GPU preparation fails.
        scene.for_each_object([&](const std::string& name,
                                   const vkkk::SceneObject& object) {
            float best_distance = max_distance;
            if (near_cursor(glm::vec3{object.model[3]}, best_distance)) {
                const auto ref = SelectionRef::scene_object(name);
                hits.push_back(Hit{ref, depth(ref)});
            }
        });
    }

    bool closest_controller(double cursor_x, double cursor_y, SelectionRef& chosen,
        float* best_dist = nullptr)
    {
        if (window == nullptr) {
            return false;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            return false;
        }
        float best = best_dist != nullptr ? *best_dist : kPickPixels * kPickPixels;
        bool found = false;
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Controller) {
                return;
            }
            if (components.controller(meta.id) == nullptr) {
                return;
            }
            const auto world = components.controller_world_xform(meta.id);
            const auto points = orlviewer::controller_shape_points(
                components.controller_shape(meta.id));
            for (const auto& local : points) {
                glm::vec2 pixel{};
                if (!project(glm::vec3{world * glm::vec4{local, 1.0f}},
                        width, height, pixel))
                {
                    continue;
                }
                const float dx = pixel.x - static_cast<float>(cursor_x);
                const float dy = pixel.y - static_cast<float>(cursor_y);
                const float dist = dx * dx + dy * dy;
                if (dist < best) {
                    best = dist;
                    chosen = SelectionRef::controller(meta.id);
                    found = true;
                }
            }
        });
        if (found && best_dist != nullptr) {
            *best_dist = best;
        }
        return found;
    }

    bool closest_locator(double cursor_x, double cursor_y,
        SelectionRef& chosen, float* best_dist = nullptr)
    {
        if (window == nullptr) {
            return false;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            return false;
        }
        float best = best_dist != nullptr ? *best_dist
            : kPickPixels * kPickPixels;
        bool found = false;
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Locator) {
                return;
            }
            const auto* locator = components.locator(meta.id);
            if (locator == nullptr) {
                return;
            }
            glm::vec2 pixel{};
            if (!project(glm::vec3{locator->xform[3]}, width, height, pixel)) {
                return;
            }
            const float dx = pixel.x - static_cast<float>(cursor_x);
            const float dy = pixel.y - static_cast<float>(cursor_y);
            const float dist = dx * dx + dy * dy;
            if (dist < best) {
                best = dist;
                chosen = SelectionRef::locator(meta.id);
                found = true;
            }
        });
        if (found && best_dist != nullptr) {
            *best_dist = best;
        }
        return found;
    }

    bool closest_joint(double cursor_x, double cursor_y,
        SelectionRef& chosen, float* best_dist = nullptr)
    {
        if (window == nullptr) {
            return false;
        }
        const auto size = window->window_size();
        const int width = static_cast<int>(size.width);
        const int height = static_cast<int>(size.height);
        if (width <= 0 || height <= 0) {
            return false;
        }

        float best = best_dist != nullptr ? *best_dist
            : kPickPixels * kPickPixels;
        const auto packed = components.packed_joints();
        components.for_each([&](const Component& meta) {
            if (meta.kind != ComponentKind::Joint) {
                return;
            }
            const auto index = components.joint_index(meta.id);
            if (index < 0 || static_cast<std::size_t>(index) >= packed.size()) {
                return;
            }
            const glm::vec3 world{orlviewer::joint_world_matrix(packed, index)[3]};
            glm::vec2 pixel{};
            if (!project(world, width, height, pixel)) {
                return;
            }
            const float dx = pixel.x - static_cast<float>(cursor_x);
            const float dy = pixel.y - static_cast<float>(cursor_y);
            const float dist = dx * dx + dy * dy;
            if (dist < best) {
                best = dist;
                chosen = SelectionRef::joint(meta.id);
            }
        });
        if (best_dist != nullptr && chosen) {
            *best_dist = best;
        }
        return static_cast<bool>(chosen);
    }

    float depth(const SelectionRef& ref) const {
        glm::vec3 world{};
        switch (ref.kind) {
        case SelectionRef::Kind::Joint: {
            const auto packed = components.packed_joints();
            const auto index = components.joint_index(ref.component);
            if (index < 0 || static_cast<std::size_t>(index) >= packed.size()) {
                return std::numeric_limits<float>::infinity();
            }
            world = glm::vec3{
                orlviewer::joint_world_matrix(packed, index)[3]};
            break;
        }
        case SelectionRef::Kind::SceneObject: {
            const auto* object = scene.find_object(ref.object_name);
            if (object == nullptr) {
                return std::numeric_limits<float>::infinity();
            }
            world = glm::vec3{object->model[3]};
            break;
        }
        case SelectionRef::Kind::Controller:
            if (components.controller(ref.component) == nullptr) {
                return std::numeric_limits<float>::infinity();
            }
            world = glm::vec3{
                components.controller_world_xform(ref.component)[3]};
            break;
        case SelectionRef::Kind::Locator: {
            const auto* locator = components.locator(ref.component);
            if (locator == nullptr) {
                return std::numeric_limits<float>::infinity();
            }
            world = glm::vec3{locator->xform[3]};
            break;
        }
        case SelectionRef::Kind::Vector:
            if (ref.vector != nullptr) {
                world = *ref.vector;
            }
            break;
        case SelectionRef::Kind::None:
            return std::numeric_limits<float>::infinity();
        }
        return glm::dot(world - camera.pos, camera.front);
    }

    bool project(const glm::vec3& world, int width, int height, glm::vec2& pixel) const {
        const glm::vec4 clip = camera.ubo_data.proj * camera.ubo_data.view
            * glm::vec4{world, 1.0f};
        if (std::abs(clip.w) < 1.0e-8f) {
            return false;
        }
        const glm::vec3 ndc = glm::vec3{clip} / clip.w;
        pixel.x = (ndc.x * 0.5f + 0.5f) * static_cast<float>(width);
        pixel.y = (ndc.y * 0.5f + 0.5f) * static_cast<float>(height);
        return true;
    }

    Selection& selection;
    ComponentManager& components;
    vkkk::Scene& scene;
    const vkkk::Camera& camera;
    vkkk::WindowBackend* window = nullptr;
    const CreateJointOp& create_joint;
    JointPickingFeature* gpu_picking = nullptr;
    MeshPickingFeature* mesh_picking = nullptr;
    RigPickingFeature* rig_picking = nullptr;
    double fallback_x = 0.0;
    double fallback_y = 0.0;
    SelectionAction pending_action = SelectionAction::Replace;
    std::vector<Hit> pending_hits;
    SelectionMask pending_mask = kAllSelectionKinds;
    std::uint64_t pending_mask_revision = 0;
    std::uint64_t transaction_id = 0;
    std::uint64_t expected_joint_request = 0;
    std::uint64_t expected_mesh_request = 0;
    std::uint64_t expected_rig_request = 0;
    bool pending_overflow = false;
    bool last_pick_overflow_ = false;
    glm::mat4 pending_view_{1.0f};
    glm::mat4 pending_proj_{1.0f};
    std::int64_t click_x_ = 0;
    std::int64_t click_y_ = 0;
    std::int64_t cycle_x_ = 0;
    std::int64_t cycle_y_ = 0;
    std::uint64_t cycle_mask_revision_ = 0;
    glm::mat4 cycle_view_{1.0f};
    glm::mat4 cycle_proj_{1.0f};
    bool cycle_valid_ = false;
    bool awaiting_joint = false;
    bool awaiting_mesh = false;
    bool awaiting_rig = false;
};

} // namespace ORL
