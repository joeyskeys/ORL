#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include <glm/mat4x4.hpp>

#include "../orlexec/orlrig/component_store.hpp"
#include "comps/controller.hpp"
#include "comps/joint.hpp"
#include "comps/weight.hpp"
#include "comps/deformer.hpp"
#include "concepts/curve.hpp"

namespace ORL
{

using ComponentId = orlrig::ComponentId;
using ComponentKind = orlrig::ComponentKind;
using ConstraintData = orlrig::ConstraintData;

// Named handle into vkkk draw storage. ComponentManager never owns GPU
// resources; it only records which draw object (if any) represents this
// CPU component.
//
//   SceneObject -> vkkk::Scene objects
//   Mesh        -> DrawableMgr / Context meshes
//   Lines       -> DrawableMgr / Context lines
//   Points      -> Context points
enum class DisplayResourceKind {
    None,
    SceneObject,
    Mesh,
    Lines,
    Points,
};

struct DisplayLink {
    DisplayResourceKind kind = DisplayResourceKind::None;
    std::string name;

    bool bound() const {
        return kind != DisplayResourceKind::None && !name.empty();
    }
};

// Non-owning pointer to a vkkk curve. Control points and evaluation stay in
// vkkk; the viewport component only keeps this link. Tessellated draw data
// (if any) is a separate DisplayLink, typically DisplayResourceKind::Lines.
struct CurveLink {
    using Handle = std::variant<
        std::monostate,
        const vkkk::BezierCurve*,
        const vkkk::BSplineCurve*,
        const vkkk::NurbsCurve*,
        const vkkk::CatmullRomCurve*>;

    Handle handle;

    CurveLink() = default;
    CurveLink(const vkkk::BezierCurve& curve) : handle(&curve) {}
    CurveLink(const vkkk::BSplineCurve& curve) : handle(&curve) {}
    CurveLink(const vkkk::NurbsCurve& curve) : handle(&curve) {}
    CurveLink(const vkkk::CatmullRomCurve& curve) : handle(&curve) {}

    bool empty() const { return std::holds_alternative<std::monostate>(handle); }
};

struct Component {
    ComponentId id;
    std::string name;
    ComponentKind kind = ComponentKind::Joint;
    DisplayLink display;
};

enum class AttachmentTargetKind {
    None,
    Joint,
    Locator,
};

// Viewer-only controller relationship. The xform is the target-local setup
// offset; controller animation input is stored on Controller::input_xform.
// Controllers are resolved to world transforms for interaction, display, and
// scene-input packing.
struct ControllerAttachment {
    AttachmentTargetKind target_kind = AttachmentTargetKind::None;
    ComponentId target;
    glm::mat4 xform{1.0f};
    bool input_drives_target = false;
};

// Viewer adapter around the standalone rigging store. Runtime rigging data
// lives in orlrig::ComponentStore; display links, curve handles, controller
// shapes, and authoring-only controller attachments stay here.
class ComponentManager {
public:
    ComponentId create_joint(std::string name, orlviewer::Joint joint = orlviewer::make_identity_joint());
    ComponentId create_controller(std::string name,
        orlrig::Controller controller = orlrig::Controller{},
        orlviewer::ControllerShape shape = orlviewer::ControllerShape::Curve);
    ComponentId create_locator(std::string name,
        orlrig::Locator locator = orlrig::Locator{});
    ComponentId create_curve(std::string name, CurveLink curve = {});
    ComponentId create_weight(std::string name, WeightData weight = {});
    ComponentId create_constraint(std::string name, ConstraintData constraint = {});
    ComponentId create_deformer(std::string name, DeformerData deformer = {});

    bool destroy(ComponentId id);
    bool destroy(std::string_view name);
    void destroy_kind(ComponentKind kind);
    bool rename(ComponentId id, std::string new_name);

    bool bind_display(ComponentId id, DisplayLink display);
    bool unbind_display(ComponentId id);

    const Component* find(ComponentId id) const;
    Component* find(ComponentId id);
    const Component* find(std::string_view name) const;
    Component* find(std::string_view name);

    orlviewer::Joint* joint(ComponentId id);
    const orlviewer::Joint* joint(ComponentId id) const;
    orlrig::Controller* controller(ComponentId id);
    const orlrig::Controller* controller(ComponentId id) const;
    orlrig::Locator* locator(ComponentId id);
    const orlrig::Locator* locator(ComponentId id) const;
    orlviewer::ControllerShape controller_shape(ComponentId id) const;
    bool set_controller_shape(ComponentId id, orlviewer::ControllerShape shape);
    CurveLink* curve(ComponentId id);
    const CurveLink* curve(ComponentId id) const;
    WeightData* weight(ComponentId id);
    const WeightData* weight(ComponentId id) const;
    ConstraintData* constraint(ComponentId id);
    const ConstraintData* constraint(ComponentId id) const;
    DeformerData* deformer(ComponentId id);
    const DeformerData* deformer(ComponentId id) const;

    std::size_t size() const { return store.size(); }
    std::size_t size(ComponentKind kind) const;
    bool contains(std::string_view name) const;

    // Joints in creation order, suitable for packing into Joint joints[].
    std::vector<orlviewer::Joint> packed_joints() const;
    std::vector<ComponentId> packed_joint_ids() const;
    std::int64_t joint_index(ComponentId id) const;
    std::vector<orlrig::Locator> packed_locators() const;
    std::vector<ComponentId> packed_locator_ids() const;
    std::int64_t locator_index(ComponentId id) const;

    bool attach_controller(ComponentId controller, ComponentId target,
        std::string* error = nullptr);
    bool detach_controller(ComponentId controller);
    ControllerAttachment* controller_attachment(ComponentId controller);
    const ControllerAttachment* controller_attachment(
        ComponentId controller) const;
    ComponentId attached_controller(ComponentId target) const;
    bool validate_controller_attachments(std::string* error = nullptr) const;
    // Effective transform used by display and runtime scene inputs.
    glm::mat4 controller_world_xform(ComponentId controller) const;
    // Animation mode: record input. Runtime evaluation applies it to an
    // attached target before solver/deformer kernels run.
    bool set_controller_world_xform(ComponentId controller,
        const glm::mat4& world, std::string* error = nullptr);
    // Rigging mode: update setup placement without changing the target.
    bool set_controller_setup_world_xform(ComponentId controller,
        const glm::mat4& world, std::string* error = nullptr);
    bool apply_controller_inputs(std::string* error = nullptr);
    bool set_locator_world_xform(ComponentId locator,
        const glm::mat4& world);

    orlrig::ComponentStore& rigging() { return store; }
    const orlrig::ComponentStore& rigging() const { return store; }

    template <typename Fn>
    void for_each(Fn&& fn) const {
        for (const auto& [_, meta] : metadata) {
            fn(meta);
        }
    }

private:
    bool target_world_xform(ComponentId target, glm::mat4& world) const;
    bool set_target_world_xform(ComponentId target,
        const glm::mat4& world, std::string* error);
    bool set_error(std::string* error, std::string message) const;

    orlrig::ComponentStore store;
    std::unordered_map<std::uint64_t, Component> metadata;
    std::unordered_map<std::uint64_t, orlviewer::ControllerShape> controller_shapes;
    std::unordered_map<std::uint64_t, CurveLink> curves;
    std::unordered_map<std::uint64_t, ControllerAttachment>
        controller_attachments;
    std::unordered_map<std::uint64_t, ComponentId> target_controllers;
};

} // namespace ORL
