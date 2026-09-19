#include "component_manager.hpp"

#include <glm/gtc/matrix_inverse.hpp>

#include <utility>

namespace ORL
{

namespace
{

Component make_meta(ComponentId id, std::string name, ComponentKind kind) {
    return Component{id, std::move(name), kind, {}};
}

} // namespace

ComponentId ComponentManager::create_joint(std::string name, orlviewer::Joint joint) {
    const auto id = store.create_joint(name, std::move(joint));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Joint));
    }
    return id;
}

ComponentId ComponentManager::create_controller(std::string name,
    orlrig::Controller controller,
    orlviewer::ControllerShape shape)
{
    const auto id = store.create_controller(name, std::move(controller));
    if (id) {
        metadata.emplace(id.value, make_meta(id, name, ComponentKind::Controller));
        controller_shapes.emplace(id.value, shape);
    }
    return id;
}

ComponentId ComponentManager::create_locator(std::string name,
    orlrig::Locator locator)
{
    const auto id = store.create_locator(name, std::move(locator));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name),
            ComponentKind::Locator));
    }
    return id;
}

ComponentId ComponentManager::create_curve(std::string name, CurveLink curve) {
    const auto id = store.create_curve(name);
    if (id) {
        metadata.emplace(id.value, make_meta(id, name, ComponentKind::Curve));
        curves.emplace(id.value, std::move(curve));
    }
    return id;
}

ComponentId ComponentManager::create_weight(std::string name, WeightData weight) {
    const auto id = store.create_weight(name, std::move(weight));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Weight));
    }
    return id;
}

ComponentId ComponentManager::create_constraint(std::string name, ConstraintData constraint) {
    const auto id = store.create_constraint(name, std::move(constraint));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Constraint));
    }
    return id;
}

ComponentId ComponentManager::create_deformer(std::string name, DeformerData deformer) {
    const auto id = store.create_deformer(name, std::move(deformer));
    if (id) {
        metadata.emplace(id.value, make_meta(id, std::move(name), ComponentKind::Deformer));
    }
    return id;
}

bool ComponentManager::destroy(ComponentId id) {
    if (controller_attachments.contains(id.value)) {
        detach_controller(id);
    }
    if (const auto found = target_controllers.find(id.value);
        found != target_controllers.end())
    {
        detach_controller(found->second);
    }
    if (!store.destroy(id)) {
        return false;
    }
    metadata.erase(id.value);
    controller_shapes.erase(id.value);
    curves.erase(id.value);
    return true;
}

bool ComponentManager::destroy(std::string_view name) {
    const auto* comp = find(name);
    return comp != nullptr && destroy(comp->id);
}

bool ComponentManager::destroy_joint_recursive(ComponentId root) {
    const auto* meta = find(root);
    if (meta == nullptr || meta->kind != ComponentKind::Joint) {
        return false;
    }

    const auto ids = store.packed_joint_ids();
    const auto packed = store.packed_joints();
    const auto root_index = store.joint_index(root);
    if (root_index < 0
        || static_cast<std::size_t>(root_index) >= packed.size()
        || packed.size() != ids.size())
    {
        return false;
    }

    std::vector<bool> removed(packed.size(), false);
    removed[static_cast<std::size_t>(root_index)] = true;
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t index = 0; index < packed.size(); ++index) {
            if (removed[index]) {
                continue;
            }
            const auto parent = packed[index].parent;
            if (parent >= 0
                && static_cast<std::size_t>(parent) < removed.size()
                && removed[static_cast<std::size_t>(parent)])
            {
                removed[index] = true;
                changed = true;
            }
        }
    }

    std::vector<std::int64_t> remapped(packed.size(), -1);
    std::int64_t next_index = 0;
    for (std::size_t index = 0; index < packed.size(); ++index) {
        if (!removed[index]) {
            remapped[index] = next_index++;
        }
    }

    // Destroy by stable IDs, in reverse packed order so descendants are
    // removed before their parents without depending on shifting indices.
    for (std::size_t index = packed.size(); index-- > 0;) {
        if (removed[index] && !destroy(ids[index])) {
            return false;
        }
    }

    // Joint::parent stores a packed-array index, not a stable ComponentId.
    // Rebuild it after the deletion so surviving joints still address the
    // correct parent in the compacted buffer.
    for (std::size_t index = 0; index < packed.size(); ++index) {
        if (removed[index]) {
            continue;
        }
        auto* joint_value = joint(ids[index]);
        if (joint_value == nullptr || joint_value->parent < 0) {
            if (joint_value != nullptr) {
                joint_value->parent = -1;
            }
            continue;
        }
        const auto parent = joint_value->parent;
        if (static_cast<std::size_t>(parent) >= packed.size()
            || removed[static_cast<std::size_t>(parent)])
        {
            joint_value->parent = -1;
            continue;
        }
        joint_value->parent = remapped[static_cast<std::size_t>(parent)];
    }
    return true;
}

void ComponentManager::destroy_kind(ComponentKind kind) {
    std::vector<ComponentId> ids;
    for (const auto& [_, meta] : metadata) {
        if (meta.kind == kind) {
            ids.push_back(meta.id);
        }
    }
    for (const ComponentId id : ids) {
        destroy(id);
    }
}

bool ComponentManager::rename(ComponentId id, std::string new_name) {
    auto* meta = find(id);
    if (meta == nullptr || !store.rename(id, new_name)) {
        return false;
    }
    meta->name = std::move(new_name);
    return true;
}

bool ComponentManager::bind_display(ComponentId id, DisplayLink display) {
    auto* meta = find(id);
    if (meta == nullptr) {
        return false;
    }
    meta->display = std::move(display);
    return true;
}

bool ComponentManager::unbind_display(ComponentId id) {
    return bind_display(id, {});
}

const Component* ComponentManager::find(ComponentId id) const {
    const auto found = metadata.find(id.value);
    return found == metadata.end() ? nullptr : &found->second;
}

Component* ComponentManager::find(ComponentId id) {
    const auto found = metadata.find(id.value);
    return found == metadata.end() ? nullptr : &found->second;
}

const Component* ComponentManager::find(std::string_view name) const {
    for (const auto& [_, meta] : metadata) {
        if (meta.name == name) {
            return &meta;
        }
    }
    return nullptr;
}

Component* ComponentManager::find(std::string_view name) {
    for (auto& [_, meta] : metadata) {
        if (meta.name == name) {
            return &meta;
        }
    }
    return nullptr;
}

orlviewer::Joint* ComponentManager::joint(ComponentId id) {
    return store.joint(id);
}

const orlviewer::Joint* ComponentManager::joint(ComponentId id) const {
    return store.joint(id);
}

orlrig::Controller* ComponentManager::controller(ComponentId id) {
    return store.controller(id);
}

const orlrig::Controller* ComponentManager::controller(ComponentId id) const {
    return store.controller(id);
}

orlrig::Locator* ComponentManager::locator(ComponentId id) {
    return store.locator(id);
}

const orlrig::Locator* ComponentManager::locator(ComponentId id) const {
    return store.locator(id);
}

orlviewer::ControllerShape ComponentManager::controller_shape(ComponentId id) const {
    const auto found = controller_shapes.find(id.value);
    return found == controller_shapes.end()
        ? orlviewer::ControllerShape::Curve
        : found->second;
}

bool ComponentManager::set_controller_shape(
    ComponentId id, orlviewer::ControllerShape shape)
{
    if (store.controller(id) == nullptr) {
        return false;
    }
    const auto found = controller_shapes.find(id.value);
    if (found == controller_shapes.end()) {
        return false;
    }
    found->second = shape;
    return true;
}

CurveLink* ComponentManager::curve(ComponentId id) {
    const auto found = curves.find(id.value);
    return found == curves.end() ? nullptr : &found->second;
}

const CurveLink* ComponentManager::curve(ComponentId id) const {
    const auto found = curves.find(id.value);
    return found == curves.end() ? nullptr : &found->second;
}

WeightData* ComponentManager::weight(ComponentId id) {
    return store.weight(id);
}

const WeightData* ComponentManager::weight(ComponentId id) const {
    return store.weight(id);
}

ConstraintData* ComponentManager::constraint(ComponentId id) {
    return store.constraint(id);
}

const ConstraintData* ComponentManager::constraint(ComponentId id) const {
    return store.constraint(id);
}

DeformerData* ComponentManager::deformer(ComponentId id) {
    return store.deformer(id);
}

const DeformerData* ComponentManager::deformer(ComponentId id) const {
    return store.deformer(id);
}

std::size_t ComponentManager::size(ComponentKind kind) const {
    return store.size(kind);
}

bool ComponentManager::contains(std::string_view name) const {
    return store.contains(name);
}

std::vector<orlviewer::Joint> ComponentManager::packed_joints() const {
    return store.packed_joints();
}

std::vector<ComponentId> ComponentManager::packed_joint_ids() const {
    return store.packed_joint_ids();
}

std::int64_t ComponentManager::joint_index(ComponentId id) const {
    return store.joint_index(id);
}

std::vector<orlrig::Locator> ComponentManager::packed_locators() const {
    return store.packed_locators();
}

std::vector<ComponentId> ComponentManager::packed_locator_ids() const {
    return store.packed_locator_ids();
}

std::int64_t ComponentManager::locator_index(ComponentId id) const {
    return store.locator_index(id);
}

bool ComponentManager::attach_controller(ComponentId controller,
    ComponentId target, std::string* error)
{
    const auto* controller_meta = find(controller);
    if (controller_meta == nullptr
        || controller_meta->kind != ComponentKind::Controller)
    {
        return set_error(error, "Attachment source is not a controller");
    }
    const auto* target_meta = find(target);
    if (target_meta == nullptr
        || (target_meta->kind != ComponentKind::Joint
            && target_meta->kind != ComponentKind::Locator))
    {
        return set_error(error, "Attachment target must be a joint or locator");
    }
    if (const auto found = target_controllers.find(target.value);
        found != target_controllers.end() && found->second != controller)
    {
        return set_error(error, "Attachment target already has a controller");
    }

    const glm::mat4 controller_world = controller_world_xform(controller);
    if (controller_attachments.contains(controller.value)) {
        detach_controller(controller);
    }
    glm::mat4 target_world{1.0f};
    if (!target_world_xform(target, target_world)) {
        return set_error(error, "Attachment target transform is unavailable");
    }

    ControllerAttachment attachment;
    attachment.target_kind = target_meta->kind == ComponentKind::Joint
        ? AttachmentTargetKind::Joint
        : AttachmentTargetKind::Locator;
    attachment.target = target;
    attachment.xform = glm::inverse(target_world) * controller_world;
    controller_attachments[controller.value] = attachment;
    target_controllers[target.value] = controller;
    if (auto* value = this->controller(controller)) {
        // Keep the current world placement as the setup transform. The
        // target-local attachment offset must not absorb the control's
        // display scale as animation input.
        value->xform = controller_world;
        value->input_xform = glm::mat4{1.0f};
    }
    return true;
}

bool ComponentManager::detach_controller(ComponentId controller) {
    const auto found = controller_attachments.find(controller.value);
    if (found == controller_attachments.end()) {
        return false;
    }
    if (auto* value = this->controller(controller)) {
        value->xform = controller_world_xform(controller);
        value->input_xform = glm::mat4{1.0f};
    }
    const auto target = found->second.target;
    if (const auto target_controller = target_controllers.find(target.value);
        target_controller != target_controllers.end()
        && target_controller->second == controller)
    {
        target_controllers.erase(target_controller);
    }
    controller_attachments.erase(found);
    return true;
}

ControllerAttachment* ComponentManager::controller_attachment(
    ComponentId controller)
{
    const auto found = controller_attachments.find(controller.value);
    return found == controller_attachments.end() ? nullptr : &found->second;
}

const ControllerAttachment* ComponentManager::controller_attachment(
    ComponentId controller) const
{
    const auto found = controller_attachments.find(controller.value);
    return found == controller_attachments.end() ? nullptr : &found->second;
}

ComponentId ComponentManager::attached_controller(ComponentId target) const {
    const auto found = target_controllers.find(target.value);
    return found == target_controllers.end() ? ComponentId{} : found->second;
}

bool ComponentManager::validate_controller_attachments(
    std::string* error) const
{
    for (const auto& [controller_id, attachment] : controller_attachments) {
        const ComponentId controller{controller_id};
        const auto* controller_meta = find(controller);
        const auto* target_meta = find(attachment.target);
        if (controller_meta == nullptr
            || controller_meta->kind != ComponentKind::Controller)
        {
            return set_error(error, "Attachment source is not a live controller");
        }
        if (target_meta == nullptr
            || (target_meta->kind != ComponentKind::Joint
                && target_meta->kind != ComponentKind::Locator))
        {
            return set_error(error, "Attachment target is not a live joint or locator");
        }
        const auto expected_kind = target_meta->kind == ComponentKind::Joint
            ? AttachmentTargetKind::Joint
            : AttachmentTargetKind::Locator;
        if (attachment.target_kind != expected_kind) {
            return set_error(error, "Attachment target kind is inconsistent");
        }
        const auto reverse = target_controllers.find(
            attachment.target.value);
        if (reverse == target_controllers.end()
            || reverse->second != controller)
        {
            return set_error(error, "Attachment reverse index is inconsistent");
        }
        if (attachment.target == controller) {
            return set_error(error, "Controller attachment cycle detected");
        }
    }
    for (const auto& [target_id, controller] : target_controllers) {
        const auto found = controller_attachments.find(controller.value);
        if (found == controller_attachments.end()
            || found->second.target.value != target_id)
        {
            return set_error(error, "Attachment target index is inconsistent");
        }
    }
    return true;
}

glm::mat4 ComponentManager::controller_world_xform(ComponentId controller) const {
    const auto* value = this->controller(controller);
    if (value == nullptr) {
        return glm::mat4{1.0f};
    }
    return value->xform * value->input_xform;
}

bool ComponentManager::set_controller_world_xform(ComponentId controller,
    const glm::mat4& world, std::string* error)
{
    auto* value = this->controller(controller);
    if (value == nullptr) {
        return set_error(error, "Transform target is not a controller");
    }
    auto* attachment = controller_attachment(controller);
    if (attachment == nullptr) {
        value->input_xform = glm::inverse(value->xform) * world;
        return true;
    }
    // xform is the stable rig setup. Keep input_xform cumulative relative
    // to that setup instead of replacing it with the per-frame drag delta.
    const glm::mat4 input = glm::inverse(value->xform) * world;
    value->input_xform = input;
    attachment->input_drives_target = true;
    return true;
}

bool ComponentManager::set_controller_setup_world_xform(
    ComponentId controller, const glm::mat4& world, std::string* error)
{
    auto* value = this->controller(controller);
    if (value == nullptr) {
        return set_error(error, "Transform target is not a controller");
    }
    auto* attachment = controller_attachment(controller);
    if (attachment == nullptr) {
        value->xform = world;
        value->input_xform = glm::mat4{1.0f};
        return true;
    }
    const glm::mat4 target_setup_world =
        value->xform * glm::inverse(attachment->xform);
    attachment->xform = glm::inverse(target_setup_world) * world;
    value->xform = world;
    value->input_xform = glm::mat4{1.0f};
    attachment->input_drives_target = false;
    return true;
}

bool ComponentManager::apply_controller_inputs(std::string* error) {
    for (const auto& [controller_id, attachment] : controller_attachments) {
        if (!attachment.input_drives_target) {
            continue;
        }
        const auto* controller = this->controller(ComponentId{controller_id});
        if (controller == nullptr) {
            return set_error(error,
                "Controller input source is no longer available");
        }
        const glm::mat4 controller_world =
            controller->xform * controller->input_xform;
        const glm::mat4 target_world =
            controller_world * glm::inverse(attachment.xform);
        if (!set_target_world_xform(
                attachment.target, target_world, error))
        {
            return false;
        }
    }
    return true;
}

bool ComponentManager::set_locator_world_xform(ComponentId locator,
    const glm::mat4& world)
{
    auto* value = this->locator(locator);
    if (value == nullptr) {
        return false;
    }
    value->xform = world;
    return true;
}

bool ComponentManager::target_world_xform(ComponentId target,
    glm::mat4& world) const
{
    const auto* meta = find(target);
    if (meta == nullptr) {
        return false;
    }
    if (meta->kind == ComponentKind::Locator) {
        const auto* value = locator(target);
        if (value == nullptr) {
            return false;
        }
        world = value->xform;
        return true;
    }
    if (meta->kind != ComponentKind::Joint) {
        return false;
    }
    const auto index = store.joint_index(target);
    const auto packed = store.packed_joints();
    if (index < 0 || static_cast<std::size_t>(index) >= packed.size()) {
        return false;
    }
    world = orlrig::joint_world_matrix(packed, index);
    return true;
}

bool ComponentManager::set_target_world_xform(ComponentId target,
    const glm::mat4& world, std::string* error)
{
    const auto* meta = find(target);
    if (meta == nullptr) {
        return set_error(error, "Transform target no longer exists");
    }
    if (meta->kind == ComponentKind::Locator) {
        return set_locator_world_xform(target, world)
            || set_error(error, "Locator transform is unavailable");
    }
    if (meta->kind != ComponentKind::Joint) {
        return set_error(error, "Transform target is not a joint or locator");
    }
    auto* joint = this->joint(target);
    const auto index = store.joint_index(target);
    const auto packed = store.packed_joints();
    if (joint == nullptr || index < 0
        || static_cast<std::size_t>(index) >= packed.size())
    {
        return set_error(error, "Joint transform is unavailable");
    }
    if (!orlrig::write_joint_world_matrix(packed, index, world, *joint)) {
        return set_error(error, "Joint world transform cannot be decomposed");
    }
    return true;
}

bool ComponentManager::set_error(std::string* error, std::string message) const {
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

} // namespace ORL
