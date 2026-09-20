#include "project_serialization.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "comps/controller_curves.hpp"
#include "orlgraph/graph_serialization.hpp"
#include "orlrig/abi.hpp"
#include "orlrig/weight.hpp"

namespace ORL
{
namespace
{

using Json = rapidjson::Value;
using Allocator = rapidjson::Document::AllocatorType;

constexpr const char* kProjectMagic = "ORL_PROJECT";
constexpr std::uint32_t kProjectFormatVersion = 1;

struct SavedJoint {
    std::uint64_t id = 0;
    std::string name;
    orlviewer::Joint value = orlviewer::make_identity_joint();
};

struct SavedLocator {
    std::uint64_t id = 0;
    std::string name;
    orlrig::Locator value;
};

struct SavedController {
    std::uint64_t id = 0;
    std::string name;
    orlrig::Controller value;
    orlviewer::ControllerShape shape = orlviewer::ControllerShape::Curve;
};

struct SavedConstraint {
    std::uint64_t id = 0;
    std::string name;
    ConstraintData value;
};

struct SavedAttachment {
    std::uint64_t controller = 0;
    std::uint64_t target = 0;
    AttachmentTargetKind target_kind = AttachmentTargetKind::None;
    glm::mat4 xform{1.0f};
    bool input_drives_target = false;
};

struct SavedWeights {
    bool present = false;
    std::int64_t weight_count =
        static_cast<std::int64_t>(orlrig::kDefaultWeightCount);
    std::vector<orlrig::Weight> values;
};

struct SavedDeformer {
    bool present = false;
    std::string type = "lbs";
    std::string mesh_name;
    glm::mat4 bind_model{1.0f};
    bool bound = false;
    std::vector<std::array<double, 4>> bind_positions;
    std::vector<std::array<double, 16>> inverse_binds;
};

struct SavedComponents {
    std::vector<SavedJoint> joints;
    std::vector<SavedLocator> locators;
    std::vector<SavedController> controllers;
    std::vector<SavedConstraint> constraints;
    std::vector<SavedAttachment> attachments;
    SavedWeights weights;
    SavedDeformer deformer;
};

ProjectIoResult failure(std::string message) {
    ProjectIoResult result;
    result.errors.push_back(std::move(message));
    return result;
}

void add(Json& object, const char* key, Json value, Allocator& allocator) {
    object.AddMember(
        Json{key, allocator}, std::move(value), allocator);
}

Json string_value(std::string_view value, Allocator& allocator) {
    return Json{
        value.data(), static_cast<rapidjson::SizeType>(value.size()),
        allocator};
}

void add_matrix(Json& object, const char* key, const glm::mat4& matrix,
    Allocator& allocator)
{
    Json values(rapidjson::kArrayType);
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            values.PushBack(static_cast<double>(matrix[column][row]),
                allocator);
        }
    }
    add(object, key, std::move(values), allocator);
}

void add_double_array(Json& object, const char* key,
    const double* values, std::size_t count, Allocator& allocator)
{
    Json result(rapidjson::kArrayType);
    for (std::size_t index = 0; index < count; ++index) {
        result.PushBack(values[index], allocator);
    }
    add(object, key, std::move(result), allocator);
}

void add_joint(Json& object, const char* key, const orlviewer::Joint& joint,
    Allocator& allocator)
{
    Json value(rapidjson::kObjectType);
    add(value, "parent", Json{joint.parent}, allocator);
    add(value, "selected", Json{joint.selected}, allocator);
    add_double_array(value, "translation", joint.translation, 3, allocator);
    add_double_array(value, "rotation", joint.rotation, 4, allocator);
    add_double_array(value, "scale", joint.scale, 3, allocator);
    add(object, key, std::move(value), allocator);
}

void add_locator(Json& object, const char* key, const orlrig::Locator& locator,
    Allocator& allocator)
{
    add_matrix(object, key, locator.xform, allocator);
}

void add_controller(Json& object, const char* key,
    const orlrig::Controller& controller, Allocator& allocator)
{
    Json value(rapidjson::kObjectType);
    add_matrix(value, "xform", controller.xform, allocator);
    add_matrix(value, "input_xform", controller.input_xform, allocator);
    add(object, key, std::move(value), allocator);
}

void add_component_header(Json& object, std::uint64_t id,
    std::string_view name, Allocator& allocator)
{
    add(object, "id", Json{id}, allocator);
    add(object, "name", string_value(name, allocator), allocator);
}

void add_weight_buffer(Json& object, const exec::OrlBuffer& buffer,
    Allocator& allocator)
{
    Json values(rapidjson::kArrayType);
    const auto* source =
        static_cast<const orlrig::Weight*>(buffer.data());
    for (std::size_t index = 0; index < buffer.count(); ++index) {
        Json value(rapidjson::kObjectType);
        add(value, "weight", Json{source[index].weight}, allocator);
        add(value, "joint", Json{source[index].joint}, allocator);
        values.PushBack(std::move(value), allocator);
    }
    add(object, "values", std::move(values), allocator);
}

void add_deformer_buffer(Json& object, const exec::OrlBuffer& buffer,
    const char* key, std::size_t value_count, std::size_t value_stride,
    Allocator& allocator)
{
    Json values(rapidjson::kArrayType);
    const auto* source = static_cast<const double*>(buffer.data());
    for (std::size_t index = 0; index < value_count; ++index) {
        Json value(rapidjson::kArrayType);
        for (std::size_t component = 0; component < value_stride;
             ++component)
        {
            value.PushBack(source[index * value_stride + component],
                allocator);
        }
        values.PushBack(std::move(value), allocator);
    }
    add(object, key, std::move(values), allocator);
}

std::string write_json(const Json& value) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    value.Accept(writer);
    return buffer.GetString();
}

std::string write_pretty_json(const Json& value) {
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    writer.SetIndent(' ', 2);
    value.Accept(writer);
    return buffer.GetString();
}

template <typename T>
const Json* member(const Json& object, const char* name,
    std::vector<std::string>& errors, bool required = true)
{
    if (!object.IsObject() || !object.HasMember(name)) {
        if (required) {
            errors.emplace_back(
                "Project JSON is missing '" + std::string{name} + "'");
        }
        return nullptr;
    }
    return &object[name];
}

bool read_string(const Json& object, const char* name, std::string& result,
    std::vector<std::string>& errors, bool required = true)
{
    const auto* value = member<void>(object, name, errors, required);
    if (value == nullptr) {
        return !required;
    }
    if (!value->IsString()) {
        errors.emplace_back(
            "Project field '" + std::string{name} + "' must be a string");
        return false;
    }
    result = value->GetString();
    return true;
}

bool read_bool(const Json& object, const char* name, bool& result,
    std::vector<std::string>& errors, bool required = true)
{
    const auto* value = member<void>(object, name, errors, required);
    if (value == nullptr) {
        return !required;
    }
    if (!value->IsBool()) {
        errors.emplace_back(
            "Project field '" + std::string{name} + "' must be a boolean");
        return false;
    }
    result = value->GetBool();
    return true;
}

bool read_u64(const Json& object, const char* name, std::uint64_t& result,
    std::vector<std::string>& errors)
{
    const auto* value = member<void>(object, name, errors);
    if (value == nullptr) {
        return false;
    }
    if (!value->IsUint64()) {
        errors.emplace_back(
            "Project field '" + std::string{name} + "' must be an unsigned integer");
        return false;
    }
    result = value->GetUint64();
    return true;
}

bool read_i64(const Json& object, const char* name, std::int64_t& result,
    std::vector<std::string>& errors, bool required = true)
{
    const auto* value = member<void>(object, name, errors, required);
    if (value == nullptr) {
        return !required;
    }
    if (!value->IsInt64()) {
        errors.emplace_back(
            "Project field '" + std::string{name} + "' must be an integer");
        return false;
    }
    result = value->GetInt64();
    return true;
}

bool read_double(const Json& object, const char* name, double& result,
    std::vector<std::string>& errors)
{
    const auto* value = member<void>(object, name, errors);
    if (value == nullptr) {
        return false;
    }
    if (!value->IsNumber()) {
        errors.emplace_back(
            "Project field '" + std::string{name} + "' must be numeric");
        return false;
    }
    result = value->GetDouble();
    return true;
}

bool read_array(const Json& object, const char* name, std::size_t count,
    std::vector<double>& result, std::vector<std::string>& errors)
{
    const auto* value = member<void>(object, name, errors);
    if (value == nullptr) {
        return false;
    }
    if (!value->IsArray() || value->Size() != count) {
        errors.emplace_back(
            "Project field '" + std::string{name}
            + "' must be an array with " + std::to_string(count)
            + " values");
        return false;
    }
    result.clear();
    result.reserve(count);
    bool valid = true;
    for (const auto& entry : value->GetArray()) {
        if (!entry.IsNumber()) {
            valid = false;
            break;
        }
        result.push_back(entry.GetDouble());
    }
    if (!valid) {
        errors.emplace_back(
            "Project field '" + std::string{name}
            + "' contains a non-numeric value");
    }
    return valid;
}

bool read_matrix(const Json& object, const char* name, glm::mat4& result,
    std::vector<std::string>& errors)
{
    std::vector<double> values;
    if (!read_array(object, name, 16, values, errors)) {
        return false;
    }
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            result[column][row] = static_cast<float>(
                values[static_cast<std::size_t>(column * 4 + row)]);
        }
    }
    return true;
}

bool read_joint(const Json& object, orlviewer::Joint& result,
    std::vector<std::string>& errors)
{
    bool valid = true;
    valid = read_i64(object, "parent", result.parent, errors) && valid;
    read_i64(object, "selected", result.selected, errors, false);
    std::vector<double> values;
    if (read_array(object, "translation", 3, values, errors)) {
        std::copy(values.begin(), values.end(), result.translation);
    } else {
        valid = false;
    }
    if (read_array(object, "rotation", 4, values, errors)) {
        std::copy(values.begin(), values.end(), result.rotation);
    } else {
        valid = false;
    }
    if (read_array(object, "scale", 3, values, errors)) {
        std::copy(values.begin(), values.end(), result.scale);
    } else {
        valid = false;
    }
    return valid;
}

bool read_id_name(const Json& object, std::uint64_t& id,
    std::string& name, std::vector<std::string>& errors)
{
    return read_u64(object, "id", id, errors)
        && read_string(object, "name", name, errors);
}

bool read_locator(const Json& object, SavedLocator& result,
    std::vector<std::string>& errors)
{
    return read_id_name(object, result.id, result.name, errors)
        && read_matrix(object, "xform", result.value.xform, errors);
}

bool read_controller(const Json& object, SavedController& result,
    std::vector<std::string>& errors)
{
    bool valid = read_id_name(object, result.id, result.name, errors);
    const auto* value = member<void>(object, "value", errors);
    if (value == nullptr || !value->IsObject()) {
        errors.emplace_back("Project controller.value must be an object");
        return false;
    }
    valid = read_matrix(*value, "xform", result.value.xform, errors)
        && valid;
    valid = read_matrix(*value, "input_xform",
        result.value.input_xform, errors) && valid;
    std::int64_t shape = 0;
    valid = read_i64(object, "shape", shape, errors) && valid;
    if (shape < 0
        || shape > static_cast<std::int64_t>(
            orlviewer::ControllerShape::Polygon))
    {
        errors.emplace_back("Controller shape is out of range");
        valid = false;
    } else {
        result.shape = static_cast<orlviewer::ControllerShape>(shape);
    }
    return valid;
}

bool read_constraint(const Json& object, SavedConstraint& result,
    std::vector<std::string>& errors)
{
    bool valid = read_id_name(object, result.id, result.name, errors);
    valid = read_string(object, "type", result.value.type, errors)
        && valid;
    std::uint64_t id = 0;
    valid = read_u64(object, "root", id, errors) && valid;
    result.value.root = ComponentId{id};
    valid = read_u64(object, "mid", id, errors) && valid;
    result.value.mid = ComponentId{id};
    valid = read_u64(object, "end", id, errors) && valid;
    result.value.end = ComponentId{id};
    valid = read_u64(object, "target", id, errors) && valid;
    result.value.target = ComponentId{id};
    valid = read_u64(object, "pole", id, errors) && valid;
    result.value.pole = ComponentId{id};
    valid = read_bool(object, "bound", result.value.bound, errors)
        && valid;
    return valid;
}

bool read_weight_data(const Json& object, SavedWeights& result,
    std::vector<std::string>& errors)
{
    result.present = true;
    bool valid = read_i64(object, "weight_count", result.weight_count,
        errors);
    const auto* values = member<void>(object, "values", errors);
    if (values == nullptr || !values->IsArray()) {
        errors.emplace_back("Project weights.values must be an array");
        return false;
    }
    for (const auto& entry : values->GetArray()) {
        if (!entry.IsObject()) {
            errors.emplace_back("Project weight entries must be objects");
            valid = false;
            continue;
        }
        orlrig::Weight value{};
        valid = read_double(entry, "weight", value.weight, errors)
            && valid;
        valid = read_i64(entry, "joint", value.joint, errors) && valid;
        result.values.push_back(value);
    }
    return valid;
}

bool read_deformer_data(const Json& object, SavedDeformer& result,
    std::vector<std::string>& errors)
{
    result.present = true;
    bool valid = read_string(object, "type", result.type, errors);
    valid = read_string(object, "mesh_name", result.mesh_name, errors)
        && valid;
    valid = read_matrix(object, "bind_model", result.bind_model, errors)
        && valid;
    valid = read_bool(object, "bound", result.bound, errors) && valid;

    const auto* positions = member<void>(object, "bind_positions", errors);
    if (positions == nullptr || !positions->IsArray()) {
        errors.emplace_back(
            "Project deformer.bind_positions must be an array");
        valid = false;
    } else {
        for (const auto& entry : positions->GetArray()) {
            std::vector<double> values;
            if (!entry.IsArray() || entry.Size() != 4) {
                errors.emplace_back(
                    "Project bind position must contain four values");
                valid = false;
                continue;
            }
            for (const auto& component : entry.GetArray()) {
                if (!component.IsNumber()) {
                    valid = false;
                    break;
                }
                values.push_back(component.GetDouble());
            }
            if (values.size() == 4) {
                result.bind_positions.push_back(
                    {values[0], values[1], values[2], values[3]});
            }
        }
    }

    const auto* inverse_binds =
        member<void>(object, "inverse_binds", errors);
    if (inverse_binds == nullptr || !inverse_binds->IsArray()) {
        errors.emplace_back(
            "Project deformer.inverse_binds must be an array");
        valid = false;
    } else {
        for (const auto& entry : inverse_binds->GetArray()) {
            std::vector<double> values;
            if (!entry.IsArray() || entry.Size() != 16) {
                errors.emplace_back(
                    "Project inverse bind must contain sixteen values");
                valid = false;
                continue;
            }
            for (const auto& component : entry.GetArray()) {
                if (!component.IsNumber()) {
                    valid = false;
                    break;
                }
                values.push_back(component.GetDouble());
            }
            if (values.size() == 16) {
                std::array<double, 16> matrix{};
                std::copy(values.begin(), values.end(), matrix.begin());
                result.inverse_binds.push_back(matrix);
            }
        }
    }
    return valid;
}

bool read_components(const Json& object, SavedComponents& result,
    std::vector<std::string>& errors)
{
    if (!object.IsObject()) {
        errors.emplace_back("Project components must be an object");
        return false;
    }
    bool valid = true;
    const auto read_array_member =
        [&object, &errors](const char* name) -> const Json* {
        const auto* value = member<void>(object, name, errors);
        if (value == nullptr || !value->IsArray()) {
            errors.emplace_back(
                "Project components." + std::string{name}
                + " must be an array");
            return nullptr;
        }
        return value;
    };

    if (const auto* joints = read_array_member("joints")) {
        for (const auto& entry : joints->GetArray()) {
            if (!entry.IsObject()) {
                errors.emplace_back("Project joint entries must be objects");
                valid = false;
                continue;
            }
            SavedJoint value;
            valid = read_id_name(entry, value.id, value.name, errors)
                && valid;
            const auto* joint = member<void>(entry, "value", errors);
            valid = joint != nullptr && joint->IsObject()
                && read_joint(*joint, value.value, errors) && valid;
            result.joints.push_back(std::move(value));
        }
    } else {
        valid = false;
    }
    if (const auto* locators = read_array_member("locators")) {
        for (const auto& entry : locators->GetArray()) {
            if (!entry.IsObject()) {
                errors.emplace_back("Project locator entries must be objects");
                valid = false;
                continue;
            }
            SavedLocator value;
            valid = read_locator(entry, value, errors) && valid;
            result.locators.push_back(std::move(value));
        }
    } else {
        valid = false;
    }
    if (const auto* controllers = read_array_member("controllers")) {
        for (const auto& entry : controllers->GetArray()) {
            if (!entry.IsObject()) {
                errors.emplace_back(
                    "Project controller entries must be objects");
                valid = false;
                continue;
            }
            SavedController value;
            valid = read_controller(entry, value, errors) && valid;
            result.controllers.push_back(std::move(value));
        }
    } else {
        valid = false;
    }
    if (const auto* constraints = read_array_member("constraints")) {
        for (const auto& entry : constraints->GetArray()) {
            if (!entry.IsObject()) {
                errors.emplace_back(
                    "Project constraint entries must be objects");
                valid = false;
                continue;
            }
            SavedConstraint value;
            valid = read_constraint(entry, value, errors) && valid;
            result.constraints.push_back(std::move(value));
        }
    } else {
        valid = false;
    }
    if (const auto* attachments = read_array_member("attachments")) {
        for (const auto& entry : attachments->GetArray()) {
            if (!entry.IsObject()) {
                errors.emplace_back(
                    "Project attachment entries must be objects");
                valid = false;
                continue;
            }
            SavedAttachment value;
            std::int64_t target_kind = 0;
            valid = read_u64(entry, "controller", value.controller,
                errors) && valid;
            valid = read_u64(entry, "target", value.target, errors)
                && valid;
            valid = read_i64(entry, "target_kind", target_kind, errors)
                && valid;
            if (target_kind < 0
                || target_kind > static_cast<std::int64_t>(
                    AttachmentTargetKind::Locator))
            {
                errors.emplace_back(
                    "Project attachment target kind is out of range");
                valid = false;
            }
            value.target_kind =
                static_cast<AttachmentTargetKind>(target_kind);
            valid = read_matrix(entry, "xform", value.xform, errors)
                && valid;
            valid = read_bool(entry, "input_drives_target",
                value.input_drives_target, errors) && valid;
            result.attachments.push_back(value);
        }
    } else {
        valid = false;
    }

    if (const auto* weights = member<void>(
            object, "weights", errors, false))
    {
        valid = read_weight_data(*weights, result.weights, errors)
            && valid;
    }
    if (const auto* deformer = member<void>(
            object, "deformer", errors, false))
    {
        valid = read_deformer_data(*deformer, result.deformer, errors)
            && valid;
    }
    return valid;
}

bool validate_component_ids(const SavedComponents& components,
    std::vector<std::string>& errors)
{
    std::set<std::uint64_t> ids;
    std::set<std::string> names;
    bool valid = true;
    const auto check = [&ids, &names, &errors, &valid](
                           std::uint64_t id, const std::string& name) {
        if (id == 0 || !ids.insert(id).second) {
            errors.emplace_back("Project component IDs must be unique");
            valid = false;
        }
        if (name.empty() || !names.insert(name).second) {
            errors.emplace_back(
                "Project component names must be unique and non-empty");
            valid = false;
        }
    };
    for (const auto& value : components.joints) {
        check(value.id, value.name);
    }
    for (const auto& value : components.locators) {
        check(value.id, value.name);
    }
    for (const auto& value : components.controllers) {
        check(value.id, value.name);
    }
    for (const auto& value : components.constraints) {
        check(value.id, value.name);
    }
    return valid;
}

bool validate_component_references(const SavedComponents& components,
    std::vector<std::string>& errors)
{
    std::unordered_map<std::uint64_t, ComponentKind> kinds;
    for (const auto& value : components.joints) {
        kinds.emplace(value.id, ComponentKind::Joint);
        if (value.value.parent < -1
            || value.value.parent
                >= static_cast<std::int64_t>(components.joints.size()))
        {
            errors.emplace_back("Project joint parent index is invalid");
        }
    }
    for (const auto& value : components.locators) {
        kinds.emplace(value.id, ComponentKind::Locator);
    }
    for (const auto& value : components.controllers) {
        kinds.emplace(value.id, ComponentKind::Controller);
    }
    for (const auto& value : components.constraints) {
        kinds.emplace(value.id, ComponentKind::Constraint);
    }

    const auto exists = [&kinds](ComponentId id) {
        return id && kinds.contains(id.value);
    };
    for (const auto& value : components.constraints) {
        if (!exists(value.value.root) || !exists(value.value.mid)
            || !exists(value.value.end) || !exists(value.value.target)
            || !exists(value.value.pole))
        {
            errors.emplace_back(
                "Project constraint references a missing component");
        }
    }
    for (const auto& value : components.attachments) {
        const auto controller = kinds.find(value.controller);
        const auto target = kinds.find(value.target);
        const bool valid_target_kind =
            value.target_kind == AttachmentTargetKind::Joint
                ? target != kinds.end()
                    && target->second == ComponentKind::Joint
                : value.target_kind == AttachmentTargetKind::Locator
                    ? target != kinds.end()
                        && target->second == ComponentKind::Locator
                    : false;
        if (controller == kinds.end()
            || controller->second != ComponentKind::Controller
            || !valid_target_kind)
        {
            errors.emplace_back(
                "Project controller attachment references invalid components");
        }
    }
    return errors.empty();
}

std::string diagnostics_text(
    const std::vector<orlgraph::Diagnostic>& diagnostics)
{
    std::ostringstream result;
    for (const auto& diagnostic : diagnostics) {
        if (result.tellp() != std::streampos{0}) {
            result << '\n';
        }
        result << diagnostic.code << ": " << diagnostic.message;
    }
    return result.str();
}

Json serialize_components(const ComponentManager& components,
    ComponentId weight_id, ComponentId deformer_id, Allocator& allocator)
{
    Json result(rapidjson::kObjectType);

    Json joints(rapidjson::kArrayType);
    for (const ComponentId id : components.packed_joint_ids()) {
        const auto* meta = components.find(id);
        const auto* value = components.joint(id);
        if (meta == nullptr || value == nullptr) {
            continue;
        }
        Json entry(rapidjson::kObjectType);
        add_component_header(entry, id.value, meta->name, allocator);
        add_joint(entry, "value", *value, allocator);
        joints.PushBack(std::move(entry), allocator);
    }
    add(result, "joints", std::move(joints), allocator);

    Json locators(rapidjson::kArrayType);
    for (const ComponentId id : components.packed_locator_ids()) {
        const auto* meta = components.find(id);
        const auto* value = components.locator(id);
        if (meta == nullptr || value == nullptr) {
            continue;
        }
        Json entry(rapidjson::kObjectType);
        add_component_header(entry, id.value, meta->name, allocator);
        add_locator(entry, "xform", *value, allocator);
        locators.PushBack(std::move(entry), allocator);
    }
    add(result, "locators", std::move(locators), allocator);

    std::vector<ComponentId> other_ids;
    components.for_each([&other_ids](const Component& meta) {
        if (meta.kind == ComponentKind::Controller
            || meta.kind == ComponentKind::Constraint)
        {
            other_ids.push_back(meta.id);
        }
    });
    std::sort(other_ids.begin(), other_ids.end(),
        [](ComponentId first, ComponentId second) {
            return first.value < second.value;
        });

    Json controllers(rapidjson::kArrayType);
    Json constraints(rapidjson::kArrayType);
    Json attachments(rapidjson::kArrayType);
    for (const ComponentId id : other_ids) {
        const auto* meta = components.find(id);
        if (meta == nullptr) {
            continue;
        }
        if (meta->kind == ComponentKind::Controller) {
            const auto* value = components.controller(id);
            if (value == nullptr) {
                continue;
            }
            Json entry(rapidjson::kObjectType);
            add_component_header(entry, id.value, meta->name, allocator);
            add_controller(entry, "value", *value, allocator);
            add(entry, "shape",
                Json{static_cast<std::int64_t>(
                    components.controller_shape(id))}, allocator);
            controllers.PushBack(std::move(entry), allocator);

            if (const auto* attachment =
                    components.controller_attachment(id))
            {
                Json attached(rapidjson::kObjectType);
                add(attached, "controller", Json{id.value}, allocator);
                add(attached, "target",
                    Json{attachment->target.value}, allocator);
                add(attached, "target_kind",
                    Json{static_cast<std::int64_t>(
                        attachment->target_kind)}, allocator);
                add_matrix(attached, "xform", attachment->xform, allocator);
                add(attached, "input_drives_target",
                    Json{attachment->input_drives_target}, allocator);
                attachments.PushBack(std::move(attached), allocator);
            }
        } else if (meta->kind == ComponentKind::Constraint) {
            const auto* value = components.constraint(id);
            if (value == nullptr) {
                continue;
            }
            Json entry(rapidjson::kObjectType);
            add_component_header(entry, id.value, meta->name, allocator);
            add(entry, "type", string_value(value->type, allocator),
                allocator);
            add(entry, "root", Json{value->root.value}, allocator);
            add(entry, "mid", Json{value->mid.value}, allocator);
            add(entry, "end", Json{value->end.value}, allocator);
            add(entry, "target", Json{value->target.value}, allocator);
            add(entry, "pole", Json{value->pole.value}, allocator);
            add(entry, "bound", Json{value->bound}, allocator);
            constraints.PushBack(std::move(entry), allocator);
        }
    }
    add(result, "controllers", std::move(controllers), allocator);
    add(result, "constraints", std::move(constraints), allocator);
    add(result, "attachments", std::move(attachments), allocator);

    Json weights(rapidjson::kObjectType);
    if (const auto* value = components.weight(weight_id)) {
        if (const auto* meta = components.find(weight_id)) {
            add(weights, "name", string_value(meta->name, allocator),
                allocator);
        }
        add(weights, "weight_count", Json{value->weight_cnt}, allocator);
        add_weight_buffer(weights, value->weights, allocator);
        add(result, "weights", std::move(weights), allocator);
    }

    Json deformer(rapidjson::kObjectType);
    if (const auto* value = components.deformer(deformer_id)) {
        add(deformer, "type", string_value(value->type, allocator),
            allocator);
        add(deformer, "mesh_name",
            string_value(value->mesh_name, allocator), allocator);
        add_matrix(deformer, "bind_model", value->bind_model, allocator);
        add(deformer, "bound", Json{value->bound}, allocator);
        add_deformer_buffer(deformer, value->bind_positions,
            "bind_positions", value->bind_positions.count(), 4, allocator);
        Json inverse(rapidjson::kArrayType);
        const auto* source =
            static_cast<const double*>(value->inverse_binds.data());
        for (std::size_t index = 0;
             index < value->inverse_binds.count(); ++index)
        {
            Json matrix(rapidjson::kArrayType);
            for (std::size_t component = 0; component < 16; ++component) {
                matrix.PushBack(
                    source[index * 16 + component], allocator);
            }
            inverse.PushBack(std::move(matrix), allocator);
        }
        add(deformer, "inverse_binds", std::move(inverse), allocator);
        add(result, "deformer", std::move(deformer), allocator);
    }
    return result;
}

bool restore_buffer(exec::OrlBuffer& destination,
    const std::vector<orlrig::Weight>& values)
{
    if (!destination.resize(values.size())) {
        return false;
    }
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!destination.write(index, values[index])) {
            return false;
        }
    }
    return true;
}

bool restore_double_buffer(exec::OrlBuffer& destination,
    const std::vector<std::array<double, 4>>& values)
{
    if (!destination.resize(values.size())) {
        return false;
    }
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!destination.write(index, values[index].data(),
                sizeof(double) * values[index].size()))
        {
            return false;
        }
    }
    return true;
}

bool restore_matrix_buffer(exec::OrlBuffer& destination,
    const std::vector<std::array<double, 16>>& values)
{
    if (!destination.resize(values.size())) {
        return false;
    }
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!destination.write(index, values[index].data(),
                sizeof(double) * values[index].size()))
        {
            return false;
        }
    }
    return true;
}

bool apply_components(const SavedComponents& saved,
    ComponentManager& components, ComponentId weight_id,
    ComponentId deformer_id, std::vector<std::string>& errors)
{
    std::unordered_map<std::uint64_t, ComponentId> id_map;
    for (const auto& value : saved.joints) {
        const auto id = components.create_joint(value.name, value.value);
        if (!id) {
            errors.emplace_back("Unable to restore joint '" + value.name + "'");
            return false;
        }
        id_map.emplace(value.id, id);
    }
    for (const auto& value : saved.locators) {
        const auto id = components.create_locator(value.name, value.value);
        if (!id) {
            errors.emplace_back(
                "Unable to restore locator '" + value.name + "'");
            return false;
        }
        id_map.emplace(value.id, id);
    }
    for (const auto& value : saved.controllers) {
        const auto id = components.create_controller(
            value.name, value.value, value.shape);
        if (!id) {
            errors.emplace_back(
                "Unable to restore controller '" + value.name + "'");
            return false;
        }
        id_map.emplace(value.id, id);
    }

    for (const auto& value : saved.attachments) {
        const auto controller = id_map.find(value.controller);
        const auto target = id_map.find(value.target);
        if (controller == id_map.end() || target == id_map.end()
            || !components.attach_controller(
                controller->second, target->second))
        {
            errors.emplace_back("Unable to restore controller attachment");
            return false;
        }
        auto* attachment =
            components.controller_attachment(controller->second);
        if (attachment == nullptr) {
            errors.emplace_back(
                "Restored controller attachment is unavailable");
            return false;
        }
        attachment->target_kind = value.target_kind;
        attachment->xform = value.xform;
        attachment->input_drives_target = value.input_drives_target;
    }

    // attach_controller() initializes the controller payload as a setup
    // transform. Restore the saved setup/input values after rebuilding the
    // relationship indices.
    for (const auto& value : saved.controllers) {
        const auto found = id_map.find(value.id);
        if (found != id_map.end()) {
            if (auto* controller = components.controller(found->second)) {
                *controller = value.value;
            }
        }
    }

    for (const auto& value : saved.constraints) {
        const auto remap = [&id_map](ComponentId old) {
            const auto found = id_map.find(old.value);
            return found == id_map.end() ? ComponentId{} : found->second;
        };
        ConstraintData data = value.value;
        data.root = remap(data.root);
        data.mid = remap(data.mid);
        data.end = remap(data.end);
        data.target = remap(data.target);
        data.pole = remap(data.pole);
        if (!data.root || !data.mid || !data.end
            || !data.target || !data.pole
            || !components.create_constraint(value.name, data))
        {
            errors.emplace_back(
                "Unable to restore constraint '" + value.name + "'");
            return false;
        }
    }

    if (saved.weights.present) {
        auto* weights = components.weight(weight_id);
        if (weights == nullptr
            || !restore_buffer(weights->weights, saved.weights.values))
        {
            errors.emplace_back("Unable to restore project weights");
            return false;
        }
        weights->weight_cnt = saved.weights.weight_count;
    }
    if (saved.deformer.present) {
        auto* deformer = components.deformer(deformer_id);
        if (deformer == nullptr
            || !restore_double_buffer(
                deformer->bind_positions,
                saved.deformer.bind_positions)
            || !restore_matrix_buffer(
                deformer->inverse_binds,
                saved.deformer.inverse_binds))
        {
            errors.emplace_back("Unable to restore project deformer buffers");
            return false;
        }
        deformer->type = saved.deformer.type;
        deformer->mesh_name = saved.deformer.mesh_name;
        deformer->bind_model = saved.deformer.bind_model;
        deformer->bound = saved.deformer.bound;
    }
    return true;
}

bool load_text(const std::string& path, std::string& text,
    std::vector<std::string>& errors)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        errors.emplace_back("Unable to open project file: " + path);
        return false;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    text = contents.str();
    return true;
}

} // namespace

ProjectIoResult save_project_json(
    const std::string& path,
    SceneGraphContext& graph_context,
    const ComponentManager& components,
    ComponentId weight_id,
    ComponentId deformer_id,
    bool evaluate_orl)
{
    graph_context.ensure_stage_graphs();
    const auto serialized_graph = orlgraph::serialize_graph_stages_json(
        graph_context.stage_graph(orlgraph::GraphStage::Solver),
        graph_context.stage_graph(orlgraph::GraphStage::Deformer));
    if (!serialized_graph.ok) {
        return failure(
            serialized_graph.diagnostics.empty()
                ? "Unable to serialize project graphs"
                : diagnostics_text(serialized_graph.diagnostics));
    }

    rapidjson::Document graph_document;
    graph_document.Parse(
        serialized_graph.text.data(), serialized_graph.text.size());
    if (graph_document.HasParseError() || !graph_document.IsObject()) {
        return failure("Unable to parse the serialized project graph");
    }

    rapidjson::Document document;
    document.SetObject();
    auto& allocator = document.GetAllocator();

    Json header(rapidjson::kObjectType);
    add(header, "magic", string_value(kProjectMagic, allocator), allocator);
    add(header, "format_version",
        Json{kProjectFormatVersion}, allocator);
    add(document, "header", std::move(header), allocator);

    Json evaluation(rapidjson::kObjectType);
    add(evaluation, "evaluate_orl", Json{evaluate_orl}, allocator);
    add(document, "evaluation", std::move(evaluation), allocator);

    Json graph(rapidjson::kObjectType);
    graph.CopyFrom(graph_document, allocator);
    add(document, "graph", std::move(graph), allocator);
    add(document, "components",
        serialize_components(components, weight_id, deformer_id, allocator),
        allocator);

    std::ofstream output(path, std::ios::binary);
    if (!output.is_open()) {
        return failure("Unable to open project output file: " + path);
    }
    output << write_pretty_json(document);
    if (!output) {
        return failure("Unable to write project output file: " + path);
    }

    ProjectIoResult result;
    result.ok = true;
    result.evaluate_orl = evaluate_orl;
    return result;
}

ProjectIoResult load_project_json(
    const std::string& path,
    SceneGraphContext& graph_context,
    ComponentManager& components,
    ComponentId weight_id,
    ComponentId deformer_id)
{
    std::string text;
    ProjectIoResult result;
    if (!load_text(path, text, result.errors)) {
        return result;
    }

    rapidjson::Document document;
    document.Parse(text.data(), text.size());
    if (document.HasParseError() || !document.IsObject()) {
        return failure("The project file is not valid JSON");
    }

    const auto* header = member<void>(
        document, "header", result.errors);
    if (header == nullptr || !header->IsObject()) {
        return result;
    }
    std::string magic;
    std::uint64_t version = 0;
    if (!read_string(*header, "magic", magic, result.errors)
        || magic != kProjectMagic
        || !read_u64(*header, "format_version", version, result.errors)
        || version != kProjectFormatVersion)
    {
        if (magic != kProjectMagic) {
            result.errors.emplace_back("Project file magic is invalid");
        } else if (version != kProjectFormatVersion) {
            result.errors.emplace_back(
                "Unsupported project file format version");
        }
        return result;
    }

    if (const auto* evaluation = member<void>(
            document, "evaluation", result.errors, false))
    {
        read_bool(*evaluation, "evaluate_orl", result.evaluate_orl,
            result.errors, false);
    }

    const auto* graph = member<void>(document, "graph", result.errors);
    const auto* component_value =
        member<void>(document, "components", result.errors);
    if (graph == nullptr || component_value == nullptr
        || !graph->IsObject())
    {
        return result;
    }

    const auto graph_document = orlgraph::deserialize_graph_stages_json(
        write_json(*graph));
    if (!graph_document.ok) {
        result.errors.push_back(
            graph_document.diagnostics.empty()
                ? "Unable to load project graph"
                : diagnostics_text(graph_document.diagnostics));
        return result;
    }
    const auto solver_validation = orlgraph::validate(
        graph_document.solver, graph_context.registry(),
        orlgraph::GraphStage::Solver);
    const auto deformer_validation = orlgraph::validate(
        graph_document.deformer, graph_context.registry(),
        orlgraph::GraphStage::Deformer);
    if (!solver_validation.ok()) {
        result.errors.push_back(
            "Solver graph validation failed:\n"
            + diagnostics_text(solver_validation.diagnostics));
    }
    if (!deformer_validation.ok()) {
        result.errors.push_back(
            "Deformer graph validation failed:\n"
            + diagnostics_text(deformer_validation.diagnostics));
    }
    if (!result.errors.empty()) {
        return result;
    }

    SavedComponents saved;
    if (!read_components(*component_value, saved, result.errors)
        || !validate_component_ids(saved, result.errors)
        || !validate_component_references(saved, result.errors))
    {
        return result;
    }

    const auto persistent_name = [&components](
                                     ComponentId id) -> std::string {
        const auto* meta = components.find(id);
        return meta == nullptr ? std::string{} : meta->name;
    };
    std::set<std::string> reserved_names;
    if (const auto name = persistent_name(weight_id); !name.empty()) {
        reserved_names.insert(name);
    }
    if (const auto name = persistent_name(deformer_id); !name.empty()) {
        reserved_names.insert(name);
    }
    for (const auto& value : saved.joints) {
        if (reserved_names.contains(value.name)) {
            result.errors.emplace_back(
                "Project component name conflicts with a persistent component");
        }
    }
    for (const auto& value : saved.locators) {
        if (reserved_names.contains(value.name)) {
            result.errors.emplace_back(
                "Project component name conflicts with a persistent component");
        }
    }
    for (const auto& value : saved.controllers) {
        if (reserved_names.contains(value.name)) {
            result.errors.emplace_back(
                "Project component name conflicts with a persistent component");
        }
    }
    for (const auto& value : saved.constraints) {
        if (reserved_names.contains(value.name)) {
            result.errors.emplace_back(
                "Project component name conflicts with a persistent component");
        }
    }
    if (!result.errors.empty()) {
        return result;
    }

    components.destroy_kind(ComponentKind::Constraint);
    components.destroy_kind(ComponentKind::Controller);
    components.destroy_kind(ComponentKind::Locator);
    components.destroy_kind(ComponentKind::Joint);
    components.destroy_kind(ComponentKind::Curve);

    if (!apply_components(
            saved, components, weight_id, deformer_id, result.errors))
    {
        return result;
    }

    auto registry = graph_context.registry();
    graph_context.set_stage_graphs(
        std::move(graph_document.solver),
        std::move(graph_document.deformer),
        std::move(registry));

    result.ok = true;
    return result;
}

} // namespace ORL
