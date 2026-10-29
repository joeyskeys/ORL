#include "vkkk_adapter.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "vkkk_adapter_impl.hpp"
#include "vp/object_picking.hpp"

namespace ORL::Rhi {
namespace {

using vkkk_detail::Pass;
using vkkk_detail::PickLayerState;
using vkkk_detail::PipelineDesc;
using vkkk_detail::to_glm;

constexpr std::uint32_t kHeadBinding = 3;
constexpr std::uint32_t kNodeBinding = 4;

constexpr const char* kMeshPickPipeline = "orl_mesh_picking";
constexpr const char* kMeshPickABuffer = "orl_mesh_picking_abuffer";

constexpr const char* kJointPickPipeline = "orl_joint_picking";
constexpr const char* kJointPickABuffer = "orl_joint_picking_abuffer";
constexpr const char* kJointsBlock = "Joints";
constexpr const char* kPointSizeBlock = "PointSizeUBO";
constexpr const char* kJointPickPoints = "orl_joint_pick_points";

constexpr const char* kControllerPickLines = "orl_controller_picking_lines";
constexpr const char* kControllerPickPoly = "orl_controller_picking_poly";
constexpr const char* kLocatorPickPipeline = "orl_locator_picking";
constexpr const char* kRigPickABuffer = "orl_rig_picking_abuffer";
constexpr const char* kModelBlock = "ModelUBO";
constexpr const char* kLocatorTransformsBlock = "LocatorTransforms";
constexpr const char* kLocatorIdsBlock = "LocatorIds";
constexpr const char* kPickIdBlock = "PickId";
constexpr std::size_t kDeviceLocatorStride = sizeof(double) * 16;

struct PickId {
    std::uint32_t value = 0;
};

// Matches the std430 Locators record in locator_picking.vert.
struct LocatorPickTransform {
    double xform[16] = {};
};

struct LocatorPickId {
    std::uint32_t id = 0;
    std::uint32_t pad0 = 0;
    std::uint32_t pad1 = 0;
    std::uint32_t pad2 = 0;
};

LocatorPickTransform locator_pick_transform(const Mat4& model) {
    const auto value = to_glm(model);
    LocatorPickTransform result;
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            result.xform[column * 4 + row] =
                static_cast<double>(value[column][row]);
        }
    }
    return result;
}

struct ShapePick {
    std::string shape;
    bool lines = true;
    glm::mat4 model{1.0f};
    std::uint32_t id = 0;
};

std::size_t index_of(PickLayer layer) {
    return static_cast<std::size_t>(layer);
}

const char* abuffer_for(PickLayer layer) {
    switch (layer) {
    case PickLayer::mesh:
        return kMeshPickABuffer;
    case PickLayer::joint:
        return kJointPickABuffer;
    case PickLayer::rig:
        return kRigPickABuffer;
    }
    return kMeshPickABuffer;
}

std::vector<const char*> pipelines_for(PickLayer layer) {
    switch (layer) {
    case PickLayer::mesh:
        return {kMeshPickPipeline};
    case PickLayer::joint:
        return {kJointPickPipeline};
    case PickLayer::rig:
        return {kControllerPickLines, kControllerPickPoly, kLocatorPickPipeline};
    }
    return {};
}

PipelineDesc pick_pipeline(const VkkkAdapter::Impl& impl, const char* name,
    std::string_view vertex, std::string_view fragment, vk::PrimitiveTopology topology)
{
    PipelineDesc desc;
    desc.name = name;
    desc.vertex.path = impl.shader_path(vertex);
    desc.fragment.path = impl.shader_path(fragment);
    desc.topology = topology;
    desc.depth_test = false;
    desc.depth_write = false;
    desc.depth_compare = vk::CompareOp::eAlways;
    desc.pick_target = true;
    return desc;
}

bool create_layer_resources(VkkkAdapter::Impl& impl, const PickLayerSetup& setup) {
    auto& context = *impl.context;
    switch (setup.layer) {
    case PickLayer::mesh: {
        auto desc = pick_pipeline(impl, kMeshPickPipeline, "object_picking.vert",
            "object_picking_abuffer.frag", vk::PrimitiveTopology::eTriangleList);
        desc.components = {vkkk::VERTEX, vkkk::NORMAL};
        return vkkk_detail::create_pipeline(context, desc);
    }
    case PickLayer::joint:
        return vkkk_detail::create_pipeline(context, pick_pipeline(impl, kJointPickPipeline,
                   "joint_picking.vert", "joint_picking.frag",
                   vk::PrimitiveTopology::ePointList))
            && vkkk_detail::load_dummy_points(context, kJointPickPoints)
            && context.resize_pipeline_ssbo(kJointPickPipeline, kJointsBlock, 1)
            && context.alloc_pipeline_ssbo(kJointPickPipeline, kJointsBlock);
    case PickLayer::rig:
        return vkkk_detail::create_pipeline(context, pick_pipeline(impl, kControllerPickLines,
                   "controller_picking.vert", "selection_abuffer.frag",
                   vk::PrimitiveTopology::eLineList))
            && vkkk_detail::create_pipeline(context, pick_pipeline(impl, kControllerPickPoly,
                   "controller_picking.vert", "selection_abuffer.frag",
                   vk::PrimitiveTopology::eTriangleList))
            && vkkk_detail::create_pipeline(context, pick_pipeline(impl, kLocatorPickPipeline,
                   "locator_picking.vert", "selection_abuffer.frag",
                   vk::PrimitiveTopology::eLineList))
            && context.resize_pipeline_ssbo(
                kLocatorPickPipeline, kLocatorTransformsBlock, 1)
            && context.alloc_pipeline_ssbo(
                kLocatorPickPipeline, kLocatorTransformsBlock)
            && context.resize_pipeline_ssbo(
                kLocatorPickPipeline, kLocatorIdsBlock, 1)
            && context.alloc_pipeline_ssbo(
                kLocatorPickPipeline, kLocatorIdsBlock);
    }
    return false;
}

Status require_layer(const VkkkAdapter::Impl& impl, PickLayer layer, std::string_view call) {
    if (auto status = impl.require_frame(call); !status) {
        return status;
    }
    if (!impl.pick_layers[index_of(layer)].prepared) {
        return Status::failure(StatusCode::not_initialized,
            std::string{call} + " requires prepare_pick_layer");
    }
    return Status::success();
}

// Every layer renders into its A-buffer inside its own depth-only pass, then
// makes the A-buffer visible to the host for read_pick.
template <typename DrawFn>
void queue_pick_pass(VkkkAdapter::Impl& impl, PickLayer layer, DrawFn draw) {
    impl.queue(Pass::picking,
        [impl = &impl, layer, serial = impl.active_frame->serial, draw = std::move(draw)](
            vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            auto& context = *impl->context;
            if (!context.clear_abuffer(abuffer_for(layer), image_index)) {
                return;
            }
            vkkk::PassDesc pass{};
            pass.colors.clear();
            pass.present = false;
            context.begin_pass(cmd, image_index, pass);
            draw(context, cmd, image_index);
            context.end_pass(cmd, image_index, pass);
            context.barrier_abuffer_for_host(*cmd);
            auto& state = impl->pick_layers[index_of(layer)];
            state.rendered_serial = serial;
            state.rendered_image_index = image_index;
        });
}

} // namespace

Status VkkkAdapter::prepare_pick_layer(const PickLayerSetup& setup) {
    if (auto status = impl_->require_context("prepare_pick_layer"); !status) {
        return status;
    }
    if (setup.nodes_per_pixel == 0) {
        return Status::failure(StatusCode::invalid_argument,
            "prepare_pick_layer needs at least one A-buffer node per pixel");
    }
    if (setup.layer == PickLayer::joint && setup.joint_stride == 0) {
        return Status::failure(StatusCode::invalid_argument,
            "the joint pick layer needs a joint stride");
    }

    auto& state = impl_->pick_layers[index_of(setup.layer)];
    state.prepared = false;
    state.rendered_serial.reset();
    state.device_upload_serial.reset();
    state.device_locator_upload_serial.reset();
    state.device_locator_count = 0;
    try {
        if (!create_layer_resources(*impl_, setup)) {
            return Status::failure(StatusCode::resource_failure,
                "pick layer pipelines are unavailable");
        }
        auto& context = *impl_->context;
        const char* abuffer = abuffer_for(setup.layer);
        if (!context.resize_abuffer(abuffer, context.extent(), setup.nodes_per_pixel)) {
            return Status::failure(StatusCode::resource_failure,
                "vkkk could not size the pick A-buffer");
        }
        for (const auto* pipeline : pipelines_for(setup.layer)) {
            if (!context.bind_pipeline_abuffer(pipeline, abuffer, kHeadBinding, kNodeBinding)) {
                return Status::failure(StatusCode::resource_failure,
                    "vkkk could not bind the pick A-buffer");
            }
        }
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::resource_failure, error);
    }
    state.nodes_per_pixel = setup.nodes_per_pixel;
    if (setup.layer == PickLayer::joint) {
        state.joint_stride = setup.joint_stride;
    }
    state.prepared = true;
    impl_->caps.picking = true;
    return Status::success();
}

bool VkkkAdapter::map_cursor(double x, double y, std::uint32_t& px, std::uint32_t& py) const {
    if (!impl_->initialized || impl_->context == nullptr || impl_->window == nullptr) {
        return false;
    }
    return impl_->window->map_cursor(x, y, impl_->context->extent(), px, py);
}

Status VkkkAdapter::draw_mesh_pick(std::span<const MeshPickInstance> instances) {
    if (auto status = require_layer(*impl_, PickLayer::mesh, "draw_mesh_pick"); !status) {
        return status;
    }
    auto& state = impl_->pick_layers[index_of(PickLayer::mesh)];
    std::vector<std::string> meshes;
    std::vector<vkkk::vp::ObjectPickingInstance> records;
    meshes.reserve(instances.size());
    records.reserve(instances.size());
    for (const auto& instance : instances) {
        vkkk::vp::ObjectPickingInstance record{};
        record.model = to_glm(instance.model);
        record.object_id = instance.object_id;
        meshes.push_back(instance.mesh);
        records.push_back(record);
    }
    if (!records.empty() && records.size() != state.instance_capacity) {
        auto& context = *impl_->context;
        if (!context.resize_pipeline_ssbo(kMeshPickPipeline,
                vkkk::buf::ObjectPickingInstances, records.size())
            || !context.alloc_pipeline_ssbo(kMeshPickPipeline,
                vkkk::buf::ObjectPickingInstances))
        {
            return Status::failure(StatusCode::resource_failure,
                "vkkk could not size the mesh pick instance buffer");
        }
        state.instance_capacity = records.size();
    }

    queue_pick_pass(*impl_, PickLayer::mesh,
        [impl = impl_.get(), meshes = std::move(meshes), records = std::move(records)](
            vkkk::Context& context, vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            if (records.empty()) {
                return;
            }
            context.sync_ubo(kMeshPickPipeline, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ssbo(kMeshPickPipeline, vkkk::buf::ObjectPickingInstances,
                records.data(), image_index, static_cast<std::uint32_t>(
                    records.size() * sizeof(vkkk::vp::ObjectPickingInstance)));
            if (!context.bind(*cmd, kMeshPickPipeline, image_index)) {
                return;
            }
            for (std::uint32_t index = 0; index < static_cast<std::uint32_t>(records.size());
                ++index)
            {
                context.draw(*cmd, kMeshPickPipeline, meshes[index], 1, index);
            }
        });
    return Status::success();
}

Status VkkkAdapter::upload_device_pick_joints(DeviceBuffer device, std::uint32_t count) {
    if (auto status = require_layer(*impl_, PickLayer::joint, "upload_device_pick_joints");
        !status)
    {
        return status;
    }
    auto& state = impl_->pick_layers[index_of(PickLayer::joint)];
    state.device_upload_serial.reset();
    state.device_joint_count = 0;
    const auto bytes = static_cast<vk::DeviceSize>(count) * state.joint_stride;
    if (count == 0 || device.device_ptr == 0 || device.bytes < bytes) {
        return Status::failure(StatusCode::invalid_argument,
            "device joints are empty or smaller than the requested count");
    }
    auto& context = *impl_->context;
    try {
        const bool uploaded = context.resize_pipeline_ssbo(
                kJointPickPipeline, kJointsBlock, count)
            && context.write_pipeline_ssbo_from_cuda(kJointPickPipeline, kJointsBlock,
                impl_->active_frame->image_index, device.device_ptr, bytes);
        if (!uploaded) {
            return Status::failure(StatusCode::resource_failure,
                "vkkk could not copy device joints into the joint pick buffer");
        }
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::resource_failure, error);
    }
    state.device_upload_serial = impl_->active_frame->serial;
    state.device_joint_count = count;
    return Status::success();
}

Status VkkkAdapter::draw_joint_pick(const JointPickDraw& draw) {
    if (auto status = require_layer(*impl_, PickLayer::joint, "draw_joint_pick"); !status) {
        return status;
    }
    const auto& state = impl_->pick_layers[index_of(PickLayer::joint)];
    std::uint32_t joint_count = draw.joint_count;
    std::vector<std::byte> joints;
    if (draw.use_device_joints) {
        if (state.device_upload_serial != impl_->active_frame->serial) {
            return Status::failure(StatusCode::invalid_argument,
                "draw_joint_pick uses device joints that were not uploaded this frame");
        }
        joint_count = std::min(joint_count, state.device_joint_count);
    }
    else {
        const auto bytes = static_cast<std::size_t>(joint_count) * state.joint_stride;
        if (draw.joints.size() < bytes) {
            return Status::failure(StatusCode::invalid_argument,
                "draw_joint_pick host data is smaller than joint_count records");
        }
        joints.assign(draw.joints.begin(),
            draw.joints.begin() + static_cast<std::ptrdiff_t>(bytes));
    }

    const glm::vec4 point_size{
        vkkk_detail::point_size_for(*impl_->context, draw.point_size), 0.0f, 0.0f, 0.0f};
    queue_pick_pass(*impl_, PickLayer::joint,
        [impl = impl_.get(), host = !draw.use_device_joints, joint_count,
            joints = std::move(joints), point_size](
            vkkk::Context& context, vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            if (joint_count == 0
                || (host && !context.resize_pipeline_ssbo(
                    kJointPickPipeline, kJointsBlock, joint_count)))
            {
                return;
            }
            context.sync_ubo(kJointPickPipeline, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ubo(kJointPickPipeline, kPointSizeBlock, &point_size, image_index);
            if (host) {
                context.sync_ssbo(kJointPickPipeline, kJointsBlock, joints.data(),
                    image_index, static_cast<std::uint32_t>(joints.size()));
            }
            if (context.bind(*cmd, kJointPickPipeline, image_index)) {
                context.draw_points(*cmd, kJointPickPoints, 0, joint_count);
            }
        });
    return Status::success();
}

Status VkkkAdapter::upload_device_pick_locators(
    DeviceBuffer device, std::uint32_t count)
{
    if (auto status = require_layer(
            *impl_, PickLayer::rig, "upload_device_pick_locators");
        !status)
    {
        return status;
    }
    auto& state = impl_->pick_layers[index_of(PickLayer::rig)];
    state.device_locator_upload_serial.reset();
    state.device_locator_count = 0;
    const auto bytes = static_cast<vk::DeviceSize>(count)
        * static_cast<vk::DeviceSize>(kDeviceLocatorStride);
    if (count == 0 || device.device_ptr == 0
        || device.bytes < static_cast<std::size_t>(bytes))
    {
        return Status::failure(StatusCode::invalid_argument,
            "device locators are empty or smaller than the requested count");
    }
    auto& context = *impl_->context;
    try {
        const bool uploaded = context.resize_pipeline_ssbo(
                kLocatorPickPipeline, kLocatorTransformsBlock, count)
            && context.write_pipeline_ssbo_from_cuda(
                kLocatorPickPipeline, kLocatorTransformsBlock,
                impl_->active_frame->image_index, device.device_ptr, bytes);
        if (!uploaded) {
            return Status::failure(
                StatusCode::resource_failure,
                "vkkk could not copy device locators into the rig pick buffer");
        }
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::resource_failure, error);
    }
    state.device_locator_upload_serial = impl_->active_frame->serial;
    state.device_locator_count = count;
    return Status::success();
}

Status VkkkAdapter::draw_rig_pick(const RigPickDraw& draw) {
    if (auto status = require_layer(*impl_, PickLayer::rig, "draw_rig_pick"); !status) {
        return status;
    }
    std::vector<ShapePick> controllers;
    controllers.reserve(draw.controllers.size());
    for (const auto& controller : draw.controllers) {
        const auto found = impl_->shapes.find(controller.shape);
        if (found == impl_->shapes.end()) {
            return Status::failure(StatusCode::invalid_argument,
                "draw_rig_pick: shape '" + controller.shape + "' is not registered");
        }
        controllers.push_back({controller.shape, found->second == ShapeKind::lines,
            to_glm(controller.model), controller.id});
    }

    std::vector<LocatorPickTransform> locator_transforms;
    std::vector<LocatorPickId> locator_ids;
    if (!draw.locators.empty()) {
        const auto found = impl_->shapes.find(draw.locator_shape);
        if (found == impl_->shapes.end() || found->second != ShapeKind::lines) {
            return Status::failure(StatusCode::invalid_argument,
                "draw_rig_pick: '" + draw.locator_shape + "' is not a registered line shape");
        }
        locator_transforms.reserve(draw.locators.size());
        locator_ids.reserve(draw.locators.size());
        for (const auto& locator : draw.locators) {
            locator_transforms.push_back(locator_pick_transform(locator.model));
            locator_ids.push_back({.id = locator.id});
        }
    }
    const auto& state = impl_->pick_layers[index_of(PickLayer::rig)];
    const bool use_device_locators = draw.use_device_locators;
    if (use_device_locators
        && state.device_locator_upload_serial != impl_->active_frame->serial)
    {
        return Status::failure(
            StatusCode::invalid_argument,
            "draw_rig_pick uses device locators that were not uploaded this frame");
    }
    const auto locator_count = use_device_locators
        ? std::min(
            static_cast<std::uint32_t>(locator_ids.size()),
            state.device_locator_count)
        : static_cast<std::uint32_t>(locator_ids.size());

    queue_pick_pass(*impl_, PickLayer::rig,
        [impl = impl_.get(), controllers = std::move(controllers),
            locator_shape = draw.locator_shape,
            host = !use_device_locators,
            locator_count,
            locator_transforms = std::move(locator_transforms),
            locator_ids = std::move(locator_ids)](
            vkkk::Context& context, vk::raii::CommandBuffer& cmd, std::uint32_t image_index) {
            const bool locator_buffer_ready = locator_count != 0
                && context.resize_pipeline_ssbo(
                    kLocatorPickPipeline, kLocatorIdsBlock, locator_count)
                && context.alloc_pipeline_ssbo(
                    kLocatorPickPipeline, kLocatorIdsBlock)
                && (!host || (
                    context.resize_pipeline_ssbo(
                        kLocatorPickPipeline,
                        kLocatorTransformsBlock,
                        locator_count)
                    && context.alloc_pipeline_ssbo(
                        kLocatorPickPipeline, kLocatorTransformsBlock)));

            context.sync_ubo(kControllerPickLines, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ubo(kControllerPickPoly, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);
            context.sync_ubo(kLocatorPickPipeline, vkkk::buf::CameraUBO,
                &impl->camera_ubo, image_index);

            for (const auto& controller : controllers) {
                const char* pipeline = controller.lines
                    ? kControllerPickLines : kControllerPickPoly;
                const PickId pick_id{controller.id};
                context.sync_ubo(pipeline, kModelBlock, &controller.model, image_index);
                if (!context.bind(*cmd, pipeline, image_index)) {
                    continue;
                }
                context.push_constants(*cmd, pipeline, kPickIdBlock, &pick_id, sizeof(pick_id));
                if (controller.lines) {
                    context.draw_lines(*cmd, controller.shape);
                }
                else {
                    context.draw(*cmd, pipeline, controller.shape, 1);
                }
            }

            if (!locator_buffer_ready) {
                return;
            }
            if (host) {
                context.sync_ssbo(
                    kLocatorPickPipeline, kLocatorTransformsBlock,
                    locator_transforms.data(), image_index,
                    locator_count * sizeof(LocatorPickTransform));
            }
            context.sync_ssbo(
                kLocatorPickPipeline, kLocatorIdsBlock, locator_ids.data(),
                image_index, locator_count * sizeof(LocatorPickId));
            if (context.bind(*cmd, kLocatorPickPipeline, image_index)) {
                const PickId no_controller_id{};
                context.push_constants(*cmd, kLocatorPickPipeline, kPickIdBlock,
                    &no_controller_id, sizeof(no_controller_id));
                context.draw_lines(*cmd, locator_shape, 0, locator_count);
            }
        });
    return Status::success();
}

Status VkkkAdapter::read_pick(PickLayer layer, std::uint64_t min_serial,
    std::uint32_t x, std::uint32_t y, PickReadback& readback)
{
    readback = {};
    if (auto status = impl_->require_context("read_pick"); !status) {
        return status;
    }
    const auto& state = impl_->pick_layers[index_of(layer)];
    if (!state.prepared) {
        return Status::failure(StatusCode::not_initialized,
            "read_pick requires prepare_pick_layer");
    }
    if (!state.rendered_serial.has_value() || *state.rendered_serial < min_serial) {
        return Status::success();
    }

    std::vector<vkkk::ABufferHit> raw_hits;
    try {
        impl_->context->read_abuffer_pixel(abuffer_for(layer), state.rendered_image_index,
            x, y, raw_hits, readback.overflow);
    }
    catch (const std::exception& error) {
        return exception_status(StatusCode::device_failure, error);
    }
    readback.rendered = true;
    readback.hits.reserve(raw_hits.size());
    for (const auto& hit : raw_hits) {
        readback.hits.push_back({hit.vertex_id, hit.depth});
    }
    return Status::success();
}

} // namespace ORL::Rhi
