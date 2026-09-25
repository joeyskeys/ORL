#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <atomic>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "orlrig/abi.hpp"
#include "orlrig/component_store.hpp"
#include "orlrig/graph_resources.hpp"
#include "orlrig/handle_registry.hpp"
#include "orlrig/hierarchy.hpp"
#include "orlrig/ik.hpp"
#include "orlrig/locator.hpp"
#include "orlrig/runners.hpp"
#include "orlrig/two_stage_runner.hpp"

namespace
{

struct LbsFixture {
    orlrig::MeshData mesh;
    std::vector<orlrig::Joint> joints;
    orlrig::WeightData weights;
    orlrig::DeformerData deformer;
};

LbsFixture make_lbs_fixture() {
    LbsFixture fixture;
    fixture.mesh.positions = {glm::vec3{1.0f, 0.0f, 0.0f}};
    fixture.joints.resize(2);
    fixture.joints[0] = orlrig::make_identity_joint();
    fixture.joints[1] = orlrig::make_identity_joint();
    fixture.joints[1].parent = 0;
    fixture.joints[1].translation[0] = 1.0;
    fixture.weights.weight_cnt = 1;
    if (!fixture.weights.weights.resize(1)
        || !fixture.weights.weights.write<orlrig::Weight>(
            0, orlrig::Weight{1.0, 0}))
    {
        throw std::runtime_error{"failed to initialize LBS fixture"};
    }
    return fixture;
}

std::string last_error(const orlrig::RunnerStatus& status) {
    return status.errors.empty() ? "orlrig runner failed" : status.errors.back();
}

struct AutoWeightFixture {
    orlrig::MeshData mesh;
    std::vector<orlrig::Joint> joints;
    orlrig::MeshCsrData csr;
};

AutoWeightFixture make_auto_weight_fixture() {
    AutoWeightFixture fixture;
    fixture.mesh.positions = {
        glm::vec3{0.0f, 0.2f, 0.0f},
        glm::vec3{0.5f, 0.2f, 0.0f},
        glm::vec3{1.5f, 0.2f, 0.0f},
        glm::vec3{2.0f, 0.2f, 0.0f},
    };
    fixture.joints.resize(3);
    fixture.joints[0] = orlrig::make_identity_joint();
    fixture.joints[1] = orlrig::make_identity_joint();
    fixture.joints[1].parent = 0;
    fixture.joints[1].translation[0] = 1.0;
    fixture.joints[2] = orlrig::make_identity_joint();
    fixture.joints[2].parent = 1;
    fixture.joints[2].translation[0] = 1.0;

    // Four vertices connected as a line graph. The offsets array has one
    // entry per vertex plus the terminal neighbor offset.
    fixture.csr.offsets = {0, 1, 3, 5, 6};
    fixture.csr.neighbors = {1, 0, 2, 1, 3, 2};
    return fixture;
}

void check_auto_weight_algorithm(std::string_view algorithm,
    bool needs_csr)
{
    auto fixture = make_auto_weight_fixture();
    orlrig::WeightData weights;
    weights.weight_cnt = 2;
    orlrig::AutoWeightRunner runner(std::string{algorithm});
    const auto* csr = needs_csr ? &fixture.csr : nullptr;
    const auto status = runner.run(
        fixture.mesh, fixture.joints, csr, weights, 4.0);

    INFO("auto-weight algorithm: " << algorithm);
    REQUIRE(status);
    REQUIRE(weights.weights.count()
        == fixture.mesh.positions.size()
            * static_cast<std::size_t>(weights.weight_cnt));

    for (std::size_t vertex = 0;
         vertex < fixture.mesh.positions.size();
         ++vertex)
    {
        double sum = 0.0;
        for (std::size_t slot = 0;
             slot < static_cast<std::size_t>(weights.weight_cnt);
             ++slot)
        {
            orlrig::Weight cell{};
            REQUIRE(weights.weights.read(
                vertex * static_cast<std::size_t>(weights.weight_cnt) + slot,
                &cell));
            REQUIRE(cell.weight >= 0.0);
            REQUIRE(cell.weight <= 1.0);
            if (cell.joint >= 0) {
                REQUIRE(static_cast<std::size_t>(cell.joint)
                    < fixture.joints.size());
                sum += cell.weight;
            } else {
                REQUIRE(cell.weight == Catch::Approx(0.0));
            }
        }
        REQUIRE(sum == Catch::Approx(1.0));
    }
}

orlrig::TwoStageProgram make_two_stage_program() {
    return {
        {
            R"(
use joint;
int solve(Joint skeleton[], point target[], int joint_count) {
    Joint current = skeleton[0];
    vec4 translation(
        target[0].x, current.translation.y,
        current.translation.z, current.translation.w);
    Joint updated(
        current.parent, current.selected, current.pad0, current.pad1,
        translation, current.rotation, current.scale);
    skeleton[0] = updated;
    return joint_count;
}
)",
            {.entry_function = "solve", .source_name = "orlrig_test_solver"},
        },
        {
            R"(
use joint;
int deform(Joint pose[], point bind_positions[],
           point output_positions[], int vertex_count, float offset) {
    parallel for (int vertex = 0;
                  vertex < vertex_count;
                  vertex = vertex + 1) {
        point bind = bind_positions[vertex];
        point result(
            bind.x + pose[0].translation.x + offset,
            bind.y, bind.z);
        output_positions[vertex] = result;
    }
    return vertex_count;
}
)",
            {.entry_function = "deform", .source_name = "orlrig_test_deformer"},
        },
    };
}

} // namespace

TEST_CASE("orlrig preserves host and stdlib buffer ABI", "[orlrig][abi]") {
    REQUIRE(sizeof(orlrig::Joint) == orlrig::kJointStride);
    REQUIRE(sizeof(orlrig::Weight) == orlrig::kWeightStride);
    REQUIRE(orlrig::kLocatorStride == orlrig::kMatrixStride);

    orlrig::Controller controller = orlrig::make_controller(
        glm::vec3{2.0f, 3.0f, 4.0f});
    double packed[16] = {};
    orlrig::pack_xform(controller, packed);
    REQUIRE(packed[3] == Catch::Approx(2.0));
    REQUIRE(packed[7] == Catch::Approx(3.0));
    REQUIRE(packed[11] == Catch::Approx(4.0));

    const auto locator = orlrig::make_locator(glm::vec3{5.0f, 6.0f, 7.0f});
    orlrig::pack_xform(locator, packed);
    REQUIRE(packed[3] == Catch::Approx(5.0));
    REQUIRE(packed[7] == Catch::Approx(6.0));
    REQUIRE(packed[11] == Catch::Approx(7.0));
}

TEST_CASE("orlrig registers exact handle views and guards storage access",
    "[orlrig][handles]")
{
    orlcomp::HandleViewRegistry registry;
    std::string error;
    REQUIRE(orlrig::register_rig_handle_views(registry, &error));
    REQUIRE(registry.find(
        orlrig::kJointHandleCanonical, "Joint") != nullptr);
    REQUIRE(registry.find(
        orlrig::kLocatorHandleCanonical, "Locator") != nullptr);
    REQUIRE(registry.find(
        orlrig::kJointHandleCanonical,
        orlrig::kWorldTransformStruct) != nullptr);
    REQUIRE(registry.register_view({
        "test::custom_handle", "Custom", 1, 1, "custom_arena",
        {{"value", orlcomp::HandleViewFieldKind::Int64, 0,
            "custom_read", "custom_write"}}}, &error));
    REQUIRE_FALSE(registry.register_view({
        "test::custom_handle", "Custom", 1, 1, "custom_arena",
        {{"value", orlcomp::HandleViewFieldKind::Int64, 0,
            "custom_read", "custom_write"}}}, &error));

    std::vector<orlrig::Joint> joints(2);
    joints[0] = orlrig::make_identity_joint();
    joints[1] = orlrig::make_identity_joint();
    joints[1].parent = 0;
    joints[0].translation[0] = 2.0;
    joints[1].translation[0] = 1.0;
    orlrig::HandleViewContext context;
    context.joints = {
        joints.data(), joints.size(), sizeof(orlrig::Joint), true};
    context.topology_revision = 7;

    const auto joint_handle = orlcomp::HandleValue{
        orlcomp::HandleTypeIdFor(orlrig::kJointHandleCanonical), 1};
    double translation[4] = {};
    REQUIRE(orlrig::read_joint_vec4(
        context, joint_handle, 4, translation, &error));
    REQUIRE(translation[0] == Catch::Approx(1.0));
    translation[0] = 3.0;
    REQUIRE(orlrig::write_joint_vec4(
        context, joint_handle, 4, translation, &error));
    REQUIRE(joints[1].translation[0] == Catch::Approx(3.0));

    double world[16] = {};
    REQUIRE(orlrig::read_world_matrix(
        context, joint_handle, world, &error));
    REQUIRE(world[3] == Catch::Approx(5.0));
    world[3] = 8.0;
    REQUIRE(orlrig::write_world_matrix(
        context, joint_handle, world, &error));
    REQUIRE(joints[1].translation[0] == Catch::Approx(6.0));

    REQUIRE_FALSE(orlrig::read_joint_vec4(
        context,
        {orlcomp::HandleTypeIdFor(orlrig::kLocatorHandleCanonical), 1},
        4, translation, &error));
    REQUIRE_FALSE(orlrig::read_joint_vec4(
        context,
        {orlcomp::HandleTypeIdFor(orlrig::kJointHandleCanonical), 7},
        4, translation, &error));
    context.joints.writable = false;
    REQUIRE_FALSE(orlrig::write_joint_vec4(
        context, joint_handle, 4, translation, &error));
}

TEST_CASE("orlrig component store packs joints deterministically",
    "[orlrig][store]")
{
    orlrig::ComponentStore store;
    auto root = orlrig::make_identity_joint();
    const auto root_id = store.create_joint("root", root);

    auto child = orlrig::make_identity_joint();
    child.parent = 0;
    child.translation[0] = 1.0;
    const auto child_id = store.create_joint("child", child);

    REQUIRE(root_id);
    REQUIRE(child_id);
    REQUIRE(store.joint_index(root_id) == 0);
    REQUIRE(store.joint_index(child_id) == 1);
    REQUIRE(store.packed_joint_ids() == std::vector<orlrig::ComponentId>{
        root_id, child_id});
    REQUIRE(store.packed_joints()[1].parent == 0);
}

TEST_CASE("orlrig compiles stable-ID preorder hierarchy",
    "[orlrig][hierarchy]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");
    const auto other_root = store.create_joint("other_root");

    auto child_joint = orlrig::make_identity_joint();
    child_joint.parent = 0;
    const auto child = store.create_joint("child", child_joint);

    auto grandchild_joint = orlrig::make_identity_joint();
    grandchild_joint.parent = 2;
    const auto grandchild = store.create_joint(
        "grandchild", grandchild_joint);

    auto other_child_joint = orlrig::make_identity_joint();
    other_child_joint.parent = 1;
    const auto other_child = store.create_joint(
        "other_child", other_child_joint);

    orlrig::HierarchyCompileOptions options;
    options.topology_revision = 42;
    const auto result = orlrig::compile_hierarchy_plan(store, options);

    REQUIRE(result);
    REQUIRE(result.plan->topology_revision == 42);
    REQUIRE(result.plan->joint_count == 5);
    REQUIRE(result.plan->preorder_joints
        == std::vector<orlrig::ComponentId>{
            root, child, grandchild, other_root, other_child});
    REQUIRE(result.plan->depth
        == std::vector<std::uint32_t>{0, 1, 2, 0, 1});
    REQUIRE(result.plan->subtree_begin
        == std::vector<std::uint32_t>{0, 1, 2, 3, 4});
    REQUIRE(result.plan->subtree_end
        == std::vector<std::uint32_t>{3, 3, 3, 5, 5});
    REQUIRE(result.plan->level_offsets
        == std::vector<std::uint32_t>{0, 2, 4, 5});
    REQUIRE(result.plan->level_joints
        == std::vector<orlrig::ComponentId>{
            root, other_root, child, other_child, grandchild});
}

TEST_CASE("orlrig hierarchy defaults to flattened ancestors",
    "[orlrig][hierarchy][ancestors]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");

    auto mid_joint = orlrig::make_identity_joint();
    mid_joint.parent = 0;
    const auto mid = store.create_joint("mid", mid_joint);

    auto end_joint = orlrig::make_identity_joint();
    end_joint.parent = 1;
    const auto end = store.create_joint("end", end_joint);

    const auto result = orlrig::compile_hierarchy_plan(store);

    REQUIRE(result);
    REQUIRE(result.plan->ancestor_storage
        == orlrig::AncestorStorageMode::Flattened);
    REQUIRE(result.plan->parent_joints.empty());
    REQUIRE(result.plan->ancestor_offsets
        == std::vector<std::uint32_t>{0, 0, 1, 3});
    REQUIRE(result.plan->ancestor_joints
        == std::vector<orlrig::ComponentId>{root, root, mid});
    REQUIRE(result.plan->preorder_joints
        == std::vector<orlrig::ComponentId>{root, mid, end});
}

TEST_CASE("orlrig hierarchy supports parent-chain ancestors",
    "[orlrig][hierarchy][ancestors]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");

    auto child_joint = orlrig::make_identity_joint();
    child_joint.parent = 0;
    const auto child = store.create_joint("child", child_joint);

    orlrig::HierarchyCompileOptions options;
    options.ancestor_storage = orlrig::AncestorStorageMode::ParentChain;
    const auto result = orlrig::compile_hierarchy_plan(store, options);

    REQUIRE(result);
    REQUIRE(result.plan->ancestor_storage
        == orlrig::AncestorStorageMode::ParentChain);
    REQUIRE(result.plan->parent_joints
        == std::vector<orlrig::ComponentId>{
            orlrig::ComponentId{}, root});
    REQUIRE(result.plan->ancestor_offsets.empty());
    REQUIRE(result.plan->ancestor_joints.empty());
    REQUIRE(result.plan->preorder_joints
        == std::vector<orlrig::ComponentId>{root, child});
}

TEST_CASE("orlrig hierarchy packs a deterministic ABI buffer",
    "[orlrig][hierarchy][abi]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");
    auto child_joint = orlrig::make_identity_joint();
    child_joint.parent = 0;
    const auto child = store.create_joint("child", child_joint);

    const auto result = orlrig::compile_hierarchy_plan(store);
    REQUIRE(result);
    const auto packed = orlrig::pack_hierarchy_plan(*result.plan);

    REQUIRE(packed.context.joint_count == 2);
    REQUIRE(packed.context.level_count == 2);
    REQUIRE(packed.context.ancestor_storage == 1);
    REQUIRE(packed.context.preorder_offset == 0);
    REQUIRE(packed.context.depth_offset == 2);
    REQUIRE(packed.context.subtree_begin_offset == 4);
    REQUIRE(packed.context.subtree_end_offset == 6);
    REQUIRE(packed.context.level_offsets_offset == 8);
    REQUIRE(packed.context.level_joints_offset == 11);
    REQUIRE(packed.context.parent_joints_offset == 13);
    REQUIRE(packed.context.ancestor_offsets_offset == 13);
    REQUIRE(packed.context.ancestor_joints_offset == 16);
    REQUIRE(packed.context.data_count
        == static_cast<std::int64_t>(packed.data.size()));
    REQUIRE(packed.data == std::vector<std::int64_t>{
        static_cast<std::int64_t>(root.value),
        static_cast<std::int64_t>(child.value),
        0, 1,
        0, 1,
        2, 2,
        0, 1, 2,
        static_cast<std::int64_t>(root.value),
        static_cast<std::int64_t>(child.value),
        0, 0, 1,
        static_cast<std::int64_t>(root.value),
    });

    orlrig::HierarchyCompileOptions chain_options;
    chain_options.ancestor_storage =
        orlrig::AncestorStorageMode::ParentChain;
    const auto chain_result =
        orlrig::compile_hierarchy_plan(store, chain_options);
    REQUIRE(chain_result);
    const auto chain = orlrig::pack_hierarchy_plan(*chain_result.plan);
    REQUIRE(chain.context.ancestor_storage == 0);
    REQUIRE(chain.context.ancestor_count == 2);
    REQUIRE(chain.context.parent_joints_offset == 13);
    REQUIRE(chain.context.ancestor_offsets_offset == 15);
    REQUIRE(chain.context.ancestor_joints_offset == 15);
    REQUIRE(chain.data == std::vector<std::int64_t>{
        static_cast<std::int64_t>(root.value),
        static_cast<std::int64_t>(child.value),
        0, 1,
        0, 1,
        2, 2,
        0, 1, 2,
        static_cast<std::int64_t>(root.value),
        static_cast<std::int64_t>(child.value),
        0, static_cast<std::int64_t>(root.value),
    });
}

TEST_CASE("orlrig hierarchy resolves stable parent relationships",
    "[orlrig][hierarchy][lookup]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");
    auto child_joint = orlrig::make_identity_joint();
    child_joint.parent = 0;
    const auto child = store.create_joint("child", child_joint);

    const auto result = orlrig::compile_hierarchy_plan(store);

    REQUIRE(result);
    REQUIRE(result.plan->preorder_position(root)
        == std::optional<std::size_t>{0});
    REQUIRE(result.plan->preorder_position(child)
        == std::optional<std::size_t>{1});
    REQUIRE_FALSE(result.plan->preorder_position(
        orlrig::ComponentId{999}).has_value());
    REQUIRE(result.plan->parent_of(root)
        == std::optional<orlrig::ComponentId>{orlrig::ComponentId{}});
    REQUIRE(result.plan->parent_of(child)
        == std::optional<orlrig::ComponentId>{root});
    REQUIRE(result.plan->is_direct_parent(root, child));
    REQUIRE_FALSE(result.plan->is_direct_parent(child, root));
}

TEST_CASE("orlrig hierarchy handles empty and single-root stores",
    "[orlrig][hierarchy]")
{
    const auto empty_result =
        orlrig::compile_hierarchy_plan(orlrig::ComponentStore{});
    REQUIRE(empty_result);
    REQUIRE(empty_result.plan->joint_count == 0);
    REQUIRE(empty_result.plan->preorder_joints.empty());
    REQUIRE(empty_result.plan->ancestor_offsets
        == std::vector<std::uint32_t>{0});
    REQUIRE(empty_result.plan->level_offsets
        == std::vector<std::uint32_t>{0});

    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");
    const auto result = orlrig::compile_hierarchy_plan(store);

    REQUIRE(result);
    REQUIRE(result.plan->preorder_joints
        == std::vector<orlrig::ComponentId>{root});
    REQUIRE(result.plan->subtree_begin
        == std::vector<std::uint32_t>{0});
    REQUIRE(result.plan->subtree_end
        == std::vector<std::uint32_t>{1});
    REQUIRE(result.plan->ancestor_offsets
        == std::vector<std::uint32_t>{0, 0});
}

TEST_CASE("orlrig hierarchy rejects invalid parent indices",
    "[orlrig][hierarchy][validation]")
{
    orlrig::ComponentStore store;
    auto invalid = orlrig::make_identity_joint();
    invalid.parent = 7;
    store.create_joint("invalid", invalid);

    const auto result = orlrig::compile_hierarchy_plan(store);

    REQUIRE_FALSE(result);
    REQUIRE_FALSE(result.plan.has_value());
    REQUIRE_FALSE(result.errors.empty());
    REQUIRE(result.errors.front().find("out of range")
        != std::string::npos);
}

TEST_CASE("orlrig hierarchy rejects self-parenting and cycles",
    "[orlrig][hierarchy][validation]")
{
    orlrig::ComponentStore self_store;
    auto self = orlrig::make_identity_joint();
    self.parent = 0;
    self_store.create_joint("self", self);
    const auto self_result =
        orlrig::compile_hierarchy_plan(self_store);
    REQUIRE_FALSE(self_result);
    REQUIRE(self_result.errors.front().find("own parent")
        != std::string::npos);

    orlrig::ComponentStore cycle_store;
    auto first = orlrig::make_identity_joint();
    const auto first_id = cycle_store.create_joint("first", first);
    auto second = orlrig::make_identity_joint();
    second.parent = 0;
    const auto second_id = cycle_store.create_joint("second", second);
    REQUIRE(first_id);
    REQUIRE(second_id);
    cycle_store.joint(first_id)->parent = 1;

    const auto cycle_result =
        orlrig::compile_hierarchy_plan(cycle_store);
    REQUIRE_FALSE(cycle_result);
    REQUIRE(cycle_result.errors.front().find("cycle")
        != std::string::npos);
}

TEST_CASE("orlrig component store packs locators deterministically",
    "[orlrig][store][locator]")
{
    orlrig::ComponentStore store;
    const auto first = store.create_locator(
        "first", orlrig::make_locator(glm::vec3{1.0f, 2.0f, 3.0f}));
    const auto second = store.create_locator(
        "second", orlrig::make_locator(glm::vec3{4.0f, 5.0f, 6.0f}));

    REQUIRE(first);
    REQUIRE(second);
    REQUIRE(store.locator_index(first) == 0);
    REQUIRE(store.locator_index(second) == 1);
    REQUIRE(store.packed_locator_ids()
        == std::vector<orlrig::ComponentId>{first, second});
    REQUIRE(orlrig::locator_origin(store.packed_locators()[1]).x
        == Catch::Approx(4.0f));
}

TEST_CASE("orlrig runs bind capture and CPU LBS without a window",
    "[orlrig][cpu][lbs]")
{
    auto fixture = make_lbs_fixture();
    orlrig::LbsRunner runner(ORL::exec::Backend::Cpu);

    auto status = runner.capture_bind(
        fixture.deformer, fixture.mesh, fixture.joints);
    REQUIRE(status);
    REQUIRE(fixture.deformer.bound);
    REQUIRE(fixture.deformer.inverse_binds.count() == 2);

    fixture.joints[0].translation[0] = 1.0;
    status = runner.evaluate(fixture.deformer, fixture.weights, fixture.joints);
    REQUIRE(status);
    const auto* output = static_cast<const double*>(
        runner.output_positions().data());
    REQUIRE(output[0] == Catch::Approx(2.0));
}

TEST_CASE("orlrig LBS accepts a shared Joint stage buffer",
    "[orlrig][cpu][two_stage][lbs]")
{
    auto fixture = make_lbs_fixture();
    ORL::exec::OrlBuffer joint_buffer(
        orlrig::kJointOrlType, orlrig::kJointStride);
    REQUIRE(joint_buffer.resize(fixture.joints.size()));
    for (std::size_t index = 0; index < fixture.joints.size(); ++index) {
        REQUIRE(joint_buffer.write(index, fixture.joints[index]));
    }

    orlrig::LbsRunner runner(ORL::exec::Backend::Cpu);
    auto status = runner.capture_bind(
        fixture.deformer, fixture.mesh, joint_buffer);
    REQUIRE(status);

    auto posed = fixture.joints[0];
    posed.translation[0] = 1.0;
    REQUIRE(joint_buffer.write(0, posed));
    status = runner.evaluate(
        fixture.deformer, fixture.weights, joint_buffer);
    REQUIRE(status);
    const auto* output = static_cast<const double*>(
        runner.output_positions().data());
    REQUIRE(output[0] == Catch::Approx(2.0));
}

TEST_CASE("orlrig forwards solver varyings into a deformer stage",
    "[orlrig][cpu][two_stage]")
{
    const auto program = make_two_stage_program();

    orlrig::TwoStageRunner runner(program);
    ORL::exec::OrlBuffer skeleton(
        orlrig::kJointOrlType, orlrig::kJointStride);
    ORL::exec::OrlBuffer target(
        orlrig::kPointOrlType, orlrig::kPointStride);
    ORL::exec::OrlBuffer bind_positions(
        orlrig::kPointOrlType, orlrig::kPointStride);
    ORL::exec::OrlBuffer output_positions(
        orlrig::kPointOrlType, orlrig::kPointStride);

    REQUIRE(skeleton.resize(1));
    REQUIRE(skeleton.write(0, orlrig::make_identity_joint()));
    REQUIRE(target.resize(1));
    REQUIRE(bind_positions.resize(1));
    REQUIRE(output_positions.resize(1));
    auto* target_data = static_cast<double*>(target.data());
    target_data[0] = 2.0;
    auto* bind_data = static_cast<double*>(bind_positions.data());
    bind_data[0] = 1.0;

    REQUIRE(runner.bind_varying("skeleton", skeleton, "skeleton", "pose"));
    REQUIRE(runner.solver_inputs().bind_buffer("target", target));
    REQUIRE(runner.solver_inputs().bind_int("joint_count", 1));
    REQUIRE(runner.deformer_inputs().bind_buffer(
        "bind_positions", bind_positions));
    REQUIRE(runner.deformer_inputs().bind_buffer(
        "output_positions", output_positions));
    REQUIRE(runner.deformer_inputs().bind_int("vertex_count", 1));
    REQUIRE(runner.deformer_inputs().bind_float("offset", 0.5));

    const auto status = runner.execute(1, 1);
    INFO("two-stage status: " << last_error(status));
    REQUIRE(status);
    REQUIRE(runner.solver_complete());

    orlrig::Joint solved{};
    REQUIRE(skeleton.read(0, &solved));
    REQUIRE(solved.translation[0] == Catch::Approx(2.0));

    const auto* output = static_cast<const double*>(
        static_cast<const ORL::exec::OrlBuffer&>(output_positions).data());
    REQUIRE(output[0] == Catch::Approx(3.5));
}

TEST_CASE("orlrig validates two-stage buffer signatures",
    "[orlrig][cpu][two_stage][validation]")
{
    orlrig::TwoStageRunner runner(make_two_stage_program());
    ORL::exec::OrlBuffer wrong_type(
        orlrig::kMatrixOrlType, orlrig::kMatrixStride);
    ORL::exec::OrlBuffer target(
        orlrig::kPointOrlType, orlrig::kPointStride);
    REQUIRE(wrong_type.resize(1));
    REQUIRE(target.resize(1));
    REQUIRE(runner.bind_varying("skeleton", wrong_type));
    REQUIRE(runner.solver_inputs().bind_buffer("target", target));
    REQUIRE(runner.solver_inputs().bind_int("joint_count", 1));

    const auto status = runner.execute_solver();
    REQUIRE_FALSE(status);
    REQUIRE_FALSE(status.errors.empty());
    REQUIRE(status.errors.front().find("expects type 'Joint'")
        != std::string::npos);
}

TEST_CASE("orlrig forwards device varyings between CUDA stages",
    "[orlrig][cuda][two_stage]")
{
    const auto program = make_two_stage_program();
    orlrig::TwoStageRunner runner(
        program, ORL::exec::Backend::Cuda);
    ORL::exec::OrlBuffer skeleton(
        orlrig::kJointOrlType, orlrig::kJointStride);
    ORL::exec::OrlBuffer target(
        orlrig::kPointOrlType, orlrig::kPointStride);
    ORL::exec::OrlBuffer bind_positions(
        orlrig::kPointOrlType, orlrig::kPointStride);
    ORL::exec::OrlBuffer output_positions(
        orlrig::kPointOrlType, orlrig::kPointStride);

    REQUIRE(skeleton.resize(1));
    REQUIRE(skeleton.write(0, orlrig::make_identity_joint()));
    REQUIRE(target.resize(1));
    REQUIRE(bind_positions.resize(1));
    REQUIRE(output_positions.resize(1));
    auto* target_data = static_cast<double*>(target.data());
    target_data[0] = 2.0;
    auto* bind_data = static_cast<double*>(bind_positions.data());
    bind_data[0] = 1.0;

    REQUIRE(runner.bind_varying("skeleton", skeleton, "skeleton", "pose"));
    REQUIRE(runner.solver_inputs().bind_buffer("target", target));
    REQUIRE(runner.solver_inputs().bind_int("joint_count", 1));
    REQUIRE(runner.deformer_inputs().bind_buffer(
        "bind_positions", bind_positions));
    REQUIRE(runner.deformer_inputs().bind_buffer(
        "output_positions", output_positions));
    REQUIRE(runner.deformer_inputs().bind_int("vertex_count", 1));
    REQUIRE(runner.deformer_inputs().bind_float("offset", 0.5));

    auto status = runner.execute_solver(1, true);
    if (!status) {
        WARN(last_error(status));
        return;
    }
    REQUIRE(runner.solver_complete());
    REQUIRE(runner.varying_device_buffer("skeleton").has_value());

    // The deformer consumes the solver-owned device allocation and reads its
    // output back only at this final verification boundary.
    status = runner.execute_deformer(1, false);
    if (!status) {
        WARN(last_error(status));
        return;
    }
    const auto* output = static_cast<const double*>(
        static_cast<const ORL::exec::OrlBuffer&>(output_positions).data());
    REQUIRE(output[0] == Catch::Approx(3.5));
}

TEST_CASE("orlrig runs auto-weight and two-bone solver on CPU",
    "[orlrig][cpu][rigging]")
{
    orlrig::MeshData mesh;
    mesh.positions = {
        glm::vec3{0.1f, 0.0f, 0.0f},
        glm::vec3{1.9f, 0.0f, 0.0f},
    };
    std::vector<orlrig::Joint> joints(3);
    joints[0] = orlrig::make_identity_joint();
    joints[1] = orlrig::make_identity_joint();
    joints[1].parent = 0;
    joints[1].translation[0] = 1.0;
    joints[2] = orlrig::make_identity_joint();
    joints[2].parent = 1;
    joints[2].translation[0] = 1.0;

    orlrig::WeightData weights;
    weights.weight_cnt = 2;
    orlrig::AutoWeightRunner auto_weight("closest_distance");
    auto status = auto_weight.run(mesh, joints, nullptr, weights, 4.0);
    REQUIRE(status);
    REQUIRE(weights.weights.count() == 4);

    const auto chain = orlrig::make_two_bone_chain(joints, 2);
    REQUIRE(chain.has_value());
    orlrig::SolverRunner solver(ORL::exec::Backend::Cpu);
    const auto target = orlrig::make_locator(glm::vec3{1.0f, 1.0f, 0.0f});
    const auto pole = orlrig::make_locator(glm::vec3{0.0f, 0.0f, 1.0f});
    status = solver.evaluate_two_bone(
        joints, chain->root, chain->mid, chain->end, target, pole);
    REQUIRE(status);

    ORL::exec::OrlBuffer shared_joints(
        orlrig::kJointOrlType, orlrig::kJointStride);
    REQUIRE(shared_joints.resize(joints.size()));
    for (std::size_t index = 0; index < joints.size(); ++index) {
        REQUIRE(shared_joints.write(index, joints[index]));
    }
    orlrig::SolverRunner shared_solver(ORL::exec::Backend::Cpu);
    status = shared_solver.evaluate_two_bone(
        shared_joints, chain->root, chain->mid, chain->end, target, pole);
    REQUIRE(status);
}

TEST_CASE("solver runner consumes a compiled hierarchy plan",
    "[orlrig][cpu][hierarchy][ik]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");

    auto mid_joint = orlrig::make_identity_joint();
    mid_joint.parent = 0;
    mid_joint.translation[0] = 1.0;
    const auto mid = store.create_joint("mid", mid_joint);

    auto end_joint = orlrig::make_identity_joint();
    end_joint.parent = 1;
    end_joint.translation[0] = 1.0;
    const auto end = store.create_joint("end", end_joint);
    const auto other_root = store.create_joint("other_root");

    const auto hierarchy = orlrig::compile_hierarchy_plan(store);
    REQUIRE(hierarchy);

    orlrig::SolverRunner solver(ORL::exec::Backend::Cpu);
    REQUIRE(solver.set_hierarchy_plan(*hierarchy.plan));
    const auto target = orlrig::make_locator(
        glm::vec3{1.0f, 1.0f, 0.0f});
    const auto pole = orlrig::make_locator(
        glm::vec3{0.0f, 0.0f, 1.0f});

    REQUIRE(solver.evaluate_two_bone(
        store, root, mid, end, target, pole));
    REQUIRE(store.joint(root)->rotation[3] != Catch::Approx(1.0));

    const auto invalid_chain = solver.evaluate_two_bone(
        store, root, other_root, end, target, pole);
    REQUIRE_FALSE(invalid_chain);
    REQUIRE_FALSE(invalid_chain.errors.empty());
    REQUIRE(invalid_chain.errors.front().find("compiled hierarchy")
        != std::string::npos);
}

TEST_CASE("two-bone solver keeps a near-collinear pole bend stable",
    "[orlrig][cpu][rigging][ik]")
{
    std::vector<orlrig::Joint> joints(3);
    joints[0] = orlrig::make_identity_joint();
    joints[1] = orlrig::make_identity_joint();
    joints[1].parent = 0;
    joints[1].translation[0] = 1.0;
    joints[2] = orlrig::make_identity_joint();
    joints[2].parent = 1;
    joints[2].translation[0] = 1.0;

    orlrig::SolverRunner solver(ORL::exec::Backend::Cpu);
    float previous_mid_z = 0.0f;
    bool have_previous = false;
    for (const float target_x : {2.0e-6f, -2.0e-6f, 2.0e-6f, -2.0e-6f}) {
        const auto target = orlrig::make_locator(
            glm::vec3{target_x, 1.5f, 0.0f});
        // Keep the pole on the reach axis. Its projection is intentionally
        // too small to define a reliable bend plane.
        const auto pole = orlrig::make_locator(
            glm::vec3{target_x * 2.0f, 3.0f, 0.0f});
        const auto status = solver.evaluate_two_bone(
            joints, 0, 1, 2, target, pole);
        REQUIRE(status);

        const float mid_z =
            glm::vec3{orlrig::joint_world_matrix(joints, 1)[3]}.z;
        if (have_previous) {
            REQUIRE(std::abs(mid_z - previous_mid_z) < 1.0e-5f);
        }
        previous_mid_z = mid_z;
        have_previous = true;
    }
}

TEST_CASE("orlrig auto-weight closest distance", "[orlrig][cpu][auto_weight]") {
    check_auto_weight_algorithm("closest_distance", false);
}

TEST_CASE("orlrig auto-weight closest hierarchy", "[orlrig][cpu][auto_weight]") {
    check_auto_weight_algorithm("closest_hierarchy", false);
}

TEST_CASE("orlrig auto-weight heat", "[orlrig][cpu][auto_weight]") {
    check_auto_weight_algorithm("heat", true);
}

TEST_CASE("orlrig auto-weight geodesic", "[orlrig][cpu][auto_weight]") {
    check_auto_weight_algorithm("geodesic", true);
}

TEST_CASE("orlrig exposes CUDA output for no-window tests",
    "[orlrig][cuda]")
{
    auto fixture = make_lbs_fixture();
    orlrig::LbsRunner runner(ORL::exec::Backend::Cuda);
    auto status = runner.capture_bind(
        fixture.deformer, fixture.mesh, fixture.joints);
    if (!status) {
        WARN(last_error(status));
        return;
    }

    fixture.joints[0].translation[0] = 1.0;
    status = runner.evaluate(
        fixture.deformer, fixture.weights, fixture.joints, true);
    if (!status) {
        WARN(last_error(status));
        return;
    }
    REQUIRE(runner.output_device().has_value());

    status = runner.readback();
    if (!status) {
        WARN(last_error(status));
        return;
    }
    const auto* output = static_cast<const double*>(
        runner.output_positions().data());
    REQUIRE(output[0] == Catch::Approx(2.0));
}

TEST_CASE("evaluation plan resolves typed solver footprints and dirty levels",
    "[orlrig][partial][plan]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");
    auto mid_value = orlrig::make_identity_joint();
    mid_value.parent = 0;
    mid_value.translation[0] = 1.0;
    const auto mid = store.create_joint("mid", mid_value);
    auto end_value = orlrig::make_identity_joint();
    end_value.parent = 1;
    end_value.translation[0] = 1.0;
    const auto end = store.create_joint("end", end_value);
    const auto target = store.create_locator("target");
    const auto pole = store.create_locator("pole");

    orlgraph::NodeRegistry registry;
    REQUIRE(orlrig::register_rig_node_definitions(registry));
    orlgraph::GraphModule graph;
    graph.module_id = "partial.ik";

    const auto add_find = [&](std::string id, std::string definition,
        std::string name) {
        orlgraph::NodeInstance node;
        node.id = orlgraph::StableId{std::move(id)};
        node.definition = orlgraph::StableId{std::move(definition)};
        node.name = node.id.value;
        node.parameter_values.emplace("name",
            orlgraph::ConstantValue{
                orlgraph::LogicalType::string(), std::move(name)});
        return graph.add_node(std::move(node));
    };
    REQUIRE(add_find("find.root", "orlrig.input.find_joint", "root"));
    REQUIRE(add_find("find.mid", "orlrig.input.find_joint", "mid"));
    REQUIRE(add_find("find.end", "orlrig.input.find_joint", "end"));
    REQUIRE(add_find("find.target", "orlrig.input.find_locator", "target"));
    REQUIRE(add_find("find.pole", "orlrig.input.find_locator", "pole"));

    orlgraph::NodeInstance solver;
    solver.id = orlgraph::StableId{"ik"};
    solver.definition = orlgraph::StableId{"orlrig.solver.ik_two_bone"};
    solver.name = "ik";
    REQUIRE(graph.add_node(std::move(solver)));

    const auto connect = [&](std::string source_node,
        std::string source_port, std::string destination_port) {
        orlgraph::Connection connection;
        connection.source = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(source_node)},
            orlgraph::StableId{std::move(source_port)});
        connection.destination = orlgraph::Endpoint::node_port(
            orlgraph::StableId{"ik"},
            orlgraph::StableId{std::move(destination_port)});
        return graph.add_connection(std::move(connection));
    };
    REQUIRE(connect("find.root", "handle", "root"));
    REQUIRE(connect("find.mid", "handle", "mid"));
    REQUIRE(connect("find.end", "handle", "end"));
    REQUIRE(connect("find.target", "handle", "target"));
    REQUIRE(connect("find.pole", "handle", "pole"));

    const auto hierarchy = orlrig::compile_hierarchy_plan(store);
    REQUIRE(hierarchy);
    const auto compiled = orlrig::compile_evaluation_plan(
        graph, registry, store, *hierarchy.plan);
    REQUIRE(compiled);
    REQUIRE(compiled.plan->regions.size() == 1);
    const auto& region = compiled.plan->regions.front();
    REQUIRE_FALSE(region.global);
    REQUIRE(region.read_joints.size() == 3);
    REQUIRE(region.affected_joints.size() == 3);
    REQUIRE(std::find(region.read_joints.begin(), region.read_joints.end(),
        root) != region.read_joints.end());
    REQUIRE(std::find(region.read_joints.begin(), region.read_joints.end(),
        mid) != region.read_joints.end());
    REQUIRE(std::find(region.read_joints.begin(), region.read_joints.end(),
        end) != region.read_joints.end());
    const std::vector<orlrig::ComponentId> expected_locators = {
        target, pole};
    REQUIRE(region.read_locators == expected_locators);

    orlrig::DirtyInputs clean;
    const auto clean_dispatch =
        orlrig::build_dynamic_dispatch_plan(*compiled.plan, clean);
    REQUIRE(clean_dispatch.region_indices.empty());

    orlrig::DirtyInputs locator_dirty;
    locator_dirty.locators = {target};
    const auto partial_dispatch =
        orlrig::build_dynamic_dispatch_plan(
            *compiled.plan, locator_dirty, 1.0);
    REQUIRE_FALSE(partial_dispatch.full_evaluation);
    REQUIRE(partial_dispatch.region_indices.size() == 1);
    std::atomic<int> calls{0};
    REQUIRE(orlrig::dispatch_cpu_levels(
        *compiled.plan, partial_dispatch,
        [&](const orlrig::SolverRegion&) {
            ++calls;
            return orlrig::RunnerStatus{true, {}};
        }));
    REQUIRE(calls.load() == 1);

    const auto cuda_dispatch = orlrig::build_cuda_dispatch_plan(
        *compiled.plan, locator_dirty, 1.0);
    REQUIRE(cuda_dispatch.full_evaluation);
    const auto packed_dispatch =
        orlrig::pack_dynamic_dispatch_plan(cuda_dispatch);
    REQUIRE(packed_dispatch.context.full_evaluation == 1);
    REQUIRE(packed_dispatch.context.range_count == 1);
    REQUIRE(packed_dispatch.data.size() == 3);

    orlrig::DirtyInputs full;
    full.full_evaluation = true;
    const auto full_dispatch =
        orlrig::build_dynamic_dispatch_plan(*compiled.plan, full);
    REQUIRE(full_dispatch.full_evaluation);
    REQUIRE(full_dispatch.region_indices.size() == 1);

    std::vector<glm::mat4> worlds;
    REQUIRE(orlrig::compute_world_matrices_parallel(
        *hierarchy.plan, store, &worlds));
    REQUIRE(worlds.size() == 3);
    REQUIRE(glm::vec3{worlds[2][3]}.x == Catch::Approx(2.0f));
}

#if 0 // Locator index constraints await typed constraint migration.
TEST_CASE("evaluation plan orders a locator writer before its reader",
    "[orlrig][partial][order]")
{
    orlrig::ComponentStore store;
    const auto root = store.create_joint("root");
    auto mid_value = orlrig::make_identity_joint();
    mid_value.parent = 0;
    const auto mid = store.create_joint("mid", mid_value);
    auto end_value = orlrig::make_identity_joint();
    end_value.parent = 1;
    store.create_joint("end", end_value);
    store.create_locator("target");
    store.create_locator("pole");
    store.create_locator("aim_at");
    (void)root;
    (void)mid;

    orlgraph::NodeRegistry registry;
    REQUIRE(orlrig::register_rig_node_definitions(registry));
    orlgraph::GraphModule graph;
    graph.module_id = "partial.order";

    const auto add_find = [&](std::string id, std::string definition,
        std::string name) {
        orlgraph::NodeInstance node;
        node.id = orlgraph::StableId{id};
        node.definition = orlgraph::StableId{std::move(definition)};
        node.name = id;
        node.parameter_values.emplace("name",
            orlgraph::ConstantValue{
                orlgraph::LogicalType::string(), std::move(name)});
        return graph.add_node(std::move(node));
    };
    const auto connect = [&](std::string source_node,
        std::string source_port, std::string destination_node,
        std::string destination_port) {
        orlgraph::Connection connection;
        connection.source = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(source_node)},
            orlgraph::StableId{std::move(source_port)});
        connection.destination = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(destination_node)},
            orlgraph::StableId{std::move(destination_port)});
        return graph.add_connection(std::move(connection));
    };
    REQUIRE(add_find("find.root", "orlrig.input.find_joint", "root"));
    REQUIRE(add_find("find.mid", "orlrig.input.find_joint", "mid"));
    REQUIRE(add_find("find.end", "orlrig.input.find_joint", "end"));
    REQUIRE(add_find("find.target", "orlrig.input.find_locator", "target"));
    REQUIRE(add_find("find.pole", "orlrig.input.find_locator", "pole"));
    REQUIRE(add_find("find.aim", "orlrig.input.find_locator", "aim_at"));

    orlgraph::NodeInstance aim;
    aim.id = orlgraph::StableId{"aim"};
    aim.definition = orlgraph::StableId{"orlrig.constraint.aim_locator"};
    aim.name = "aim";
    REQUIRE(graph.add_node(std::move(aim)));
    orlgraph::NodeInstance solver;
    solver.id = orlgraph::StableId{"ik"};
    solver.definition = orlgraph::StableId{"orlrig.solver.ik_two_bone"};
    solver.name = "ik";
    REQUIRE(graph.add_node(std::move(solver)));

    REQUIRE(connect("find.aim", "index", "aim", "target_index"));
    REQUIRE(connect("find.target", "index", "aim", "subject_index"));
    REQUIRE(connect("find.root", "handle", "ik", "root"));
    REQUIRE(connect("find.mid", "handle", "ik", "mid"));
    REQUIRE(connect("find.end", "handle", "ik", "end"));
    REQUIRE(connect("find.target", "index", "ik", "target_index"));
    REQUIRE(connect("find.pole", "index", "ik", "pole_index"));

    const auto hierarchy = orlrig::compile_hierarchy_plan(store);
    REQUIRE(hierarchy);
    const auto compiled = orlrig::compile_evaluation_plan(
        graph, registry, store, *hierarchy.plan);
    REQUIRE(compiled);
    std::size_t aim_level = compiled.plan->batches.size();
    std::size_t ik_level = compiled.plan->batches.size();
    for (std::size_t level = 0; level < compiled.plan->batches.size(); ++level) {
        for (const auto& id : compiled.plan->batches[level]) {
            if (id.value == "aim") {
                aim_level = level;
            }
            if (id.value == "ik") {
                ik_level = level;
            }
        }
    }
    REQUIRE(aim_level < ik_level);
}

#endif

TEST_CASE("evaluation plan rejects two solvers writing the same joints",
    "[orlrig][partial][order]")
{
    orlrig::ComponentStore store;
    store.create_joint("root");
    auto mid_value = orlrig::make_identity_joint();
    mid_value.parent = 0;
    store.create_joint("mid", mid_value);
    auto end_value = orlrig::make_identity_joint();
    end_value.parent = 1;
    store.create_joint("end", end_value);
    store.create_locator("target");
    store.create_locator("pole");

    orlgraph::NodeRegistry registry;
    REQUIRE(orlrig::register_rig_node_definitions(registry));
    orlgraph::GraphModule graph;
    graph.module_id = "partial.duplicate";

    const auto add_find = [&](std::string id, std::string definition,
        std::string name) {
        orlgraph::NodeInstance node;
        node.id = orlgraph::StableId{id};
        node.definition = orlgraph::StableId{std::move(definition)};
        node.name = id;
        node.parameter_values.emplace("name",
            orlgraph::ConstantValue{
                orlgraph::LogicalType::string(), std::move(name)});
        return graph.add_node(std::move(node));
    };
    const auto connect = [&](std::string source_node,
        std::string source_port, std::string destination_node,
        std::string destination_port) {
        orlgraph::Connection connection;
        connection.source = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(source_node)},
            orlgraph::StableId{std::move(source_port)});
        connection.destination = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(destination_node)},
            orlgraph::StableId{std::move(destination_port)});
        return graph.add_connection(std::move(connection));
    };
    REQUIRE(add_find("find.root", "orlrig.input.find_joint", "root"));
    REQUIRE(add_find("find.mid", "orlrig.input.find_joint", "mid"));
    REQUIRE(add_find("find.end", "orlrig.input.find_joint", "end"));
    REQUIRE(add_find("find.target", "orlrig.input.find_locator", "target"));
    REQUIRE(add_find("find.pole", "orlrig.input.find_locator", "pole"));
    for (const char* id : {"ik_a", "ik_b"}) {
        orlgraph::NodeInstance solver;
        solver.id = orlgraph::StableId{id};
        solver.definition = orlgraph::StableId{"orlrig.solver.ik_two_bone"};
        solver.name = id;
        REQUIRE(graph.add_node(std::move(solver)));
        REQUIRE(connect("find.root", "handle", id, "root"));
        REQUIRE(connect("find.mid", "handle", id, "mid"));
        REQUIRE(connect("find.end", "handle", id, "end"));
        REQUIRE(connect("find.target", "handle", id, "target"));
        REQUIRE(connect("find.pole", "handle", id, "pole"));
    }

    const auto hierarchy = orlrig::compile_hierarchy_plan(store);
    REQUIRE(hierarchy);
    const auto compiled = orlrig::compile_evaluation_plan(
        graph, registry, store, *hierarchy.plan);
    REQUIRE_FALSE(compiled);
    REQUIRE_FALSE(compiled.plan.has_value());
    REQUIRE(std::any_of(compiled.errors.begin(), compiled.errors.end(),
        [](const std::string& error) {
            return error.find("Competing writers") != std::string::npos;
        }));
}

#if 0 // Locator index constraints await typed constraint migration.
TEST_CASE("evaluation plan rejects a locator dependency cycle",
    "[orlrig][partial][order]")
{
    orlrig::ComponentStore store;
    store.create_locator("locator_a");
    store.create_locator("locator_b");

    orlgraph::NodeRegistry registry;
    REQUIRE(orlrig::register_rig_node_definitions(registry));
    orlgraph::GraphModule graph;
    graph.module_id = "partial.cycle";

    const auto add_find = [&](std::string id, std::string name) {
        orlgraph::NodeInstance node;
        node.id = orlgraph::StableId{id};
        node.definition = orlgraph::StableId{"orlrig.input.find_locator"};
        node.name = id;
        node.parameter_values.emplace("name",
            orlgraph::ConstantValue{
                orlgraph::LogicalType::string(), std::move(name)});
        return graph.add_node(std::move(node));
    };
    const auto connect = [&](std::string source_node,
        std::string destination_node, std::string destination_port) {
        orlgraph::Connection connection;
        connection.source = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(source_node)},
            orlgraph::StableId{"index"});
        connection.destination = orlgraph::Endpoint::node_port(
            orlgraph::StableId{std::move(destination_node)},
            orlgraph::StableId{std::move(destination_port)});
        return graph.add_connection(std::move(connection));
    };
    REQUIRE(add_find("find.a", "locator_a"));
    REQUIRE(add_find("find.b", "locator_b"));
    for (const char* id : {"aim_a", "aim_b"}) {
        orlgraph::NodeInstance aim;
        aim.id = orlgraph::StableId{id};
        aim.definition = orlgraph::StableId{"orlrig.constraint.aim_locator"};
        aim.name = id;
        REQUIRE(graph.add_node(std::move(aim)));
    }
    REQUIRE(connect("find.b", "aim_a", "target_index"));
    REQUIRE(connect("find.a", "aim_a", "subject_index"));
    REQUIRE(connect("find.a", "aim_b", "target_index"));
    REQUIRE(connect("find.b", "aim_b", "subject_index"));

    const auto hierarchy = orlrig::compile_hierarchy_plan(store);
    REQUIRE(hierarchy);
    const auto compiled = orlrig::compile_evaluation_plan(
        graph, registry, store, *hierarchy.plan);
    REQUIRE_FALSE(compiled);
    REQUIRE_FALSE(compiled.plan.has_value());
    REQUIRE(std::any_of(compiled.errors.begin(), compiled.errors.end(),
        [](const std::string& error) {
            return error.find("Evaluation dependency cycle")
                != std::string::npos;
        }));
}

#endif

TEST_CASE("opaque solver nodes force conservative full evaluation",
    "[orlrig][partial][fallback]")
{
    orlrig::ComponentStore store;
    store.create_joint("root");
    const auto hierarchy = orlrig::compile_hierarchy_plan(store);
    REQUIRE(hierarchy);

    orlgraph::NodeRegistry registry;
    orlgraph::NodeDefinition definition;
    definition.id = orlgraph::StableId{"opaque.solver"};
    definition.qualified_name = "opaque.solver";
    definition.operation = "solver";
    REQUIRE(registry.register_definition(std::move(definition)));

    orlgraph::GraphModule graph;
    graph.module_id = "opaque";
    orlgraph::NodeInstance node;
    node.id = orlgraph::StableId{"solver"};
    node.definition = orlgraph::StableId{"opaque.solver"};
    REQUIRE(graph.add_node(std::move(node)));

    const auto plan = orlrig::compile_evaluation_plan(
        graph, registry, store, *hierarchy.plan);
    REQUIRE(plan);
    REQUIRE(plan.plan->full_evaluation_required);
    REQUIRE(plan.plan->regions.size() == 1);
    REQUIRE(plan.plan->regions.front().global);
}

#endif
