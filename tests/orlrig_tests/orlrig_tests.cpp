#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

#include "orlrig/abi.hpp"
#include "orlrig/component_store.hpp"
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

#endif
