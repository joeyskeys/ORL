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
#include "orlrig/runners.hpp"

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

} // namespace

TEST_CASE("orlrig preserves host and stdlib buffer ABI", "[orlrig][abi]") {
    REQUIRE(sizeof(orlrig::Joint) == orlrig::kJointStride);
    REQUIRE(sizeof(orlrig::Weight) == orlrig::kWeightStride);

    orlrig::Controller controller = orlrig::make_controller(
        glm::vec3{2.0f, 3.0f, 4.0f});
    double packed[16] = {};
    orlrig::pack_xform(controller, packed);
    REQUIRE(packed[3] == Catch::Approx(2.0));
    REQUIRE(packed[7] == Catch::Approx(3.0));
    REQUIRE(packed[11] == Catch::Approx(4.0));
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
    const auto target = orlrig::make_controller(glm::vec3{1.0f, 1.0f, 0.0f});
    const auto pole = orlrig::make_controller(glm::vec3{0.0f, 0.0f, 1.0f});
    status = solver.evaluate_two_bone(
        joints, chain->root, chain->mid, chain->end, target, pole);
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
