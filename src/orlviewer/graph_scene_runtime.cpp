#include "graph_scene_runtime.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "asset_mgr/drawable_mgr.h"
#include "comps/joint.hpp"
#include "concepts/mesh.h"
#include "orlrig/graph_resources.hpp"
#include "runtime_config.hpp"
#include "vk_ins/context.hpp"

#if ORL_USE_QT6
#include <QMessageBox>
#include <QString>
#endif

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

namespace ORL
{
namespace
{

constexpr std::string_view kBindOperation = "bind";
constexpr std::string_view kCaptureBindRuntime =
    "orlrig.deformer.lbs.capture_bind";
constexpr std::string_view kEvaluateRuntime =
    "orlrig.deformer.lbs.evaluate";

exec::Backend backend_from_config() {
    return runtime_config.device == ComputeDevice::Gpu
        ? exec::Backend::Cuda
        : exec::Backend::Cpu;
}

bool known_type(std::string_view name) {
    return name == "lbs";
}

void print_runner_errors(const std::vector<std::string>& errors) {
    for (const auto& error : errors) {
        std::cerr << "Deformer: " << error << '\n';
    }
}

int vertex_float_offset(const vkkk::Mesh& mesh) {
    int packed = 0;
    for (const auto component : mesh.comps) {
        if (component == vkkk::VERTEX) {
            return packed;
        }
        packed += static_cast<int>(vkkk::comp_sizes[component]);
    }
    return -1;
}

bool extract_mesh(orlrig::MeshData& destination,
    const vkkk::Mesh& source,
    const glm::mat4& model)
{
    const int offset = vertex_float_offset(source);
    if (offset < 0 || source.vbuf == nullptr || source.vcnt == 0) {
        return false;
    }
    destination.positions.resize(source.vcnt);
    destination.model = model;
    for (std::uint32_t vertex = 0; vertex < source.vcnt; ++vertex) {
        const float* source_position =
            source.vbuf + vertex * source.comp_size + offset;
        destination.positions[vertex] = {
            source_position[0], source_position[1], source_position[2]};
    }
    if (source.ibuf != nullptr && source.icnt != 0) {
        destination.indices.assign(source.ibuf, source.ibuf + source.icnt * 3);
    }
    return true;
}

bool write_positions(vkkk::Mesh& mesh,
    const exec::OrlBuffer& positions,
    const glm::mat4& bind_model)
{
    const int offset = vertex_float_offset(mesh);
    if (offset < 0 || mesh.vbuf == nullptr || positions.count() < mesh.vcnt) {
        return false;
    }
    const glm::mat4 to_object = glm::inverse(bind_model);
    const auto* source = static_cast<const double*>(positions.data());
    for (std::uint32_t vertex = 0; vertex < mesh.vcnt; ++vertex) {
        float* destination = mesh.vbuf + vertex * mesh.comp_size + offset;
        const glm::vec4 world{
            static_cast<float>(source[vertex * 4 + 0]),
            static_cast<float>(source[vertex * 4 + 1]),
            static_cast<float>(source[vertex * 4 + 2]),
            1.0f};
        const glm::vec4 local = to_object * world;
        destination[0] = local.x;
        destination[1] = local.y;
        destination[2] = local.z;
    }
    return true;
}

void dump_bind_snapshot(const std::vector<orlviewer::Joint>& joints,
    const exec::OrlBuffer& inverse_binds)
{
    const auto* matrices = static_cast<const double*>(inverse_binds.data());
    std::ofstream output("orl_debug_bind.txt", std::ios::trunc);
    if (!output) {
        std::cerr << "Deformer: could not write orl_debug_bind.txt\n";
        return;
    }
    output << "# joint parent selected tx ty tz cpp_world_xyz inv_row_tx ty tz\n";
    std::cout << "Deformer: bind snapshot\n";
    for (std::size_t index = 0; index < joints.size(); ++index) {
        const auto& joint = joints[index];
        const glm::vec3 world{
            orlviewer::joint_world_matrix(joints,
                static_cast<std::int64_t>(index))[3]};
        double inverse_tx = 0.0;
        double inverse_ty = 0.0;
        double inverse_tz = 0.0;
        if (matrices != nullptr && index < inverse_binds.count()) {
            const auto* matrix = matrices + index * 16;
            inverse_tx = matrix[3];
            inverse_ty = matrix[7];
            inverse_tz = matrix[11];
        }
        output << index << '\t' << joint.parent << '\t' << joint.selected << '\t'
            << joint.translation[0] << '\t' << joint.translation[1] << '\t'
            << joint.translation[2] << '\t' << world.x << '\t' << world.y << '\t'
            << world.z << '\t' << inverse_tx << '\t' << inverse_ty << '\t'
            << inverse_tz << '\n';
    }
    std::cout << "Deformer: wrote orl_debug_bind.txt\n";
}

double max_position_delta(const exec::OrlBuffer& bind,
    const exec::OrlBuffer& posed)
{
    const auto count = std::min(bind.count(), posed.count());
    const auto* first = static_cast<const double*>(bind.data());
    const auto* second = static_cast<const double*>(posed.data());
    if (first == nullptr || second == nullptr || count == 0) {
        return 0.0;
    }
    double maximum = 0.0;
    for (std::size_t vertex = 0; vertex < count; ++vertex) {
        const double dx = first[vertex * 4 + 0] - second[vertex * 4 + 0];
        const double dy = first[vertex * 4 + 1] - second[vertex * 4 + 1];
        const double dz = first[vertex * 4 + 2] - second[vertex * 4 + 2];
        maximum = std::max(maximum, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    return maximum;
}

void dump_posed(const exec::OrlBuffer& bind,
    const exec::OrlBuffer& posed,
    const vkkk::Mesh& mesh)
{
    const auto count = std::min(bind.count(), posed.count());
    const auto* first = static_cast<const double*>(bind.data());
    const auto* second = static_cast<const double*>(posed.data());
    const int offset = vertex_float_offset(mesh);
    std::ofstream output("orl_debug_posed.txt", std::ios::trunc);
    if (!output || first == nullptr || second == nullptr) {
        std::cerr << "Deformer: could not write orl_debug_posed.txt\n";
        return;
    }
    output << "# vertex\tbind_xyz\tposed_xyz\tcpu_vbuf_xyz\n";
    const std::size_t preview = std::min<std::size_t>(count, 8);
    for (std::size_t vertex = 0; vertex < preview; ++vertex) {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        if (mesh.vbuf != nullptr && offset >= 0) {
            const float* source = mesh.vbuf + vertex * mesh.comp_size + offset;
            x = source[0];
            y = source[1];
            z = source[2];
        }
        output << vertex << '\t'
            << first[vertex * 4 + 0] << '\t' << first[vertex * 4 + 1] << '\t'
            << first[vertex * 4 + 2] << '\t' << second[vertex * 4 + 0] << '\t'
            << second[vertex * 4 + 1] << '\t' << second[vertex * 4 + 2] << '\t'
            << x << '\t' << y << '\t' << z << '\n';
    }
    std::cout << "Deformer: wrote orl_debug_posed.txt\n";
}

std::string selected_scene_object_name(const Selection& selection) {
    for (const auto& ref : selection.refs()) {
        if (ref.kind == SelectionRef::Kind::SceneObject) {
            return ref.object_name;
        }
    }
    return {};
}

std::string constant_fingerprint(const orlgraph::ConstantValue& value) {
    std::ostringstream stream;
    stream << value.value.index();
    if (const auto* text = std::get_if<std::string>(&value.value)) {
        stream << ':' << *text;
    } else if (const auto* integer = std::get_if<std::int64_t>(
                   &value.value)) {
        stream << ':' << *integer;
    } else if (const auto* floating = std::get_if<double>(&value.value)) {
        stream << ':' << *floating;
    } else if (const auto* boolean = std::get_if<bool>(&value.value)) {
        stream << ':' << (*boolean ? 1 : 0);
    }
    return stream.str();
}

std::string runtime_identifier(std::string_view value) {
    std::string result;
    result.reserve(value.size());
    for (const char character : value) {
        result.push_back(
            std::isalnum(static_cast<unsigned char>(character))
                || character == '_'
            ? character : '_');
    }
    return result;
}

} // namespace

GraphSceneRuntime::GraphSceneRuntime(SceneGraphContext& graph_context,
    const Selection& selection, ComponentId deformer_id,
    ComponentId weight_id, std::optional<orlgraph::GraphStage> stage)
    : graph_context_(graph_context)
    , selection_(selection)
    , deformer_id(deformer_id)
    , weight_id(weight_id)
    , stage_(stage)
    , runner_(backend_from_config())
{
    register_runtime_adapters();
}

orlgraph::GraphModule& GraphSceneRuntime::active_graph() {
    return stage_.has_value()
        ? graph_context_.stage_graph(*stage_)
        : graph_context_.graph();
}

const orlgraph::GraphModule& GraphSceneRuntime::active_graph() const {
    return stage_.has_value()
        ? graph_context_.stage_graph(*stage_)
        : graph_context_.graph();
}

bool GraphSceneRuntime::has_active_graph_content() const {
    const auto& graph = active_graph();
    return !graph.nodes().empty()
        || !graph.inputs().empty()
        || !graph.outputs().empty();
}

void GraphSceneRuntime::register_runtime_adapters() {
    runtime_adapters_.emplace(
        std::string{kCaptureBindRuntime},
        &GraphSceneRuntime::execute_lbs_capture_adapter);
    runtime_adapters_.emplace(
        std::string{kEvaluateRuntime},
        &GraphSceneRuntime::execute_lbs_evaluate_adapter);
    runtime_adapters_.emplace(
        std::string{orlrig::kComputedJointsNodeDefinition},
        &GraphSceneRuntime::execute_computed_joints_adapter);
    for (const std::string_view runtime_name : {
             std::string_view{"orlrig.input.find_joint"},
             std::string_view{"orlrig.input.find_locator"},
             std::string_view{"orlrig.input.find_mesh"}})
    {
        runtime_adapters_.emplace(
            std::string{runtime_name},
            &GraphSceneRuntime::execute_scene_input_adapter);
    }
}

bool GraphSceneRuntime::set_type(std::string_view name) {
    if (!known_type(name)) {
        std::cerr << "Deformer: unknown type '" << name << "'\n";
        return false;
    }
    type_name = std::string{name};
    if (auto* deformer = graph_context_.components().deformer(deformer_id)) {
        deformer->type = type_name;
        deformer->bound = false;
    }
    return true;
}

void GraphSceneRuntime::set_mesh(std::string name) {
    if (auto* deformer = graph_context_.components().deformer(deformer_id)) {
        deformer->mesh_name = std::move(name);
        deformer->bound = false;
    }
}

void GraphSceneRuntime::request_bind()
{
    graph_context_.request_operation(std::string{kBindOperation});
}

void GraphSceneRuntime::unbind() {
    while (graph_context_.take_operation(kBindOperation)) {
    }
    if (auto* deformer = graph_context_.components().deformer(deformer_id)) {
        deformer->bound = false;
        deformer->mesh_name.clear();
        deformer->bind_positions.clear();
        deformer->inverse_binds.clear();
        deformer->bind_model = glm::mat4{1.0f};
    }
    execution_plan_ready_ = false;
    execution_plan_.clear();
    orl_segments_.clear();
    graph_active_ = false;
    logged_rest = false;
    logged_move = false;
}

void GraphSceneRuntime::on_update(vkkk::Context& context) {
    const bool solver_stage = stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver;
    if (!runtime_config.evaluate_orl) {
        graph_context_.scene_inputs().set_cuda_evaluation(false);
        if (solver_stage) {
            graph_context_.clear_computed_joints_device();
        }
        return;
    }

    if (solver_stage) {
        std::string hierarchy_error;
        if (!graph_context_.ensure_hierarchy_plan(&hierarchy_error)) {
            report_compile_error("Solver: " + hierarchy_error);
            graph_context_.clear_computed_joints_device();
            return;
        }
    }

    const bool graph_invalidated =
        observed_graph_edit_revision_
            != graph_context_.graph_edit_revision()
        || observed_evaluation_revision_
            != graph_context_.evaluation_revision();
    if (graph_invalidated) {
        observed_graph_edit_revision_ =
            graph_context_.graph_edit_revision();
        observed_evaluation_revision_ =
            graph_context_.evaluation_revision();
        execution_plan_ready_ = false;
        execution_plan_.clear();
        orl_segments_.clear();
        graph_active_ = true;
    }

    if (solver_stage) {
        // Keep the previous computed-joints device view alive when a partial
        // frame has no dirty solver region. A solver execution replaces this
        // view when it actually dispatches.
        const auto& graph = active_graph();
        if (graph.nodes().empty()
            && graph.inputs().empty()
            && graph.outputs().empty())
        {
            graph_active_ = false;
            return;
        }
        std::string input_error;
        if (!graph_context_.components().apply_controller_inputs(
                &input_error))
        {
            std::cerr << "Solver: controller input application failed: "
                      << input_error << '\n';
            graph_active_ = false;
            return;
        }
        if (graph_active_ && !dispatch_graph(context, false)) {
            graph_active_ = false;
        }
        return;
    }

    if (graph_context_.take_operation(kBindOperation)) {
        auto* deformer = graph_context_.components().deformer(deformer_id);
        OrlEvaluationPause pause;
        const bool setup_ok = setup(context);
        pause.resume();
        if (setup_ok) {
            observed_graph_edit_revision_ =
                graph_context_.graph_edit_revision();
            observed_evaluation_revision_ =
                graph_context_.evaluation_revision();
        }
        if (!setup_ok
            || !dispatch_graph(context, true))
        {
            if (deformer != nullptr) {
                deformer->bound = false;
            }
            graph_active_ = false;
        } else {
            graph_active_ = true;
        }
        return;
    }

    if (graph_invalidated && has_lbs_nodes()) {
        auto* deformer = graph_context_.components().deformer(deformer_id);
        if (deformer == nullptr || !deformer->bound) {
            OrlEvaluationPause pause;
            const bool setup_ok = setup(context);
            pause.resume();
            if (setup_ok) {
                observed_graph_edit_revision_ =
                    graph_context_.graph_edit_revision();
                observed_evaluation_revision_ =
                    graph_context_.evaluation_revision();
            }
            if (!setup_ok
                || !dispatch_graph(context, true))
            {
                if (deformer != nullptr) {
                    deformer->bound = false;
                }
                graph_active_ = false;
            } else {
                graph_active_ = true;
            }
            return;
        }
    }

    const auto* deformer = graph_context_.components().deformer(deformer_id);
    const bool should_evaluate = graph_active_
        || (deformer != nullptr && deformer->bound);
    if (should_evaluate && !dispatch_graph(context, false))
    {
        graph_active_ = false;
        if (auto* active_deformer =
                graph_context_.components().deformer(deformer_id)) {
            active_deformer->bound = false;
        }
    }
}

bool GraphSceneRuntime::prepare_lbs_inputs(std::string* error)
{
    const std::string object_name = selected_scene_object_name(selection_);
    const auto* weight = graph_context_.components().weight(weight_id);
    const auto* weight_meta = graph_context_.components().find(weight_id);
    if (object_name.empty()) {
        if (error != nullptr) {
            *error = "No scene object is selected for the graph";
        }
        return false;
    }
    if (weight == nullptr || weight_meta == nullptr
        || weight->weights.count() == 0)
    {
        if (error != nullptr) {
            *error = "The graph has no populated weight input";
        }
        return false;
    }

    graph_context_.refresh_scene_inputs();
    graph_context_.clear_input_mappings();
    if (!graph_context_.map_input_by_binding(
            "bind_positions",
            orlrig::scene_mesh_positions_binding(object_name), error)
        || !graph_context_.map_input_by_binding(
            "weights",
            orlrig::scene_weight_buffer_binding(weight_meta->name), error))
    {
        return false;
    }

    for (const auto& [_, input] : active_graph().inputs()) {
        exec::GraphInputBinding binding;
        if (!graph_context_.resolve_graph_input(input, binding, error)) {
            return false;
        }
    }
    return true;
}

bool GraphSceneRuntime::ensure_lbs_graph()
{
    if (stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver) {
        return true;
    }
    const auto& graph = active_graph();
    if (!graph.nodes().empty()
        || !graph.inputs().empty()
        || !graph.outputs().empty())
    {
        return true;
    }

    // Keep the editor blank on startup, but make the first bind request
    // useful without requiring users to manually rebuild the standard LBS
    // pipeline before they can inspect or save it.
    auto lbs_graph = orlrig::make_lbs_graph();
    if (graph_context_.has_stage_graphs()) {
        auto solver_graph = graph_context_.stage_graph(
            orlgraph::GraphStage::Solver);
        graph_context_.set_stage_graphs(
            std::move(solver_graph),
            std::move(lbs_graph.module),
            std::move(lbs_graph.registry));
    } else {
        graph_context_.set_graph(
            std::move(lbs_graph.module), std::move(lbs_graph.registry));
    }
    return true;
}

bool GraphSceneRuntime::has_lbs_nodes() const
{
    for (const auto& [_, instance] : active_graph().nodes()) {
        const auto* definition =
            graph_context_.registry().find(instance.definition);
        if (definition == nullptr) {
            continue;
        }
        const auto runtime_name = definition->implementation.runtime_name;
        if (runtime_name == kCaptureBindRuntime
            || runtime_name == kEvaluateRuntime)
        {
            return true;
        }
    }
    return false;
}

bool GraphSceneRuntime::has_orl_nodes() const
{
    for (const auto& [_, instance] : active_graph().nodes()) {
        const auto* definition =
            graph_context_.registry().find(instance.definition);
        if (definition != nullptr
            && definition->implementation.kind
                == orlgraph::ImplementationKind::OrlFunction)
        {
            return true;
        }
    }
    return false;
}

bool GraphSceneRuntime::setup(vkkk::Context& context) {
    if (!ensure_lbs_graph()) {
        return false;
    }
    if (!has_lbs_nodes()) {
        graph_context_.refresh_scene_inputs();
        return true;
    }

    auto* deformer = graph_context_.components().deformer(deformer_id);
    const auto* weight = graph_context_.components().weight(weight_id);
    if (deformer == nullptr || weight == nullptr) {
        std::cerr << "Deformer: missing deformer or weight component\n";
        return false;
    }
    std::string error;
    if (!prepare_lbs_inputs(&error)) {
        std::cerr << "Deformer: graph input setup failed: " << error << '\n';
        return false;
    }

    std::string mesh_name = selection_.selected_mesh_name();
    if (mesh_name.empty()) {
        mesh_name = deformer->mesh_name;
    }
    if (mesh_name.empty() || !selection_.has_selected_joint()) {
        std::cerr << "Deformer: select a mesh and joints first\n";
        return false;
    }
    auto& scene = graph_context_.scene();
    if (scene.drawable_mgr == nullptr) {
        std::cerr << "Deformer: no mesh loaded\n";
        return false;
    }
    const auto* mesh = scene.drawable_mgr->find_mesh(mesh_name);
    if (mesh == nullptr) {
        std::cerr << "Deformer: mesh '" << mesh_name << "' not found\n";
        return false;
    }
    const auto packed = graph_context_.components().packed_joints();
    if (packed.empty()) {
        std::cerr << "Deformer: no joints\n";
        return false;
    }
    if (weight->weights.count() == 0) {
        std::cerr << "Deformer: skipped, auto-weight did not fill weight buffers\n";
        return false;
    }
    if (runner_.backend() == exec::Backend::Cuda
        && !context.make_mesh_deformable(mesh_name, *mesh))
    {
        std::cerr << "Deformer: failed to create rest/draw GPU buffers for '"
            << mesh_name << "'\n";
        return false;
    }

    orlrig::MeshData input;
    if (!extract_mesh(input, *mesh, selection_.selected_mesh_model())) {
        std::cerr << "Deformer: failed to pack bind pose\n";
        return false;
    }
    deformer->mesh_name = mesh_name;
    deformer->type = type_name;
    logged_rest = false;
    logged_move = false;

    // Keep the graph's scene mapping authoritative while the runner consumes
    // the same packed scene data through its existing ABI.
    std::cout << "Deformer: graph bind request for '" << mesh_name << "' "
        << mesh->vcnt << " verts, " << packed.size() << " joints\n";
    return true;
}

bool GraphSceneRuntime::dispatch_graph(vkkk::Context& context, bool capture)
{
    graph_context_.refresh_scene_inputs();
    const bool solver_stage = stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver;
    if (solver_stage) {
        std::string evaluation_error;
        if (!graph_context_.ensure_evaluation_plan(
                orlgraph::GraphStage::Solver, &evaluation_error))
        {
            report_compile_error(
                "Solver: evaluation plan: " + evaluation_error);
            return false;
        }
        evaluation_plan_ = graph_context_.evaluation_plan();
        active_change_set_ = graph_context_.take_change_set();
        orlrig::DirtyInputs dirty;
        dirty.full_evaluation = active_change_set_.full_evaluation
            || active_change_set_.plan_invalidated;
        dirty.joints = active_change_set_.joints;
        dirty.controllers = active_change_set_.controllers;
        dirty.locators = active_change_set_.locators;
        if (evaluation_plan_ == nullptr) {
            active_dispatch_ = {};
        } else if (runner_.backend() == exec::Backend::Cuda) {
            active_dispatch_ = orlrig::build_cuda_dispatch_plan(
                *evaluation_plan_, dirty);
        } else {
            active_dispatch_ = orlrig::build_dynamic_dispatch_plan(
                *evaluation_plan_, dirty);
        }
        active_dispatch_data_ =
            orlrig::pack_dynamic_dispatch_plan(active_dispatch_);
    }
    const auto validation = stage_.has_value()
        ? graph_context_.validate(*stage_)
        : graph_context_.validate();
    const char* label = stage_.has_value()
            && *stage_ == orlgraph::GraphStage::Solver
        ? "Solver" : "Deformer";
    if (!validation.ok()) {
        std::ostringstream message;
        for (const auto& diagnostic : validation.diagnostics) {
            message << label << ": graph validation: "
                    << diagnostic.message << '\n';
        }
        for (const auto& error : validation.schedule.errors) {
            message << label << ": graph schedule: " << error << '\n';
        }
        report_compile_error(message.str());
        return false;
    }

    std::string plan_error;
    if (!ensure_execution_plan(validation, &plan_error)) {
        report_compile_error(std::string{label}
            + ": graph execution plan failed: " + plan_error);
        return false;
    }

    bool saw_capture = false;
    bool saw_evaluate = false;
    for (const auto& step : execution_plan_) {
        if (step.kind == ExecutionStep::Kind::OrlSegment) {
            if (!execute_orl_segment(context, step.index)) {
                return false;
            }
            continue;
        }

        const auto* instance = active_graph().node(step.node_id);
        if (instance == nullptr
            || !execute_runtime_node(context, *instance, capture))
        {
            return false;
        }
        const auto* definition =
            graph_context_.registry().find(instance->definition);
        if (definition != nullptr) {
            saw_capture = saw_capture
                || definition->implementation.runtime_name == kCaptureBindRuntime;
            saw_evaluate = saw_evaluate
                || definition->implementation.runtime_name == kEvaluateRuntime;
        }
    }
    if ((!stage_.has_value()
            || *stage_ != orlgraph::GraphStage::Solver)
        && has_lbs_nodes()
        && ((capture && !saw_capture) || !saw_evaluate)) {
        std::cerr << "Deformer: active graph does not contain the required "
                     "LBS capture/deform nodes\n";
        return false;
    }
    reportedPlanError.clear();
    reportedPlanRevision = 0;
    return true;
}

void GraphSceneRuntime::report_compile_error(const std::string& message)
{
    std::cerr << message << '\n';
    const std::size_t revision = graph_context_.graph_edit_revision();
    if (message == reportedPlanError && revision == reportedPlanRevision) {
        return;
    }
    reportedPlanError = message;
    reportedPlanRevision = revision;
#if ORL_USE_QT6
    QMessageBox::warning(nullptr, QStringLiteral("ORL evaluation"),
        QString::fromStdString(message));
#endif
}

bool GraphSceneRuntime::resolve_scene_input_node(
    const orlgraph::NodeDefinition& definition,
    const orlgraph::NodeInstance& instance)
{
    const std::string_view runtime_name =
        definition.implementation.runtime_name;
    graph_context_.refresh_scene_inputs();

    if (runtime_name
            == std::string{orlrig::kComputedJointsNodeDefinition})
    {
        exec::GraphInputBinding resolved;
        std::string error;
        if (!graph_context_.scene_inputs().resolve_binding(
                orlrig::kComputedJointsBinding, resolved, &error))
        {
            std::cerr << "Deformer: scene input node resolution failed: "
                << error << '\n';
            return false;
        }
        return true;
    }

    SceneElementKind element_kind;
    const std::string_view element_name = runtime_name.substr(
        std::string_view{"orlrig.input.find_"}.size());
    if (element_name == "joint") {
        element_kind = SceneElementKind::Joint;
    } else if (element_name == "locator") {
        element_kind = SceneElementKind::Locator;
    } else if (element_name == "mesh") {
        element_kind = SceneElementKind::Mesh;
    } else {
        return false;
    }

    const auto parameter = instance.parameter_values.find("name");
    const auto* name = parameter == instance.parameter_values.end()
        ? nullptr
        : std::get_if<std::string>(&parameter->second.value);
    if (name == nullptr || name->empty()) {
        std::cerr << "Deformer: scene find node '" << instance.name
            << "' has no selected element\n";
        return false;
    }
    if (!graph_context_.scene_inputs().resolve_element_handle(
            element_kind, *name).has_value())
    {
        std::cerr << "Deformer: scene find node '" << instance.name
            << "' cannot resolve '" << *name << "'\n";
        return false;
    }
    return true;
}

std::size_t GraphSceneRuntime::graph_fingerprint() const
{
    std::ostringstream stream;
    const auto& graph = active_graph();
    stream << graph.module_id;
    for (const auto& [id, node] : graph.nodes()) {
        stream << "|node:" << id.value << ':' << node.definition.value
               << ':' << node.name;
        for (const auto& [name, value] : node.parameter_values) {
            stream << "|param:" << name << '=' << constant_fingerprint(value);
        }
    }
    for (const auto& connection : graph.connections()) {
        stream << "|connection:" << static_cast<int>(connection.source.kind)
               << ':' << connection.source.owner.value << ':'
               << connection.source.port.value << "->"
               << static_cast<int>(connection.destination.kind) << ':'
               << connection.destination.owner.value << ':'
               << connection.destination.port.value << ':'
               << connection.conversion << ':'
               << (connection.feedback ? 1 : 0);
    }
    for (const auto& [id, input] : graph.inputs()) {
        stream << "|input:" << id.value << ':' << input.name << ':'
               << static_cast<int>(input.type.kind) << ':'
               << input.type.canonical_name() << ':'
               << input.binding << ':' << input.semantic << ':'
               << input.coordinate_space;
    }
    for (const auto& [id, output] : graph.outputs()) {
        stream << "|output:" << id.value << ':' << output.name << ':'
               << static_cast<int>(output.type.kind) << ':'
               << output.type.canonical_name() << ':'
               << output.binding << ':' << output.semantic << ':'
               << output.coordinate_space;
    }
    return std::hash<std::string>{}(stream.str());
}

std::string GraphSceneRuntime::runtime_output_key(
    const orlgraph::StableId& node, const orlgraph::StableId& port) const
{
    return node.value + ":" + port.value;
}

bool GraphSceneRuntime::add_scene_execution_input(
    orlgraph::GraphModule& module,
    std::string_view binding,
    std::string_view name,
    const orlgraph::LogicalType& element_type,
    const orlgraph::Domain& domain,
    const orlgraph::Shape& shape,
    std::string_view semantic,
    std::string_view coordinate_space,
    std::string* expression,
    std::string* error)
{
    if (expression == nullptr) {
        if (error != nullptr) {
            *error = "Scene execution input expression destination is null";
        }
        return false;
    }

    const bool scalar_handle = element_type.is_handle();
    const auto expected_type = scalar_handle
        ? element_type
        : orlgraph::LogicalType::buffer(element_type);
    const bool require_exact_binding = scalar_handle
        || ((binding.rfind("scene.rig.controller.", 0) == 0
                || binding.rfind("scene.rig.locator.", 0) == 0)
            && binding.ends_with(".xform"));
    for (const auto& [id, input] : module.inputs()) {
        if (input.binding != binding
            && (require_exact_binding || semantic.empty()
                || input.semantic != semantic))
        {
            continue;
        }
        if (input.type != expected_type
            || input.domain != domain
            || input.shape != shape)
        {
            if (error != nullptr) {
                *error = "Scene execution input '" + std::string{binding}
                    + "' has incompatible graph metadata";
            }
            return false;
        }
        if (input.binding != binding
            && active_graph().input(id) != nullptr)
        {
            if (!graph_context_.map_input(
                    id, orlgraph::StableId{std::string{binding}}, error))
            {
                return false;
            }
        }
        *expression = runtime_identifier(
            input.name.empty() ? id.value : input.name);
        if (expression->empty()
            || std::isdigit(static_cast<unsigned char>((*expression)[0])))
        {
            expression->insert(expression->begin(), '_');
        }
        return true;
    }

    std::string base = "__orl_runtime_"
        + runtime_identifier(name) + "_"
        + runtime_identifier(binding);
    std::string id_value = base;
    for (std::size_t suffix = 1;
         module.input(orlgraph::StableId{id_value}) != nullptr; ++suffix)
    {
        id_value = base + "_" + std::to_string(suffix);
    }

    orlgraph::InterfacePort input;
    input.id = orlgraph::StableId{id_value};
    input.name = id_value;
    input.direction = orlgraph::PortDirection::Input;
    input.type = expected_type;
    input.domain = domain;
    input.shape = shape;
    input.required = true;
    input.binding = std::string{binding};
    input.semantic = std::string{semantic};
    input.coordinate_space = std::string{coordinate_space};
    if (!module.add_input(input, error)) {
        return false;
    }
    *expression = input.name;
    return true;
}

bool GraphSceneRuntime::prepare_runtime_output_expressions(
    orlgraph::GraphModule& module,
    const std::set<orlgraph::StableId>& node_ids,
    std::map<std::string, std::string>* expressions,
    std::string* error)
{
    if (expressions == nullptr) {
        if (error != nullptr) {
            *error = "Runtime output expression destination is null";
        }
        return false;
    }

    const auto element_name = [](const orlgraph::NodeInstance& instance)
        -> const std::string* {
        const auto found = instance.parameter_values.find("name");
        if (found == instance.parameter_values.end()) {
            return nullptr;
        }
        return std::get_if<std::string>(&found->second.value);
    };
    const auto fail = [error](std::string message) {
        if (error != nullptr) {
            *error = std::move(message);
        }
        return false;
    };

    const auto& source_graph = active_graph();
    for (const auto& node_id : node_ids) {
        const auto* instance = source_graph.node(node_id);
        const auto* definition = instance == nullptr
            ? nullptr : graph_context_.registry().find(instance->definition);
        if (instance == nullptr || definition == nullptr
            || definition->implementation.kind
                != orlgraph::ImplementationKind::Runtime)
        {
            continue;
        }

        const auto runtime_name = definition->implementation.runtime_name;
        for (const auto& output : definition->outputs) {
            std::string output_expression;
            if (runtime_name
                    == std::string{orlrig::kComputedJointsNodeDefinition}
                && output.name == "joints")
            {
                if (!add_scene_execution_input(
                        module, orlrig::kComputedJointsBinding,
                        "computed_joints",
                        orlgraph::LogicalType::struct_type("Joint"),
                        orlgraph::Domain::joint(),
                        orlgraph::Shape::one("joint_count"),
                        "joints", "world", &output_expression, error))
                {
                    return false;
                }
            } else if (runtime_name == "orlrig.input.find_joint"
                || runtime_name == "orlrig.input.find_locator"
                || runtime_name == "orlrig.input.find_mesh")
            {
                const auto* name = element_name(*instance);
                if (name == nullptr || name->empty()) {
                    return fail("Scene find node '" + instance->name
                        + "' has no selected element");
                }

                const std::string_view element_name = runtime_name.substr(
                    std::string_view{"orlrig.input.find_"}.size());
                SceneElementKind kind = SceneElementKind::Mesh;
                const auto* handle_output = definition->output("handle");
                if (handle_output != nullptr
                    && handle_output->type.is_handle()
                    && handle_output->type.name
                        == "orlrig::joint_handle")
                {
                    kind = SceneElementKind::Joint;
                } else if (handle_output != nullptr
                    && handle_output->type.is_handle()
                    && handle_output->type.name
                        == "orlrig::locator_handle")
                {
                    kind = SceneElementKind::Locator;
                } else if (element_name == "joint") {
                    kind = SceneElementKind::Joint;
                } else if (element_name == "locator") {
                    kind = SceneElementKind::Locator;
                }

                if (output.name == "handle") {
                    if (kind != SceneElementKind::Mesh) {
                        const std::string binding =
                            kind == SceneElementKind::Joint
                            ? scene_joint_handle_binding(*name)
                            : scene_locator_handle_binding(*name);
                        const auto handle_type =
                            kind == SceneElementKind::Joint
                            ? orlgraph::LogicalType::handle(
                                "orlrig::joint_handle")
                            : orlgraph::LogicalType::handle(
                                "orlrig::locator_handle");
                        if (!add_scene_execution_input(
                                module, binding,
                                "find_" + instance->name + "_handle",
                                handle_type, orlgraph::Domain::rig(),
                                orlgraph::Shape::scalar(),
                                kind == SceneElementKind::Joint
                                    ? "scene.joint.handle"
                                    : "scene.locator.handle",
                                {}, &output_expression, error))
                        {
                            return false;
                        }
                        (*expressions)[runtime_output_key(
                            instance->id, output.id)] =
                            std::move(output_expression);
                        continue;
                    }
                    const auto handle =
                        graph_context_.scene_inputs().resolve_element_handle(
                            kind, *name);
                    if (!handle.has_value()) {
                        return fail("Scene find node '" + instance->name
                            + "' cannot resolve '" + *name + "'");
                    }
                    output_expression = std::to_string(*handle);
                }
            }

            if (!output_expression.empty()) {
                (*expressions)[runtime_output_key(
                    instance->id, output.id)] = std::move(output_expression);
            }
        }
    }
    return true;
}

bool GraphSceneRuntime::add_segment_connection(
    const orlgraph::GraphModule& source,
    orlgraph::GraphModule& destination,
    const orlgraph::Connection& connection,
    const std::set<orlgraph::StableId>& node_ids,
    std::string* error)
{
    if (connection.destination.kind
            != orlgraph::EndpointKind::NodePort
        || node_ids.find(connection.destination.owner) == node_ids.end())
    {
        return true;
    }

    if (connection.source.kind == orlgraph::EndpointKind::GraphInput) {
        const auto* input = source.input(connection.source.owner);
        if (input == nullptr) {
            if (error != nullptr) {
                *error = "Graph segment references an unknown graph input";
            }
            return false;
        }
        if (destination.input(input->id) == nullptr
            && !destination.add_input(*input, error))
        {
            return false;
        }
    } else if (connection.source.kind
        == orlgraph::EndpointKind::NodePort)
    {
        if (node_ids.find(connection.source.owner) == node_ids.end()) {
            if (error != nullptr) {
                *error = "Graph segment has a data dependency across a "
                         "runtime execution boundary";
            }
            return false;
        }
    } else {
        if (error != nullptr) {
            *error = "Graph segment has an unsupported connection source";
        }
        return false;
    }

    return destination.add_connection(connection, error);
}

bool GraphSceneRuntime::build_orl_segment(
    const std::vector<orlgraph::StableId>& node_ids,
    std::size_t segment_index,
    std::string* error)
{
    if (node_ids.empty()) {
        return true;
    }

    std::set<orlgraph::StableId> node_set(
        node_ids.begin(), node_ids.end());
    const auto& source_graph = active_graph();
    OrlSegment segment;
    segment.graph.module_id = source_graph.module_id
        + ".runtime_segment." + std::to_string(segment_index);

    for (const auto& [id, resource] : source_graph.resources()) {
        if (!segment.graph.add_resource(resource, error)) {
            return false;
        }
    }
    for (const auto& node_id : node_ids) {
        const auto* node = source_graph.node(node_id);
        if (node == nullptr || !segment.graph.add_node(*node, error)) {
            return false;
        }
    }

    std::map<std::string, std::string> expressions;
    if (!prepare_runtime_output_expressions(
            segment.graph, node_set, &expressions, error))
    {
        return false;
    }

    for (const auto& connection : source_graph.connections()) {
        if (connection.destination.kind
            == orlgraph::EndpointKind::NodePort)
        {
            if (!add_segment_connection(
                    source_graph, segment.graph,
                    connection, node_set, error))
            {
                return false;
            }
            continue;
        }

        if (connection.destination.kind
                == orlgraph::EndpointKind::GraphOutput
            && connection.source.kind
                == orlgraph::EndpointKind::NodePort
            && node_set.find(connection.source.owner) != node_set.end())
        {
            const auto* output =
                source_graph.output(connection.destination.owner);
            if (output == nullptr) {
                if (error != nullptr) {
                    *error = "Graph segment references an unknown output";
                }
                return false;
            }
            if (segment.graph.output(output->id) == nullptr
                && !segment.graph.add_output(*output, error))
            {
                return false;
            }
            if (!segment.graph.add_connection(connection, error)) {
                return false;
            }
        }
    }

    orlcomp::GraphLoweringOptions options;
    options.entry_function = "orl_graph_segment_"
        + std::to_string(segment_index);
    options.scene_revision = graph_context_.scene_input_revision();
    options.runtime_output_expression =
        [expressions = std::move(expressions), this](
            const orlgraph::NodeInstance& node,
            const orlgraph::NodeDefinition&,
            const orlgraph::Port& output)
            -> std::optional<std::string> {
        const auto found = expressions.find(
            runtime_output_key(node.id, output.id));
        if (found == expressions.end()) {
            return std::nullopt;
        }
        return found->second;
    };
    auto program = exec::OrlGraphProgram::Compile(
        segment.graph, graph_context_.registry(), std::move(options));
    if (!program.valid()) {
        if (error != nullptr) {
            *error = program.errors().empty()
                ? "ORL graph segment compilation failed"
                : program.errors().front();
        }
        return false;
    }
    auto execution = exec::OrlGraphExecution::Create(
        program, runner_.backend());
    if (!execution.valid()) {
        if (error != nullptr) {
            *error = execution.errors().empty()
                ? "ORL graph segment execution creation failed"
                : execution.errors().front();
        }
        return false;
    }
    segment.program = std::move(program);
    segment.execution = std::move(execution);
    orl_segments_.push_back(std::move(segment));
    return true;
}

bool GraphSceneRuntime::ensure_execution_plan(
    const orlgraph::ValidationResult& validation,
    std::string* error)
{
    const auto current_graph_revision = graph_context_.graph_revision();
    const auto current_scene_revision =
        graph_context_.scene_input_revision();
    const auto current_fingerprint = graph_fingerprint();
    if (execution_plan_ready_
        && planned_graph_revision_ == current_graph_revision
        && planned_scene_revision_ == current_scene_revision
        && planned_graph_fingerprint_ == current_fingerprint
        && planned_backend_ == runner_.backend())
    {
        return true;
    }

    execution_plan_ready_ = false;
    const bool has_solver_graph =
        graph_context_.has_stage_graphs()
        && [&] {
            const auto& solver_graph =
                graph_context_.stage_graph(orlgraph::GraphStage::Solver);
            return !solver_graph.nodes().empty()
                || !solver_graph.inputs().empty()
                || !solver_graph.outputs().empty();
        }();
    if (!stage_.has_value()
        || *stage_ != orlgraph::GraphStage::Deformer
        || !has_solver_graph) {
        graph_context_.clear_computed_joints_device();
    }
    execution_plan_.clear();
    orl_segments_.clear();

    const bool contains_orl = has_orl_nodes();
    const auto& source_graph = active_graph();
    std::vector<orlgraph::StableId> segment_nodes;
    bool segment_has_orl = false;
    const auto flush_segment = [&]() -> bool {
        if (!segment_has_orl) {
            segment_nodes.clear();
            return true;
        }
        const std::size_t segment_index = orl_segments_.size();
        if (!build_orl_segment(segment_nodes, segment_index, error)) {
            return false;
        }
        if (stage_.has_value()
            && *stage_ == orlgraph::GraphStage::Solver
            && evaluation_plan_ != nullptr)
        {
            for (const auto& node_id : segment_nodes) {
                if (evaluation_plan_->region(node_id) != nullptr) {
                    orl_segments_.back().region_nodes.push_back(node_id);
                }
            }
        }
        execution_plan_.push_back(ExecutionStep{
            ExecutionStep::Kind::OrlSegment,
            segment_index,
            {},
        });
        segment_nodes.clear();
        segment_has_orl = false;
        return true;
    };

    std::map<orlgraph::StableId, std::size_t> region_batches;
    if (stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver
        && evaluation_plan_ != nullptr)
    {
        for (std::size_t level = 0;
             level < evaluation_plan_->batches.size(); ++level)
        {
            for (const auto& id : evaluation_plan_->batches[level]) {
                region_batches.emplace(id, level);
            }
        }
    }
    std::optional<std::size_t> segment_batch;
    std::map<std::size_t, std::vector<orlgraph::StableId>> stashed_inputs;
    const auto consumer_batch = [&](const orlgraph::StableId& id)
        -> std::optional<std::size_t> {
        std::optional<std::size_t> batch;
        for (const auto& connection : source_graph.connections()) {
            if (connection.source.kind != orlgraph::EndpointKind::NodePort
                || connection.source.owner != id
                || connection.destination.kind
                    != orlgraph::EndpointKind::NodePort)
            {
                continue;
            }
            const auto found = region_batches.find(
                connection.destination.owner);
            if (found == region_batches.end()) {
                continue;
            }
            if (!batch.has_value() || found->second < *batch) {
                batch = found->second;
            }
        }
        return batch;
    };
    const auto begin_batch = [&](std::size_t batch) -> bool {
        if (segment_batch.has_value() && *segment_batch == batch) {
            return true;
        }
        if (segment_has_orl) {
            if (!flush_segment()) {
                return false;
            }
        } else if (segment_batch.has_value() && !segment_nodes.empty()) {
            auto& slot = stashed_inputs[*segment_batch];
            slot.insert(slot.end(),
                segment_nodes.begin(), segment_nodes.end());
            segment_nodes.clear();
        }
        segment_batch = batch;
        const auto stashed = stashed_inputs.find(batch);
        if (stashed != stashed_inputs.end()) {
            segment_nodes.insert(segment_nodes.end(),
                stashed->second.begin(), stashed->second.end());
            stashed_inputs.erase(stashed);
        }
        return true;
    };

    for (const auto& node_id : validation.schedule.order) {
        const auto* instance = source_graph.node(node_id);
        const auto* definition = instance == nullptr
            ? nullptr : graph_context_.registry().find(instance->definition);
        if (instance == nullptr || definition == nullptr) {
            if (error != nullptr) {
                *error = "Graph schedule contains an unknown node";
            }
            return false;
        }

        if (definition->implementation.kind
            == orlgraph::ImplementationKind::OrlFunction)
        {
            const auto found = region_batches.find(node_id);
            if (found != region_batches.end()
                && !begin_batch(found->second))
            {
                return false;
            }
            segment_nodes.push_back(node_id);
            segment_has_orl = true;
            continue;
        }

        if (definition->implementation.kind
            == orlgraph::ImplementationKind::Runtime)
        {
            const std::string runtime_name =
                definition->implementation.runtime_name;
            if (runtime_adapters_.find(runtime_name)
                == runtime_adapters_.end())
            {
                if (error != nullptr) {
                    *error = "No runtime adapter is registered for '"
                        + runtime_name + "'";
                }
                return false;
            }
            const bool is_scene_input =
                runtime_name.rfind("orlrig.input.", 0) == 0;
            const bool is_computed_joints_source =
                runtime_name
                    == std::string{orlrig::kComputedJointsNodeDefinition};
            if (contains_orl && (is_scene_input || is_computed_joints_source)) {
                if (is_scene_input) {
                    const auto batch = consumer_batch(node_id);
                    if (batch.has_value() && !begin_batch(*batch)) {
                        return false;
                    }
                }
                segment_nodes.push_back(node_id);
                continue;
            }
            if (!flush_segment()) {
                return false;
            }
            segment_batch.reset();
            execution_plan_.push_back(ExecutionStep{
                ExecutionStep::Kind::RuntimeNode,
                0,
                node_id,
            });
            continue;
        }

        if (definition->operation == "identity") {
            segment_nodes.push_back(node_id);
            segment_has_orl = true;
            continue;
        }

        if (error != nullptr) {
            *error = "Graph node '" + definition->qualified_name
                + "' is not supported by the scene execution plan";
        }
        return false;
    }
    if (!flush_segment()) {
        return false;
    }
    if (!region_batches.empty()) {
        std::vector<std::size_t> positions;
        std::vector<ExecutionStep> segments;
        for (std::size_t index = 0; index < execution_plan_.size(); ++index) {
            const auto& step = execution_plan_[index];
            if (step.kind != ExecutionStep::Kind::OrlSegment
                || step.index >= orl_segments_.size()
                || orl_segments_[step.index].region_nodes.empty())
            {
                continue;
            }
            positions.push_back(index);
            segments.push_back(step);
        }
        std::stable_sort(segments.begin(), segments.end(),
            [&](const ExecutionStep& left, const ExecutionStep& right) {
                const auto batch_of = [&](const ExecutionStep& step) {
                    const auto& nodes =
                        orl_segments_[step.index].region_nodes;
                    const auto found = nodes.empty()
                        ? region_batches.end()
                        : region_batches.find(nodes.front());
                    return found == region_batches.end()
                        ? std::size_t{0} : found->second;
                };
                return batch_of(left) < batch_of(right);
            });
        for (std::size_t index = 0; index < positions.size(); ++index) {
            execution_plan_[positions[index]] = segments[index];
        }
    }

    planned_graph_revision_ = current_graph_revision;
    planned_scene_revision_ = current_scene_revision;
    planned_graph_fingerprint_ = current_fingerprint;
    planned_backend_ = runner_.backend();
    execution_plan_ready_ = true;
    return true;
}

bool GraphSceneRuntime::segment_is_clean(std::size_t segment_index) const
{
    if (stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver
        && evaluation_plan_ != nullptr
        && segment_index < orl_segments_.size()
        && !orl_segments_[segment_index].region_nodes.empty())
    {
        for (const auto& node_id : orl_segments_[segment_index].region_nodes) {
            const auto* region = evaluation_plan_->region(node_id);
            if (region == nullptr) {
                return false;
            }
            const auto region_index = static_cast<std::size_t>(
                std::distance(evaluation_plan_->regions.data(), region));
            if (std::find(active_dispatch_.region_indices.begin(),
                    active_dispatch_.region_indices.end(), region_index)
                != active_dispatch_.region_indices.end())
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

bool GraphSceneRuntime::execute_orl_segment(
    vkkk::Context& context, std::size_t segment_index)
{
    (void)context;
    if (segment_index >= orl_segments_.size()
        || !orl_segments_[segment_index].execution.has_value())
    {
        std::cerr << "Deformer: ORL graph segment is unavailable\n";
        return false;
    }
    if (segment_is_clean(segment_index)) {
        return true;
    }

    auto& segment = orl_segments_[segment_index];
    auto& execution = *segment.execution;
    if (!graph_context_.bind_graph_inputs(execution, segment.graph))
    {
        for (const auto& error : execution.errors()) {
            std::cerr << "Deformer: graph input binding: "
                      << error << '\n';
        }
        return false;
    }
    std::string hierarchy_error;
    if (!graph_context_.ensure_hierarchy_plan(&hierarchy_error)
        || !execution.set_hierarchy_context(
            graph_context_.hierarchy_context())
        || !execution.bind_hierarchy_data(
            graph_context_.hierarchy_data()))
    {
        std::cerr << "Deformer: hierarchy binding: "
                  << (hierarchy_error.empty()
                      ? (execution.errors().empty()
                          ? "unknown hierarchy binding error"
                          : execution.errors().front())
                      : hierarchy_error)
                  << '\n';
        return false;
    }

    if (!execution.bind_handle_view_context(
            graph_context_.scene_inputs().handle_view_context()))
    {
        for (const auto& error : execution.errors()) {
            std::cerr << "Deformer: handle-view binding: "
                      << error << '\n';
        }
        return false;
    }

    const bool solver_device_evaluation =
        stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver
        && execution.backend() == exec::Backend::Cuda;
    std::optional<exec::GraphEvaluationResult> result;
    if (solver_device_evaluation) {
        if (!execution.evaluate_device()) {
            for (const auto& error : execution.errors()) {
                std::cerr << "Solver: ORL graph evaluation: "
                          << error << '\n';
            }
            return false;
        }
    } else {
        result = execution.evaluate_result();
        if (!result->ok) {
            for (const auto& error : result->errors) {
                std::cerr << "Deformer: ORL graph evaluation: "
                          << error << '\n';
            }
            return false;
        }
    }

    // Typed solver views write the internal Joint arena directly. Keep that
    // device allocation available to the deformer without forcing a
    // graph-level joints socket or a host readback.
    if (solver_device_evaluation) {
        auto device = execution.handle_joint_device_view();
        if (!device.has_value()) {
            std::cerr << "Deformer: typed solver Joint device buffer is "
                         "unavailable\n";
            return false;
        }
        graph_context_.set_computed_joints_device(
            device, graph_context_.scene_inputs()
                .computed_joints_buffer().count());
    } else if (stage_.has_value()
        && *stage_ == orlgraph::GraphStage::Solver)
    {
        graph_context_.clear_computed_joints_device();
    }

    if (solver_device_evaluation) {
        // The solver result remains in CUDA/Vulkan-shared storage. Reading it
        // back here would defeat the device-only path and is unnecessary for
        // the deformer or GPU joint rendering.
        return true;
    }

    std::string error;
    if (!graph_context_.commit_scene_writes(*result, true, &error)) {
        std::cerr << "Deformer: graph scene writeback failed: "
                  << error << '\n';
        return false;
    }
    return true;
}

bool GraphSceneRuntime::execute_runtime_node(
    vkkk::Context& context,
    const orlgraph::NodeInstance& instance,
    bool capture)
{
    const auto* definition =
        graph_context_.registry().find(instance.definition);
    if (definition == nullptr) {
        return false;
    }
    const auto runtime_name = definition->implementation.runtime_name;
    const auto found = runtime_adapters_.find(runtime_name);
    if (found == runtime_adapters_.end()) {
        std::cerr << "Deformer: unsupported runtime graph node '"
                  << definition->qualified_name << "'\n";
        return false;
    }
    return (this->*found->second)(context, instance, capture);
}

bool GraphSceneRuntime::execute_scene_input_adapter(
    vkkk::Context& context,
    const orlgraph::NodeInstance& instance,
    bool capture)
{
    (void)context;
    (void)capture;
    const auto* definition =
        graph_context_.registry().find(instance.definition);
    return definition != nullptr
        && resolve_scene_input_node(*definition, instance);
}

bool GraphSceneRuntime::execute_computed_joints_adapter(
    vkkk::Context& context,
    const orlgraph::NodeInstance& instance,
    bool capture)
{
    (void)context;
    (void)capture;
    const auto* definition =
        graph_context_.registry().find(instance.definition);
    // The computed-joints node is lowered as a read-only stage input when it
    // is connected to an ORL segment. If a graph contains only this node,
    // resolving the binding is the only runtime work required.
    return definition != nullptr
        && resolve_scene_input_node(*definition, instance);
}

bool GraphSceneRuntime::execute_lbs_capture_adapter(
    vkkk::Context& context,
    const orlgraph::NodeInstance& instance,
    bool capture)
{
    (void)instance;
    if (!capture) {
        return true;
    }
    auto* deformer = graph_context_.components().deformer(deformer_id);
    std::string mesh_name = selection_.selected_mesh_name();
    if (mesh_name.empty() && deformer != nullptr) {
        mesh_name = deformer->mesh_name;
    }
    const auto* mesh = graph_context_.scene().drawable_mgr == nullptr
        ? nullptr
        : graph_context_.scene().drawable_mgr->find_mesh(mesh_name);
    const auto packed = graph_context_.components().packed_joints();
    if (deformer == nullptr || mesh == nullptr || packed.empty()) {
        return false;
    }
    orlrig::MeshData input;
    if (!extract_mesh(input, *mesh, selection_.selected_mesh_model())) {
        return false;
    }
    const auto status = runner_.capture_bind(*deformer, input, packed);
    if (!status) {
        print_runner_errors(status.errors);
        return false;
    }
    deformer->bound = true;
    dump_bind_snapshot(packed, deformer->inverse_binds);
    return true;
}

bool GraphSceneRuntime::execute_lbs_evaluate_adapter(
    vkkk::Context& context,
    const orlgraph::NodeInstance& instance,
    bool capture)
{
    (void)instance;
    (void)capture;
    auto* deformer = graph_context_.components().deformer(deformer_id);
    auto* weight = graph_context_.components().weight(weight_id);
    if (deformer == nullptr || weight == nullptr || !deformer->bound) {
        return false;
    }
    const auto packed = graph_context_.components().packed_joints();
    if (packed.empty()
        || packed.size() != deformer->inverse_binds.count())
    {
        return false;
    }
    const bool device_only = runner_.backend() == exec::Backend::Cuda;
    const auto status = runner_.evaluate(
        *deformer, *weight, packed, device_only);
    if (!status) {
        print_runner_errors(status.errors);
        return false;
    }

    auto& scene = graph_context_.scene();
    const std::string mesh_name = deformer->mesh_name;
    auto* mesh = scene.drawable_mgr == nullptr
        ? nullptr : scene.drawable_mgr->find_mesh(mesh_name);
    if (mesh == nullptr) {
        return false;
    }
    if (device_only) {
        const auto output_device = runner_.output_device();
        if (!output_device.has_value()) {
            std::cerr << "Deformer: CUDA output buffer is unavailable\n";
            return false;
        }
        const glm::mat4 to_object = glm::inverse(deformer->bind_model);
        float matrix_float[16] = {};
        std::memcpy(matrix_float, &to_object, sizeof(matrix_float));
        double world_to_object[16] = {};
        for (int index = 0; index < 16; ++index) {
            world_to_object[index] = static_cast<double>(matrix_float[index]);
        }
        const int offset = vertex_float_offset(*mesh);
        if (offset < 0 || !context.write_mesh_positions_from_cuda(
                mesh_name, output_device->device_ptr, output_device->bytes,
                mesh->vcnt, mesh->comp_size,
                static_cast<std::uint32_t>(offset), world_to_object))
        {
            std::cerr << "Deformer: CUDA-Vulkan mesh update failed for '"
                      << mesh_name << "'\n";
            return false;
        }
        if (!logged_rest) {
            std::cout << "Deformer: GPU-resident output committed to mesh\n";
            logged_rest = true;
        }
        return true;
    }

    const auto& output = runner_.output_positions();
    if (!write_positions(*mesh, output, deformer->bind_model)) {
        std::cerr << "Deformer: failed to write mesh positions\n";
        return false;
    }
    const double delta = max_position_delta(
        deformer->bind_positions, output);
    if (!logged_rest) {
        std::cout << "Deformer: rest evaluate max |posed-bind|="
                  << delta << '\n';
        logged_rest = true;
    } else if (!logged_move && delta > 1.0e-3) {
        std::cout << "Deformer: mesh moved, max |posed-bind|="
                  << delta << '\n';
        dump_posed(deformer->bind_positions, output, *mesh);
        logged_move = true;
    }
    if (!context.update_mesh(mesh_name, *mesh)) {
        std::cerr << "Deformer: GPU mesh update failed for '"
                  << mesh_name << "'\n";
        return false;
    }
    return true;
}

bool GraphSceneRuntime::evaluate(vkkk::Context& context)
{
    if (!runtime_config.evaluate_orl) {
        return true;
    }
    return dispatch_graph(context, false);
}

} // namespace ORL
