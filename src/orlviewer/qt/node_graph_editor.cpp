#include "node_graph_editor.hpp"

#if ORL_USE_QT6

#include <QAbstractItemView>
#include <QComboBox>
#include <QCursor>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFontMetrics>
#include <QHash>
#include <QKeyEvent>
#include <QLineEdit>
#include <QMouseEvent>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QSet>
#include <QSignalBlocker>
#include <QStringList>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "../graph_scene_inputs.hpp"
#include "../project_serialization.hpp"
#include "../runtime_config.hpp"
#include "../scene_graph_context.hpp"
#include "../selection.hpp"
#include "node_graph/node_ops.hpp"
#include "orlgraph/graph_serialization.hpp"

namespace ORL
{

namespace
{

const QVector<QColor> kNodeColors = {
    QColor{QStringLiteral("#4c78a8")},
    QColor{QStringLiteral("#5b8e7d")},
    QColor{QStringLiteral("#b7791f")},
    QColor{QStringLiteral("#805ad5")},
};

std::optional<SceneElementKind> find_element_kind(
    std::string_view qualified_name)
{
    if (qualified_name == "orlrig.input.find_joint") {
        return SceneElementKind::Joint;
    }
    if (qualified_name == "orlrig.input.find_locator") {
        return SceneElementKind::Locator;
    }
    if (qualified_name == "orlrig.input.find_mesh") {
        return SceneElementKind::Mesh;
    }
    return std::nullopt;
}

} // namespace

NodeGraphEditor::NodeGraphEditor(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(480, 320);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

void NodeGraphEditor::set_graph(const orlgraph::GraphModule& module,
    const orlgraph::NodeRegistry& registry)
{
    solver_graph_storage_ = module;
    deformer_graph_storage_ = module;
    registry_storage_ = registry;
    graph_ = stage_ == orlgraph::GraphStage::Solver
        ? &solver_graph_storage_ : &deformer_graph_storage_;
    registry_ = &registry_storage_;
    scene_graph_context_ = nullptr;
    attached_graph_revision_ = 0;
    graph_file_path_.reset();
    clear_stage_layouts();
    nodes_.clear();
    rebuild_view();
}

void NodeGraphEditor::set_graph(orlgraph::GraphModule& module,
    orlgraph::NodeRegistry& registry)
{
    solver_graph_storage_ = module;
    deformer_graph_storage_ = module;
    registry_storage_ = registry;
    graph_ = stage_ == orlgraph::GraphStage::Solver
        ? &solver_graph_storage_ : &deformer_graph_storage_;
    registry_ = &registry_storage_;
    scene_graph_context_ = nullptr;
    attached_graph_revision_ = 0;
    graph_file_path_.reset();
    clear_stage_layouts();
    nodes_.clear();
    rebuild_view();
}

void NodeGraphEditor::set_scene_graph_context(SceneGraphContext* context)
{
    scene_graph_context_ = context;
    if (scene_graph_context_ == nullptr) {
        graph_ = stage_ == orlgraph::GraphStage::Solver
            ? &solver_graph_storage_ : &deformer_graph_storage_;
        registry_ = &registry_storage_;
        scene_input_catalog_ = nullptr;
        attached_graph_revision_ = 0;
    } else {
        scene_graph_context_->ensure_stage_graphs();
        graph_ = &scene_graph_context_->stage_graph(stage_);
        registry_ = &scene_graph_context_->registry();
        scene_input_catalog_ = &scene_graph_context_->scene_inputs();
        attached_graph_revision_ = scene_graph_context_->graph_revision();
    }
    graph_file_path_.reset();
    clear_stage_layouts();
    nodes_.clear();
    rebuild_view();
}

void NodeGraphEditor::set_project_context(ComponentManager* components,
    ComponentId weight_id, ComponentId deformer_id)
{
    project_components = components;
    project_weight_id = weight_id;
    project_deformer_id = deformer_id;
}

bool NodeGraphEditor::load_project_file()
{
    if (scene_graph_context_ == nullptr || project_components == nullptr) {
        QMessageBox::warning(this, QStringLiteral("Open Project"),
            QStringLiteral("The node graph is not attached to a project."));
        return false;
    }

    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("Open ORL Project"),
        graph_file_path_.value_or(QString()),
        QStringLiteral("ORL Project (*.json);;All Files (*)"));
    if (path.isEmpty()) {
        return false;
    }

    const bool was_enabled = runtime_config.evaluate_orl;
    runtime_config.evaluate_orl = false;
    scene_graph_context_->scene_inputs().set_cuda_evaluation(false);
    scene_graph_context_->clear_computed_joints_device();
    if (project_selection != nullptr) {
        project_selection->clear();
    }
    const ProjectIoResult loaded = load_project_json(
        path.toStdString(), *scene_graph_context_, *project_components,
        project_weight_id, project_deformer_id);
    if (!loaded.ok) {
        runtime_config.evaluate_orl = was_enabled;
        QString message = QStringLiteral("Unable to open ORL project.");
        if (!loaded.errors.empty()) {
            message += QStringLiteral("\n")
                + QString::fromStdString(loaded.errors.front());
        }
        QMessageBox::warning(this, QStringLiteral("Open ORL Project"),
            message);
        return false;
    }

    graph_ = &scene_graph_context_->stage_graph(stage_);
    registry_ = &scene_graph_context_->registry();
    scene_input_catalog_ = &scene_graph_context_->scene_inputs();
    scene_graph_context_->refresh_scene_inputs();
    attached_graph_revision_ = scene_graph_context_->graph_revision();
    graph_file_path_ = path;
    import_node_graph_layout(loaded.node_graph_layout);
    nodes_.clear();
    apply_stage_view();
    rebuild_view();

    runtime_config.evaluate_orl = loaded.evaluate_orl;
    if (project_selection != nullptr) {
        project_selection->set_controller_input_mode(
            runtime_config.evaluate_orl);
    }
    if (runtime_config.evaluate_orl) {
        scene_graph_context_->request_evaluation();
    }
    if (!loaded.warnings.empty()) {
        QString message = QStringLiteral(
            "The project opened with unfinished graph connections.");
        for (const auto& warning : loaded.warnings) {
            message += QStringLiteral("\n")
                + QString::fromStdString(warning);
        }
        QMessageBox::warning(this, QStringLiteral("Open ORL Project"),
            message);
    }
    return true;
}

void NodeGraphEditor::set_stage(orlgraph::GraphStage stage)
{
    if (stage_ == stage && graph_ != nullptr) {
        return;
    }
    capture_stage_layout();
    nodes_.clear();
    stage_ = stage;
    if (scene_graph_context_ != nullptr) {
        scene_graph_context_->ensure_stage_graphs();
        graph_ = &scene_graph_context_->stage_graph(stage_);
        attached_graph_revision_ = scene_graph_context_->graph_revision();
    } else {
        graph_ = stage_ == orlgraph::GraphStage::Solver
            ? &solver_graph_storage_ : &deformer_graph_storage_;
    }
    apply_stage_view();
    rebuild_view();
}

void NodeGraphEditor::set_scene_input_catalog(
    const SceneInputCatalog* catalog)
{
    scene_input_catalog_ = catalog;
    rebuild_view();
}

void NodeGraphEditor::create_frame_from_selection()
{
    if (qobject_cast<QLineEdit*>(focusWidget()) != nullptr
        || qobject_cast<QComboBox*>(focusWidget()) != nullptr)
    {
        return;
    }

    std::vector<node_graph::FrameMemberBounds> members;
    members.reserve(static_cast<std::size_t>(selectedNodes.size()));
    for (int index : selectedNodes) {
        if (index < 0 || index >= nodes_.size() || node_is_hidden(nodes_[index])) {
            continue;
        }
        const Node& node = nodes_[index];
        members.push_back({
            node.id.toStdString(),
            node.position.x(),
            node.position.y(),
            node.size.width(),
            node.size.height(),
        });
    }

    std::vector<std::string> existing_ids;
    auto& frames = current_frames();
    existing_ids.reserve(static_cast<std::size_t>(frames.size()));
    for (const auto& frame : frames) {
        existing_ids.push_back(frame.id.toStdString());
    }

    node_graph::FrameDesc created;
    std::string error;
    if (!node_graph::create_frame(members, existing_ids, &created, &error)) {
        return;
    }

    const QString created_id = QString::fromStdString(created.id);
    for (auto& frame : frames) {
        frame.members.erase(std::remove_if(frame.members.begin(),
            frame.members.end(),
            [&created](const QString& member) {
                return std::find(created.members.begin(), created.members.end(),
                    member.toStdString()) != created.members.end();
            }), frame.members.end());
    }
    frames.erase(std::remove_if(frames.begin(), frames.end(),
        [](const Frame& frame) { return frame.members.isEmpty(); }),
        frames.end());

    Frame view;
    view.id = created_id;
    view.title = QString::fromStdString(created.title);
    view.position = QPointF{created.x, created.y};
    view.size = QSizeF{created.width, created.height};
    view.collapsed = created.collapsed;
    for (const auto& member : created.members) {
        view.members.push_back(QString::fromStdString(member));
    }
    frames.push_back(std::move(view));
    capture_stage_layout();
    rebuild_frame_controls();
    select_only_frame(frames.size() - 1);
    update();
}

void NodeGraphEditor::copy_selected_nodes()
{
    if (qobject_cast<QLineEdit*>(focusWidget()) != nullptr
        || qobject_cast<QComboBox*>(focusWidget()) != nullptr)
    {
        return;
    }

    std::vector<node_graph::CopyNodeRef> refs;
    QSet<QString> seen;
    const auto add_view_node = [&](const Node& node) {
        if (seen.contains(node.id)) {
            return;
        }
        seen.insert(node.id);
        node_graph::CopyNodeRef ref;
        ref.editor_id = node.id.toStdString();
        ref.x = node.position.x();
        ref.y = node.position.y();
        if (node.kind == Node::Kind::GraphInput) {
            ref.kind = node_graph::CopiedNodeKind::GraphInput;
            ref.owner_id = node.interface_id.value;
        } else if (node.kind == Node::Kind::GraphOutput) {
            ref.kind = node_graph::CopiedNodeKind::GraphOutput;
            ref.owner_id = node.interface_id.value;
        } else {
            ref.kind = node_graph::CopiedNodeKind::Definition;
            ref.owner_id = node.id.toStdString();
        }
        refs.push_back(std::move(ref));
    };
    const auto find_view_node = [this](const QString& id) -> const Node* {
        for (const auto& node : nodes_) {
            if (node.id == id) {
                return &node;
            }
        }
        return nullptr;
    };

    std::vector<node_graph::CopiedFrame> frames;
    const auto& stage_frames = current_frames();
    for (int index : selectedFrames) {
        if (index < 0 || index >= stage_frames.size()) {
            continue;
        }
        const Frame& frame = stage_frames[index];
        node_graph::CopiedFrame copied;
        copied.title = frame.title.toStdString();
        copied.x = frame.position.x();
        copied.y = frame.position.y();
        copied.width = frame.size.width();
        copied.height = frame.size.height();
        copied.collapsed = frame.collapsed;
        for (const auto& member : frame.members) {
            const Node* node = find_view_node(member);
            if (node == nullptr) {
                continue;
            }
            add_view_node(*node);
            copied.members.push_back(member.toStdString());
        }
        if (!copied.members.empty()) {
            frames.push_back(std::move(copied));
        }
    }
    for (int index : selectedNodes) {
        if (index < 0 || index >= nodes_.size()) {
            continue;
        }
        add_view_node(nodes_[index]);
    }

    node_graph::NodeClipboard clipboard;
    std::string error;
    if (!node_graph::copy_nodes(
            active_graph(), refs, frames, &clipboard, &error))
    {
        return;
    }
    clipboard.stage = stage_;
    nodeClipboard = std::move(clipboard);
    pasteSerial = 0;
}

void NodeGraphEditor::paste_nodes()
{
    if (qobject_cast<QLineEdit*>(focusWidget()) != nullptr
        || qobject_cast<QComboBox*>(focusWidget()) != nullptr)
    {
        return;
    }
    if (!nodeClipboard.has_value() || nodeClipboard->nodes.empty()
        || nodeClipboard->stage != stage_)
    {
        return;
    }

    runtime_config.evaluate_orl = false;
    if (project_selection != nullptr) {
        project_selection->set_controller_input_mode(false);
    }
    if (scene_graph_context_ != nullptr) {
        scene_graph_context_->scene_inputs().set_cuda_evaluation(false);
    }

    ++pasteSerial;
    const double offset = 48.0 * static_cast<double>(pasteSerial);
    std::vector<std::string> frame_ids;
    std::vector<std::string> frame_titles;
    for (const auto& frame : current_frames()) {
        frame_ids.push_back(frame.id.toStdString());
        frame_titles.push_back(frame.title.toStdString());
    }

    node_graph::PasteResult pasted;
    std::string error;
    if (!node_graph::paste_nodes(active_graph(), active_registry(), stage_,
            *nodeClipboard, offset, offset, frame_ids, frame_titles,
            &pasted, &error))
    {
        if (!error.empty()) {
            std::cerr << "Node graph: failed to paste: " << error << '\n';
        }
        return;
    }

    auto& stored = stage_layout(stage_).nodes;
    QSet<QString> pasted_ids;
    for (const auto& node : pasted.nodes) {
        const QString id = QString::fromStdString(node.editor_id);
        stored.insert(id, QPointF{node.x, node.y});
        pasted_ids.insert(id);
    }
    QSet<QString> pasted_frames;
    auto& frames = current_frames();
    for (const auto& frame : pasted.frames) {
        Frame view;
        view.id = QString::fromStdString(frame.id);
        view.title = QString::fromStdString(frame.title);
        view.position = QPointF{frame.x, frame.y};
        view.size = QSizeF{frame.width, frame.height};
        view.collapsed = frame.collapsed;
        for (const auto& member : frame.members) {
            view.members.push_back(QString::fromStdString(member));
        }
        pasted_frames.insert(view.id);
        frames.push_back(std::move(view));
    }

    notify_graph_changed();
    rebuild_view();

    selectedNodes.clear();
    selectedFrames.clear();
    selected_node_ = -1;
    for (int index = 0; index < nodes_.size(); ++index) {
        if (pasted_ids.contains(nodes_[index].id)) {
            selectedNodes.push_back(index);
            selected_node_ = index;
        }
    }
    const auto& live_frames = current_frames();
    for (int index = 0; index < live_frames.size(); ++index) {
        if (pasted_frames.contains(live_frames[index].id)) {
            selectedFrames.push_back(index);
        }
    }
    capture_stage_layout();
    update();
}

void NodeGraphEditor::refresh_scene_inputs()
{
    if (scene_graph_context_ != nullptr
        && attached_graph_revision_ != scene_graph_context_->graph_revision())
    {
        attached_graph_revision_ = scene_graph_context_->graph_revision();
        rebuild_view();
    }
    refresh_find_controls();
    position_find_controls();
    position_frame_controls();
}

void NodeGraphEditor::notify_graph_changed()
{
    if (scene_graph_context_ != nullptr) {
        scene_graph_context_->touch_graph();
        // A graph can be temporarily invalid while it is being authored.
        // Keep runtime evaluation paused until the user explicitly enables
        // it again, which is also the point where the graph is JIT compiled.
        runtime_config.evaluate_orl = false;
    }
}

NodeGraphEditor::StageLayout& NodeGraphEditor::stage_layout(
    orlgraph::GraphStage stage)
{
    return stage == orlgraph::GraphStage::Solver
        ? solver_layout_ : deformer_layout_;
}

const NodeGraphEditor::StageLayout& NodeGraphEditor::stage_layout(
    orlgraph::GraphStage stage) const
{
    return stage == orlgraph::GraphStage::Solver
        ? solver_layout_ : deformer_layout_;
}

void NodeGraphEditor::capture_stage_layout()
{
    auto& layout = stage_layout(stage_);
    layout.pan = pan_;
    layout.zoom = zoom_;
    layout.nodes.clear();
    for (const auto& node : nodes_) {
        layout.nodes.insert(node.id, node.position);
    }
}

void NodeGraphEditor::apply_stage_view()
{
    const auto& layout = stage_layout(stage_);
    pan_ = layout.pan;
    zoom_ = layout.zoom;
}

void NodeGraphEditor::clear_stage_layouts()
{
    solver_layout_ = {};
    deformer_layout_ = {};
    pan_ = solver_layout_.pan;
    zoom_ = solver_layout_.zoom;
}

NodeGraphLayout NodeGraphEditor::export_node_graph_layout()
{
    capture_stage_layout();
    NodeGraphLayout layout;
    const auto copy_stage = [](const StageLayout& source,
        NodeGraphStageLayout& destination) {
        destination.pan_x = source.pan.x();
        destination.pan_y = source.pan.y();
        destination.zoom = source.zoom;
        for (auto it = source.nodes.constBegin();
             it != source.nodes.constEnd(); ++it)
        {
            destination.nodes[it.key().toStdString()] = {
                it.value().x(), it.value().y()};
        }
        for (const auto& frame : source.frames) {
            NodeGraphFrameLayout saved;
            saved.id = frame.id.toStdString();
            saved.title = frame.title.toStdString();
            saved.x = frame.position.x();
            saved.y = frame.position.y();
            saved.width = frame.size.width();
            saved.height = frame.size.height();
            saved.collapsed = frame.collapsed;
            saved.members.reserve(static_cast<std::size_t>(frame.members.size()));
            for (const auto& member : frame.members) {
                saved.members.push_back(member.toStdString());
            }
            destination.frames.push_back(std::move(saved));
        }
    };
    copy_stage(solver_layout_, layout.solver);
    copy_stage(deformer_layout_, layout.deformer);
    return layout;
}

void NodeGraphEditor::import_node_graph_layout(const NodeGraphLayout& layout)
{
    const auto copy_stage = [](const NodeGraphStageLayout& source,
        StageLayout& destination) {
        destination.pan = QPointF{source.pan_x, source.pan_y};
        destination.zoom = source.zoom;
        destination.nodes.clear();
        destination.frames.clear();
        for (const auto& [id, position] : source.nodes) {
            destination.nodes.insert(
                QString::fromStdString(id),
                QPointF{position[0], position[1]});
        }
        for (const auto& frame : source.frames) {
            Frame view;
            view.id = QString::fromStdString(frame.id);
            view.title = QString::fromStdString(frame.title);
            view.position = QPointF{frame.x, frame.y};
            view.size = QSizeF{frame.width, frame.height};
            view.collapsed = frame.collapsed;
            for (const auto& member : frame.members) {
                view.members.push_back(QString::fromStdString(member));
            }
            destination.frames.push_back(std::move(view));
        }
    };
    copy_stage(layout.solver, solver_layout_);
    copy_stage(layout.deformer, deformer_layout_);
}

void NodeGraphEditor::rebuild_view()
{
    clear_find_controls();
    clear_frame_controls();
    auto& stored_nodes = stage_layout(stage_).nodes;
    for (const auto& node : nodes_) {
        stored_nodes.insert(node.id, node.position);
    }

    nodes_.clear();
    links_.clear();
    selected_node_ = -1;
    selectedNodes.clear();
    selectedFrames.clear();
    dragging_node_ = -1;
    draggingFrame = -1;
    boxSelecting = false;
    pending_connection_ = {};

    const auto restore_position = [&stored_nodes](Node& node) {
        if (const auto previous = stored_nodes.constFind(node.id);
            previous != stored_nodes.constEnd())
        {
            node.position = previous.value();
        }
    };

    int index = 0;
    for (const auto& [id, instance] : active_graph().nodes()) {
        Node node;
        node.id = QString::fromStdString(id.value);
        node.title = QString::fromStdString(instance.name.empty()
            ? id.value : instance.name);
        node.position = QPointF(
            64.0 + static_cast<double>(index % 3) * 300.0,
            80.0 + static_cast<double>(index / 3) * 210.0);
        node.color = kNodeColors[index % kNodeColors.size()];
        bool has_find_control = false;

        if (const auto* definition = active_registry().find(instance.definition)) {
            node.title = QString::fromStdString(definition->qualified_name);
            has_find_control = find_element_kind(
                definition->qualified_name).has_value();
            for (const auto& port : definition->inputs) {
                Port view_port;
                view_port.id = QString::fromStdString(port.id.value);
                view_port.name = QString::fromStdString(port.name);
                view_port.direction = port.direction;
                view_port.cardinality = port.cardinality;
                view_port.type = port.type;
                view_port.domain = port.domain;
                view_port.shape = port.shape;
                view_port.semantic =
                    QString::fromStdString(port.semantic);
                node.inputs.push_back(std::move(view_port));
            }
            for (const auto& port : definition->outputs) {
                Port view_port;
                view_port.id = QString::fromStdString(port.id.value);
                view_port.name = QString::fromStdString(port.name);
                view_port.output = true;
                view_port.direction = port.direction;
                view_port.cardinality = port.cardinality;
                view_port.type = port.type;
                view_port.domain = port.domain;
                view_port.shape = port.shape;
                view_port.semantic =
                    QString::fromStdString(port.semantic);
                node.outputs.push_back(std::move(view_port));
            }
        }
        const int rows = std::max(node.inputs.size(), node.outputs.size());
        const double control_space = has_find_control ? 34.0 : 0.0;
        node.size.setHeight(std::max(
            86.0, 42.0 + rows * 22.0 + control_space));
        restore_position(node);
        nodes_.push_back(std::move(node));
        ++index;
    }

    const auto add_interface_node =
        [this, &restore_position](const orlgraph::StableId& id,
            const orlgraph::InterfacePort& interface_port,
            Node::Kind kind, int boundary_index) {
        Node node;
        node.kind = kind;
        node.interface_id = id;
        node.id = (kind == Node::Kind::GraphInput
                ? QStringLiteral("__graph_input__:")
                : QStringLiteral("__graph_output__:"))
            + QString::fromStdString(id.value);
        const QString label = QString::fromStdString(
            interface_port.name.empty() ? id.value : interface_port.name);
        node.title = (kind == Node::Kind::GraphInput
                ? QStringLiteral("Input: ")
                : QStringLiteral("Output: "))
            + label;
        node.position = kind == Node::Kind::GraphInput
            ? QPointF{-280.0, 80.0 + boundary_index * 150.0}
            : QPointF{980.0, 80.0 + boundary_index * 150.0};
        node.size = QSizeF{240.0, 86.0};
        node.color = kind == Node::Kind::GraphInput
            ? QColor{QStringLiteral("#4c78a8")}
            : QColor{QStringLiteral("#5b8e7d")};

        Port port;
        port.id = QString::fromStdString(id.value);
        port.name = label;
        port.output = kind == Node::Kind::GraphInput;
        port.direction = interface_port.direction;
        port.cardinality = interface_port.type.kind
            == orlgraph::LogicalTypeKind::Buffer
            ? orlgraph::PortCardinality::Buffer
            : orlgraph::PortCardinality::Scalar;
        port.type = interface_port.type;
        port.domain = interface_port.domain;
        port.shape = interface_port.shape;
        port.semantic =
            QString::fromStdString(interface_port.semantic);
        if (port.output) {
            node.outputs.push_back(std::move(port));
        } else {
            node.inputs.push_back(std::move(port));
        }
        restore_position(node);
        nodes_.push_back(std::move(node));
    };

    int boundary_index = 0;
    for (const auto& [id, input] : active_graph().inputs()) {
        add_interface_node(id, input, Node::Kind::GraphInput, boundary_index++);
    }
    boundary_index = 0;
    for (const auto& [id, output] : active_graph().outputs()) {
        add_interface_node(id, output, Node::Kind::GraphOutput, boundary_index++);
    }

    const auto find_port = [](const QVector<Port>& ports, const QString& id) {
        for (int port_index = 0; port_index < ports.size(); ++port_index) {
            if (ports[port_index].id == id) {
                return port_index;
            }
        }
        return -1;
    };

    const auto find_endpoint_node = [this](const orlgraph::Endpoint& endpoint) {
        for (int node_index = 0; node_index < nodes_.size(); ++node_index) {
            const Node& node = nodes_[node_index];
            if (endpoint.kind == orlgraph::EndpointKind::NodePort
                && node.kind == Node::Kind::Definition
                && node.id.toStdString() == endpoint.owner.value)
            {
                return node_index;
            }
            if (endpoint.kind == orlgraph::EndpointKind::GraphInput
                && node.kind == Node::Kind::GraphInput
                && node.interface_id == endpoint.owner)
            {
                return node_index;
            }
            if (endpoint.kind == orlgraph::EndpointKind::GraphOutput
                && node.kind == Node::Kind::GraphOutput
                && node.interface_id == endpoint.owner)
            {
                return node_index;
            }
        }
        return -1;
    };

    for (const auto& connection : active_graph().connections()) {
        const int source_node = find_endpoint_node(connection.source);
        const int destination_node = find_endpoint_node(connection.destination);
        if (source_node < 0 || destination_node < 0) {
            continue;
        }

        const auto endpoint_port_id = [](const orlgraph::Endpoint& endpoint) {
            return QString::fromStdString(
                endpoint.kind == orlgraph::EndpointKind::NodePort
                    ? endpoint.port.value : endpoint.owner.value);
        };
        const int source_port = find_port(nodes_[source_node].outputs,
            endpoint_port_id(connection.source));
        const int destination_port = find_port(nodes_[destination_node].inputs,
            endpoint_port_id(connection.destination));
        if (source_port >= 0 && destination_port >= 0) {
            links_.push_back(Link{
                source_node, source_port, destination_node, destination_port});
        }
    }
    prune_frames();
    rebuild_find_controls();
    rebuild_frame_controls();
    update();
}

bool NodeGraphEditor::save_project_file(bool save_as)
{
    QString path;
    if (!save_as && graph_file_path_.has_value()) {
        path = *graph_file_path_;
    } else {
        const QString suggested_path = graph_file_path_.value_or(
            QStringLiteral("project.json"));
        path = QFileDialog::getSaveFileName(
            this,
            save_as ? QStringLiteral("Save ORL Project As")
                    : QStringLiteral("Save ORL Project"),
            suggested_path,
            QStringLiteral("ORL Project (*.json);;All Files (*)"));
        if (path.isEmpty()) {
            return false;
        }
    }

    if (QFileInfo(path).suffix().isEmpty()) {
        path += QStringLiteral(".json");
    }

    ProjectIoResult saved;
    if (scene_graph_context_ != nullptr && project_components != nullptr) {
        const NodeGraphLayout layout = export_node_graph_layout();
        saved = save_project_json(path.toStdString(), *scene_graph_context_,
            *project_components, project_weight_id, project_deformer_id,
            runtime_config.evaluate_orl, &layout);
    } else {
        std::vector<orlgraph::Diagnostic> diagnostics;
        if (scene_graph_context_ != nullptr) {
            scene_graph_context_->ensure_stage_graphs();
        }
        const auto& solver = scene_graph_context_ != nullptr
            ? scene_graph_context_->stage_graph(orlgraph::GraphStage::Solver)
            : solver_graph_storage_;
        const auto& deformer = scene_graph_context_ != nullptr
            ? scene_graph_context_->stage_graph(orlgraph::GraphStage::Deformer)
            : deformer_graph_storage_;
        saved.ok = orlgraph::save_graph_stages_json(
            path.toStdString(), solver, deformer, &diagnostics);
        if (!saved.ok && !diagnostics.empty()) {
            saved.errors.push_back(diagnostics.front().message);
        }
    }
    if (!saved.ok) {
        QString message = QStringLiteral("Unable to save ORL project.");
        if (!saved.errors.empty()) {
            message += QStringLiteral("\n")
                + QString::fromStdString(saved.errors.front());
        }
        QMessageBox::warning(this, QStringLiteral("Save ORL Project"),
            message);
        return false;
    }

    graph_file_path_ = path;
    return true;
}

void NodeGraphEditor::create_node(const orlgraph::StableId& definition_id,
    const QPointF& scene_position_value)
{
    orlgraph::StableId created_id;
    std::string error;
    if (!node_graph::create_node(active_graph(), active_registry(),
            stage_, definition_id, &created_id, &error)) {
        std::cerr << "Node graph: failed to create node: " << error << '\n';
        return;
    }

    notify_graph_changed();
    rebuild_view();
    for (int index = 0; index < nodes_.size(); ++index) {
        if (nodes_[index].id != QString::fromStdString(created_id.value)) {
            continue;
        }
        nodes_[index].position = scene_position_value
            - QPointF{nodes_[index].size.width() * 0.5,
                nodes_[index].size.height() * 0.5};
        select_only(index);
        break;
    }
    capture_stage_layout();
    update();
}

void NodeGraphEditor::create_graph_input(
    const orlgraph::StableId& template_id,
    const QPointF& scene_position_value)
{
    const auto* template_input = active_graph().input(template_id);
    if (template_input == nullptr) {
        std::cerr << "Node graph: unknown graph input template: "
                  << template_id.value << '\n';
        return;
    }

    orlgraph::InterfacePort input = *template_input;
    const std::string base_id = template_id.value + ".copy";
    std::size_t suffix = 1;
    for (;;) {
        const orlgraph::StableId candidate{
            base_id + std::to_string(suffix)};
        if (active_graph().input(candidate) == nullptr
            && active_graph().output(candidate) == nullptr)
        {
            input.id = candidate;
            break;
        }
        ++suffix;
    }
    const std::string base_name = input.name.empty()
        ? template_id.value : input.name;
    input.name = base_name + "_copy" + std::to_string(suffix);
    const orlgraph::StableId created_id = input.id;

    std::string error;
    if (!active_graph().add_input(std::move(input), &error)) {
        std::cerr << "Node graph: failed to add graph input: "
                  << error << '\n';
        return;
    }

    notify_graph_changed();
    rebuild_view();
    for (int index = 0; index < nodes_.size(); ++index) {
        if (nodes_[index].kind != Node::Kind::GraphInput
            || nodes_[index].interface_id != created_id)
        {
            continue;
        }
        nodes_[index].position = scene_position_value
            - QPointF{nodes_[index].size.width() * 0.5,
                nodes_[index].size.height() * 0.5};
        select_only(index);
        break;
    }
    capture_stage_layout();
    update();
}

void NodeGraphEditor::clear_find_controls()
{
    for (const auto& control : find_controls_) {
        delete control.combo;
    }
    find_controls_.clear();
}

void NodeGraphEditor::rebuild_find_controls()
{
    for (const auto& node : nodes_) {
        if (node.kind != Node::Kind::Definition) {
            continue;
        }
        const auto* instance = active_graph().node(
            orlgraph::StableId{node.id.toStdString()});
        const auto* definition = instance == nullptr
            ? nullptr : active_registry().find(instance->definition);
        if (definition == nullptr
            || !find_element_kind(definition->qualified_name).has_value())
        {
            continue;
        }

        auto* combo = new QComboBox(this);
        combo->setEditable(false);
        combo->setPlaceholderText(QStringLiteral("Select scene element"));
        combo->setToolTip(QStringLiteral(
            "Select the scene element used by this find node"));
        const QString node_id = node.id;
        QObject::connect(combo, &QComboBox::currentTextChanged, this,
            [this, node_id](const QString& text) {
                const auto iterator = active_graph().mutable_nodes().find(
                    orlgraph::StableId{node_id.toStdString()});
                if (iterator == active_graph().mutable_nodes().end()) {
                    return;
                }
                if (text.isEmpty()) {
                    if (iterator->second.parameter_values.erase("name") != 0) {
                        notify_graph_changed();
                    }
                    return;
                }
                orlgraph::ConstantValue value;
                value.type = orlgraph::LogicalType::string();
                value.value = text.toStdString();
                const auto parameter =
                    iterator->second.parameter_values.find("name");
                const bool changed = parameter
                    == iterator->second.parameter_values.end()
                    || parameter->second.type != value.type
                    || parameter->second.value != value.value;
                iterator->second.parameter_values["name"] = std::move(value);
                if (changed) {
                    notify_graph_changed();
                }
            });
        find_controls_.push_back(FindControl{node.id, combo});
    }
    refresh_find_controls();
    position_find_controls();
}

void NodeGraphEditor::refresh_find_controls()
{
    for (const auto& control : find_controls_) {
        if (control.combo == nullptr) {
            continue;
        }
        const auto instance_iterator = active_graph().nodes().find(
            orlgraph::StableId{control.node_id.toStdString()});
        if (instance_iterator == active_graph().nodes().end()) {
            control.combo->setVisible(false);
            continue;
        }
        const auto* definition = active_registry().find(
            instance_iterator->second.definition);
        if (definition == nullptr) {
            control.combo->setVisible(false);
            continue;
        }
        const auto element_kind = find_element_kind(
            definition->qualified_name);
        if (!element_kind.has_value()) {
            control.combo->setVisible(false);
            continue;
        }

        std::vector<std::string> names;
        if (scene_input_catalog_ != nullptr) {
            names = scene_input_catalog_->element_names(*element_kind);
        }
        std::string selected;
        const auto parameter = instance_iterator->second.parameter_values.find("name");
        if (parameter != instance_iterator->second.parameter_values.end()) {
            if (const auto* value = std::get_if<std::string>(
                    &parameter->second.value)) {
                selected = *value;
            }
        }

        {
            const QSignalBlocker blocker{control.combo};
            if (selected.empty() && !names.empty()) {
                selected = names.front();
            }
            QStringList desired_items;
            desired_items.reserve(static_cast<qsizetype>(names.size()));
            for (const auto& name : names) {
                desired_items.push_back(QString::fromStdString(name));
            }
            const QString selected_text = QString::fromStdString(selected);
            if (!selected.empty()
                && !desired_items.contains(selected_text))
            {
                desired_items.push_back(selected_text);
            }

            bool items_match = control.combo->count()
                == desired_items.size();
            for (int index = 0; items_match
                && index < control.combo->count(); ++index)
            {
                items_match = control.combo->itemText(index)
                    == desired_items.at(index);
            }
            const bool popup_visible = control.combo->view() != nullptr
                && control.combo->view()->isVisible();
            if (!popup_visible || items_match) {
                if (!items_match) {
                    control.combo->clear();
                    control.combo->addItems(desired_items);
                }
                const int selected_index = selected.empty()
                    ? -1 : control.combo->findText(selected_text);
                if (control.combo->currentIndex() != selected_index) {
                    control.combo->setCurrentIndex(selected_index);
                }
            }
            control.combo->setEnabled(!names.empty());
        }

        auto& node = active_graph().mutable_nodes().find(
            orlgraph::StableId{control.node_id.toStdString()})->second;
        bool changed = false;
        if (selected.empty()) {
            changed = node.parameter_values.erase("name") != 0;
        } else {
            orlgraph::ConstantValue value;
            value.type = orlgraph::LogicalType::string();
            value.value = selected;
            const auto parameter = node.parameter_values.find("name");
            changed = parameter == node.parameter_values.end()
                || parameter->second.type != value.type
                || parameter->second.value != value.value;
            node.parameter_values["name"] = std::move(value);
        }
        if (changed) {
            notify_graph_changed();
        }
    }
}

void NodeGraphEditor::position_find_controls()
{
    for (const auto& control : find_controls_) {
        if (control.combo == nullptr) {
            continue;
        }
        const auto node_iterator = std::find_if(nodes_.cbegin(), nodes_.cend(),
            [&control](const Node& node) {
                return node.id == control.node_id;
            });
        if (node_iterator == nodes_.cend() || node_is_hidden(*node_iterator)) {
            control.combo->setVisible(false);
            continue;
        }
        const QRectF& node = node_rect(*node_iterator);
        const QRectF local{
            node.left() + 12.0,
            node.bottom() - 29.0,
            node.width() - 24.0,
            22.0};
        const QRectF viewport{
            pan_.x() + local.left() * zoom_,
            pan_.y() + local.top() * zoom_,
            local.width() * zoom_,
            std::max(18.0, local.height() * zoom_)};
        control.combo->setGeometry(viewport.toRect());
        control.combo->setVisible(true);
        control.combo->raise();
    }
}

void NodeGraphEditor::clear_frame_controls()
{
    for (const auto& control : frameControls) {
        delete control.title;
    }
    frameControls.clear();
}

void NodeGraphEditor::rebuild_frame_controls()
{
    clear_frame_controls();
    auto& frames = current_frames();
    for (int index = 0; index < frames.size(); ++index) {
        auto* title = new QLineEdit(this);
        title->setText(frames[index].title);
        title->setPlaceholderText(QStringLiteral("Frame"));
        title->setFrame(false);
        title->setStyleSheet(QStringLiteral(
            "QLineEdit { background: transparent; color: #f6f6f6; border: none; }"
            "QLineEdit:focus { background: #1b1b1b; border: 1px solid #3d3d3d; }"));
        const QString frame_id = frames[index].id;
        QObject::connect(title, &QLineEdit::textChanged, this,
            [this, frame_id](const QString& text) {
                auto& frames = current_frames();
                for (auto& frame : frames) {
                    if (frame.id != frame_id) {
                        continue;
                    }
                    frame.title = text;
                    capture_stage_layout();
                    position_frame_controls();
                    update();
                    return;
                }
            });
        frameControls.push_back(FrameControl{frame_id, title});
    }
    position_frame_controls();
}

void NodeGraphEditor::position_frame_controls()
{
    const auto& frames = current_frames();
    for (const auto& control : frameControls) {
        if (control.title == nullptr) {
            continue;
        }
        const auto frame_iterator = std::find_if(frames.cbegin(), frames.cend(),
            [&control](const Frame& frame) {
                return frame.id == control.frame_id;
            });
        if (frame_iterator == frames.cend()) {
            control.title->setVisible(false);
            continue;
        }
        const QRectF header = frame_header_rect(*frame_iterator);
        const QRectF button = collapse_button_rect(*frame_iterator);
        const QRectF local{
            button.right() + 8.0,
            header.top() + 6.0,
            std::max(40.0, header.right() - 10.0 - (button.right() + 8.0)),
            header.height() - 12.0};
        const QRectF viewport{
            pan_.x() + local.left() * zoom_,
            pan_.y() + local.top() * zoom_,
            local.width() * zoom_,
            std::max(16.0, local.height() * zoom_)};
        control.title->setGeometry(viewport.toRect());
        control.title->setVisible(true);
        control.title->raise();
    }
}

void NodeGraphEditor::prune_frames()
{
    QSet<QString> node_ids;
    for (const auto& node : nodes_) {
        node_ids.insert(node.id);
    }
    auto& frames = current_frames();
    for (auto& frame : frames) {
        frame.members.erase(std::remove_if(frame.members.begin(),
            frame.members.end(),
            [&node_ids](const QString& member) {
                return !node_ids.contains(member);
            }), frame.members.end());
    }
    frames.erase(std::remove_if(frames.begin(), frames.end(),
        [](const Frame& frame) { return frame.members.isEmpty(); }),
        frames.end());
}

QVector<NodeGraphEditor::Frame>& NodeGraphEditor::current_frames()
{
    return stage_layout(stage_).frames;
}

const QVector<NodeGraphEditor::Frame>& NodeGraphEditor::current_frames() const
{
    return stage_layout(stage_).frames;
}

QPointF NodeGraphEditor::scene_position(const QPointF& viewport_position) const
{
    return (viewport_position - pan_) / zoom_;
}

QRectF NodeGraphEditor::node_rect(const Node& node) const
{
    return QRectF(node.position, node.size);
}

QRectF NodeGraphEditor::expanded_frame_rect(const Frame& frame) const
{
    return QRectF(frame.position, frame.size);
}

QRectF NodeGraphEditor::collapsed_frame_rect(const Frame& frame) const
{
    const QFontMetrics metrics(font());
    const double title_width = metrics.horizontalAdvance(
        frame.title.isEmpty() ? QStringLiteral("Frame") : frame.title);
    const double width = std::max(node_graph::kFrameCollapsedMinWidth,
        std::min(frame.size.width(), title_width + 56.0));
    return QRectF(frame.position,
        QSizeF(width, node_graph::kFrameCollapsedHeight));
}

QRectF NodeGraphEditor::frame_rect(const Frame& frame) const
{
    return frame.collapsed
        ? collapsed_frame_rect(frame) : expanded_frame_rect(frame);
}

QRectF NodeGraphEditor::frame_header_rect(const Frame& frame) const
{
    const QRectF rect = frame_rect(frame);
    return QRectF(rect.topLeft(),
        QSizeF(rect.width(), std::min(rect.height(), node_graph::kFrameHeader)));
}

QRectF NodeGraphEditor::collapse_button_rect(const Frame& frame) const
{
    const QRectF header = frame_header_rect(frame);
    const double size = 16.0;
    return QRectF(
        header.left() + 8.0,
        header.top() + (header.height() - size) * 0.5,
        size, size);
}

QPointF NodeGraphEditor::frame_socket_position(const Frame& frame,
    bool output) const
{
    const QRectF rect = collapsed_frame_rect(frame);
    return {
        output ? rect.right() : rect.left(),
        rect.center().y(),
    };
}

QPointF NodeGraphEditor::port_position(const Node& node, bool output, int index) const
{
    const double y = node.position.y() + 42.0 + index * 22.0;
    const double x = output
        ? node.position.x() + node.size.width()
        : node.position.x();
    return {x, y};
}

int NodeGraphEditor::node_at(const QPointF& scene) const
{
    for (int index = nodes_.size() - 1; index >= 0; --index) {
        if (node_is_hidden(nodes_[index])) {
            continue;
        }
        if (node_rect(nodes_[index]).contains(scene)) {
            return index;
        }
    }
    return -1;
}

int NodeGraphEditor::frame_at(const QPointF& scene) const
{
    const auto& frames = current_frames();
    for (int index = frames.size() - 1; index >= 0; --index) {
        if (frame_rect(frames[index]).contains(scene)) {
            return index;
        }
    }
    return -1;
}

int NodeGraphEditor::collapse_button_at(const QPointF& scene) const
{
    const auto& frames = current_frames();
    for (int index = frames.size() - 1; index >= 0; --index) {
        if (collapse_button_rect(frames[index]).contains(scene)) {
            return index;
        }
    }
    return -1;
}

int NodeGraphEditor::collapsed_frame_for_node(const QString& node_id) const
{
    const auto& frames = current_frames();
    for (int index = 0; index < frames.size(); ++index) {
        if (frames[index].collapsed && frames[index].members.contains(node_id)) {
            return index;
        }
    }
    return -1;
}

bool NodeGraphEditor::node_is_hidden(const Node& node) const
{
    return collapsed_frame_for_node(node.id) >= 0;
}

int NodeGraphEditor::port_at(const Node& node, bool output,
    const QPointF& scene) const
{
    const auto& ports = output ? node.outputs : node.inputs;
    for (int index = 0; index < ports.size(); ++index) {
        const QPointF position = port_position(node, output, index);
        const double distance = std::hypot(
            position.x() - scene.x(), position.y() - scene.y());
        if (distance <= 10.0 / zoom_) {
            return index;
        }
    }
    return -1;
}

NodeGraphEditor::Socket NodeGraphEditor::socket_at(const QPointF& scene) const
{
    for (int node_index = nodes_.size() - 1; node_index >= 0; --node_index) {
        const Node& node = nodes_[node_index];
        if (node_is_hidden(node)) {
            continue;
        }
        const int input = port_at(node, false, scene);
        if (input >= 0) {
            return Socket{node_index, input, false};
        }
        const int output = port_at(node, true, scene);
        if (output >= 0) {
            return Socket{node_index, output, true};
        }
    }

    const auto& frames = current_frames();
    const double radius = 10.0 / zoom_;
    for (int index = frames.size() - 1; index >= 0; --index) {
        if (!frames[index].collapsed) {
            continue;
        }
        const QPointF input = frame_socket_position(frames[index], false);
        if (std::hypot(input.x() - scene.x(), input.y() - scene.y()) <= radius) {
            return Socket{-1, 0, false, index};
        }
        const QPointF output = frame_socket_position(frames[index], true);
        if (std::hypot(output.x() - scene.x(), output.y() - scene.y()) <= radius) {
            return Socket{-1, 0, true, index};
        }
    }
    return {};
}

NodeGraphEditor::Socket NodeGraphEditor::resolve_socket(const Socket& socket,
    const Socket* other) const
{
    if (socket.frame < 0) {
        return socket;
    }
    const auto& frames = current_frames();
    if (socket.frame >= frames.size()) {
        return {};
    }
    const Frame& frame = frames[socket.frame];
    Socket fallback;
    for (const auto& member_id : frame.members) {
        int node_index = -1;
        for (int index = 0; index < nodes_.size(); ++index) {
            if (nodes_[index].id == member_id) {
                node_index = index;
                break;
            }
        }
        if (node_index < 0) {
            continue;
        }
        const Node& node = nodes_[node_index];
        const auto& ports = socket.output ? node.outputs : node.inputs;
        for (int port = 0; port < ports.size(); ++port) {
            Socket candidate{node_index, port, socket.output};
            if (!fallback.valid()) {
                fallback = candidate;
            }
            if (other == nullptr || !other->valid()) {
                continue;
            }
            const Socket resolved_other = other->frame >= 0
                ? Socket{} : *other;
            if (!resolved_other.valid()) {
                continue;
            }
            if (sockets_compatible(candidate, resolved_other)) {
                return candidate;
            }
        }
    }
    return fallback;
}

bool NodeGraphEditor::sockets_compatible(const Socket& first,
    const Socket& second) const
{
    if (!first.valid() || !second.valid()
        || first.output == second.output)
    {
        return false;
    }
    if (first.frame >= 0 && first.frame == second.frame) {
        return false;
    }

    const Socket resolved_first = resolve_socket(first, &second);
    const Socket resolved_second = resolve_socket(second, &first);
    if (!resolved_first.valid() || !resolved_second.valid()
        || resolved_first.output == resolved_second.output
        || resolved_first.node == resolved_second.node)
    {
        return false;
    }
    if (resolved_first.node < 0 || resolved_first.node >= nodes_.size()
        || resolved_second.node < 0 || resolved_second.node >= nodes_.size())
    {
        return false;
    }

    const Socket source = resolved_first.output ? resolved_first : resolved_second;
    const Socket destination = resolved_first.output ? resolved_second : resolved_first;
    const Node& source_node = nodes_[source.node];
    const Node& destination_node = nodes_[destination.node];
    if (source.port < 0 || source.port >= source_node.outputs.size()
        || destination.port < 0
        || destination.port >= destination_node.inputs.size())
    {
        return false;
    }

    const Port& source_port = source_node.outputs[source.port];
    const Port& destination_port = destination_node.inputs[destination.port];
    if (!source_port.type.is_known() || !destination_port.type.is_known()) {
        return false;
    }

    orlgraph::Port source_contract;
    source_contract.id = orlgraph::StableId{source_port.id.toStdString()};
    source_contract.name = source_port.name.toStdString();
    source_contract.direction =
        source_port.direction == orlgraph::PortDirection::Input
            ? orlgraph::PortDirection::Output : source_port.direction;
    source_contract.cardinality = source_port.cardinality;
    source_contract.type = source_port.type;
    source_contract.domain = source_port.domain;
    source_contract.shape = source_port.shape;
    source_contract.semantic = source_port.semantic.toStdString();

    orlgraph::Port destination_contract;
    destination_contract.id = orlgraph::StableId{
        destination_port.id.toStdString()};
    destination_contract.name = destination_port.name.toStdString();
    destination_contract.direction =
        destination_port.direction == orlgraph::PortDirection::Output
            ? orlgraph::PortDirection::Input : destination_port.direction;
    destination_contract.cardinality = destination_port.cardinality;
    destination_contract.type = destination_port.type;
    destination_contract.domain = destination_port.domain;
    destination_contract.shape = destination_port.shape;
    destination_contract.semantic =
        destination_port.semantic.toStdString();

    if (!destination_contract.compatible_value(source_contract)) {
        return false;
    }
    if (source_port.shape != destination_port.shape
        && !source_port.shape.is_scalar()
        && !destination_port.shape.is_scalar())
    {
        return false;
    }
    return true;
}

bool NodeGraphEditor::add_connection(const Socket& first, const Socket& second)
{
    if (!sockets_compatible(first, second)) {
        return false;
    }

    const Socket resolved_first = resolve_socket(first, &second);
    const Socket resolved_second = resolve_socket(second, &first);
    const Socket source = resolved_first.output ? resolved_first : resolved_second;
    const Socket destination = resolved_first.output ? resolved_second : resolved_first;
    const auto endpoint_for = [this](const Socket& socket, bool source_endpoint) {
        const Node& node = nodes_[socket.node];
        if (source_endpoint && node.kind == Node::Kind::GraphInput) {
            return orlgraph::Endpoint::graph_input(node.interface_id);
        }
        if (!source_endpoint && node.kind == Node::Kind::GraphOutput) {
            return orlgraph::Endpoint::graph_output(node.interface_id);
        }
        const auto& ports = source_endpoint ? node.outputs : node.inputs;
        return orlgraph::Endpoint::node_port(
            orlgraph::StableId{node.id.toStdString()},
            orlgraph::StableId{ports[socket.port].id.toStdString()});
    };
    const orlgraph::Endpoint source_endpoint = endpoint_for(source, true);
    const orlgraph::Endpoint destination_endpoint =
        endpoint_for(destination, false);

    auto& connections = active_graph().mutable_connections();
    std::vector<orlgraph::Connection> replaced_connections;
    for (auto connection = connections.begin(); connection != connections.end();) {
        if (connection->destination == destination_endpoint)
        {
            replaced_connections.push_back(*connection);
            connection = connections.erase(connection);
        } else {
            ++connection;
        }
    }

    std::string error;
    if (!active_graph().add_connection(orlgraph::Connection{
            source_endpoint, destination_endpoint},
            &error))
    {
        connections.insert(connections.end(),
            replaced_connections.begin(), replaced_connections.end());
        if (!error.empty()) {
            std::cerr << "Node graph: failed to connect sockets: "
                      << error << '\n';
        }
        return false;
    }
    notify_graph_changed();
    rebuild_view();
    return true;
}

void NodeGraphEditor::draw_grid(QPainter& painter) const
{
    const QRectF visible(
        -pan_.x() / zoom_,
        -pan_.y() / zoom_,
        width() / zoom_,
        height() / zoom_);
    const double minor_step = 20.0;
    const double major_step = 100.0;
    const int first_x = static_cast<int>(std::floor(visible.left() / minor_step)) - 1;
    const int last_x = static_cast<int>(std::ceil(visible.right() / minor_step)) + 1;
    const int first_y = static_cast<int>(std::floor(visible.top() / minor_step)) - 1;
    const int last_y = static_cast<int>(std::ceil(visible.bottom() / minor_step)) + 1;

    painter.setPen(QPen(QColor{QStringLiteral("#2a2a2a")}, 1.0 / zoom_));
    for (int x = first_x; x <= last_x; ++x) {
        painter.drawLine(QPointF{x * minor_step, visible.top()},
            QPointF{x * minor_step, visible.bottom()});
    }
    for (int y = first_y; y <= last_y; ++y) {
        painter.drawLine(QPointF{visible.left(), y * minor_step},
            QPointF{visible.right(), y * minor_step});
    }

    painter.setPen(QPen(QColor{QStringLiteral("#373737")}, 1.5 / zoom_));
    const int first_major_x = static_cast<int>(std::floor(visible.left() / major_step)) - 1;
    const int last_major_x = static_cast<int>(std::ceil(visible.right() / major_step)) + 1;
    const int first_major_y = static_cast<int>(std::floor(visible.top() / major_step)) - 1;
    const int last_major_y = static_cast<int>(std::ceil(visible.bottom() / major_step)) + 1;
    for (int x = first_major_x; x <= last_major_x; ++x) {
        painter.drawLine(QPointF{x * major_step, visible.top()},
            QPointF{x * major_step, visible.bottom()});
    }
    for (int y = first_major_y; y <= last_major_y; ++y) {
        painter.drawLine(QPointF{visible.left(), y * major_step},
            QPointF{visible.right(), y * major_step});
    }
}

void NodeGraphEditor::draw_link(QPainter& painter, const Link& link) const
{
    if (link.source_node < 0 || link.source_node >= nodes_.size()
        || link.destination_node < 0 || link.destination_node >= nodes_.size())
    {
        return;
    }
    const Node& source_node = nodes_[link.source_node];
    const Node& destination_node = nodes_[link.destination_node];
    if (link.source_port < 0 || link.source_port >= source_node.outputs.size()
        || link.destination_port < 0
        || link.destination_port >= destination_node.inputs.size())
    {
        return;
    }
    const int source_frame = collapsed_frame_for_node(source_node.id);
    const int destination_frame = collapsed_frame_for_node(destination_node.id);
    if (source_frame >= 0 && source_frame == destination_frame) {
        return;
    }
    const auto& frames = current_frames();
    const QPointF source = source_frame >= 0
        ? frame_socket_position(frames[source_frame], true)
        : port_position(source_node, true, link.source_port);
    const QPointF destination = destination_frame >= 0
        ? frame_socket_position(frames[destination_frame], false)
        : port_position(destination_node, false, link.destination_port);
    const double distance = std::max(60.0, std::abs(destination.x() - source.x()) * 0.5);
    QPainterPath path(source);
    path.cubicTo(source + QPointF{distance, 0.0},
        destination - QPointF{distance, 0.0}, destination);
    painter.setPen(QPen(QColor{QStringLiteral("#e4b85c")}, 2.5 / zoom_));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
}

void NodeGraphEditor::draw_pending_connection(QPainter& painter) const
{
    if (!pending_connection_.start.valid()) {
        return;
    }

    QPointF start;
    if (pending_connection_.start.frame >= 0) {
        const auto& frames = current_frames();
        if (pending_connection_.start.frame >= frames.size()
            || !frames[pending_connection_.start.frame].collapsed)
        {
            return;
        }
        start = frame_socket_position(frames[pending_connection_.start.frame],
            pending_connection_.start.output);
    } else {
        if (pending_connection_.start.node < 0
            || pending_connection_.start.node >= nodes_.size())
        {
            return;
        }
        const Node& node = nodes_[pending_connection_.start.node];
        const auto& ports = pending_connection_.start.output
            ? node.outputs : node.inputs;
        if (pending_connection_.start.port < 0
            || pending_connection_.start.port >= ports.size())
        {
            return;
        }
        start = port_position(node,
            pending_connection_.start.output, pending_connection_.start.port);
    }
    const QPointF end = pending_connection_.current;
    const double distance = std::max(
        50.0, std::abs(end.x() - start.x()) * 0.5);
    QPainterPath path(start);
    path.cubicTo(start + QPointF{distance, 0.0},
        end - QPointF{distance, 0.0}, end);

    QColor color{QStringLiteral("#8b8b8b")};
    if (pending_connection_.hovering_socket) {
        color = pending_connection_.valid_drop
            ? QColor{QStringLiteral("#e4b85c")}
            : QColor{QStringLiteral("#e05a5a")};
    }
    painter.setPen(QPen(color, 2.5 / zoom_));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
}

void NodeGraphEditor::draw_selection_box(QPainter& painter) const
{
    if (!boxSelecting) {
        return;
    }
    const QRectF box = QRectF(boxSelectOrigin, boxSelectCurrent).normalized();
    if (box.width() <= 0.0 && box.height() <= 0.0) {
        return;
    }
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor{QStringLiteral("#f2c14e")}, 1.0 / zoom_,
        Qt::DashLine));
    painter.drawRect(box);
}

bool NodeGraphEditor::node_is_selected(int index) const
{
    return selectedNodes.contains(index);
}

bool NodeGraphEditor::frame_is_selected(int index) const
{
    return selectedFrames.contains(index);
}

void NodeGraphEditor::select_only(int index)
{
    selected_node_ = index;
    selectedNodes.clear();
    selectedFrames.clear();
    if (index >= 0) {
        selectedNodes.push_back(index);
    }
}

void NodeGraphEditor::select_only_frame(int index)
{
    selected_node_ = -1;
    selectedNodes.clear();
    selectedFrames.clear();
    if (index >= 0) {
        selectedFrames.push_back(index);
    }
}

void NodeGraphEditor::move_frame(int index, const QPointF& delta)
{
    auto& frames = current_frames();
    if (index < 0 || index >= frames.size()) {
        return;
    }
    auto& frame = frames[index];
    frame.position += delta;
    for (const auto& member : frame.members) {
        for (auto& node : nodes_) {
            if (node.id == member) {
                node.position += delta;
                break;
            }
        }
    }
}

void NodeGraphEditor::toggle_frame_collapsed(int index)
{
    auto& frames = current_frames();
    if (index < 0 || index >= frames.size()) {
        return;
    }
    frames[index].collapsed = !frames[index].collapsed;
    capture_stage_layout();
    position_find_controls();
    position_frame_controls();
    update();
}

void NodeGraphEditor::apply_box_selection()
{
    const QPointF viewport_delta =
        (boxSelectCurrent - boxSelectOrigin) * zoom_;
    if (std::hypot(viewport_delta.x(), viewport_delta.y()) < 4.0) {
        select_only(-1);
        return;
    }

    const QRectF box = QRectF(boxSelectOrigin, boxSelectCurrent).normalized();
    selectedNodes.clear();
    selectedFrames.clear();
    selected_node_ = -1;
    for (int index = 0; index < nodes_.size(); ++index) {
        if (node_is_hidden(nodes_[index])
            || !node_rect(nodes_[index]).intersects(box))
        {
            continue;
        }
        selectedNodes.push_back(index);
        selected_node_ = index;
    }
    const auto& frames = current_frames();
    for (int index = 0; index < frames.size(); ++index) {
        if (frame_rect(frames[index]).intersects(box)) {
            selectedFrames.push_back(index);
        }
    }
}

void NodeGraphEditor::draw_frame(QPainter& painter, int index) const
{
    const auto& frames = current_frames();
    const Frame& frame = frames[index];
    const QRectF rect = frame_rect(frame);
    const QRectF header = frame_header_rect(frame);
    const QRectF button = collapse_button_rect(frame);
    const bool selected = frame_is_selected(index);

    painter.setPen(QPen(selected
            ? QColor{QStringLiteral("#f2c14e")}
            : QColor{QStringLiteral("#3d3d3d")}, 1.5 / zoom_));
    painter.setBrush(frame.collapsed
        ? QColor{36, 36, 36, 220}
        : QColor{36, 36, 36, 80});
    painter.drawRoundedRect(rect, 7.0, 7.0);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor{QStringLiteral("#242424")});
    painter.drawRoundedRect(header, 7.0, 7.0);
    if (header.height() > 7.0) {
        painter.drawRect(QRectF(header.left(), header.top() + 7.0,
            header.width(), header.height() - 7.0));
    }

    painter.setPen(QPen(QColor{QStringLiteral("#3d3d3d")}, 1.0 / zoom_));
    painter.setBrush(QColor{QStringLiteral("#2a2a2a")});
    painter.drawRoundedRect(button, 3.0, 3.0);
    painter.setPen(QPen(QColor{QStringLiteral("#d7d7d7")}, 1.5 / zoom_));
    const QPointF center = button.center();
    painter.drawLine(QPointF{button.left() + 4.0, center.y()},
        QPointF{button.right() - 4.0, center.y()});
    if (frame.collapsed) {
        painter.drawLine(QPointF{center.x(), button.top() + 4.0},
            QPointF{center.x(), button.bottom() - 4.0});
    }

    if (frame.collapsed) {
        const QPointF input = frame_socket_position(frame, false);
        const QPointF output = frame_socket_position(frame, true);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{QStringLiteral("#d7d7d7")});
        painter.drawEllipse(input, 5.0, 5.0);
        painter.drawEllipse(output, 5.0, 5.0);
    }
}

void NodeGraphEditor::draw_node(QPainter& painter, int index) const
{
    const Node& node = nodes_[index];
    const QRectF rect = node_rect(node);
    const QRectF header(rect.topLeft(), QSizeF(rect.width(), 30.0));

    painter.setPen(QPen(node_is_selected(index)
            ? QColor{QStringLiteral("#f2c14e")}
            : QColor{QStringLiteral("#121212")}, 2.0 / zoom_));
    painter.setBrush(QColor{QStringLiteral("#242424")});
    painter.drawRoundedRect(rect, 7.0, 7.0);
    painter.setPen(Qt::NoPen);
    painter.setBrush(node.color);
    painter.drawRoundedRect(header, 7.0, 7.0);
    painter.drawRect(QRectF(header.left(), header.top() + 7.0,
        header.width(), header.height() - 7.0));

    painter.setPen(QColor{QStringLiteral("#f6f6f6")});
    painter.drawText(header.adjusted(12.0, 0.0, -12.0, 0.0),
        Qt::AlignVCenter | Qt::AlignLeft, node.title);

    const QFontMetrics metrics(painter.font());
    const auto port_label = [](const Port& port) {
        if (port.semantic.endsWith(QStringLiteral(".handle"))) {
            return port.name + QStringLiteral(" (stable)");
        }
        return port.name;
    };
    for (int port = 0; port < node.inputs.size(); ++port) {
        const QPointF position = port_position(node, false, port);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{QStringLiteral("#d7d7d7")});
        painter.drawEllipse(position, 5.0, 5.0);
        painter.setPen(QColor{QStringLiteral("#d7d7d7")});
        const QRectF text_rect(
            position.x() + 12.0,
            position.y() - metrics.height() * 0.5,
            node.size.width() - 24.0,
            metrics.height());
        painter.drawText(text_rect,
            Qt::AlignLeft | Qt::AlignVCenter, port_label(node.inputs[port]));
    }
    for (int port = 0; port < node.outputs.size(); ++port) {
        const QPointF position = port_position(node, true, port);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{QStringLiteral("#d7d7d7")});
        painter.drawEllipse(position, 5.0, 5.0);
        painter.setPen(QColor{QStringLiteral("#d7d7d7")});
        const QRectF text_rect(
            position.x() - node.size.width() + 12.0,
            position.y() - metrics.height() * 0.5,
            node.size.width() - 24.0,
            metrics.height());
        painter.drawText(text_rect,
            Qt::AlignRight | Qt::AlignVCenter, port_label(node.outputs[port]));
    }
}

void NodeGraphEditor::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor{QStringLiteral("#1b1b1b")});

    painter.save();
    painter.translate(pan_);
    painter.scale(zoom_, zoom_);
    draw_grid(painter);
    const auto& frames = current_frames();
    for (int index = 0; index < frames.size(); ++index) {
        if (!frames[index].collapsed) {
            draw_frame(painter, index);
        }
    }
    for (const auto& link : links_) {
        draw_link(painter, link);
    }
    draw_pending_connection(painter);
    for (int index = 0; index < nodes_.size(); ++index) {
        if (!node_is_hidden(nodes_[index])) {
            draw_node(painter, index);
        }
    }
    for (int index = 0; index < frames.size(); ++index) {
        if (frames[index].collapsed) {
            draw_frame(painter, index);
        }
    }
    draw_selection_box(painter);
    painter.restore();

    painter.setPen(QColor{QStringLiteral("#f6f6f6")});
    QFont stage_font = painter.font();
    stage_font.setBold(true);
    stage_font.setPointSizeF(std::max(10.0, stage_font.pointSizeF() + 2.0));
    painter.setFont(stage_font);
    painter.drawText(
        QRectF{16.0, 12.0, 260.0, 28.0},
        Qt::AlignLeft | Qt::AlignVCenter,
        stage_ == orlgraph::GraphStage::Solver
            ? QStringLiteral("Stage: Solver")
            : QStringLiteral("Stage: Deformer"));
}

void NodeGraphEditor::mousePressEvent(QMouseEvent* event)
{
    setFocus(Qt::MouseFocusReason);
    const QPointF scene = scene_position(event->position());
    if (event->button() == Qt::MiddleButton) {
        panning_ = true;
        last_mouse_position_ = event->position().toPoint();
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton) {
        const Socket socket = socket_at(scene);
        if (socket.valid()) {
            if (socket.frame >= 0) {
                select_only_frame(socket.frame);
            } else {
                select_only(socket.node);
            }
            dragging_node_ = -1;
            draggingFrame = -1;
            boxSelecting = false;
            pending_connection_ = PendingConnection{
                socket, scene, false, false};
            update();
            event->accept();
            return;
        }
        const int collapse = collapse_button_at(scene);
        if (collapse >= 0) {
            toggle_frame_collapsed(collapse);
            event->accept();
            return;
        }
        const int hit = node_at(scene);
        if (hit >= 0) {
            select_only(hit);
            dragging_node_ = hit;
            draggingFrame = -1;
            boxSelecting = false;
            drag_offset_ = scene - nodes_[dragging_node_].position;
        } else {
            const int frame = frame_at(scene);
            if (frame >= 0) {
                select_only_frame(frame);
                dragging_node_ = -1;
                draggingFrame = frame;
                boxSelecting = false;
                drag_offset_ = scene - current_frames()[frame].position;
            } else {
                select_only(-1);
                dragging_node_ = -1;
                draggingFrame = -1;
                boxSelecting = true;
                boxSelectOrigin = scene;
                boxSelectCurrent = scene;
            }
        }
        update();
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void NodeGraphEditor::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint current = event->position().toPoint();
    if (panning_) {
        const QPoint delta = current - last_mouse_position_;
        pan_ += QPointF{
            static_cast<qreal>(delta.x()),
            static_cast<qreal>(delta.y())};
        last_mouse_position_ = current;
        position_find_controls();
        position_frame_controls();
        update();
        event->accept();
        return;
    }
    if (pending_connection_.start.valid()) {
        pending_connection_.current = scene_position(event->position());
        const Socket hover = socket_at(pending_connection_.current);
        pending_connection_.hovering_socket = hover.valid();
        pending_connection_.valid_drop = hover.valid()
            && sockets_compatible(pending_connection_.start, hover);
        update();
        event->accept();
        return;
    }
    if (boxSelecting && (event->buttons() & Qt::LeftButton) != 0) {
        boxSelectCurrent = scene_position(event->position());
        update();
        event->accept();
        return;
    }
    if (draggingFrame >= 0 && (event->buttons() & Qt::LeftButton) != 0) {
        const QPointF next = scene_position(event->position()) - drag_offset_;
        auto& frames = current_frames();
        if (draggingFrame < frames.size()) {
            move_frame(draggingFrame, next - frames[draggingFrame].position);
        }
        position_find_controls();
        position_frame_controls();
        update();
        event->accept();
        return;
    }
    if (dragging_node_ >= 0 && (event->buttons() & Qt::LeftButton) != 0) {
        nodes_[dragging_node_].position = scene_position(event->position()) - drag_offset_;
        position_find_controls();
        position_frame_controls();
        update();
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void NodeGraphEditor::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton) {
        panning_ = false;
        capture_stage_layout();
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton) {
        if (pending_connection_.start.valid()) {
            const QPointF scene = scene_position(event->position());
            const Socket target = socket_at(scene);
            if (target.valid()
                && sockets_compatible(pending_connection_.start, target))
            {
                add_connection(pending_connection_.start, target);
            }
            pending_connection_ = {};
            dragging_node_ = -1;
            draggingFrame = -1;
            update();
            event->accept();
            return;
        }
        if (boxSelecting) {
            boxSelectCurrent = scene_position(event->position());
            apply_box_selection();
            boxSelecting = false;
            dragging_node_ = -1;
            draggingFrame = -1;
            update();
            event->accept();
            return;
        }
        dragging_node_ = -1;
        draggingFrame = -1;
        capture_stage_layout();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void NodeGraphEditor::wheelEvent(QWheelEvent* event)
{
    const QPointF cursor = event->position();
    const QPointF before = scene_position(cursor);
    const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
    zoom_ = std::clamp(zoom_ * factor, 0.35, 2.5);
    pan_ = cursor - before * zoom_;
    capture_stage_layout();
    position_find_controls();
    position_frame_controls();
    update();
    event->accept();
}

void NodeGraphEditor::keyPressEvent(QKeyEvent* event)
{
    if (!event->isAutoRepeat()
        && event->modifiers() == Qt::NoModifier)
    {
        if (event->key() == Qt::Key_1) {
            set_stage(orlgraph::GraphStage::Solver);
            event->accept();
            return;
        }
        if (event->key() == Qt::Key_2) {
            set_stage(orlgraph::GraphStage::Deformer);
            event->accept();
            return;
        }
    }
    if (!event->isAutoRepeat()
        && (event->key() == Qt::Key_Delete
            || event->key() == Qt::Key_Backspace))
    {
        bool removed_any = false;
        for (int index : selectedNodes) {
            if (index < 0 || index >= nodes_.size()) {
                continue;
            }
            const Node& selected = nodes_[index];
            std::string error;
            bool removed = false;
            if (selected.kind == Node::Kind::GraphInput) {
                removed = active_graph().remove_input(
                    selected.interface_id, &error);
            } else if (selected.kind == Node::Kind::GraphOutput) {
                removed = active_graph().remove_output(
                    selected.interface_id, &error);
            } else {
                removed = node_graph::delete_node(
                    active_graph(),
                    orlgraph::StableId{selected.id.toStdString()},
                    &error);
            }
            if (!removed) {
                std::cerr << "Node graph: failed to delete node: "
                          << error << '\n';
            } else {
                removed_any = true;
            }
        }
        auto& frames = current_frames();
        if (!selectedFrames.isEmpty()) {
            QVector<Frame> kept;
            kept.reserve(frames.size());
            for (int index = 0; index < frames.size(); ++index) {
                if (!selectedFrames.contains(index)) {
                    kept.push_back(frames[index]);
                }
            }
            frames = std::move(kept);
            selectedFrames.clear();
            rebuild_frame_controls();
            capture_stage_layout();
            update();
        }
        if (removed_any) {
            notify_graph_changed();
            rebuild_view();
        }
        event->accept();
        return;
    }
    if (!event->isAutoRepeat()
        && event->key() == Qt::Key_O
        && event->modifiers().testFlag(Qt::ShiftModifier)
        && !event->modifiers().testFlag(Qt::ControlModifier))
    {
        create_frame_from_selection();
        event->accept();
        return;
    }
    if (!event->isAutoRepeat()
        && event->key() == Qt::Key_A
        && event->modifiers().testFlag(Qt::ShiftModifier))
    {
        const QPoint global_position = QCursor::pos();
        const QPointF scene = scene_position(mapFromGlobal(global_position));
        std::vector<node_graph::GraphInputMenuEntry> graph_inputs;
        for (const auto& [id, input] : active_graph().inputs()) {
            graph_inputs.push_back({
                id,
                input.name.empty() ? id.value : input.name,
            });
        }
        node_graph::show_create_menu(
            this, active_registry(), stage_, graph_inputs,
            global_position,
            [this, scene](const orlgraph::StableId& definition_id) {
                create_node(definition_id, scene);
            },
            [this, scene](const orlgraph::StableId& template_id) {
                create_graph_input(template_id, scene);
            });
        event->accept();
        return;
    }
    QWidget::keyPressEvent(event);
}

} // namespace ORL

#endif
