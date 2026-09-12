#include "node_graph_editor.hpp"

#if ORL_USE_QT6

#include <QComboBox>
#include <QCursor>
#include <QFontMetrics>
#include <QHash>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QSignalBlocker>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "../graph_scene_inputs.hpp"
#include "node_graph/node_ops.hpp"

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
    if (qualified_name == "orlrig.input.find_controller") {
        return SceneElementKind::Controller;
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
    reset_demo_graph();
}

void NodeGraphEditor::set_graph(const orlgraph::GraphModule& module,
    const orlgraph::NodeRegistry& registry)
{
    graph_ = module;
    registry_ = registry;
    rebuild_view();
}

void NodeGraphEditor::set_scene_input_catalog(
    const SceneInputCatalog* catalog)
{
    scene_input_catalog_ = catalog;
    rebuild_view();
}

void NodeGraphEditor::refresh_scene_inputs()
{
    refresh_find_controls();
    position_find_controls();
}

void NodeGraphEditor::rebuild_view()
{
    clear_find_controls();
    QHash<QString, QPointF> previous_positions;
    for (const auto& node : nodes_) {
        previous_positions.insert(node.id, node.position);
    }

    nodes_.clear();
    links_.clear();
    selected_node_ = -1;
    dragging_node_ = -1;
    pending_connection_ = {};

    const auto restore_position = [&previous_positions](Node& node) {
        if (const auto previous = previous_positions.constFind(node.id);
            previous != previous_positions.constEnd())
        {
            node.position = previous.value();
        }
    };

    int index = 0;
    for (const auto& [id, instance] : graph_.nodes()) {
        Node node;
        node.id = QString::fromStdString(id.value);
        node.title = QString::fromStdString(instance.name.empty()
            ? id.value : instance.name);
        node.position = QPointF(
            64.0 + static_cast<double>(index % 3) * 300.0,
            80.0 + static_cast<double>(index / 3) * 210.0);
        node.color = kNodeColors[index % kNodeColors.size()];

        if (const auto* definition = registry_.find(instance.definition)) {
            node.title = QString::fromStdString(definition->qualified_name);
            for (const auto& port : definition->inputs) {
                Port view_port;
                view_port.id = QString::fromStdString(port.id.value);
                view_port.name = QString::fromStdString(port.name);
                view_port.direction = port.direction;
                view_port.cardinality = port.cardinality;
                view_port.type = port.type;
                view_port.domain = port.domain;
                view_port.shape = port.shape;
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
                node.outputs.push_back(std::move(view_port));
            }
        }
        const int rows = std::max(node.inputs.size(), node.outputs.size());
        node.size.setHeight(std::max(86.0, 42.0 + rows * 22.0));
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
        if (port.output) {
            node.outputs.push_back(std::move(port));
        } else {
            node.inputs.push_back(std::move(port));
        }
        restore_position(node);
        nodes_.push_back(std::move(node));
    };

    int boundary_index = 0;
    for (const auto& [id, input] : graph_.inputs()) {
        add_interface_node(id, input, Node::Kind::GraphInput, boundary_index++);
    }
    boundary_index = 0;
    for (const auto& [id, output] : graph_.outputs()) {
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

    for (const auto& connection : graph_.connections()) {
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
    rebuild_find_controls();
    update();
}

void NodeGraphEditor::create_node(const orlgraph::StableId& definition_id,
    const QPointF& scene_position_value)
{
    orlgraph::StableId created_id;
    std::string error;
    if (!node_graph::create_node(graph_, registry_, definition_id, &created_id, &error)) {
        std::cerr << "Node graph: failed to create node: " << error << '\n';
        return;
    }

    rebuild_view();
    for (int index = 0; index < nodes_.size(); ++index) {
        if (nodes_[index].id != QString::fromStdString(created_id.value)) {
            continue;
        }
        nodes_[index].position = scene_position_value
            - QPointF{nodes_[index].size.width() * 0.5,
                nodes_[index].size.height() * 0.5};
        selected_node_ = index;
        break;
    }
    update();
}

void NodeGraphEditor::create_graph_input(
    const orlgraph::StableId& template_id,
    const QPointF& scene_position_value)
{
    const auto* template_input = graph_.input(template_id);
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
        if (graph_.input(candidate) == nullptr
            && graph_.output(candidate) == nullptr)
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
    if (!graph_.add_input(std::move(input), &error)) {
        std::cerr << "Node graph: failed to add graph input: "
                  << error << '\n';
        return;
    }

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
        selected_node_ = index;
        break;
    }
    update();
}

void NodeGraphEditor::create_scene_input(
    const orlgraph::StableId& input_id,
    const QPointF& scene_position_value)
{
    if (scene_input_catalog_ == nullptr) {
        std::cerr << "Node graph: scene input catalog is unavailable\n";
        return;
    }

    orlgraph::InterfacePort input;
    std::string error;
    if (!scene_input_catalog_->make_interface_port(input_id, &input, &error)) {
        std::cerr << "Node graph: failed to create scene input: "
                  << error << '\n';
        return;
    }
    if (graph_.input(input.id) != nullptr) {
        std::cerr << "Node graph: graph input already exists: "
                  << input.id.value << '\n';
        return;
    }
    if (!graph_.add_input(std::move(input), &error)) {
        std::cerr << "Node graph: failed to add scene input: "
                  << error << '\n';
        return;
    }

    rebuild_view();
    for (int index = 0; index < nodes_.size(); ++index) {
        if (nodes_[index].kind != Node::Kind::GraphInput
            || nodes_[index].interface_id != input_id)
        {
            continue;
        }
        nodes_[index].position = scene_position_value
            - QPointF{nodes_[index].size.width() * 0.5,
                nodes_[index].size.height() * 0.5};
        selected_node_ = index;
        break;
    }
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
        const auto* instance = graph_.node(
            orlgraph::StableId{node.id.toStdString()});
        const auto* definition = instance == nullptr
            ? nullptr : registry_.find(instance->definition);
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
                const auto iterator = graph_.mutable_nodes().find(
                    orlgraph::StableId{node_id.toStdString()});
                if (iterator == graph_.mutable_nodes().end()) {
                    return;
                }
                if (text.isEmpty()) {
                    iterator->second.parameter_values.erase("name");
                    return;
                }
                orlgraph::ConstantValue value;
                value.type = orlgraph::LogicalType::string();
                value.value = text.toStdString();
                iterator->second.parameter_values["name"] = std::move(value);
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
        const auto instance_iterator = graph_.nodes().find(
            orlgraph::StableId{control.node_id.toStdString()});
        if (instance_iterator == graph_.nodes().end()) {
            control.combo->setVisible(false);
            continue;
        }
        const auto* definition = registry_.find(
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
            control.combo->clear();
            for (const auto& name : names) {
                control.combo->addItem(QString::fromStdString(name));
            }
            if (!selected.empty()
                && control.combo->findText(
                    QString::fromStdString(selected)) < 0)
            {
                control.combo->addItem(
                    QString::fromStdString(selected));
            }
            if (selected.empty() && control.combo->count() > 0) {
                selected = control.combo->itemText(0).toStdString();
            }
            if (!selected.empty()) {
                control.combo->setCurrentText(
                    QString::fromStdString(selected));
            } else {
                control.combo->setCurrentIndex(-1);
            }
            control.combo->setEnabled(!names.empty());
        }

        auto& node = graph_.mutable_nodes().find(
            orlgraph::StableId{control.node_id.toStdString()})->second;
        if (selected.empty()) {
            node.parameter_values.erase("name");
        } else {
            orlgraph::ConstantValue value;
            value.type = orlgraph::LogicalType::string();
            value.value = selected;
            node.parameter_values["name"] = std::move(value);
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
        if (node_iterator == nodes_.cend()) {
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

void NodeGraphEditor::reset_demo_graph()
{
    clear_find_controls();
    nodes_.clear();
    links_.clear();
    selected_node_ = -1;
    dragging_node_ = -1;
    pending_connection_ = {};

    nodes_.push_back(Node{
        QStringLiteral("input"), QStringLiteral("Graph Input"), {64.0, 96.0}, {220.0, 100.0},
        QColor{QStringLiteral("#4c78a8")},
        {}, {Port{QStringLiteral("value"), QStringLiteral("value"), true}},
    });
    nodes_.push_back(Node{
        QStringLiteral("deform"), QStringLiteral("LBS Deformer"), {370.0, 96.0}, {240.0, 160.0},
        QColor{QStringLiteral("#b7791f")},
        {
            Port{QStringLiteral("positions"), QStringLiteral("positions"), false},
            Port{QStringLiteral("joints"), QStringLiteral("joints"), false},
            Port{QStringLiteral("weights"), QStringLiteral("weights"), false},
        },
        {Port{QStringLiteral("posed"), QStringLiteral("posed_positions"), true}},
    });
    nodes_.push_back(Node{
        QStringLiteral("output"), QStringLiteral("Graph Output"), {700.0, 96.0}, {220.0, 100.0},
        QColor{QStringLiteral("#5b8e7d")},
        {Port{QStringLiteral("posed"), QStringLiteral("posed_positions"), false}}, {},
    });
    links_ = {
        {0, 0, 1, 0},
        {1, 0, 2, 0},
    };
    update();
}

QPointF NodeGraphEditor::scene_position(const QPointF& viewport_position) const
{
    return (viewport_position - pan_) / zoom_;
}

QRectF NodeGraphEditor::node_rect(const Node& node) const
{
    return QRectF(node.position, node.size);
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
        if (node_rect(nodes_[index]).contains(scene)) {
            return index;
        }
    }
    return -1;
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
        const int input = port_at(node, false, scene);
        if (input >= 0) {
            return Socket{node_index, input, false};
        }
        const int output = port_at(node, true, scene);
        if (output >= 0) {
            return Socket{node_index, output, true};
        }
    }
    return {};
}

bool NodeGraphEditor::sockets_compatible(const Socket& first,
    const Socket& second) const
{
    if (!first.valid() || !second.valid()
        || first.output == second.output
        || first.node == second.node)
    {
        return false;
    }
    if (first.node < 0 || first.node >= nodes_.size()
        || second.node < 0 || second.node >= nodes_.size())
    {
        return false;
    }

    const Socket source = first.output ? first : second;
    const Socket destination = first.output ? second : first;
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

    const Socket source = first.output ? first : second;
    const Socket destination = first.output ? second : first;
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

    auto& connections = graph_.mutable_connections();
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
    if (!graph_.add_connection(orlgraph::Connection{
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
    const QPointF source = port_position(source_node, true, link.source_port);
    const QPointF destination = port_position(destination_node, false, link.destination_port);
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
    if (!pending_connection_.start.valid()
        || pending_connection_.start.node < 0
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

    const QPointF start = port_position(node,
        pending_connection_.start.output, pending_connection_.start.port);
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

void NodeGraphEditor::draw_node(QPainter& painter, int index) const
{
    const Node& node = nodes_[index];
    const QRectF rect = node_rect(node);
    const QRectF header(rect.topLeft(), QSizeF(rect.width(), 30.0));

    painter.setPen(QPen(index == selected_node_
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
    for (int port = 0; port < node.inputs.size(); ++port) {
        const QPointF position = port_position(node, false, port);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{QStringLiteral("#d7d7d7")});
        painter.drawEllipse(position, 5.0, 5.0);
        painter.setPen(QColor{QStringLiteral("#d7d7d7")});
        painter.drawText(QPointF{position.x() + 12.0,
                position.y() + metrics.ascent() * 0.35},
            node.inputs[port].name);
    }
    for (int port = 0; port < node.outputs.size(); ++port) {
        const QPointF position = port_position(node, true, port);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor{QStringLiteral("#d7d7d7")});
        painter.drawEllipse(position, 5.0, 5.0);
        painter.setPen(QColor{QStringLiteral("#d7d7d7")});
        const QRectF text_rect(
            position.x() - node.size.width() + 12.0,
            position.y() - metrics.ascent(),
            node.size.width() - 24.0,
            metrics.height());
        painter.drawText(text_rect, Qt::AlignRight, node.outputs[port].name);
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
    for (const auto& link : links_) {
        draw_link(painter, link);
    }
    draw_pending_connection(painter);
    for (int index = 0; index < nodes_.size(); ++index) {
        draw_node(painter, index);
    }
    painter.restore();
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
            selected_node_ = socket.node;
            dragging_node_ = -1;
            pending_connection_ = PendingConnection{
                socket, scene, false, false};
            update();
            event->accept();
            return;
        }
        selected_node_ = node_at(scene);
        dragging_node_ = selected_node_;
        if (dragging_node_ >= 0) {
            drag_offset_ = scene - nodes_[dragging_node_].position;
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
    if (dragging_node_ >= 0 && (event->buttons() & Qt::LeftButton) != 0) {
        nodes_[dragging_node_].position = scene_position(event->position()) - drag_offset_;
        position_find_controls();
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
            update();
            event->accept();
            return;
        }
        dragging_node_ = -1;
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
    position_find_controls();
    update();
    event->accept();
}

void NodeGraphEditor::keyPressEvent(QKeyEvent* event)
{
    if (!event->isAutoRepeat()
        && (event->key() == Qt::Key_Delete
            || event->key() == Qt::Key_Backspace))
    {
        if (selected_node_ >= 0 && selected_node_ < nodes_.size()) {
            const Node& selected = nodes_[selected_node_];
            std::string error;
            bool removed = false;
            if (selected.kind == Node::Kind::GraphInput) {
                removed = graph_.remove_input(selected.interface_id, &error);
            } else if (selected.kind == Node::Kind::GraphOutput) {
                removed = graph_.remove_output(selected.interface_id, &error);
            } else {
                removed = node_graph::delete_node(
                    graph_,
                    orlgraph::StableId{selected.id.toStdString()},
                    &error);
            }
            if (!removed) {
                std::cerr << "Node graph: failed to delete node: "
                          << error << '\n';
            } else {
                rebuild_view();
            }
        }
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
        for (const auto& [id, input] : graph_.inputs()) {
            graph_inputs.push_back({
                id,
                input.name.empty() ? id.value : input.name,
            });
        }
        std::vector<node_graph::SceneInputMenuEntry> scene_inputs;
        if (scene_input_catalog_ != nullptr) {
            for (const auto& input : scene_input_catalog_->descriptors()) {
                scene_inputs.push_back({
                    input.id,
                    input.label,
                });
            }
        }
        node_graph::show_create_menu(this, registry_, graph_inputs, scene_inputs,
            global_position,
            [this, scene](const orlgraph::StableId& definition_id) {
                create_node(definition_id, scene);
            },
            [this, scene](const orlgraph::StableId& template_id) {
                create_graph_input(template_id, scene);
            },
            [this, scene](const orlgraph::StableId& input_id) {
                create_scene_input(input_id, scene);
            });
        event->accept();
        return;
    }
    QWidget::keyPressEvent(event);
}

} // namespace ORL

#endif
