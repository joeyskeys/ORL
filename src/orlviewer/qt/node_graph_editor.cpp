#include "node_graph_editor.hpp"

#if ORL_USE_QT6

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <utility>

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
    nodes_.clear();
    links_.clear();
    selected_node_ = -1;
    dragging_node_ = -1;

    int index = 0;
    for (const auto& [id, instance] : module.nodes()) {
        Node node;
        node.id = QString::fromStdString(id.value);
        node.title = QString::fromStdString(instance.name.empty()
            ? id.value : instance.name);
        node.position = QPointF(
            64.0 + static_cast<double>(index % 3) * 300.0,
            80.0 + static_cast<double>(index / 3) * 210.0);
        node.color = kNodeColors[index % kNodeColors.size()];

        if (const auto* definition = registry.find(instance.definition)) {
            node.title = QString::fromStdString(definition->qualified_name);
            for (const auto& port : definition->inputs) {
                node.inputs.push_back(Port{
                    QString::fromStdString(port.id.value),
                    QString::fromStdString(port.name),
                    false,
                });
            }
            for (const auto& port : definition->outputs) {
                node.outputs.push_back(Port{
                    QString::fromStdString(port.id.value),
                    QString::fromStdString(port.name),
                    true,
                });
            }
        }
        const int rows = std::max(node.inputs.size(), node.outputs.size());
        node.size.setHeight(std::max(86.0, 42.0 + rows * 22.0));
        nodes_.push_back(std::move(node));
        ++index;
    }

    const auto find_port = [](const QVector<Port>& ports, const QString& id) {
        for (int port_index = 0; port_index < ports.size(); ++port_index) {
            if (ports[port_index].id == id) {
                return port_index;
            }
        }
        return -1;
    };
    for (const auto& connection : module.connections()) {
        if (connection.source.kind != orlgraph::EndpointKind::NodePort
            || connection.destination.kind != orlgraph::EndpointKind::NodePort)
        {
            continue;
        }
        int source_node = -1;
        int destination_node = -1;
        for (int node_index = 0; node_index < nodes_.size(); ++node_index) {
            if (nodes_[node_index].id
                == QString::fromStdString(connection.source.owner.value))
            {
                source_node = node_index;
            }
            if (nodes_[node_index].id
                == QString::fromStdString(connection.destination.owner.value))
            {
                destination_node = node_index;
            }
        }
        if (source_node < 0 || destination_node < 0) {
            continue;
        }
        const int source_port = find_port(nodes_[source_node].outputs,
            QString::fromStdString(connection.source.port.value));
        const int destination_port = find_port(nodes_[destination_node].inputs,
            QString::fromStdString(connection.destination.port.value));
        if (source_port >= 0 && destination_port >= 0) {
            links_.push_back(Link{
                source_node, source_port, destination_node, destination_port});
        }
    }
    update();
}

void NodeGraphEditor::reset_demo_graph()
{
    nodes_.clear();
    links_.clear();
    selected_node_ = -1;
    dragging_node_ = -1;

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
    for (int index = 0; index < nodes_.size(); ++index) {
        draw_node(painter, index);
    }
    painter.restore();
}

void NodeGraphEditor::mousePressEvent(QMouseEvent* event)
{
    const QPointF scene = scene_position(event->position());
    if (event->button() == Qt::MiddleButton) {
        panning_ = true;
        last_mouse_position_ = event->position().toPoint();
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton) {
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
        update();
        event->accept();
        return;
    }
    if (dragging_node_ >= 0 && (event->buttons() & Qt::LeftButton) != 0) {
        nodes_[dragging_node_].position = scene_position(event->position()) - drag_offset_;
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
    update();
    event->accept();
}

} // namespace ORL

#endif
