#pragma once

#if ORL_USE_QT6

#include <QColor>
#include <QPoint>
#include <QPointF>
#include <QRectF>
#include <QSizeF>
#include <QString>
#include <QVector>
#include <QWidget>

#include "orlgraph/orlgraph.hpp"

class QMouseEvent;
class QPaintEvent;
class QPainter;
class QWheelEvent;

namespace ORL
{

// Draft Blender-style node canvas. It intentionally owns only a visual copy
// of the graph, so opening the editor cannot mutate the execution graph.
class NodeGraphEditor final : public QWidget {
public:
    explicit NodeGraphEditor(QWidget* parent = nullptr);

    void set_graph(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry);
    void reset_demo_graph();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    struct Port {
        QString id;
        QString name;
        bool output = false;
    };

    struct Node {
        QString id;
        QString title;
        QPointF position;
        QSizeF size{220.0, 120.0};
        QColor color;
        QVector<Port> inputs;
        QVector<Port> outputs;
    };

    struct Link {
        int source_node = -1;
        int source_port = -1;
        int destination_node = -1;
        int destination_port = -1;
    };

    QPointF scene_position(const QPointF& viewport_position) const;
    QRectF node_rect(const Node& node) const;
    QPointF port_position(const Node& node, bool output, int index) const;
    int node_at(const QPointF& scene) const;
    int port_at(const Node& node, bool output, const QPointF& scene) const;
    void draw_grid(QPainter& painter) const;
    void draw_link(QPainter& painter, const Link& link) const;
    void draw_node(QPainter& painter, int index) const;

    QVector<Node> nodes_;
    QVector<Link> links_;
    QPointF pan_{48.0, 48.0};
    QPointF drag_offset_;
    QPoint last_mouse_position_;
    double zoom_ = 1.0;
    int selected_node_ = -1;
    int dragging_node_ = -1;
    bool panning_ = false;
};

} // namespace ORL

#endif
