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
class QKeyEvent;
class QPaintEvent;
class QPainter;
class QComboBox;
class QWheelEvent;

namespace ORL
{

class SceneInputCatalog;

// Draft Blender-style node canvas. It owns an editor-local graph copy, so
// editing the canvas cannot mutate the execution graph until an explicit
// commit path is added.
class NodeGraphEditor final : public QWidget {
public:
    explicit NodeGraphEditor(QWidget* parent = nullptr);

    void set_graph(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry);
    void set_scene_input_catalog(const SceneInputCatalog* catalog);
    void refresh_scene_inputs();
    void reset_demo_graph();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    struct Port {
        QString id;
        QString name;
        bool output = false;
        orlgraph::PortDirection direction = orlgraph::PortDirection::Input;
        orlgraph::PortCardinality cardinality = orlgraph::PortCardinality::Scalar;
        orlgraph::LogicalType type;
        orlgraph::Domain domain = orlgraph::Domain::constant();
        orlgraph::Shape shape = orlgraph::Shape::scalar();
    };

    struct Node {
        enum class Kind {
            Definition,
            GraphInput,
            GraphOutput,
        };

        QString id;
        QString title;
        QPointF position;
        QSizeF size{220.0, 120.0};
        QColor color;
        QVector<Port> inputs;
        QVector<Port> outputs;
        Kind kind = Kind::Definition;
        orlgraph::StableId interface_id;
    };

    struct Link {
        int source_node = -1;
        int source_port = -1;
        int destination_node = -1;
        int destination_port = -1;
    };

    struct Socket {
        int node = -1;
        int port = -1;
        bool output = false;

        bool valid() const {
            return node >= 0 && port >= 0;
        }
    };

    struct PendingConnection {
        Socket start;
        QPointF current;
        bool hovering_socket = false;
        bool valid_drop = false;
    };

    struct FindControl {
        QString node_id;
        QComboBox* combo = nullptr;
    };

    QPointF scene_position(const QPointF& viewport_position) const;
    void rebuild_view();
    void create_node(const orlgraph::StableId& definition_id, const QPointF& scene_position);
    void create_graph_input(const orlgraph::StableId& template_id,
        const QPointF& scene_position);
    void create_scene_input(const orlgraph::StableId& input_id,
        const QPointF& scene_position);
    void clear_find_controls();
    void rebuild_find_controls();
    void refresh_find_controls();
    void position_find_controls();
    QRectF node_rect(const Node& node) const;
    QPointF port_position(const Node& node, bool output, int index) const;
    int node_at(const QPointF& scene) const;
    int port_at(const Node& node, bool output, const QPointF& scene) const;
    Socket socket_at(const QPointF& scene) const;
    bool sockets_compatible(const Socket& first, const Socket& second) const;
    bool add_connection(const Socket& first, const Socket& second);
    void draw_grid(QPainter& painter) const;
    void draw_link(QPainter& painter, const Link& link) const;
    void draw_pending_connection(QPainter& painter) const;
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
    PendingConnection pending_connection_;
    orlgraph::GraphModule graph_;
    orlgraph::NodeRegistry registry_;
    const SceneInputCatalog* scene_input_catalog_ = nullptr;
    QVector<FindControl> find_controls_;
};

} // namespace ORL

#endif
