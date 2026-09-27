#pragma once

#if ORL_USE_QT6

#include <cstddef>
#include <optional>

#include <QColor>
#include <QHash>
#include <QPoint>
#include <QPointF>
#include <QRectF>
#include <QSizeF>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include "../component_manager.hpp"
#include "../node_graph/node_ops.hpp"
#include "../project_serialization.hpp"
#include "node_color_theme.hpp"
#include "orlgraph/orlgraph.hpp"

class QMouseEvent;
class QKeyEvent;
class QPaintEvent;
class QPainter;
class QComboBox;
class QLineEdit;
class QWheelEvent;

namespace ORL
{

class SceneInputCatalog;
class SceneGraphContext;
class Selection;

struct NodeSelectionInfo {
    QString id;
    QString title;
    QString kind;
    QString definition;
    QStringList inputs;
    QStringList outputs;
};

// Draft Blender-style node canvas. When attached to a SceneGraphContext it
// edits that context's active graph directly.
class NodeGraphEditor final : public QWidget {
public:
    explicit NodeGraphEditor(QWidget* parent = nullptr);

    void set_graph(const orlgraph::GraphModule& module,
        const orlgraph::NodeRegistry& registry);
    void set_graph(orlgraph::GraphModule& module,
        orlgraph::NodeRegistry& registry);
    void set_scene_graph_context(SceneGraphContext* context);
    void set_project_context(ComponentManager* components,
        ComponentId weight_id, ComponentId deformer_id);
    void set_selection(Selection* selection) { project_selection = selection; }
    bool load_project_file();
    bool save_project_file(bool save_as);
    void set_stage(orlgraph::GraphStage stage);
    orlgraph::GraphStage stage() const { return stage_; }
    std::optional<NodeSelectionInfo> selected_node_info() const;
    void set_node_color_theme(NodeColorTheme theme);
    void set_scene_input_catalog(const SceneInputCatalog* catalog);
    void refresh_scene_inputs();
    void create_frame_from_selection();
    void copy_selected_nodes();
    void paste_nodes();

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
        QString semantic;
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
        int frame = -1;

        bool valid() const {
            return frame >= 0 || (node >= 0 && port >= 0);
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

    struct FrameControl {
        QString frame_id;
        QLineEdit* title = nullptr;
    };

    struct Frame {
        QString id;
        QString title{QStringLiteral("Frame")};
        QPointF position;
        QSizeF size;
        bool collapsed = false;
        QVector<QString> members;
    };

    QPointF scene_position(const QPointF& viewport_position) const;
    orlgraph::GraphModule& active_graph() { return *graph_; }
    const orlgraph::GraphModule& active_graph() const { return *graph_; }
    orlgraph::NodeRegistry& active_registry() { return *registry_; }
    const orlgraph::NodeRegistry& active_registry() const { return *registry_; }
    void notify_graph_changed();
    void capture_stage_layout();
    void apply_stage_view();
    void clear_stage_layouts();
    NodeGraphLayout export_node_graph_layout();
    void import_node_graph_layout(const NodeGraphLayout& layout);
    void rebuild_view();
    void create_node(const orlgraph::StableId& definition_id, const QPointF& scene_position);
    void create_graph_input(const orlgraph::StableId& template_id,
        const QPointF& scene_position);
    void clear_find_controls();
    void rebuild_find_controls();
    void refresh_find_controls();
    QRectF find_control_rect(const Node& node) const;
    int find_control_at(const QPointF& scene) const;
    void show_find_control_popup(int control_index, const QPoint& anchor);
    void position_find_controls();
    void clear_frame_controls();
    void rebuild_frame_controls();
    void position_frame_controls();
    void prune_frames();
    QVector<Frame>& current_frames();
    const QVector<Frame>& current_frames() const;
    QRectF node_rect(const Node& node) const;
    QRectF expanded_frame_rect(const Frame& frame) const;
    QRectF collapsed_frame_rect(const Frame& frame) const;
    QRectF frame_rect(const Frame& frame) const;
    QRectF frame_header_rect(const Frame& frame) const;
    QRectF collapse_button_rect(const Frame& frame) const;
    QPointF frame_socket_position(const Frame& frame, bool output) const;
    QPointF port_position(const Node& node, bool output, int index) const;
    int node_at(const QPointF& scene) const;
    int frame_at(const QPointF& scene) const;
    int collapse_button_at(const QPointF& scene) const;
    int collapsed_frame_for_node(const QString& node_id) const;
    bool node_is_hidden(const Node& node) const;
    int port_at(const Node& node, bool output, const QPointF& scene) const;
    Socket socket_at(const QPointF& scene) const;
    Socket resolve_socket(const Socket& socket, const Socket* other = nullptr) const;
    bool sockets_compatible(const Socket& first, const Socket& second) const;
    bool add_connection(const Socket& first, const Socket& second);
    void move_frame(int index, const QPointF& delta);
    void toggle_frame_collapsed(int index);
    void draw_grid(QPainter& painter) const;
    void draw_link(QPainter& painter, const Link& link) const;
    void draw_pending_connection(QPainter& painter) const;
    void draw_selection_box(QPainter& painter) const;
    void draw_frame(QPainter& painter, int index) const;
    void draw_node(QPainter& painter, int index) const;
    bool node_is_selected(int index) const;
    bool frame_is_selected(int index) const;
    void select_only(int index);
    void select_only_frame(int index);
    void apply_box_selection();

    QVector<Node> nodes_;
    QVector<Link> links_;
    QPointF pan_{48.0, 48.0};
    QPointF drag_offset_;
    QPoint last_mouse_position_;
    double zoom_ = 1.0;
    int selected_node_ = -1;
    QVector<int> selectedNodes;
    QVector<int> selectedFrames;
    int dragging_node_ = -1;
    int draggingFrame = -1;
    bool panning_ = false;
    bool boxSelecting = false;
    QPointF boxSelectOrigin;
    QPointF boxSelectCurrent;
    PendingConnection pending_connection_;
    orlgraph::GraphModule solver_graph_storage_;
    orlgraph::GraphModule deformer_graph_storage_;
    orlgraph::NodeRegistry registry_storage_;
    orlgraph::GraphModule* graph_ = &solver_graph_storage_;
    orlgraph::NodeRegistry* registry_ = &registry_storage_;
    SceneGraphContext* scene_graph_context_ = nullptr;
    ComponentManager* project_components = nullptr;
    Selection* project_selection = nullptr;
    ComponentId project_weight_id;
    ComponentId project_deformer_id;
    orlgraph::GraphStage stage_ = orlgraph::GraphStage::Solver;
    std::size_t attached_graph_revision_ = 0;
    NodeColorTheme node_color_theme_ = NodeColorTheme::default_theme();
    std::optional<QString> graph_file_path_;
    const SceneInputCatalog* scene_input_catalog_ = nullptr;
    QVector<FindControl> find_controls_;
    QVector<FrameControl> frameControls;

    struct StageLayout {
        QPointF pan{48.0, 48.0};
        double zoom = 1.0;
        QHash<QString, QPointF> nodes;
        QVector<Frame> frames;
    };

    StageLayout& stage_layout(orlgraph::GraphStage stage);
    const StageLayout& stage_layout(orlgraph::GraphStage stage) const;

    StageLayout solver_layout_;
    StageLayout deformer_layout_;
    std::optional<node_graph::NodeClipboard> nodeClipboard;
    int pasteSerial = 0;
};

} // namespace ORL

#endif
