#pragma once

#if ORL_USE_QT6

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <QPoint>
#include <QString>
#include <QWidget>

#include "../component_manager.hpp"
#include "../selection.hpp"

class QTreeWidget;
class QTreeWidgetItem;

namespace ORL
{

// Displays the authored joint hierarchy together with locator transform-parent
// relations.  The tree uses stable component IDs, so rebuilding it does not
// depend on unordered ComponentManager metadata iteration.
class HierarchyOutliner final : public QWidget {
public:
    HierarchyOutliner(Selection& selection,
        ComponentManager& components, QWidget* parent = nullptr);

    void refresh();

private:
    struct Entry {
        ComponentId id;
        ComponentId parent;
        ComponentKind kind = ComponentKind::Joint;
        std::string name;
    };

    struct DropProposal {
        enum class Action {
            None,
            World,
            ParentUnder,
            Reparent,
        };

        Action action = Action::None;
        ComponentId parent;
        QTreeWidgetItem* item = nullptr;
        bool before = false;
    };

    std::vector<Entry> collect_entries() const;
    bool entries_equal(const std::vector<Entry>& entries) const;
    void rebuild(std::vector<Entry> entries);
    void sync_selection();
    void apply_tree_selection();
    void begin_hierarchy_drag();
    void update_drop_proposal(const QPoint& viewport_pos, bool inside);
    void finish_hierarchy_drag(bool canceled);
    void apply_hierarchy_drop();
    void clear_drop_visuals();
    DropProposal propose_drop(const QPoint& viewport_pos) const;
    ComponentId parent_of(ComponentId id) const;
    std::vector<ComponentId> selection_roots(
        const std::vector<ComponentId>& ids) const;
    void report_drop_error(const QString& message);
    static QString kind_label(ComponentKind kind);
    static bool is_supported_kind(ComponentKind kind);

    Selection& selection_;
    ComponentManager& components_;
    QTreeWidget* tree_ = nullptr;
    QWidget* drop_line_ = nullptr;
    std::vector<Entry> entries_;
    std::vector<ComponentId> drag_ids_;
    std::unordered_map<std::uint64_t, QTreeWidgetItem*> items_;
    DropProposal drop_;
    std::uint64_t selection_revision_ = 0;
    bool syncing_ = false;
    bool dragging_ = false;
};

} // namespace ORL

#endif
