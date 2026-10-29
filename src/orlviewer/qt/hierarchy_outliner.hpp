#pragma once

#if ORL_USE_QT6

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <QWidget>
#include <QString>

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

    std::vector<Entry> collect_entries() const;
    bool entries_equal(const std::vector<Entry>& entries) const;
    void rebuild(std::vector<Entry> entries);
    void sync_selection();
    void apply_tree_selection();
    static QString kind_label(ComponentKind kind);
    static bool is_supported_kind(ComponentKind kind);

    Selection& selection_;
    ComponentManager& components_;
    QTreeWidget* tree_ = nullptr;
    std::vector<Entry> entries_;
    std::unordered_map<std::uint64_t, QTreeWidgetItem*> items_;
    std::uint64_t selection_revision_ = 0;
    bool syncing_ = false;
};

} // namespace ORL

#endif
