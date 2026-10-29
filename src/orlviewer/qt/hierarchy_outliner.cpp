#include "hierarchy_outliner.hpp"

#if ORL_USE_QT6

#include <algorithm>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>

#include <QAbstractItemView>
#include <QHeaderView>
#include <QSignalBlocker>
#include <QString>
#include <QVariant>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace ORL
{

namespace
{

constexpr int kKindRole = Qt::UserRole;
constexpr int kIdRole = Qt::UserRole + 1;

} // namespace

HierarchyOutliner::HierarchyOutliner(
    Selection& selection, ComponentManager& components, QWidget* parent)
    : QWidget(parent)
    , selection_(selection)
    , components_(components)
{
    setMinimumSize(220, 240);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    tree_ = new QTreeWidget(this);
    tree_->setColumnCount(2);
    tree_->setHeaderLabels({
        QStringLiteral("Hierarchy"),
        QStringLiteral("Type"),
    });
    tree_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
    tree_->setUniformRowHeights(true);
    tree_->setRootIsDecorated(true);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(
        0, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(
        1, QHeaderView::ResizeToContents);
    layout->addWidget(tree_);

    QObject::connect(tree_, &QTreeWidget::itemSelectionChanged,
        this, [this] { apply_tree_selection(); });
    refresh();
}

std::vector<HierarchyOutliner::Entry>
HierarchyOutliner::collect_entries() const
{
    std::vector<Entry> result;
    const auto joint_ids = components_.packed_joint_ids();
    const auto joints = components_.packed_joints();
    result.reserve(
        joint_ids.size() + components_.packed_locator_ids().size());

    for (std::size_t index = 0;
         index < joint_ids.size() && index < joints.size(); ++index)
    {
        ComponentId parent;
        const auto parent_index = joints[index].parent;
        if (parent_index >= 0
            && static_cast<std::size_t>(parent_index) < joint_ids.size())
        {
            parent = joint_ids[static_cast<std::size_t>(parent_index)];
        }
        const auto* component = components_.find(joint_ids[index]);
        if (component == nullptr) {
            continue;
        }
        result.push_back({
            joint_ids[index], parent, ComponentKind::Joint, component->name});
    }

    for (const ComponentId id : components_.packed_locator_ids()) {
        const auto* component = components_.find(id);
        if (component == nullptr) {
            continue;
        }
        ComponentId parent;
        if (const auto* relation = components_.transform_parent(id);
            relation != nullptr && relation->enabled
                && relation->parent_kind
                    != orlrig::TransformParentKind::World)
        {
            const auto* parent_component =
                components_.find(relation->parent);
            if (parent_component != nullptr
                && is_supported_kind(parent_component->kind))
            {
                parent = relation->parent;
            }
        }
        result.push_back({
            id, parent, ComponentKind::Locator, component->name});
    }
    return result;
}

bool HierarchyOutliner::entries_equal(
    const std::vector<Entry>& entries) const
{
    if (entries.size() != entries_.size()) {
        return false;
    }
    for (std::size_t index = 0; index < entries.size(); ++index) {
        const auto& first = entries[index];
        const auto& second = entries_[index];
        if (first.id != second.id
            || first.parent != second.parent
            || first.kind != second.kind
            || first.name != second.name)
        {
            return false;
        }
    }
    return true;
}

void HierarchyOutliner::rebuild(std::vector<Entry> entries)
{
    syncing_ = true;
    QSignalBlocker tree_blocker(tree_);
    if (tree_->selectionModel() != nullptr) {
        QSignalBlocker selection_blocker(tree_->selectionModel());
        tree_->clear();
    } else {
        tree_->clear();
    }
    syncing_ = false;

    entries_ = std::move(entries);
    items_.clear();

    std::unordered_map<std::uint64_t, std::size_t> indices;
    indices.reserve(entries_.size());
    for (std::size_t index = 0; index < entries_.size(); ++index) {
        indices.emplace(entries_[index].id.value, index);
    }

    std::vector<std::vector<std::size_t>> children(entries_.size());
    std::vector<std::size_t> roots;
    roots.reserve(entries_.size());
    for (std::size_t index = 0; index < entries_.size(); ++index) {
        const auto parent = entries_[index].parent;
        const auto found = indices.find(parent.value);
        if (!parent || found == indices.end() || found->second == index) {
            roots.push_back(index);
        } else {
            children[found->second].push_back(index);
        }
    }

    std::vector<bool> visited(entries_.size(), false);
    const std::function<void(std::size_t, QTreeWidgetItem*)> add =
        [&](std::size_t index, QTreeWidgetItem* parent) {
            if (index >= entries_.size() || visited[index]) {
                return;
            }
            visited[index] = true;
            const auto& entry = entries_[index];
            auto* item = parent == nullptr
                ? new QTreeWidgetItem(tree_)
                : new QTreeWidgetItem(parent);
            item->setText(0, QString::fromStdString(entry.name));
            item->setText(1, kind_label(entry.kind));
            item->setData(
                0, kKindRole, static_cast<int>(entry.kind));
            item->setData(
                0, kIdRole,
                QVariant::fromValue<qulonglong>(entry.id.value));
            items_[entry.id.value] = item;
            for (const auto child : children[index]) {
                add(child, item);
            }
        };

    for (const auto root : roots) {
        add(root, nullptr);
    }
    for (std::size_t index = 0; index < entries_.size(); ++index) {
        add(index, nullptr);
    }
    tree_->expandAll();
}

void HierarchyOutliner::refresh()
{
    const auto current = collect_entries();
    const bool rebuilt = !entries_equal(current);
    if (rebuilt) {
        rebuild(current);
    }
    if (rebuilt || selection_revision_ != selection_.revision()) {
        sync_selection();
        selection_revision_ = selection_.revision();
    }
}

void HierarchyOutliner::sync_selection()
{
    if (tree_ == nullptr) {
        return;
    }
    syncing_ = true;
    QSignalBlocker tree_blocker(tree_);
    std::optional<QSignalBlocker> selection_blocker;
    if (tree_->selectionModel() != nullptr) {
        selection_blocker.emplace(tree_->selectionModel());
    }
    tree_->clearSelection();
    QTreeWidgetItem* current = nullptr;
    for (const auto& ref : selection_.refs()) {
        if (ref.kind != SelectionRef::Kind::Joint
            && ref.kind != SelectionRef::Kind::Locator)
        {
            continue;
        }
        const auto found = items_.find(ref.component.value);
        if (found == items_.end()) {
            continue;
        }
        found->second->setSelected(true);
        current = found->second;
    }
    if (current != nullptr) {
        tree_->setCurrentItem(current);
    }
    syncing_ = false;
}

void HierarchyOutliner::apply_tree_selection()
{
    if (syncing_ || tree_ == nullptr) {
        return;
    }
    const auto selected = tree_->selectedItems();
    if (selected.isEmpty()) {
        selection_.clear(SelectionChangeReason::Pick);
        selection_revision_ = selection_.revision();
        return;
    }

    std::vector<SelectionRef> refs;
    refs.reserve(static_cast<std::size_t>(selected.size()));
    for (auto* item : selected) {
        if (item == nullptr) {
            continue;
        }
        const auto kind = static_cast<ComponentKind>(
            item->data(0, kKindRole).toInt());
        const ComponentId id{
            item->data(0, kIdRole).toULongLong()};
        if (kind == ComponentKind::Joint) {
            refs.push_back(SelectionRef::joint(id));
        } else if (kind == ComponentKind::Locator) {
            refs.push_back(SelectionRef::locator(id));
        }
    }
    if (refs.empty()) {
        return;
    }

    selection_.replace(
        refs.front(), SelectionChangeReason::Pick);
    for (std::size_t index = 1; index < refs.size(); ++index) {
        selection_.apply(
            SelectionAction::Add, refs[index],
            SelectionChangeReason::Pick);
    }
    selection_revision_ = selection_.revision();
}

QString HierarchyOutliner::kind_label(ComponentKind kind)
{
    return kind == ComponentKind::Joint
        ? QStringLiteral("Joint")
        : QStringLiteral("Locator");
}

bool HierarchyOutliner::is_supported_kind(ComponentKind kind)
{
    return kind == ComponentKind::Joint
        || kind == ComponentKind::Locator;
}

} // namespace ORL

#endif
