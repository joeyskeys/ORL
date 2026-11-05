#include "hierarchy_outliner.hpp"

#if ORL_USE_QT6

#include "../runtime_config.hpp"

#include <algorithm>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <QAbstractItemView>
#include <QApplication>
#include <QBrush>
#include <QColor>
#include <QCursor>
#include <QEvent>
#include <QHeaderView>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QSignalBlocker>
#include <QString>
#include <QToolTip>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVariant>
#include <QVBoxLayout>

namespace ORL
{

namespace
{

constexpr int kKindRole = Qt::UserRole;
constexpr int kIdRole = Qt::UserRole + 1;

class HierarchyTreeWidget final : public QTreeWidget {
public:
    using DragBeginHandler = std::function<void()>;
    using TargetHandler = std::function<void(const QPoint&, bool)>;
    using DropHandler = std::function<void(bool)>;

    explicit HierarchyTreeWidget(QWidget* parent = nullptr)
        : QTreeWidget(parent)
    {
    }

    ~HierarchyTreeWidget() override {
        end_middle_drag(true);
    }

    void set_drag_begin_handler(DragBeginHandler handler) {
        drag_begin_handler_ = std::move(handler);
    }

    void set_target_handler(TargetHandler handler) {
        target_handler_ = std::move(handler);
    }

    void set_drop_handler(DropHandler handler) {
        drop_handler_ = std::move(handler);
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::MiddleButton) {
            const auto viewport_pos = viewport_point(event);
            auto* item = itemAt(viewport_pos);
            if (item != nullptr) {
                if (!item->isSelected()) {
                    clearSelection();
                    item->setSelected(true);
                    setCurrentItem(item);
                }
                begin_middle_drag(viewport_pos);
                event->accept();
                return;
            }
        }
        QTreeWidget::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (update_middle_drag(event)) {
            event->accept();
            return;
        }
        QTreeWidget::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::MiddleButton
            && finish_middle_drag(event, false))
        {
            event->accept();
            return;
        }
        QTreeWidget::mouseReleaseEvent(event);
    }

    void keyPressEvent(QKeyEvent* event) override {
        if (event->key() == Qt::Key_Escape && end_middle_drag(true)) {
            event->accept();
            return;
        }
        QTreeWidget::keyPressEvent(event);
    }

    bool eventFilter(QObject* watched, QEvent* event) override {
        if (middle_drag_ && event != nullptr) {
            switch (event->type()) {
            case QEvent::MouseMove:
                update_middle_drag(static_cast<QMouseEvent*>(event));
                return false;
            case QEvent::MouseButtonRelease:
                if (static_cast<QMouseEvent*>(event)->button()
                    == Qt::MiddleButton)
                {
                    finish_middle_drag(
                        static_cast<QMouseEvent*>(event), false);
                    return true;
                }
                break;
            case QEvent::KeyPress:
                if (static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape) {
                    end_middle_drag(true);
                    return true;
                }
                break;
            case QEvent::ApplicationDeactivate:
                end_middle_drag(true);
                break;
            default:
                break;
            }
        }
        return QTreeWidget::eventFilter(watched, event);
    }

private:
    QPoint viewport_point(const QMouseEvent* event) const {
        return viewport()->mapFromGlobal(event->globalPosition().toPoint());
    }

    void set_drag_cursor(bool dragging) {
        auto* view = viewport() != nullptr ? viewport() : static_cast<QWidget*>(this);
        if (dragging) {
            view->setCursor(Qt::ClosedHandCursor);
            setCursor(Qt::ClosedHandCursor);
        } else {
            view->unsetCursor();
            unsetCursor();
        }
    }

    void begin_middle_drag(const QPoint& viewport_pos) {
        middle_drag_ = true;
        setFocus(Qt::MouseFocusReason);
        if (auto* app = QApplication::instance();
            app != nullptr && !filter_installed_)
        {
            app->installEventFilter(this);
            filter_installed_ = true;
        }
        set_drag_cursor(true);
        if (drag_begin_handler_) {
            drag_begin_handler_();
        }
        if (target_handler_) {
            target_handler_(viewport_pos, true);
        }
    }

    bool update_middle_drag(const QMouseEvent* event) {
        if (!middle_drag_ || event == nullptr) {
            return false;
        }
        const auto viewport_pos = viewport_point(event);
        if (target_handler_) {
            target_handler_(
                viewport_pos, viewport()->rect().contains(viewport_pos));
        }
        return true;
    }

    bool finish_middle_drag(const QMouseEvent* event, bool canceled) {
        if (!middle_drag_) {
            return false;
        }
        bool inside = false;
        QPoint viewport_pos;
        if (event != nullptr) {
            viewport_pos = viewport_point(event);
            inside = viewport()->rect().contains(viewport_pos);
        }
        if (!canceled && target_handler_) {
            target_handler_(viewport_pos, inside);
        }
        return end_middle_drag(canceled || !inside);
    }

    bool end_middle_drag(bool canceled) {
        if (!middle_drag_) {
            return false;
        }
        middle_drag_ = false;
        if (filter_installed_) {
            if (auto* app = QApplication::instance()) {
                app->removeEventFilter(this);
            }
            filter_installed_ = false;
        }
        set_drag_cursor(false);
        if (drop_handler_) {
            drop_handler_(canceled);
        }
        return true;
    }

    DragBeginHandler drag_begin_handler_;
    TargetHandler target_handler_;
    DropHandler drop_handler_;
    bool middle_drag_ = false;
    bool filter_installed_ = false;
};

int item_depth(const QTreeWidgetItem* item)
{
    int depth = 0;
    for (auto* parent = item != nullptr ? item->parent() : nullptr;
         parent != nullptr; parent = parent->parent())
    {
        ++depth;
    }
    return depth;
}

QTreeWidgetItem* ancestor_at_depth(QTreeWidgetItem* item, int depth)
{
    auto current_depth = item_depth(item);
    while (item != nullptr && current_depth > depth) {
        item = item->parent();
        --current_depth;
    }
    return item;
}

std::optional<ComponentId> item_id(const QTreeWidgetItem* item)
{
    if (item == nullptr) {
        return std::nullopt;
    }
    const ComponentId id{item->data(0, kIdRole).toULongLong()};
    return id ? std::optional<ComponentId>{id} : std::nullopt;
}

ComponentKind item_kind(const QTreeWidgetItem* item)
{
    return item == nullptr
        ? ComponentKind::Joint
        : static_cast<ComponentKind>(item->data(0, kKindRole).toInt());
}

bool joint_target_is_descendant(
    const ComponentManager& components,
    ComponentId child, ComponentId target)
{
    const auto ids = components.packed_joint_ids();
    const auto joints = components.packed_joints();
    auto index = components.joint_index(target);
    std::size_t steps = 0;
    while (index >= 0 && static_cast<std::size_t>(index) < joints.size()
        && steps++ <= joints.size())
    {
        if (static_cast<std::size_t>(index) >= ids.size()) {
            return false;
        }
        if (ids[static_cast<std::size_t>(index)] == child) {
            return true;
        }
        index = joints[static_cast<std::size_t>(index)].parent;
    }
    return false;
}

bool locator_target_is_descendant(
    const ComponentManager& components,
    ComponentId child, ComponentId target)
{
    ComponentId current = target;
    const std::size_t limit = components.packed_locator_ids().size() + 1;
    for (std::size_t step = 0; step < limit && current; ++step) {
        if (current == child) {
            return true;
        }
        const auto* relation = components.transform_parent(current);
        if (relation == nullptr || !relation->enabled
            || relation->parent_kind == orlrig::TransformParentKind::World)
        {
            return false;
        }
        current = relation->parent;
    }
    return false;
}

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

    tree_ = new HierarchyTreeWidget(this);
    tree_->setColumnCount(2);
    tree_->setHeaderLabels({
        QStringLiteral("Hierarchy"),
        QStringLiteral("Type"),
    });
    tree_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
    tree_->setUniformRowHeights(true);
    tree_->setRootIsDecorated(true);
    tree_->setDragEnabled(false);
    tree_->setAcceptDrops(false);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(
        0, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(
        1, QHeaderView::ResizeToContents);
    layout->addWidget(tree_);

    drop_line_ = new QWidget(tree_->viewport());
    drop_line_->setAttribute(Qt::WA_TransparentForMouseEvents);
    drop_line_->setStyleSheet(
        QStringLiteral("background-color: rgb(120, 180, 255);"));
    drop_line_->hide();

    auto* hierarchy_tree = static_cast<HierarchyTreeWidget*>(tree_);
    hierarchy_tree->set_drag_begin_handler(
        [this] { begin_hierarchy_drag(); });
    hierarchy_tree->set_target_handler(
        [this](const QPoint& pos, bool inside) {
            update_drop_proposal(pos, inside);
        });
    hierarchy_tree->set_drop_handler(
        [this](bool canceled) { finish_hierarchy_drag(canceled); });
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
    clear_drop_visuals();
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
    if (dragging_) {
        return;
    }
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
        const auto kind = item_kind(item);
        const auto id = item_id(item);
        if (!id.has_value()) {
            continue;
        }
        if (kind == ComponentKind::Joint) {
            refs.push_back(SelectionRef::joint(*id));
        } else if (kind == ComponentKind::Locator) {
            refs.push_back(SelectionRef::locator(*id));
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

void HierarchyOutliner::begin_hierarchy_drag()
{
    dragging_ = true;
    drag_ids_.clear();
    drop_ = {};
    if (tree_ == nullptr) {
        return;
    }
    std::vector<ComponentId> selected;
    for (auto* item : tree_->selectedItems()) {
        const auto id = item_id(item);
        const auto* component = id.has_value()
            ? components_.find(*id) : nullptr;
        if (component == nullptr || !is_supported_kind(component->kind)) {
            continue;
        }
        if (std::find(selected.begin(), selected.end(), *id)
            == selected.end())
        {
            selected.push_back(*id);
        }
    }
    drag_ids_ = selection_roots(selected);
}

ComponentId HierarchyOutliner::parent_of(ComponentId id) const
{
    const auto* component = components_.find(id);
    if (component == nullptr) {
        return {};
    }
    if (component->kind == ComponentKind::Joint) {
        const auto ids = components_.packed_joint_ids();
        const auto joints = components_.packed_joints();
        const auto index = components_.joint_index(id);
        if (index < 0 || static_cast<std::size_t>(index) >= joints.size()) {
            return {};
        }
        const auto parent_index = joints[static_cast<std::size_t>(index)].parent;
        if (parent_index >= 0
            && static_cast<std::size_t>(parent_index) < ids.size())
        {
            return ids[static_cast<std::size_t>(parent_index)];
        }
        return {};
    }
    if (const auto* relation = components_.transform_parent(id);
        relation != nullptr && relation->enabled
            && relation->parent_kind != orlrig::TransformParentKind::World)
    {
        return relation->parent;
    }
    return {};
}

std::vector<ComponentId> HierarchyOutliner::selection_roots(
    const std::vector<ComponentId>& ids) const
{
    std::unordered_set<std::uint64_t> selected;
    selected.reserve(ids.size());
    for (const ComponentId id : ids) {
        selected.insert(id.value);
    }

    std::vector<ComponentId> roots;
    roots.reserve(ids.size());
    for (const ComponentId id : ids) {
        ComponentId current = parent_of(id);
        bool ancestor_selected = false;
        const std::size_t limit = entries_.size() + 1;
        for (std::size_t step = 0; step < limit && current; ++step) {
            if (selected.contains(current.value)) {
                ancestor_selected = true;
                break;
            }
            current = parent_of(current);
        }
        if (!ancestor_selected) {
            roots.push_back(id);
        }
    }
    return roots;
}

HierarchyOutliner::DropProposal HierarchyOutliner::propose_drop(
    const QPoint& viewport_pos) const
{
    DropProposal proposal;
    if (tree_ == nullptr) {
        return proposal;
    }

    auto* item = tree_->itemAt(viewport_pos);
    if (item == nullptr) {
        proposal.action = DropProposal::Action::World;
        return proposal;
    }

    const auto index = tree_->indexFromItem(item);
    const QRect row = tree_->visualRect(index);
    if (!row.isValid()) {
        proposal.action = DropProposal::Action::World;
        return proposal;
    }

    const int depth = item_depth(item);
    const int indent = std::max(1, tree_->indentation());
    const int levels_left = std::max(0, (row.left() - viewport_pos.x()) / indent);
    const int target_depth = std::max(0, depth - levels_left);
    const int y_inset = std::max(2, row.height() / 4);
    const bool onto_band = viewport_pos.y() > row.top() + y_inset
        && viewport_pos.y() < row.bottom() - y_inset
        && levels_left == 0;

    if (onto_band) {
        proposal.action = DropProposal::Action::ParentUnder;
        if (const auto id = item_id(item); id.has_value()) {
            proposal.parent = *id;
        }
        proposal.item = item;
        return proposal;
    }

    auto* sibling = ancestor_at_depth(item, target_depth);
    if (sibling == nullptr) {
        proposal.action = DropProposal::Action::World;
        proposal.item = item;
        proposal.before = viewport_pos.y() < row.center().y();
        return proposal;
    }

    proposal.action = DropProposal::Action::Reparent;
    if (sibling->parent() != nullptr) {
        if (const auto id = item_id(sibling->parent()); id.has_value()) {
            proposal.parent = *id;
        } else {
            proposal.action = DropProposal::Action::World;
        }
    } else {
        proposal.action = DropProposal::Action::World;
    }
    proposal.item = sibling;
    proposal.before = viewport_pos.y() < tree_->visualRect(
        tree_->indexFromItem(sibling)).center().y();
    return proposal;
}

void HierarchyOutliner::clear_drop_visuals()
{
    if (drop_.item != nullptr) {
        drop_.item->setBackground(0, QBrush{});
        drop_.item->setBackground(1, QBrush{});
    }
    if (drop_line_ != nullptr) {
        drop_line_->hide();
    }
    drop_ = {};
}

void HierarchyOutliner::update_drop_proposal(
    const QPoint& viewport_pos, bool inside)
{
    if (!dragging_) {
        return;
    }
    if (drop_.item != nullptr) {
        drop_.item->setBackground(0, QBrush{});
        drop_.item->setBackground(1, QBrush{});
    }

    if (!inside) {
        if (drop_line_ != nullptr) {
            drop_line_->hide();
        }
        drop_ = {};
        return;
    }

    drop_ = propose_drop(viewport_pos);
    if (drop_.action == DropProposal::Action::ParentUnder
        && drop_.item != nullptr)
    {
        const QColor highlight(70, 105, 155, 180);
        drop_.item->setBackground(0, highlight);
        drop_.item->setBackground(1, highlight);
        if (drop_line_ != nullptr) {
            drop_line_->hide();
        }
        return;
    }

    if (drop_line_ == nullptr || tree_ == nullptr) {
        return;
    }
    if (drop_.item == nullptr) {
        const int y = std::clamp(
            viewport_pos.y(), 0, std::max(0, tree_->viewport()->height() - 2));
        drop_line_->setGeometry(
            4, y, std::max(8, tree_->viewport()->width() - 8), 2);
        drop_line_->show();
        drop_line_->raise();
        return;
    }

    const QRect row = tree_->visualRect(tree_->indexFromItem(drop_.item));
    const int y = drop_.before
        ? std::max(0, row.top())
        : std::max(0, row.bottom() - 1);
    const int x = std::max(4, row.left());
    drop_line_->setGeometry(
        x, y, std::max(8, tree_->viewport()->width() - x - 4), 2);
    drop_line_->show();
    drop_line_->raise();
}

void HierarchyOutliner::report_drop_error(const QString& message)
{
    if (tree_ != nullptr) {
        QToolTip::showText(QCursor::pos(), message, tree_);
    }
}

void HierarchyOutliner::finish_hierarchy_drag(bool canceled)
{
    dragging_ = false;
    if (canceled) {
        drag_ids_.clear();
        clear_drop_visuals();
        return;
    }
    apply_hierarchy_drop();
    drag_ids_.clear();
    clear_drop_visuals();
}

void HierarchyOutliner::apply_hierarchy_drop()
{
    if (drag_ids_.empty()
        || drop_.action == DropProposal::Action::None)
    {
        return;
    }
    if (runtime_config.evaluate_orl
        || runtime_config.evaluation_phase
            != OrlEvaluationPhase::Disabled)
    {
        report_drop_error(
            QStringLiteral("Disable evaluation before changing hierarchy."));
        return;
    }

    const ComponentId new_parent = drop_.parent;
    if (new_parent) {
        const auto* target_component = components_.find(new_parent);
        if (target_component == nullptr
            || !is_supported_kind(target_component->kind))
        {
            report_drop_error(QStringLiteral("Invalid hierarchy target."));
            return;
        }
    }

    std::unordered_set<std::uint64_t> selected_ids;
    selected_ids.reserve(drag_ids_.size());
    for (const ComponentId id : drag_ids_) {
        selected_ids.insert(id.value);
    }
    if (new_parent && selected_ids.contains(new_parent.value)) {
        report_drop_error(
            QStringLiteral("An item cannot be parented to itself."));
        return;
    }

    for (const ComponentId id : drag_ids_) {
        const auto* component = components_.find(id);
        if (component == nullptr || !is_supported_kind(component->kind)) {
            report_drop_error(QStringLiteral("Invalid hierarchy child."));
            return;
        }
        if (component->kind == ComponentKind::Joint) {
            if (new_parent) {
                const auto* parent_component = components_.find(new_parent);
                if (parent_component == nullptr
                    || parent_component->kind != ComponentKind::Joint)
                {
                    report_drop_error(
                        QStringLiteral("Joints can only be parented to joints."));
                    return;
                }
                if (joint_target_is_descendant(
                        components_, id, new_parent))
                {
                    report_drop_error(
                        QStringLiteral("That drop would create a joint cycle."));
                    return;
                }
            }
        } else if (new_parent) {
            const auto* parent_component = components_.find(new_parent);
            if (parent_component == nullptr
                || !is_supported_kind(parent_component->kind))
            {
                report_drop_error(
                    QStringLiteral(
                        "Locators require a joint or locator parent."));
                return;
            }
            if (parent_component->kind == ComponentKind::Locator
                && locator_target_is_descendant(
                    components_, id, new_parent))
            {
                report_drop_error(
                    QStringLiteral("That drop would create a locator cycle."));
                return;
            }
        }
    }

    bool changed = false;
    for (const ComponentId id : drag_ids_) {
        if (parent_of(id) == new_parent) {
            continue;
        }
        const auto* component = components_.find(id);
        std::string error;
        bool applied = false;
        if (component->kind == ComponentKind::Joint) {
            applied = components_.set_joint_parent(
                id, new_parent, true, &error);
        } else if (new_parent) {
            applied = components_.bind_transform_parent(
                id, new_parent, true, true, true, true, &error);
        } else if (components_.transform_parent(id) != nullptr) {
            applied = components_.clear_transform_parent(id, true);
            if (!applied) {
                error = "Unable to clear the locator parent";
            }
        } else {
            applied = true;
        }
        if (!applied) {
            report_drop_error(
                error.empty()
                    ? QStringLiteral("Unable to change hierarchy.")
                    : QString::fromStdString(error));
            if (changed) {
                refresh();
            }
            return;
        }
        changed = true;
    }
    if (changed) {
        refresh();
    }
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
