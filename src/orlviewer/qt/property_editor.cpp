#include "property_editor.hpp"

#if ORL_USE_QT6

#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QLabel>
#include <QScrollArea>
#include <QStringList>
#include <QVBoxLayout>

#include <glm/common.hpp>
#include <glm/gtc/quaternion.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../component_manager.hpp"

namespace ORL
{

namespace
{

QLabel* make_value_label(QWidget* parent) {
    auto* label = new QLabel(parent);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return label;
}

} // namespace

PropertyEditor::PropertyEditor(const Selection& selection,
    const ComponentManager& components, QWidget* parent)
    : QWidget(parent)
    , selection_(selection)
    , components_(components)
{
    setMinimumSize(280, 240);

    auto* outer_layout = new QVBoxLayout(this);
    outer_layout->setContentsMargins(0, 0, 0, 0);

    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setFrameShape(QFrame::NoFrame);
    outer_layout->addWidget(scroll_area_);

    auto* content = new QWidget();
    auto* content_layout = new QVBoxLayout(content);
    content_layout->setContentsMargins(8, 8, 8, 8);

    auto* selection_group = new QGroupBox(QStringLiteral("Selection"), content);
    auto* selection_layout = new QFormLayout(selection_group);
    selection_summary_ = new QLabel(selection_group);
    selected_items_ = new QLabel(selection_group);
    selected_items_->setWordWrap(true);
    selected_items_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    selection_layout->addRow(QStringLiteral("Count"), selection_summary_);
    selection_layout->addRow(QStringLiteral("Items"), selected_items_);
    content_layout->addWidget(selection_group);

    transform_group_ = new QGroupBox(QStringLiteral("Transform"), content);
    auto* transform_layout = new QFormLayout(transform_group_);
    transform_source_ = new QLabel(transform_group_);
    transform_source_->setWordWrap(true);
    translation_value_ = make_value_label(transform_group_);
    rotation_value_ = make_value_label(transform_group_);
    scale_value_ = make_value_label(transform_group_);
    transform_layout->addRow(QStringLiteral("Source"), transform_source_);
    transform_layout->addRow(QStringLiteral("Translation"), translation_value_);
    transform_layout->addRow(QStringLiteral("Rotation (deg)"), rotation_value_);
    transform_layout->addRow(QStringLiteral("Scale"), scale_value_);
    content_layout->addWidget(transform_group_);

    auto* draft_note = new QLabel(
        QStringLiteral("Draft property editor\n"
                       "Values are read-only for now."),
        content);
    draft_note->setWordWrap(true);
    draft_note->setStyleSheet(QStringLiteral("color: palette(mid);"));
    content_layout->addWidget(draft_note);
    content_layout->addStretch(1);

    scroll_area_->setWidget(content);
    refresh();
}

void PropertyEditor::refresh() {
    const auto& refs = selection_.refs();
    if (refs.empty()) {
        selection_summary_->setText(QStringLiteral("Nothing selected"));
        selected_items_->setText(
            QStringLiteral("Select a viewport element to inspect its properties."));
        transform_group_->setVisible(false);
        return;
    }

    selection_summary_->setText(QString::number(
        static_cast<qulonglong>(refs.size())));
    QStringList item_names;
    item_names.reserve(static_cast<qsizetype>(refs.size()));
    for (const auto& ref : refs) {
        item_names.push_back(
            QStringLiteral("%1: %2")
                .arg(kind_name(ref.kind), component_name(ref, components_)));
    }
    selected_items_->setText(item_names.join(QStringLiteral("\n")));

    const auto* focus = selection_.focus();
    TransformValues values;
    QString source;
    if (focus == nullptr || !make_transform(*focus, values, source)) {
        transform_group_->setVisible(false);
        return;
    }

    transform_source_->setText(source);
    translation_value_->setText(format_vector(values.translation));
    rotation_value_->setText(format_vector(values.rotation_degrees));
    scale_value_->setText(format_vector(values.scale));
    transform_group_->setVisible(true);
}

bool PropertyEditor::make_transform(const SelectionRef& ref,
    TransformValues& values, QString& source) const
{
    const auto attr = selection_.dest(ref);
    if (!attr) {
        return false;
    }

    if (attr.has_world_matrix()) {
        glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 skew{0.0f};
        glm::vec4 perspective{0.0f};
        if (!glm::decompose(attr.world_matrix(), values.scale, orientation,
                values.translation, skew, perspective))
        {
            return false;
        }
        if (glm::length(orientation) < 1.0e-6f) {
            return false;
        }
        values.rotation_degrees = glm::degrees(
            glm::eulerAngles(glm::normalize(orientation)));
        source = ref.kind == SelectionRef::Kind::Controller
            ? QStringLiteral("Controller matrix converted to TRS for display")
            : QStringLiteral("World transform");
        return true;
    }

    if (ref.kind == SelectionRef::Kind::Joint) {
        values.translation = attr.world_position();
        values.rotation_degrees = glm::degrees(
            glm::eulerAngles(attr.local_rotation()));
        values.scale = attr.local_scale();
        source = QStringLiteral("Joint transform");
        return true;
    }

    if (ref.kind == SelectionRef::Kind::Vector) {
        values.translation = attr.world_position();
        values.rotation_degrees = glm::vec3{0.0f};
        values.scale = glm::vec3{1.0f};
        source = QStringLiteral("Point position");
        return true;
    }

    return false;
}

QString PropertyEditor::kind_name(SelectionRef::Kind kind) {
    switch (kind) {
    case SelectionRef::Kind::Joint:
        return QStringLiteral("Joint");
    case SelectionRef::Kind::SceneObject:
        return QStringLiteral("Scene Object");
    case SelectionRef::Kind::Vector:
        return QStringLiteral("Point");
    case SelectionRef::Kind::Controller:
        return QStringLiteral("Controller");
    case SelectionRef::Kind::Locator:
        return QStringLiteral("Locator");
    case SelectionRef::Kind::None:
        return QStringLiteral("None");
    }
    return QStringLiteral("Unknown");
}

QString PropertyEditor::format_vector(const glm::vec3& value) {
    return QStringLiteral("(%1, %2, %3)")
        .arg(QString::number(value.x, 'f', 3))
        .arg(QString::number(value.y, 'f', 3))
        .arg(QString::number(value.z, 'f', 3));
}

QString PropertyEditor::component_name(const SelectionRef& ref,
    const ComponentManager& components)
{
    if (ref.kind == SelectionRef::Kind::SceneObject) {
        return QString::fromStdString(ref.object_name);
    }
    if (ref.kind == SelectionRef::Kind::Vector) {
        return QStringLiteral("Point");
    }
    const auto* component = components.find(ref.component);
    return component != nullptr
        ? QString::fromStdString(component->name)
        : QStringLiteral("<unavailable>");
}

} // namespace ORL

#endif
