#include "property_editor.hpp"

#if ORL_USE_QT6

#include <algorithm>
#include <cmath>
#include <variant>
#include <vector>

#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QStringList>
#include <QVBoxLayout>

#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../component_manager.hpp"
#include "../runtime_config.hpp"
#include "../scene_graph_context.hpp"
#include "node_graph_editor.hpp"

namespace ORL
{

namespace
{

bool decompose_transform(const glm::mat4& matrix,
    glm::vec3& scale, glm::quat& rotation, glm::vec3& translation)
{
    glm::vec3 skew{0.0f};
    glm::vec4 perspective{0.0f};
    if (!glm::decompose(
            matrix, scale, rotation, translation, skew, perspective))
    {
        return false;
    }
    if (glm::length(rotation) < 1.0e-6f) {
        return false;
    }
    rotation = glm::normalize(rotation);
    return true;
}

glm::quat euler_rotation(const glm::vec3& degrees) {
    const glm::quat rotation{glm::radians(degrees)};
    return glm::length(rotation) < 1.0e-6f
        ? glm::quat{1.0f, 0.0f, 0.0f, 0.0f}
        : glm::normalize(rotation);
}

} // namespace

PropertyEditor::PropertyEditor(const Selection& selection,
    ComponentManager& components, SceneGraphContext* graph_context,
    QWidget* parent)
    : QWidget(parent)
    , selection_(selection)
    , components_(components)
    , graphContext(graph_context)
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

    selection_group_ = new QGroupBox(
        QStringLiteral("Selection"), content);
    auto* selection_layout = new QFormLayout(selection_group_);
    selection_summary_ = new QLabel(selection_group_);
    selected_items_ = new QLabel(selection_group_);
    selected_items_->setWordWrap(true);
    selected_items_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    selection_layout->addRow(QStringLiteral("Count"), selection_summary_);
    selection_layout->addRow(QStringLiteral("Items"), selected_items_);
    content_layout->addWidget(selection_group_);

    node_group_ = new QGroupBox(
        QStringLiteral("Node Graph"), content);
    auto* node_layout = new QFormLayout(node_group_);
    node_stage_ = new QLabel(node_group_);
    node_kind_ = new QLabel(node_group_);
    node_id_ = new QLabel(node_group_);
    node_definition_ = new QLabel(node_group_);
    node_ports_ = new QLabel(node_group_);
    node_id_->setWordWrap(true);
    node_definition_->setWordWrap(true);
    node_ports_->setWordWrap(true);
    node_layout->addRow(QStringLiteral("Stage"), node_stage_);
    node_layout->addRow(QStringLiteral("Kind"), node_kind_);
    node_layout->addRow(QStringLiteral("ID"), node_id_);
    node_layout->addRow(QStringLiteral("Type"), node_definition_);
    node_layout->addRow(QStringLiteral("Ports"), node_ports_);
    content_layout->addWidget(node_group_);
    node_group_->setVisible(false);

    static_value_editor_.group = new QGroupBox(
        QStringLiteral("Static Value"), content);
    auto* static_layout = new QVBoxLayout(static_value_editor_.group);
    static_value_editor_.fields_widget = new QWidget(
        static_value_editor_.group);
    static_layout->addWidget(static_value_editor_.fields_widget);
    static_value_editor_.relative_button = new QPushButton(
        QStringLiteral("Set Relative Xform"),
        static_value_editor_.group);
    static_layout->addWidget(static_value_editor_.relative_button);
    QObject::connect(static_value_editor_.relative_button,
        &QPushButton::clicked, this, [this] {
            apply_relative_xform();
        });
    content_layout->addWidget(static_value_editor_.group);
    static_value_editor_.group->setVisible(false);

    nameGroup = new QGroupBox(QStringLiteral("Name"), content);
    auto* name_layout = new QFormLayout(nameGroup);
    nameField = new QLineEdit(nameGroup);
    nameField->setPlaceholderText(QStringLiteral("component name"));
    name_layout->addRow(QStringLiteral("Name"), nameField);
    QObject::connect(nameField, &QLineEdit::editingFinished, this, [this] {
        if (!nameField->isModified()) {
            return;
        }
        apply_component_name();
        nameField->setModified(false);
    });
    content_layout->addWidget(nameGroup);

    transform_group_ = new QGroupBox(QStringLiteral("Transform"), content);
    auto* transform_layout = new QFormLayout(transform_group_);
    transform_source_ = new QLabel(transform_group_);
    transform_source_->setWordWrap(true);

    const auto make_vector_editor = [this](const char* const labels[3]) {
        VectorEditor editor;
        editor.widget = new QWidget(transform_group_);
        auto* layout = new QHBoxLayout(editor.widget);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(4);
        for (int index = 0; index < 3; ++index) {
            auto* label = new QLabel(
                QString::fromLatin1(labels[index]), editor.widget);
            label->setAlignment(Qt::AlignCenter);
            auto* field = new QLineEdit(editor.widget);
            field->setAlignment(Qt::AlignRight);
            field->setSizePolicy(
                QSizePolicy::Expanding, QSizePolicy::Preferred);
            auto* validator = new QDoubleValidator(
                -1.0e12, 1.0e12, 6, field);
            validator->setNotation(QDoubleValidator::ScientificNotation);
            field->setValidator(validator);
            layout->addWidget(label);
            layout->addWidget(field);
            editor.fields[static_cast<std::size_t>(index)] = field;
        }
        return editor;
    };

    constexpr const char* kVectorLabels[] = {"X", "Y", "Z"};
    translation_editor_ = make_vector_editor(kVectorLabels);
    rotation_editor_ = make_vector_editor(kVectorLabels);
    scale_editor_ = make_vector_editor(kVectorLabels);
    transform_layout->addRow(QStringLiteral("Source"), transform_source_);
    transform_layout->addRow(
        QStringLiteral("Translation"), translation_editor_.widget);
    transform_layout->addRow(
        QStringLiteral("Rotation (deg)"), rotation_editor_.widget);
    transform_layout->addRow(QStringLiteral("Scale"), scale_editor_.widget);

    for (std::size_t index = 0;
         index < translation_editor_.fields.size(); ++index)
    {
        auto* field = translation_editor_.fields[index];
        QObject::connect(field, &QLineEdit::editingFinished, this,
            [this, field, index] {
                if (!field->isModified()) {
                    return;
                }
                apply_vector_edit(
                    VectorProperty::Translation, static_cast<int>(index));
                field->setModified(false);
            });
    }
    for (std::size_t index = 0;
         index < rotation_editor_.fields.size(); ++index)
    {
        auto* field = rotation_editor_.fields[index];
        QObject::connect(field, &QLineEdit::editingFinished, this,
            [this, field, index] {
                if (!field->isModified()) {
                    return;
                }
                apply_vector_edit(
                    VectorProperty::Rotation, static_cast<int>(index));
                field->setModified(false);
            });
    }
    for (std::size_t index = 0;
         index < scale_editor_.fields.size(); ++index)
    {
        auto* field = scale_editor_.fields[index];
        QObject::connect(field, &QLineEdit::editingFinished, this,
            [this, field, index] {
                if (!field->isModified()) {
                    return;
                }
                apply_vector_edit(
                    VectorProperty::Scale, static_cast<int>(index));
                field->setModified(false);
            });
    }
    content_layout->addWidget(transform_group_);

    transform_parent_group_ = new QGroupBox(
        QStringLiteral("Transform Parent"), content);
    auto* parent_layout = new QVBoxLayout(transform_parent_group_);
    transform_parent_source_ = new QLabel(transform_parent_group_);
    transform_parent_source_->setWordWrap(true);
    parent_layout->addWidget(transform_parent_source_);
    auto* parent_buttons = new QHBoxLayout();
    bind_transform_parent_button_ = new QPushButton(
        QStringLiteral("Bind selected parent"), transform_parent_group_);
    clear_transform_parent_button_ = new QPushButton(
        QStringLiteral("Clear parent"), transform_parent_group_);
    parent_buttons->addWidget(bind_transform_parent_button_);
    parent_buttons->addWidget(clear_transform_parent_button_);
    parent_layout->addLayout(parent_buttons);
    QObject::connect(bind_transform_parent_button_, &QPushButton::clicked,
        this, [this] { bind_selected_transform_parent(); });
    QObject::connect(clear_transform_parent_button_, &QPushButton::clicked,
        this, [this] { clear_selected_transform_parent(); });
    content_layout->addWidget(transform_parent_group_);
    transform_parent_group_->setVisible(false);

    draft_note_ = new QLabel(
        QStringLiteral("Edit a component and press Enter or leave the field "
                       "to write it back to the selected element."),
        content);
    draft_note_->setWordWrap(true);
    draft_note_->setStyleSheet(QStringLiteral("color: palette(mid);"));
    content_layout->addWidget(draft_note_);
    content_layout->addStretch(1);

    scroll_area_->setWidget(content);
    refresh();
}

void PropertyEditor::refresh() {
    if (context_ == Context::NodeGraph) {
        refresh_node_graph_context();
        return;
    }
    refresh_scene_context();
}

void PropertyEditor::refresh_scene_context() {
    selection_group_->setVisible(true);
    node_group_->setVisible(false);
    static_value_editor_.group->setVisible(false);
    draft_note_->setVisible(true);

    const auto& refs = selection_.refs();
    if (refs.empty()) {
        selection_summary_->setText(QStringLiteral("Nothing selected"));
        selected_items_->setText(
            QStringLiteral("Select a viewport element to inspect its properties."));
        nameGroup->setVisible(false);
        transform_group_->setVisible(false);
        transform_parent_group_->setVisible(false);
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
    const bool named_focus = focus != nullptr
        && (focus->kind == SelectionRef::Kind::Joint
            || focus->kind == SelectionRef::Kind::Locator
            || focus->kind == SelectionRef::Kind::Controller);
    nameGroup->setTitle(named_focus
        ? kind_name(focus->kind) : QStringLiteral("Name"));
    nameGroup->setVisible(named_focus);
    if (named_focus && !nameField->hasFocus()) {
        const QSignalBlocker blocker(nameField);
        nameField->setText(component_name(*focus, components_));
    }

    TransformValues values;
    QString source;
    if (focus == nullptr || !make_transform(*focus, values, source)) {
        transform_group_->setVisible(false);
        transform_parent_group_->setVisible(false);
        return;
    }

    transform_source_->setText(source);
    set_vector_editor(translation_editor_, values.translation, true);
    const bool has_trs = focus->kind != SelectionRef::Kind::Vector;
    set_vector_editor(
        rotation_editor_, values.rotation_degrees, has_trs);
    set_vector_editor(scale_editor_, values.scale, has_trs);
    transform_group_->setVisible(true);

    const bool locator_focus =
        focus != nullptr && focus->kind == SelectionRef::Kind::Locator;
    transform_parent_group_->setVisible(locator_focus);
    if (locator_focus) {
        const auto* relation = components_.transform_parent(
            focus->component);
        if (relation == nullptr
            || relation->parent_kind == orlrig::TransformParentKind::World)
        {
            transform_parent_source_->setText(
                QStringLiteral("Parent: World"));
            clear_transform_parent_button_->setEnabled(false);
        } else {
            const auto* parent = components_.find(relation->parent);
            transform_parent_source_->setText(
                QStringLiteral("Parent: %1")
                    .arg(parent == nullptr
                        ? QStringLiteral("<missing>")
                        : QString::fromStdString(parent->name)));
            clear_transform_parent_button_->setEnabled(true);
        }
        bool has_parent_candidate = false;
        for (const auto& ref : refs) {
            if (ref.component != focus->component
                && (ref.kind == SelectionRef::Kind::Joint
                    || ref.kind == SelectionRef::Kind::Locator))
            {
                has_parent_candidate = true;
                break;
            }
        }
        bind_transform_parent_button_->setEnabled(has_parent_candidate);
    }
}

void PropertyEditor::bind_selected_transform_parent()
{
    ComponentId child;
    ComponentId parent;
    const auto* focus = selection_.focus();
    for (const auto& ref : selection_.refs()) {
        if (ref.kind == SelectionRef::Kind::Locator
            && (!child
                || (focus != nullptr && ref.component == focus->component)))
        {
            child = ref.component;
        }
    }
    for (const auto& ref : selection_.refs()) {
        if (ref.component == child
            || (ref.kind != SelectionRef::Kind::Joint
                && ref.kind != SelectionRef::Kind::Locator))
        {
            continue;
        }
        if (parent) {
            QMessageBox::warning(this, QStringLiteral("Transform parent"),
                QStringLiteral(
                    "Select exactly one parent joint or locator."));
            return;
        }
        parent = ref.component;
    }
    if (!child || !parent) {
        QMessageBox::warning(this, QStringLiteral("Transform parent"),
            QStringLiteral(
                "Select a locator and exactly one joint or locator parent."));
        return;
    }
    std::string error;
    if (!components_.bind_transform_parent(
            child, parent, true, true, true, true, &error))
    {
        QMessageBox::warning(this, QStringLiteral("Transform parent"),
            QString::fromStdString(error));
        return;
    }
    if (runtime_config.evaluate_orl) {
        runtime_config.evaluate_orl = false;
        runtime_config.evaluation_phase = OrlEvaluationPhase::Disabled;
        if (graphContext != nullptr) {
            graphContext->clear_computed_joints_device();
            graphContext->clear_computed_locators_device();
        }
    }
    if (graphContext != nullptr) {
        graphContext->refresh_scene_inputs();
    }
    refresh();
}

void PropertyEditor::clear_selected_transform_parent()
{
    const auto* focus = selection_.focus();
    if (focus == nullptr || focus->kind != SelectionRef::Kind::Locator) {
        return;
    }
    if (!components_.clear_transform_parent(focus->component)) {
        return;
    }
    if (runtime_config.evaluate_orl) {
        runtime_config.evaluate_orl = false;
        runtime_config.evaluation_phase = OrlEvaluationPhase::Disabled;
        if (graphContext != nullptr) {
            graphContext->clear_computed_joints_device();
            graphContext->clear_computed_locators_device();
        }
    }
    if (graphContext != nullptr) {
        graphContext->refresh_scene_inputs();
    }
    refresh();
}

void PropertyEditor::refresh_node_graph_context() {
    selection_group_->setVisible(false);
    node_group_->setVisible(true);
    nameGroup->setVisible(false);
    transform_group_->setVisible(false);
    transform_parent_group_->setVisible(false);
    draft_note_->setVisible(false);

    if (node_graph_editor_ == nullptr) {
        node_stage_->setText(QStringLiteral("Unavailable"));
        node_kind_->setText(QStringLiteral("No graph"));
        node_id_->setText(QStringLiteral("None"));
        node_definition_->setText(QStringLiteral("Node graph is unavailable."));
        node_ports_->setText(QStringLiteral("None"));
        refresh_static_value_editor(std::nullopt, {});
        return;
    }

    node_stage_->setText(
        node_graph_editor_->stage() == orlgraph::GraphStage::Solver
            ? QStringLiteral("Solver")
            : QStringLiteral("Deformer"));
    const auto node = node_graph_editor_->selected_node_info();
    if (!node.has_value()) {
        node_kind_->setText(QStringLiteral("No node selected"));
        node_id_->setText(QStringLiteral("None"));
        node_definition_->setText(
            QStringLiteral("Click a node in the graph to inspect it."));
        node_ports_->setText(QStringLiteral("None"));
        refresh_static_value_editor(std::nullopt, {});
        return;
    }

    node_kind_->setText(node->kind);
    node_id_->setText(node->id);
    node_definition_->setText(node->definition.isEmpty()
        ? node->title : node->definition);
    const QString inputs = node->inputs.isEmpty()
        ? QStringLiteral("None") : node->inputs.join(QStringLiteral(", "));
    const QString outputs = node->outputs.isEmpty()
        ? QStringLiteral("None") : node->outputs.join(QStringLiteral(", "));
    node_ports_->setText(
        QStringLiteral("Inputs: %1\nOutputs: %2").arg(inputs, outputs));
    refresh_static_value_editor(node->static_value, node->id);
}

void PropertyEditor::refresh_static_value_editor(
    const std::optional<orlgraph::ConstantValue>& value,
    const QString& node_id)
{
    const bool graph_editing_enabled = node_graph_editor_ != nullptr
        && node_graph_editor_->graph_editing_enabled();
    static_value_editor_.group->setEnabled(graph_editing_enabled);
    if (!value.has_value()
        || value->type.lanes == 0
        || !std::holds_alternative<std::vector<double>>(value->value))
    {
        static_value_editor_.group->setVisible(false);
        static_value_editor_.kind.reset();
        static_value_editor_.node_id.clear();
        return;
    }

    const auto& lanes = std::get<std::vector<double>>(value->value);
    const bool same_node = static_value_editor_.node_id == node_id
        && static_value_editor_.kind.has_value()
        && *static_value_editor_.kind == value->type.kind
        && static_value_editor_.fields.size() == lanes.size();

    if (!same_node) {
        if (static_value_editor_.fields_widget->layout() != nullptr) {
            QLayoutItem* item = nullptr;
            while ((item = static_value_editor_.fields_widget->layout()
                ->takeAt(0)) != nullptr)
            {
                delete item->widget();
                delete item;
            }
            delete static_value_editor_.fields_widget->layout();
        }
        static_value_editor_.fields.clear();

        auto* layout = new QGridLayout(static_value_editor_.fields_widget);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setHorizontalSpacing(4);
        layout->setVerticalSpacing(4);

        const bool matrix =
            value->type.kind == orlgraph::LogicalTypeKind::Matrix;
        static_value_editor_.group->setTitle(matrix
            ? QStringLiteral("Matrix Value")
            : value->type.kind == orlgraph::LogicalTypeKind::Vec4
                ? QStringLiteral("Vec4 Value")
                : QStringLiteral("Vec3 Value"));

        for (std::size_t index = 0; index < lanes.size(); ++index) {
            auto* field = new QLineEdit(static_value_editor_.fields_widget);
            field->setAlignment(Qt::AlignRight);
            auto* validator = new QDoubleValidator(
                -1.0e12, 1.0e12, 6, field);
            validator->setNotation(QDoubleValidator::ScientificNotation);
            field->setValidator(validator);
            QObject::connect(field, &QLineEdit::editingFinished, this,
                [this, field] {
                    if (!field->isModified()) {
                        return;
                    }
                    apply_static_value_edit();
                    field->setModified(false);
                });
            if (matrix) {
                layout->addWidget(field,
                    static_cast<int>(index / 4),
                    static_cast<int>(index % 4));
            } else {
                static constexpr const char* kLabels[] = {
                    "X", "Y", "Z", "W"};
                auto* label = new QLabel(
                    QString::fromLatin1(
                        kLabels[std::min(index, std::size_t{3})]),
                    static_value_editor_.fields_widget);
                label->setAlignment(Qt::AlignCenter);
                layout->addWidget(label, 0, static_cast<int>(index * 2));
                layout->addWidget(field, 0, static_cast<int>(index * 2 + 1));
            }
            static_value_editor_.fields.push_back(field);
        }

        static_value_editor_.kind = value->type.kind;
        static_value_editor_.node_id = node_id;
        static_value_editor_.relative_button->setVisible(matrix);
    }

    for (std::size_t index = 0; index < lanes.size(); ++index) {
        auto* field = static_value_editor_.fields[index];
        if (field->hasFocus()) {
            continue;
        }
        const QSignalBlocker blocker(field);
        field->setText(QString::number(lanes[index], 'g', 8));
    }
    static_value_editor_.group->setVisible(true);
}

void PropertyEditor::apply_static_value_edit()
{
    if (node_graph_editor_ == nullptr
        || !static_value_editor_.kind.has_value())
    {
        return;
    }

    orlgraph::LogicalType type;
    switch (*static_value_editor_.kind) {
    case orlgraph::LogicalTypeKind::Vector:
        type = orlgraph::LogicalType::vector();
        break;
    case orlgraph::LogicalTypeKind::Vec4:
        type = orlgraph::LogicalType::vec4();
        break;
    case orlgraph::LogicalTypeKind::Matrix:
        type = orlgraph::LogicalType::matrix();
        break;
    default:
        return;
    }

    std::vector<double> lanes;
    lanes.reserve(static_value_editor_.fields.size());
    for (auto* field : static_value_editor_.fields) {
        bool ok = false;
        const double parsed = field->text().toDouble(&ok);
        if (!ok || !std::isfinite(parsed)) {
            refresh();
            return;
        }
        lanes.push_back(parsed);
    }
    if (lanes.size() != type.lanes) {
        refresh();
        return;
    }

    const auto result = node_graph_editor_->set_selected_static_value(
        orlgraph::ConstantValue{std::move(type), std::move(lanes)});
    if (!result.ok) {
        QMessageBox::warning(this, QStringLiteral("Static Value"),
            result.error);
    }
    refresh();
}

void PropertyEditor::apply_relative_xform()
{
    if (node_graph_editor_ == nullptr) {
        return;
    }
    const auto result =
        node_graph_editor_->apply_relative_xform_to_selected_matrix();
    if (!result.ok) {
        QMessageBox::warning(this, QStringLiteral("Set Relative Xform"),
            result.error);
    }
    refresh();
}

bool PropertyEditor::read_vector_component(
    const VectorEditor& editor, int component, float& value) const
{
    if (component < 0
        || static_cast<std::size_t>(component) >= editor.fields.size())
    {
        return false;
    }
    bool ok = false;
    const double parsed = editor.fields[static_cast<std::size_t>(component)]
        ->text().toDouble(&ok);
    if (!ok || !std::isfinite(parsed)) {
        return false;
    }
    value = static_cast<float>(parsed);
    return true;
}

void PropertyEditor::set_vector_editor(const VectorEditor& editor,
    const glm::vec3& value, bool enabled, bool preserve_focus)
{
    editor.widget->setEnabled(enabled);
    for (int index = 0; index < 3; ++index) {
        auto* field = editor.fields[static_cast<std::size_t>(index)];
        if (preserve_focus && field->hasFocus()) {
            continue;
        }
        const QSignalBlocker blocker(field);
        field->setText(QString::number(
            value[static_cast<std::size_t>(index)], 'f', 3));
    }
}

glm::vec3& PropertyEditor::vector_value(
    TransformValues& values, VectorProperty property)
{
    switch (property) {
    case VectorProperty::Translation:
        return values.translation;
    case VectorProperty::Rotation:
        return values.rotation_degrees;
    case VectorProperty::Scale:
        return values.scale;
    }
    return values.translation;
}

void PropertyEditor::apply_vector_edit(
    VectorProperty property, int component)
{
    const auto* focus = selection_.focus();
    if (focus == nullptr) {
        return;
    }

    auto attr = selection_.dest(*focus);
    TransformValues current;
    QString source;
    if (!attr || !make_transform(*focus, current, source)) {
        refresh();
        return;
    }

    const VectorEditor* editor = nullptr;
    switch (property) {
    case VectorProperty::Translation:
        editor = &translation_editor_;
        break;
    case VectorProperty::Rotation:
        editor = &rotation_editor_;
        break;
    case VectorProperty::Scale:
        editor = &scale_editor_;
        break;
    }

    glm::vec3 edited = vector_value(current, property);
    float component_value = 0.0f;
    if (editor == nullptr
        || !read_vector_component(*editor, component, component_value))
    {
        if (editor != nullptr) {
            if (component >= 0
                && static_cast<std::size_t>(component)
                    < editor->fields.size())
            {
                auto* field = editor->fields[
                    static_cast<std::size_t>(component)];
                const QSignalBlocker blocker(field);
                field->setText(QString::number(
                    vector_value(current, property)[component], 'f', 3));
            }
        }
        return;
    }
    edited[component] = component_value;

    bool written = false;
    if (focus->kind == SelectionRef::Kind::Joint) {
        switch (property) {
        case VectorProperty::Translation:
            attr.set_world_position(edited);
            written = true;
            break;
        case VectorProperty::Rotation:
            attr.set_local_rotation(euler_rotation(edited));
            written = true;
            break;
        case VectorProperty::Scale:
            attr.set_local_scale(edited);
            written = true;
            break;
        }
    }
    else if (attr.has_world_matrix()) {
        glm::vec3 scale{1.0f};
        glm::vec3 translation{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
        if (!decompose_transform(
                attr.world_matrix(), scale, rotation, translation))
        {
            refresh();
            return;
        }
        switch (property) {
        case VectorProperty::Translation:
            translation = edited;
            break;
        case VectorProperty::Rotation:
            rotation = euler_rotation(edited);
            break;
        case VectorProperty::Scale:
            scale = edited;
            break;
        }
        const glm::mat4 world = glm::translate(glm::mat4{1.0f}, translation)
            * glm::mat4_cast(rotation)
            * glm::scale(glm::mat4{1.0f}, scale);
        written = attr.set_world_matrix(world);
    }
    else if (focus->kind == SelectionRef::Kind::Vector
        && property == VectorProperty::Translation)
    {
        attr.set_world_position(edited);
        written = true;
    }

    if (!written) {
        refresh();
        return;
    }
    refresh();
}

void PropertyEditor::apply_component_name()
{
    const auto* focus = selection_.focus();
    const char* find_node = nullptr;
    if (focus != nullptr && focus->kind == SelectionRef::Kind::Joint) {
        find_node = "orlrig.input.find_joint";
    } else if (focus != nullptr && focus->kind == SelectionRef::Kind::Locator) {
        find_node = "orlrig.input.find_locator";
    }
    if (find_node == nullptr) {
        return;
    }
    const auto* component = components_.find(focus->component);
    if (component == nullptr) {
        return;
    }
    const std::string old_name = component->name;
    const std::string new_name = nameField->text().trimmed().toStdString();
    const auto restore = [this, &old_name] {
        const QSignalBlocker blocker(nameField);
        nameField->setText(QString::fromStdString(old_name));
    };
    if (new_name.empty()) {
        restore();
        return;
    }
    if (new_name == old_name) {
        restore();
        return;
    }
    if (!components_.rename(focus->component, new_name)) {
        restore();
        return;
    }
    if (graphContext != nullptr) {
        graphContext->retarget_element_name(find_node, old_name, new_name);
    }
    const QSignalBlocker blocker(nameField);
    nameField->setText(QString::fromStdString(new_name));
    refresh();
}

bool PropertyEditor::make_transform(const SelectionRef& ref,
    TransformValues& values, QString& source) const
{
    const auto attr = selection_.dest(ref);
    if (!attr) {
        return false;
    }

    if (ref.kind == SelectionRef::Kind::Joint) {
        values.translation = attr.world_position();
        values.rotation_degrees = glm::degrees(
            glm::eulerAngles(attr.local_rotation()));
        values.scale = attr.local_scale();
        source = QStringLiteral("Joint transform");
        return true;
    }

    if (attr.has_world_matrix()) {
        glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
        if (!decompose_transform(
                attr.world_matrix(), values.scale, orientation,
                values.translation))
        {
            return false;
        }
        values.rotation_degrees = glm::degrees(
            glm::eulerAngles(orientation));
        source = ref.kind == SelectionRef::Kind::Controller
            ? QStringLiteral("Controller matrix converted to TRS for display")
            : QStringLiteral("World transform");
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
