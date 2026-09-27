#include "property_editor.hpp"

#if ORL_USE_QT6

#include <cmath>

#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
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
    draft_note_->setVisible(true);

    const auto& refs = selection_.refs();
    if (refs.empty()) {
        selection_summary_->setText(QStringLiteral("Nothing selected"));
        selected_items_->setText(
            QStringLiteral("Select a viewport element to inspect its properties."));
        nameGroup->setVisible(false);
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
        return;
    }

    transform_source_->setText(source);
    set_vector_editor(translation_editor_, values.translation, true);
    const bool has_trs = focus->kind != SelectionRef::Kind::Vector;
    set_vector_editor(
        rotation_editor_, values.rotation_degrees, has_trs);
    set_vector_editor(scale_editor_, values.scale, has_trs);
    transform_group_->setVisible(true);
}

void PropertyEditor::refresh_node_graph_context() {
    selection_group_->setVisible(false);
    node_group_->setVisible(true);
    nameGroup->setVisible(false);
    transform_group_->setVisible(false);
    draft_note_->setVisible(false);

    if (node_graph_editor_ == nullptr) {
        node_stage_->setText(QStringLiteral("Unavailable"));
        node_kind_->setText(QStringLiteral("No graph"));
        node_id_->setText(QStringLiteral("None"));
        node_definition_->setText(QStringLiteral("Node graph is unavailable."));
        node_ports_->setText(QStringLiteral("None"));
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
