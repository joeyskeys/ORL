#pragma once

#if ORL_USE_QT6

#include <array>

#include <QString>
#include <QWidget>

#include <glm/vec3.hpp>

#include "../selection.hpp"

class QGroupBox;
class QLabel;
class QLineEdit;
class QScrollArea;

namespace ORL
{

class ComponentManager;
class SceneGraphContext;

class PropertyEditor final : public QWidget {
public:
    PropertyEditor(const Selection& selection,
        ComponentManager& components, SceneGraphContext* graph_context,
        QWidget* parent = nullptr);

    void refresh();

private:
    enum class VectorProperty {
        Translation,
        Rotation,
        Scale,
    };

    struct TransformValues {
        glm::vec3 translation{0.0f};
        glm::vec3 rotation_degrees{0.0f};
        glm::vec3 scale{1.0f};
    };

    struct VectorEditor {
        QWidget* widget = nullptr;
        std::array<QLineEdit*, 3> fields{};
    };

    bool make_transform(const SelectionRef& ref,
        TransformValues& values, QString& source) const;
    bool read_vector_component(const VectorEditor& editor,
        int component, float& value) const;
    void set_vector_editor(const VectorEditor& editor,
        const glm::vec3& value, bool enabled,
        bool preserve_focus = true);
    void apply_vector_edit(VectorProperty property, int component);
    void apply_component_name();
    static glm::vec3& vector_value(
        TransformValues& values, VectorProperty property);
    static QString kind_name(SelectionRef::Kind kind);
    static QString component_name(const SelectionRef& ref,
        const ComponentManager& components);

    const Selection& selection_;
    ComponentManager& components_;
    SceneGraphContext* graphContext = nullptr;
    QLabel* selection_summary_ = nullptr;
    QLabel* selected_items_ = nullptr;
    QGroupBox* nameGroup = nullptr;
    QLineEdit* nameField = nullptr;
    QGroupBox* transform_group_ = nullptr;
    QLabel* transform_source_ = nullptr;
    VectorEditor translation_editor_;
    VectorEditor rotation_editor_;
    VectorEditor scale_editor_;
    QScrollArea* scroll_area_ = nullptr;
};

} // namespace ORL

#endif
