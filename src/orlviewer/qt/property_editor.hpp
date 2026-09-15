#pragma once

#if ORL_USE_QT6

#include <QString>
#include <QWidget>

#include <glm/vec3.hpp>

#include "../selection.hpp"

class QGroupBox;
class QLabel;
class QScrollArea;

namespace ORL
{

class ComponentManager;

// Draft property panel for the current viewport selection. Values are
// intentionally read-only until property editing has a defined undo/runtime
// model.
class PropertyEditor final : public QWidget {
public:
    PropertyEditor(const Selection& selection,
        const ComponentManager& components, QWidget* parent = nullptr);

    void refresh();

private:
    struct TransformValues {
        glm::vec3 translation{0.0f};
        glm::vec3 rotation_degrees{0.0f};
        glm::vec3 scale{1.0f};
    };

    bool make_transform(const SelectionRef& ref,
        TransformValues& values, QString& source) const;
    static QString kind_name(SelectionRef::Kind kind);
    static QString format_vector(const glm::vec3& value);
    static QString component_name(const SelectionRef& ref,
        const ComponentManager& components);

    const Selection& selection_;
    const ComponentManager& components_;
    QLabel* selection_summary_ = nullptr;
    QLabel* selected_items_ = nullptr;
    QGroupBox* transform_group_ = nullptr;
    QLabel* transform_source_ = nullptr;
    QLabel* translation_value_ = nullptr;
    QLabel* rotation_value_ = nullptr;
    QLabel* scale_value_ = nullptr;
    QScrollArea* scroll_area_ = nullptr;
};

} // namespace ORL

#endif
