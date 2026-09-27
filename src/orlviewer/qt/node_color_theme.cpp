#include "node_color_theme.hpp"

#if ORL_USE_QT6

#include <QFile>
#include <QIODevice>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>

#include <string>
#include <string_view>
#include <utility>

namespace ORL
{

namespace
{

std::string category_for(std::string_view qualified_name) {
    if (qualified_name == "orlrig.stage.computed_joints") {
        return "input";
    }
    if (qualified_name.find(".stage.") != std::string_view::npos) {
        return "stage";
    }
    if (qualified_name.find(".deformer.") != std::string_view::npos) {
        return "deformer";
    }
    if (qualified_name.find(".auto_weight.") != std::string_view::npos) {
        return "auto_weight";
    }
    if (qualified_name.find(".solver.") != std::string_view::npos) {
        return "solver";
    }
    if (qualified_name.find(".constraint.") != std::string_view::npos) {
        return "constraint";
    }
    if (qualified_name.find(".input") != std::string_view::npos) {
        return "input";
    }
    if (qualified_name.find(".output") != std::string_view::npos) {
        return "output";
    }
    return "utilities";
}

void append_error(std::string* error, std::string message) {
    if (error == nullptr) {
        return;
    }
    if (!error->empty()) {
        *error += "; ";
    }
    *error += std::move(message);
}

bool read_color(const QJsonValue& value, std::string_view field,
    QColor* color, std::string* error)
{
    if (!value.isString()) {
        append_error(error, "Theme field '" + std::string{field}
            + "' must be a color string");
        return false;
    }

    const QColor parsed{value.toString()};
    if (!parsed.isValid()) {
        append_error(error, "Theme field '" + std::string{field}
            + "' is not a valid color");
        return false;
    }
    *color = parsed;
    return true;
}

void read_color_property(const QJsonObject& object, const char* key,
    QColor* color, std::string* error)
{
    const QString json_key = QString::fromLatin1(key);
    if (!object.contains(json_key)) {
        return;
    }
    read_color(object.value(json_key), key, color, error);
}

void read_color_map(const QJsonObject& object, const char* section,
    QHash<QString, QColor>* colors, std::string* error)
{
    const QString json_key = QString::fromLatin1(section);
    if (!object.contains(json_key)) {
        return;
    }
    const QJsonValue value = object.value(json_key);
    if (!value.isObject()) {
        append_error(error, "Theme field '" + std::string{section}
            + "' must be an object");
        return;
    }

    const QJsonObject entries = value.toObject();
    for (auto iterator = entries.constBegin();
         iterator != entries.constEnd(); ++iterator)
    {
        QColor color;
        const std::string field = std::string{section} + "."
            + iterator.key().toStdString();
        if (read_color(iterator.value(), field, &color, error)) {
            colors->insert(iterator.key(), color);
        }
    }
}

} // namespace

NodeColorTheme NodeColorTheme::default_theme() {
    NodeColorTheme theme;
    theme.default_color_ = QColor{QStringLiteral("#61758e")};
    theme.graph_input_color_ = QColor{QStringLiteral("#4c78a8")};
    theme.graph_output_color_ = QColor{QStringLiteral("#5b8e7d")};

    theme.set_category_color("input", QColor{QStringLiteral("#4c78a8")});
    theme.set_category_color("stage", QColor{QStringLiteral("#5b8e7d")});
    theme.set_category_color("deformer", QColor{QStringLiteral("#805ad5")});
    theme.set_category_color("auto_weight", QColor{QStringLiteral("#b7791f")});
    theme.set_category_color("solver", QColor{QStringLiteral("#5b8e7d")});
    theme.set_category_color("constraint", QColor{QStringLiteral("#c05621")});
    theme.set_category_color("output", QColor{QStringLiteral("#2f855a")});
    theme.set_category_color("utilities", QColor{QStringLiteral("#61758e")});

    // Keep the built-in theme useful even when its optional configuration
    // file is missing. Explicit definitions can distinguish input types
    // while every instance of a definition still receives the same color.
    theme.set_definition_color(
        "orlrig.input.find_joint", QColor{QStringLiteral("#4c78a8")});
    theme.set_definition_color(
        "orlrig.input.find_locator", QColor{QStringLiteral("#805ad5")});
    theme.set_definition_color(
        "orlrig.input.find_mesh", QColor{QStringLiteral("#b7791f")});
    return theme;
}

NodeColorTheme NodeColorTheme::from_file(
    const std::filesystem::path& path, std::string* error)
{
    NodeColorTheme theme = default_theme();
    QFile file(QString::fromStdString(path.string()));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        append_error(error, "Unable to open node color theme '"
            + path.string() + "': " + file.errorString().toStdString());
        return theme;
    }

    QJsonParseError parse_error;
    const QJsonDocument document =
        QJsonDocument::fromJson(file.readAll(), &parse_error);
    if (parse_error.error != QJsonParseError::NoError) {
        append_error(error, "Unable to parse node color theme '"
            + path.string() + "': "
            + parse_error.errorString().toStdString());
        return theme;
    }
    if (!document.isObject()) {
        append_error(error, "Node color theme '" + path.string()
            + "' must contain a JSON object");
        return theme;
    }

    const QJsonObject root = document.object();
    read_color_property(root, "default", &theme.default_color_, error);
    read_color_property(
        root, "graph_input", &theme.graph_input_color_, error);
    read_color_property(
        root, "graph_output", &theme.graph_output_color_, error);
    read_color_map(
        root, "categories", &theme.category_colors_, error);
    read_color_map(
        root, "definitions", &theme.definition_colors_, error);
    return theme;
}

QColor NodeColorTheme::color_for_definition(
    std::string_view qualified_name) const
{
    const QString definition_key =
        QString::fromStdString(std::string{qualified_name});
    if (const auto definition = definition_colors_.constFind(definition_key);
        definition != definition_colors_.constEnd())
    {
        return definition.value();
    }

    const QString category_key = QString::fromStdString(
        category_for(qualified_name));
    if (const auto category = category_colors_.constFind(category_key);
        category != category_colors_.constEnd())
    {
        return category.value();
    }
    return default_color_;
}

QColor NodeColorTheme::color_for_interface(bool graph_input) const {
    return graph_input ? graph_input_color_ : graph_output_color_;
}

void NodeColorTheme::set_default_color(QColor color) {
    if (color.isValid()) {
        default_color_ = std::move(color);
    }
}

void NodeColorTheme::set_category_color(
    std::string_view category, QColor color)
{
    if (!color.isValid() || category.empty()) {
        return;
    }
    category_colors_.insert(
        QString::fromStdString(std::string{category}), std::move(color));
}

void NodeColorTheme::set_definition_color(
    std::string_view qualified_name, QColor color)
{
    if (!color.isValid() || qualified_name.empty()) {
        return;
    }
    definition_colors_.insert(
        QString::fromStdString(std::string{qualified_name}),
        std::move(color));
}

void NodeColorTheme::set_interface_color(bool graph_input, QColor color) {
    if (!color.isValid()) {
        return;
    }
    if (graph_input) {
        graph_input_color_ = std::move(color);
    } else {
        graph_output_color_ = std::move(color);
    }
}

} // namespace ORL

#endif
