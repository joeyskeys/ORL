#pragma once

#if ORL_USE_QT6

#include <filesystem>
#include <string>
#include <string_view>

#include <QColor>
#include <QHash>

namespace ORL
{

// Colors used by the custom-painted node graph. Definition colors are keyed
// by qualified node type, with category colors providing a stable fallback for
// definitions that are not listed explicitly in the theme file.
class NodeColorTheme final {
public:
    static NodeColorTheme default_theme();
    static NodeColorTheme from_file(
        const std::filesystem::path& path, std::string* error = nullptr);

    QColor default_color() const { return default_color_; }
    QColor color_for_definition(std::string_view qualified_name) const;
    QColor color_for_interface(bool graph_input) const;

    void set_default_color(QColor color);
    void set_category_color(std::string_view category, QColor color);
    void set_definition_color(
        std::string_view qualified_name, QColor color);
    void set_interface_color(bool graph_input, QColor color);

private:
    NodeColorTheme() = default;

    QColor default_color_;
    QColor graph_input_color_;
    QColor graph_output_color_;
    QHash<QString, QColor> category_colors_;
    QHash<QString, QColor> definition_colors_;
};

} // namespace ORL

#endif
