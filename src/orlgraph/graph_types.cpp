#include "graph_types.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

namespace orlgraph
{

namespace
{

LogicalType named(LogicalTypeKind kind, std::string name, std::uint32_t lanes = 0) {
    LogicalType type;
    type.kind = kind;
    type.name = std::move(name);
    type.lanes = lanes;
    return type;
}

} // namespace

LogicalType LogicalType::void_type() {
    return LogicalType{LogicalTypeKind::Void};
}

LogicalType LogicalType::boolean() {
    return LogicalType{LogicalTypeKind::Bool};
}

LogicalType LogicalType::int64() {
    return LogicalType{LogicalTypeKind::Int64};
}

LogicalType LogicalType::float64() {
    return LogicalType{LogicalTypeKind::Float64};
}

LogicalType LogicalType::string() {
    return LogicalType{LogicalTypeKind::String};
}

LogicalType LogicalType::vector() {
    return named(LogicalTypeKind::Vector, "vector", 3);
}

LogicalType LogicalType::point() {
    return named(LogicalTypeKind::Point, "point", 3);
}

LogicalType LogicalType::normal() {
    return named(LogicalTypeKind::Normal, "normal", 3);
}

LogicalType LogicalType::vec4() {
    return named(LogicalTypeKind::Vec4, "vec4", 4);
}

LogicalType LogicalType::quaternion() {
    return named(LogicalTypeKind::Quaternion, "quat", 4);
}

LogicalType LogicalType::matrix() {
    return named(LogicalTypeKind::Matrix, "matrix", 16);
}

LogicalType LogicalType::struct_type(std::string type_name) {
    return named(LogicalTypeKind::Struct, std::move(type_name));
}

LogicalType LogicalType::handle(std::string canonical_name) {
    return named(LogicalTypeKind::Handle, std::move(canonical_name));
}

LogicalType LogicalType::handle_union(std::string canonical_name,
    std::vector<std::string> accepted_handles, bool open)
{
    LogicalType type = named(LogicalTypeKind::Handle,
        std::move(canonical_name));
    std::sort(accepted_handles.begin(), accepted_handles.end());
    accepted_handles.erase(std::unique(
        accepted_handles.begin(), accepted_handles.end()),
        accepted_handles.end());
    type.accepted_handles = std::move(accepted_handles);
    type.open_handle = open;
    return type;
}

LogicalType LogicalType::array(LogicalType element_type, std::size_t count) {
    LogicalType type;
    type.kind = LogicalTypeKind::Array;
    type.extent = count;
    type.element = std::make_shared<LogicalType>(std::move(element_type));
    return type;
}

LogicalType LogicalType::buffer(LogicalType element_type) {
    LogicalType type;
    type.kind = LogicalTypeKind::Buffer;
    type.element = std::make_shared<LogicalType>(std::move(element_type));
    return type;
}

LogicalType LogicalType::from_orl_name(std::string_view type_name) {
    if (type_name == "void") {
        return void_type();
    }
    if (type_name == "bool") {
        return boolean();
    }
    if (type_name == "int" || type_name == "int64") {
        return int64();
    }
    if (type_name == "float" || type_name == "double" || type_name == "float64") {
        return float64();
    }
    if (type_name == "string") {
        return string();
    }
    if (type_name == "vector" || type_name == "vec3" || type_name == "dvec3") {
        return vector();
    }
    if (type_name == "point") {
        return point();
    }
    if (type_name == "normal") {
        return normal();
    }
    if (type_name == "vec4" || type_name == "dvec4") {
        return vec4();
    }
    if (type_name == "quat") {
        return quaternion();
    }
    if (type_name == "matrix" || type_name == "mat4" || type_name == "dmat4") {
        return matrix();
    }
    if (type_name.empty()) {
        return {};
    }
    return struct_type(std::string{type_name});
}

bool LogicalType::is_scalar() const {
    return kind == LogicalTypeKind::Bool
        || kind == LogicalTypeKind::Int64
        || kind == LogicalTypeKind::Float64
        || kind == LogicalTypeKind::String
        || kind == LogicalTypeKind::Handle;
}

bool LogicalType::is_sequence() const {
    return kind == LogicalTypeKind::Array || kind == LogicalTypeKind::Buffer;
}

std::string LogicalType::canonical_name() const {
    switch (kind) {
    case LogicalTypeKind::Void: return "void";
    case LogicalTypeKind::Bool: return "bool";
    case LogicalTypeKind::Int64: return "int64";
    case LogicalTypeKind::Float64: return "float64";
    case LogicalTypeKind::String: return "string";
    case LogicalTypeKind::Vector: return "vector";
    case LogicalTypeKind::Point: return "point";
    case LogicalTypeKind::Normal: return "normal";
    case LogicalTypeKind::Vec4: return "vec4";
    case LogicalTypeKind::Quaternion: return "quat";
    case LogicalTypeKind::Matrix: return "matrix";
    case LogicalTypeKind::Struct: return "struct:" + name;
    case LogicalTypeKind::Array:
        return "array[" + std::to_string(extent) + "]<"
            + (element == nullptr ? "unknown" : element->canonical_name()) + ">";
    case LogicalTypeKind::Buffer:
        return "buffer<" + (element == nullptr ? "unknown" : element->canonical_name()) + ">";
    case LogicalTypeKind::Unknown:
        return "unknown";
    case LogicalTypeKind::Handle:
        if (open_handle) {
            return "handle:any";
        }
        if (!accepted_handles.empty()) {
            std::string result = "handle_union:" + name + "<";
            for (std::size_t index = 0;
                 index < accepted_handles.size(); ++index)
            {
                if (index != 0) {
                    result += "|";
                }
                result += accepted_handles[index];
            }
            return result + ">";
        }
        return "handle:" + name;
    }
    return "unknown";
}

bool operator==(const LogicalType& left, const LogicalType& right) {
    if (left.kind != right.kind
        || left.name != right.name
        || left.lanes != right.lanes
        || left.extent != right.extent
        || left.accepted_handles != right.accepted_handles
        || left.open_handle != right.open_handle)
    {
        return false;
    }
    if (left.element == nullptr || right.element == nullptr) {
        return left.element == nullptr && right.element == nullptr;
    }
    return *left.element == *right.element;
}

bool is_assignable(const LogicalType& source, const LogicalType& destination) {
    if (!source.is_handle() || !destination.is_handle()) {
        return source == destination;
    }
    if (destination.open_handle) {
        return true;
    }
    if (source.open_handle) {
        return false;
    }
    if (destination.accepted_handles.empty()) {
        return source.accepted_handles.empty()
            && source.name == destination.name;
    }
    if (source.accepted_handles.empty()) {
        return std::find(destination.accepted_handles.begin(),
            destination.accepted_handles.end(), source.name)
            != destination.accepted_handles.end();
    }
    return std::includes(
        destination.accepted_handles.begin(),
        destination.accepted_handles.end(),
        source.accepted_handles.begin(),
        source.accepted_handles.end());
}

bool is_valid_handle_name(std::string_view canonical_name) {
    if (canonical_name.empty()
        || canonical_name.find("::") == std::string_view::npos)
    {
        return false;
    }
    std::size_t segment_start = 0;
    while (segment_start < canonical_name.size()) {
        const std::size_t separator = canonical_name.find(
            "::", segment_start);
        const std::size_t segment_end = separator == std::string_view::npos
            ? canonical_name.size() : separator;
        if (segment_end == segment_start) {
            return false;
        }
        for (std::size_t index = segment_start;
             index < segment_end; ++index)
        {
            const unsigned char character =
                static_cast<unsigned char>(canonical_name[index]);
            if (std::isspace(character) != 0
                || std::iscntrl(character) != 0)
            {
                return false;
            }
        }
        if (separator == std::string_view::npos) {
            break;
        }
        segment_start = separator + 2;
    }
    return segment_start < canonical_name.size();
}

bool contains_handle(const LogicalType& type) {
    return type.is_handle()
        || (type.element != nullptr && contains_handle(*type.element));
}

Domain Domain::constant() {
    return Domain{DomainKind::Constant, {}};
}

Domain Domain::rig() {
    return Domain{DomainKind::Rig, {}};
}

Domain Domain::joint() {
    return Domain{DomainKind::Joint, {}};
}

Domain Domain::vertex() {
    return Domain{DomainKind::Vertex, {}};
}

Domain Domain::edge() {
    return Domain{DomainKind::Edge, {}};
}

Domain Domain::face() {
    return Domain{DomainKind::Face, {}};
}

Domain Domain::instance() {
    return Domain{DomainKind::Instance, {}};
}

Domain Domain::buffer() {
    return Domain{DomainKind::Buffer, {}};
}

Domain Domain::custom(std::string domain_name) {
    return Domain{DomainKind::Custom, std::move(domain_name)};
}

std::string Domain::canonical_name() const {
    switch (kind) {
    case DomainKind::Constant: return "constant";
    case DomainKind::Rig: return "rig";
    case DomainKind::Joint: return "joint";
    case DomainKind::Vertex: return "vertex";
    case DomainKind::Edge: return "edge";
    case DomainKind::Face: return "face";
    case DomainKind::Instance: return "instance";
    case DomainKind::Buffer: return "buffer";
    case DomainKind::Custom: return "custom:" + name;
    }
    return "constant";
}

ShapeDimension ShapeDimension::value(std::uint64_t dimension) {
    ShapeDimension result;
    result.kind = ShapeDimensionKind::Constant;
    result.constant = dimension;
    return result;
}

ShapeDimension ShapeDimension::symbol_ref(std::string symbol_name) {
    ShapeDimension result;
    result.kind = ShapeDimensionKind::Symbol;
    result.symbol = std::move(symbol_name);
    return result;
}

std::string ShapeDimension::canonical_name() const {
    return kind == ShapeDimensionKind::Constant
        ? std::to_string(constant)
        : symbol;
}

Shape Shape::scalar() {
    return {};
}

Shape Shape::one(std::string symbol) {
    Shape shape;
    shape.dimensions.push_back(ShapeDimension::symbol_ref(std::move(symbol)));
    return shape;
}

std::string Shape::canonical_name() const {
    if (dimensions.empty()) {
        return "scalar";
    }
    std::ostringstream stream;
    for (std::size_t index = 0; index < dimensions.size(); ++index) {
        if (index != 0) {
            stream << "x";
        }
        stream << dimensions[index].canonical_name();
    }
    return stream.str();
}

} // namespace orlgraph
