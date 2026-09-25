#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace orlgraph
{

enum class LogicalTypeKind : std::uint8_t {
    Void,
    Bool,
    Int64,
    Float64,
    String,
    Vector,
    Point,
    Normal,
    Vec4,
    Quaternion,
    Matrix,
    Struct,
    Array,
    Buffer,
    Unknown,
    Handle,
};

struct LogicalType {
    LogicalTypeKind kind = LogicalTypeKind::Unknown;
    std::string name;
    std::uint32_t lanes = 0;
    std::size_t extent = 0;
    std::shared_ptr<LogicalType> element;
    std::vector<std::string> accepted_handles;
    bool open_handle = false;

    static LogicalType void_type();
    static LogicalType boolean();
    static LogicalType int64();
    static LogicalType float64();
    static LogicalType string();
    static LogicalType vector();
    static LogicalType point();
    static LogicalType normal();
    static LogicalType vec4();
    static LogicalType quaternion();
    static LogicalType matrix();
    static LogicalType struct_type(std::string name);
    static LogicalType handle(std::string canonical_name);
    static LogicalType handle_union(std::string canonical_name,
        std::vector<std::string> accepted_handles,
        bool open = false);
    static LogicalType array(LogicalType element, std::size_t extent);
    static LogicalType buffer(LogicalType element);
    static LogicalType from_orl_name(std::string_view name);

    bool is_scalar() const;
    bool is_sequence() const;
    bool is_handle() const { return kind == LogicalTypeKind::Handle; }
    bool is_open_handle() const { return is_handle() && open_handle; }
    bool is_known() const { return kind != LogicalTypeKind::Unknown; }
    std::string canonical_name() const;

    friend bool operator==(const LogicalType&, const LogicalType&);
    friend bool operator!=(const LogicalType& left, const LogicalType& right) {
        return !(left == right);
    }
};

// Graph connections currently support exact type compatibility only. Keeping
// this policy behind a helper leaves room for plan 7's universal and union
// handle assignability without changing every connection validator again.
bool is_assignable(const LogicalType& source, const LogicalType& destination);
bool is_valid_handle_name(std::string_view canonical_name);
bool contains_handle(const LogicalType& type);

struct StructField {
    std::string name;
    LogicalType type;

    friend bool operator==(const StructField&, const StructField&) = default;
};

struct StructType {
    std::string name;
    std::vector<StructField> fields;

    friend bool operator==(const StructType&, const StructType&) = default;
};

enum class DomainKind : std::uint8_t {
    Constant,
    Rig,
    Joint,
    Vertex,
    Edge,
    Face,
    Instance,
    Buffer,
    Custom,
};

struct Domain {
    DomainKind kind = DomainKind::Constant;
    std::string name;

    static Domain constant();
    static Domain rig();
    static Domain joint();
    static Domain vertex();
    static Domain edge();
    static Domain face();
    static Domain instance();
    static Domain buffer();
    static Domain custom(std::string name);

    std::string canonical_name() const;

    friend bool operator==(const Domain&, const Domain&) = default;
};

enum class ShapeDimensionKind : std::uint8_t {
    Constant,
    Symbol,
};

struct ShapeDimension {
    ShapeDimensionKind kind = ShapeDimensionKind::Constant;
    std::uint64_t constant = 0;
    std::string symbol;

    static ShapeDimension value(std::uint64_t value);
    static ShapeDimension symbol_ref(std::string name);
    std::string canonical_name() const;

    friend bool operator==(const ShapeDimension&, const ShapeDimension&) = default;
};

struct Shape {
    std::vector<ShapeDimension> dimensions;

    static Shape scalar();
    static Shape one(std::string symbol);
    bool is_scalar() const { return dimensions.empty(); }
    std::string canonical_name() const;

    friend bool operator==(const Shape&, const Shape&) = default;
};

using Value = std::variant<
    std::monostate,
    bool,
    std::int64_t,
    double,
    std::string,
    std::vector<std::int64_t>,
    std::vector<double>>;

struct ConstantValue {
    LogicalType type;
    Value value;

    friend bool operator==(const ConstantValue&, const ConstantValue&) = default;
};

} // namespace orlgraph
