#include "graph_serialization.hpp"

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <type_traits>
#include <utility>

namespace orlgraph
{

namespace
{

using Json = rapidjson::Value;
using Allocator = rapidjson::Document::AllocatorType;

Json string_value(std::string_view value, Allocator& allocator) {
    Json result;
    result.SetString(value.data(), static_cast<rapidjson::SizeType>(value.size()), allocator);
    return result;
}

void add(Json& object, const char* key, Json value, Allocator& allocator) {
    object.AddMember(string_value(key, allocator), std::move(value), allocator);
}

Json string_array(const std::vector<std::string>& values, Allocator& allocator) {
    Json result(rapidjson::kArrayType);
    for (const auto& value : values) {
        result.PushBack(string_value(value, allocator), allocator);
    }
    return result;
}

Json version_value(const Version& version, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "major", Json(version.major), allocator);
    add(result, "minor", Json(version.minor), allocator);
    add(result, "patch", Json(version.patch), allocator);
    return result;
}

Json type_value(const LogicalType& type, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "kind", Json(static_cast<std::uint32_t>(type.kind)), allocator);
    add(result, "name", string_value(type.name, allocator), allocator);
    add(result, "lanes", Json(type.lanes), allocator);
    add(result, "extent", Json(static_cast<std::uint64_t>(type.extent)), allocator);
    if (type.element != nullptr) {
        add(result, "element", type_value(*type.element, allocator), allocator);
    } else {
        add(result, "element", Json(rapidjson::kNullType), allocator);
    }
    return result;
}

Json domain_value(const Domain& domain, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "kind", Json(static_cast<std::uint32_t>(domain.kind)), allocator);
    add(result, "name", string_value(domain.name, allocator), allocator);
    return result;
}

Json shape_value(const Shape& shape, Allocator& allocator) {
    Json result(rapidjson::kArrayType);
    for (const auto& dimension : shape.dimensions) {
        Json item(rapidjson::kObjectType);
        add(item, "kind", Json(static_cast<std::uint32_t>(dimension.kind)), allocator);
        add(item, "constant", Json(dimension.constant), allocator);
        add(item, "symbol", string_value(dimension.symbol, allocator), allocator);
        result.PushBack(std::move(item), allocator);
    }
    return result;
}

Json value_value(const Value& value, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    std::visit([&](const auto& item) {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            add(result, "kind", string_value("null", allocator), allocator);
        } else if constexpr (std::is_same_v<T, bool>) {
            add(result, "kind", string_value("bool", allocator), allocator);
            add(result, "value", Json(item), allocator);
        } else if constexpr (std::is_same_v<T, std::int64_t>) {
            add(result, "kind", string_value("int64", allocator), allocator);
            add(result, "value", Json(item), allocator);
        } else if constexpr (std::is_same_v<T, double>) {
            add(result, "kind", string_value("float64", allocator), allocator);
            add(result, "value", Json(item), allocator);
        } else if constexpr (std::is_same_v<T, std::string>) {
            add(result, "kind", string_value("string", allocator), allocator);
            add(result, "value", string_value(item, allocator), allocator);
        } else if constexpr (std::is_same_v<T, std::vector<std::int64_t>>
            || std::is_same_v<T, std::vector<double>>)
        {
            add(result, "kind", string_value(
                std::is_same_v<T, std::vector<std::int64_t>> ? "int64_array" : "float64_array",
                allocator), allocator);
            Json values(rapidjson::kArrayType);
            for (const auto& value_item : item) {
                values.PushBack(Json(value_item), allocator);
            }
            add(result, "value", std::move(values), allocator);
        }
    }, value);
    return result;
}

Json constant_value(const ConstantValue& constant, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "type", type_value(constant.type, allocator), allocator);
    add(result, "value", value_value(constant.value, allocator), allocator);
    return result;
}

Json provenance_value(const Provenance& provenance, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    Json locations(rapidjson::kArrayType);
    for (const auto& location : provenance.locations) {
        Json item(rapidjson::kObjectType);
        add(item, "file", string_value(location.file, allocator), allocator);
        add(item, "line", Json(location.line), allocator);
        add(item, "column", Json(location.column), allocator);
        add(item, "end_line", Json(location.end_line), allocator);
        add(item, "end_column", Json(location.end_column), allocator);
        locations.PushBack(std::move(item), allocator);
    }
    add(result, "locations", std::move(locations), allocator);
    Json source_nodes(rapidjson::kArrayType);
    for (const auto& id : provenance.source_nodes) {
        source_nodes.PushBack(string_value(id.value, allocator), allocator);
    }
    add(result, "source_nodes", std::move(source_nodes), allocator);
    add(result, "description", string_value(provenance.description, allocator), allocator);
    return result;
}

Json port_value(const Port& port, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "id", string_value(port.id.value, allocator), allocator);
    add(result, "name", string_value(port.name, allocator), allocator);
    add(result, "direction", Json(static_cast<std::uint32_t>(port.direction)), allocator);
    add(result, "cardinality", Json(static_cast<std::uint32_t>(port.cardinality)), allocator);
    add(result, "type", type_value(port.type, allocator), allocator);
    add(result, "domain", domain_value(port.domain, allocator), allocator);
    add(result, "shape", shape_value(port.shape, allocator), allocator);
    add(result, "required", Json(port.required), allocator);
    if (port.default_value.has_value()) {
        add(result, "default", constant_value(*port.default_value, allocator), allocator);
    } else {
        add(result, "default", Json(rapidjson::kNullType), allocator);
    }
    add(result, "semantic", string_value(port.semantic, allocator), allocator);
    add(result, "coordinate_space", string_value(port.coordinate_space, allocator), allocator);
    return result;
}

Json parameter_value(const ParameterSpec& parameter, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "id", string_value(parameter.id.value, allocator), allocator);
    add(result, "name", string_value(parameter.name, allocator), allocator);
    add(result, "type", type_value(parameter.type, allocator), allocator);
    add(result, "compile_time", Json(parameter.compile_time), allocator);
    if (parameter.default_value.has_value()) {
        add(result, "default", constant_value(*parameter.default_value, allocator), allocator);
    } else {
        add(result, "default", Json(rapidjson::kNullType), allocator);
    }
    add(result, "semantic", string_value(parameter.semantic, allocator), allocator);
    return result;
}

Json effect_value(const ResourceEffect& effect, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "resource", string_value(effect.resource.value, allocator), allocator);
    add(result, "access", Json(static_cast<std::uint32_t>(effect.access)), allocator);
    add(result, "observable", Json(effect.observable), allocator);
    return result;
}

Json definition_value(const NodeDefinition& definition, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "id", string_value(definition.id.value, allocator), allocator);
    add(result, "qualified_name", string_value(definition.qualified_name, allocator), allocator);
    add(result, "version", version_value(definition.version, allocator), allocator);
    Json implementation(rapidjson::kObjectType);
    add(implementation, "kind",
        Json(static_cast<std::uint32_t>(definition.implementation.kind)), allocator);
    add(implementation, "module",
        string_value(definition.implementation.module, allocator), allocator);
    add(implementation, "function",
        string_value(definition.implementation.function, allocator), allocator);
    add(implementation, "subgraph",
        string_value(definition.implementation.subgraph.value, allocator), allocator);
    add(implementation, "runtime",
        string_value(definition.implementation.runtime_name, allocator), allocator);
    add(result, "implementation", std::move(implementation), allocator);

    Json inputs(rapidjson::kArrayType);
    for (const auto& port : definition.inputs) {
        inputs.PushBack(port_value(port, allocator), allocator);
    }
    add(result, "inputs", std::move(inputs), allocator);
    Json outputs(rapidjson::kArrayType);
    for (const auto& port : definition.outputs) {
        outputs.PushBack(port_value(port, allocator), allocator);
    }
    add(result, "outputs", std::move(outputs), allocator);
    Json parameters(rapidjson::kArrayType);
    for (const auto& parameter : definition.parameters) {
        parameters.PushBack(parameter_value(parameter, allocator), allocator);
    }
    add(result, "parameters", std::move(parameters), allocator);
    Json effects(rapidjson::kArrayType);
    for (const auto& effect : definition.effects) {
        effects.PushBack(effect_value(effect, allocator), allocator);
    }
    add(result, "effects", std::move(effects), allocator);
    add(result, "capabilities", string_array(definition.capabilities, allocator), allocator);
    add(result, "inline_policy",
        Json(static_cast<std::uint32_t>(definition.inline_policy)), allocator);
    add(result, "operation", string_value(definition.operation, allocator), allocator);
    add(result, "pure", Json(definition.pure), allocator);
    add(result, "stateful", Json(definition.stateful), allocator);
    add(result, "provenance", provenance_value(definition.provenance, allocator), allocator);
    return result;
}

Json endpoint_value(const Endpoint& endpoint, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "kind", Json(static_cast<std::uint32_t>(endpoint.kind)), allocator);
    add(result, "owner", string_value(endpoint.owner.value, allocator), allocator);
    add(result, "port", string_value(endpoint.port.value, allocator), allocator);
    return result;
}

Json mapping_value(const ParameterMapping& mapping, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "parameter", string_value(mapping.parameter, allocator), allocator);
    add(result, "source_kind",
        Json(static_cast<std::uint32_t>(mapping.source_kind)), allocator);
    add(result, "source", endpoint_value(mapping.source, allocator), allocator);
    if (mapping.constant.has_value()) {
        add(result, "constant", constant_value(*mapping.constant, allocator), allocator);
    } else {
        add(result, "constant", Json(rapidjson::kNullType), allocator);
    }
    add(result, "resource_attribute",
        string_value(mapping.resource_attribute, allocator), allocator);
    add(result, "conversion", string_value(mapping.conversion, allocator), allocator);
    add(result, "compile_time", Json(mapping.compile_time), allocator);
    add(result, "provenance", provenance_value(mapping.provenance, allocator), allocator);
    return result;
}

Json node_value(const NodeInstance& node, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "id", string_value(node.id.value, allocator), allocator);
    add(result, "definition", string_value(node.definition.value, allocator), allocator);
    add(result, "name", string_value(node.name, allocator), allocator);
    Json parameters(rapidjson::kObjectType);
    for (const auto& [name, value] : node.parameter_values) {
        add(parameters, name.c_str(), constant_value(value, allocator), allocator);
    }
    add(result, "parameters", std::move(parameters), allocator);
    Json mappings(rapidjson::kArrayType);
    for (const auto& mapping : node.parameter_mappings) {
        mappings.PushBack(mapping_value(mapping, allocator), allocator);
    }
    add(result, "mappings", std::move(mappings), allocator);
    add(result, "provenance", provenance_value(node.provenance, allocator), allocator);
    add(result, "inline_policy",
        Json(static_cast<std::uint32_t>(node.inline_policy)), allocator);
    return result;
}

Json interface_value(const InterfacePort& port, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "id", string_value(port.id.value, allocator), allocator);
    add(result, "name", string_value(port.name, allocator), allocator);
    add(result, "direction", Json(static_cast<std::uint32_t>(port.direction)), allocator);
    add(result, "type", type_value(port.type, allocator), allocator);
    add(result, "domain", domain_value(port.domain, allocator), allocator);
    add(result, "shape", shape_value(port.shape, allocator), allocator);
    add(result, "required", Json(port.required), allocator);
    if (port.default_value.has_value()) {
        add(result, "default", constant_value(*port.default_value, allocator), allocator);
    } else {
        add(result, "default", Json(rapidjson::kNullType), allocator);
    }
    add(result, "compile_time", Json(port.compile_time), allocator);
    add(result, "binding", string_value(port.binding, allocator), allocator);
    add(result, "semantic", string_value(port.semantic, allocator), allocator);
    add(result, "coordinate_space", string_value(port.coordinate_space, allocator), allocator);
    return result;
}

Json resource_value(const Resource& resource, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "id", string_value(resource.id.value, allocator), allocator);
    add(result, "name", string_value(resource.name, allocator), allocator);
    add(result, "type", type_value(resource.type, allocator), allocator);
    add(result, "domain", domain_value(resource.domain, allocator), allocator);
    add(result, "shape", shape_value(resource.shape, allocator), allocator);
    add(result, "access", Json(static_cast<std::uint32_t>(resource.access)), allocator);
    add(result, "observable", Json(resource.observable), allocator);
    add(result, "binding", string_value(resource.binding, allocator), allocator);
    return result;
}

Json graph_value(const GraphModule& module, Allocator& allocator) {
    Json result(rapidjson::kObjectType);
    add(result, "module_id", string_value(module.module_id, allocator), allocator);
    add(result, "version", version_value(module.version, allocator), allocator);
    add(result, "language_version", string_value(module.language_version, allocator), allocator);
    add(result, "logical_abi_version",
        string_value(module.logical_abi_version, allocator), allocator);

    Json inputs(rapidjson::kArrayType);
    for (const auto& [_, input] : module.inputs()) {
        inputs.PushBack(interface_value(input, allocator), allocator);
    }
    add(result, "inputs", std::move(inputs), allocator);
    Json outputs(rapidjson::kArrayType);
    for (const auto& [_, output] : module.outputs()) {
        outputs.PushBack(interface_value(output, allocator), allocator);
    }
    add(result, "outputs", std::move(outputs), allocator);
    Json resources(rapidjson::kArrayType);
    for (const auto& [_, resource] : module.resources()) {
        resources.PushBack(resource_value(resource, allocator), allocator);
    }
    add(result, "resources", std::move(resources), allocator);
    Json nodes(rapidjson::kArrayType);
    for (const auto& [_, node] : module.nodes()) {
        nodes.PushBack(node_value(node, allocator), allocator);
    }
    add(result, "nodes", std::move(nodes), allocator);
    Json connections(rapidjson::kArrayType);
    for (const auto& connection : module.connections()) {
        Json item(rapidjson::kObjectType);
        add(item, "source", endpoint_value(connection.source, allocator), allocator);
        add(item, "destination", endpoint_value(connection.destination, allocator), allocator);
        add(item, "conversion", string_value(connection.conversion, allocator), allocator);
        add(item, "shape", shape_value(connection.shape, allocator), allocator);
        add(item, "provenance", provenance_value(connection.provenance, allocator), allocator);
        add(item, "feedback", Json(connection.feedback), allocator);
        connections.PushBack(std::move(item), allocator);
    }
    add(result, "connections", std::move(connections), allocator);
    return result;
}

std::string write_json(const Json& value) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    value.Accept(writer);
    return buffer.GetString();
}

std::string hash_text(std::string_view text) {
    std::uint64_t hash = 14695981039346656037ull;
    for (const char value : text) {
        hash ^= static_cast<unsigned char>(value);
        hash *= 1099511628211ull;
    }
    std::ostringstream stream;
    stream << std::hex << std::setw(16) << std::setfill('0') << hash;
    return stream.str();
}

const Json* member(const Json& object, const char* key) {
    if (!object.IsObject()) {
        return nullptr;
    }
    const auto found = object.FindMember(key);
    return found == object.MemberEnd() ? nullptr : &found->value;
}

bool require_member(const Json& object, const char* key, const Json** value,
    std::vector<Diagnostic>* diagnostics)
{
    *value = member(object, key);
    if (*value != nullptr) {
        return true;
    }
    diagnostics->push_back({
        DiagnosticSeverity::Error,
        "ORLGRAPH_MISSING_FIELD",
        std::string{"Missing field: "} + key,
        {}, {}, {},
    });
    return false;
}

bool read_string(const Json& object, const char* key, std::string* output,
    std::vector<Diagnostic>* diagnostics, bool required = true)
{
    const Json* value = member(object, key);
    if (value == nullptr) {
        if (required) {
            require_member(object, key, &value, diagnostics);
            return false;
        }
        output->clear();
        return true;
    }
    if (!value->IsString()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error,
            "ORLGRAPH_INVALID_STRING",
            std::string{"Field is not a string: "} + key,
            {}, {}, {},
        });
        return false;
    }
    *output = value->GetString();
    return true;
}

bool read_bool(const Json& object, const char* key, bool* output,
    std::vector<Diagnostic>* diagnostics, bool default_value = false)
{
    const Json* value = member(object, key);
    if (value == nullptr) {
        *output = default_value;
        return true;
    }
    if (!value->IsBool()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error,
            "ORLGRAPH_INVALID_BOOL",
            std::string{"Field is not a bool: "} + key,
            {}, {}, {},
        });
        return false;
    }
    *output = value->GetBool();
    return true;
}

bool read_u32(const Json& object, const char* key, std::uint32_t* output,
    std::vector<Diagnostic>* diagnostics, std::uint32_t default_value = 0)
{
    const Json* value = member(object, key);
    if (value == nullptr) {
        *output = default_value;
        return true;
    }
    if (!value->IsUint()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error,
            "ORLGRAPH_INVALID_INTEGER",
            std::string{"Field is not an unsigned integer: "} + key,
            {}, {}, {},
        });
        return false;
    }
    *output = value->GetUint();
    return true;
}

bool read_string_array(const Json& object, const char* key,
    std::vector<std::string>* output, std::vector<Diagnostic>* diagnostics)
{
    const Json* value = nullptr;
    if (!require_member(object, key, &value, diagnostics) || !value->IsArray()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error,
            "ORLGRAPH_INVALID_ARRAY",
            std::string{"Field is not an array: "} + key,
            {}, {}, {},
        });
        return false;
    }
    for (const auto& item : value->GetArray()) {
        if (!item.IsString()) {
            diagnostics->push_back({
                DiagnosticSeverity::Error,
                "ORLGRAPH_INVALID_STRING",
                std::string{"Array contains a non-string value: "} + key,
                {}, {}, {},
            });
            return false;
        }
        output->emplace_back(item.GetString());
    }
    return true;
}

bool parse_version(const Json& value, Version* version,
    std::vector<Diagnostic>* diagnostics)
{
    if (!value.IsObject()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_VERSION",
            "Version is not an object", {}, {}, {},
        });
        return false;
    }
    return read_u32(value, "major", &version->major, diagnostics)
        && read_u32(value, "minor", &version->minor, diagnostics)
        && read_u32(value, "patch", &version->patch, diagnostics);
}

bool parse_type(const Json& value, LogicalType* type,
    std::vector<Diagnostic>* diagnostics)
{
    if (!value.IsObject()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_TYPE",
            "Logical type is not an object", {}, {}, {},
        });
        return false;
    }
    std::uint32_t kind = 0;
    if (!read_u32(value, "kind", &kind, diagnostics)
        || kind > static_cast<std::uint32_t>(LogicalTypeKind::Unknown))
    {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_TYPE",
            "Logical type kind is invalid", {}, {}, {},
        });
        return false;
    }
    type->kind = static_cast<LogicalTypeKind>(kind);
    read_string(value, "name", &type->name, diagnostics, false);
    read_u32(value, "lanes", &type->lanes, diagnostics);
    const Json* extent = member(value, "extent");
    if (extent != nullptr && extent->IsUint64()) {
        type->extent = static_cast<std::size_t>(extent->GetUint64());
    }
    const Json* element = member(value, "element");
    if (element != nullptr && !element->IsNull()) {
        type->element = std::make_shared<LogicalType>();
        return parse_type(*element, type->element.get(), diagnostics);
    }
    return true;
}

bool parse_domain(const Json& value, Domain* domain,
    std::vector<Diagnostic>* diagnostics)
{
    if (!value.IsObject()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_DOMAIN",
            "Domain is not an object", {}, {}, {},
        });
        return false;
    }
    std::uint32_t kind = 0;
    if (!read_u32(value, "kind", &kind, diagnostics)
        || kind > static_cast<std::uint32_t>(DomainKind::Custom))
    {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_DOMAIN",
            "Domain kind is invalid", {}, {}, {},
        });
        return false;
    }
    domain->kind = static_cast<DomainKind>(kind);
    return read_string(value, "name", &domain->name, diagnostics, false);
}

bool parse_shape(const Json& value, Shape* shape,
    std::vector<Diagnostic>* diagnostics)
{
    if (!value.IsArray()) {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_SHAPE",
            "Shape is not an array", {}, {}, {},
        });
        return false;
    }
    shape->dimensions.clear();
    for (const auto& item : value.GetArray()) {
        if (!item.IsObject()) {
            diagnostics->push_back({
                DiagnosticSeverity::Error, "ORLGRAPH_INVALID_SHAPE",
                "Shape dimension is not an object", {}, {}, {},
            });
            return false;
        }
        std::uint32_t kind = 0;
        if (!read_u32(item, "kind", &kind, diagnostics)
            || kind > static_cast<std::uint32_t>(ShapeDimensionKind::Symbol))
        {
            return false;
        }
        ShapeDimension dimension;
        dimension.kind = static_cast<ShapeDimensionKind>(kind);
        const Json* constant = member(item, "constant");
        if (constant != nullptr && constant->IsUint64()) {
            dimension.constant = constant->GetUint64();
        }
        read_string(item, "symbol", &dimension.symbol, diagnostics, false);
        shape->dimensions.push_back(std::move(dimension));
    }
    return true;
}

bool parse_value(const Json& value, Value* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string kind;
    if (!read_string(value, "kind", &kind, diagnostics)) {
        return false;
    }
    const Json* item = member(value, "value");
    if (kind == "null") {
        *output = std::monostate{};
    } else if (kind == "bool" && item != nullptr && item->IsBool()) {
        *output = item->GetBool();
    } else if (kind == "int64" && item != nullptr && item->IsInt64()) {
        *output = item->GetInt64();
    } else if (kind == "float64" && item != nullptr && item->IsNumber()) {
        *output = item->GetDouble();
    } else if (kind == "string" && item != nullptr && item->IsString()) {
        *output = std::string{item->GetString()};
    } else if ((kind == "int64_array" || kind == "float64_array")
        && item != nullptr && item->IsArray())
    {
        if (kind == "int64_array") {
            std::vector<std::int64_t> values;
            for (const auto& element : item->GetArray()) {
                if (!element.IsInt64()) {
                    return false;
                }
                values.push_back(element.GetInt64());
            }
            *output = std::move(values);
        } else {
            std::vector<double> values;
            for (const auto& element : item->GetArray()) {
                if (!element.IsNumber()) {
                    return false;
                }
                values.push_back(element.GetDouble());
            }
            *output = std::move(values);
        }
    } else {
        diagnostics->push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_VALUE",
            "Constant value encoding is invalid", {}, {}, {},
        });
        return false;
    }
    return true;
}

bool parse_constant(const Json& value, ConstantValue* output,
    std::vector<Diagnostic>* diagnostics)
{
    const Json* type = member(value, "type");
    const Json* item = member(value, "value");
    if (type == nullptr || item == nullptr
        || !parse_type(*type, &output->type, diagnostics)
        || !parse_value(*item, &output->value, diagnostics))
    {
        return false;
    }
    return true;
}

bool parse_provenance(const Json& value, Provenance* output,
    std::vector<Diagnostic>* diagnostics)
{
    if (!value.IsObject()) {
        return false;
    }
    const Json* locations = member(value, "locations");
    if (locations != nullptr && locations->IsArray()) {
        for (const auto& item : locations->GetArray()) {
            SourceLocation location;
            read_string(item, "file", &location.file, diagnostics, false);
            read_u32(item, "line", &location.line, diagnostics);
            read_u32(item, "column", &location.column, diagnostics);
            read_u32(item, "end_line", &location.end_line, diagnostics);
            read_u32(item, "end_column", &location.end_column, diagnostics);
            output->locations.push_back(std::move(location));
        }
    }
    const Json* source_nodes = member(value, "source_nodes");
    if (source_nodes != nullptr && source_nodes->IsArray()) {
        for (const auto& item : source_nodes->GetArray()) {
            if (item.IsString()) {
                output->source_nodes.emplace_back(item.GetString());
            }
        }
    }
    read_string(value, "description", &output->description, diagnostics, false);
    return true;
}

bool parse_port(const Json& value, Port* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string id;
    if (!read_string(value, "id", &id, diagnostics)
        || !read_string(value, "name", &output->name, diagnostics))
    {
        return false;
    }
    output->id = StableId{std::move(id)};
    std::uint32_t direction = 0;
    std::uint32_t cardinality = 0;
    read_u32(value, "direction", &direction, diagnostics);
    read_u32(value, "cardinality", &cardinality, diagnostics);
    output->direction = static_cast<PortDirection>(direction);
    output->cardinality = static_cast<PortCardinality>(cardinality);
    const Json* type = member(value, "type");
    const Json* domain = member(value, "domain");
    const Json* shape = member(value, "shape");
    if (type == nullptr || domain == nullptr || shape == nullptr
        || !parse_type(*type, &output->type, diagnostics)
        || !parse_domain(*domain, &output->domain, diagnostics)
        || !parse_shape(*shape, &output->shape, diagnostics))
    {
        return false;
    }
    read_bool(value, "required", &output->required, diagnostics, true);
    const Json* default_value = member(value, "default");
    if (default_value != nullptr && !default_value->IsNull()) {
        output->default_value = ConstantValue{};
        parse_constant(*default_value, &*output->default_value, diagnostics);
    }
    read_string(value, "semantic", &output->semantic, diagnostics, false);
    read_string(value, "coordinate_space", &output->coordinate_space, diagnostics, false);
    return true;
}

bool parse_parameter(const Json& value, ParameterSpec* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string id;
    if (!read_string(value, "id", &id, diagnostics)
        || !read_string(value, "name", &output->name, diagnostics))
    {
        return false;
    }
    output->id = StableId{std::move(id)};
    const Json* type = member(value, "type");
    if (type == nullptr || !parse_type(*type, &output->type, diagnostics)) {
        return false;
    }
    read_bool(value, "compile_time", &output->compile_time, diagnostics);
    const Json* default_value = member(value, "default");
    if (default_value != nullptr && !default_value->IsNull()) {
        output->default_value = ConstantValue{};
        parse_constant(*default_value, &*output->default_value, diagnostics);
    }
    read_string(value, "semantic", &output->semantic, diagnostics, false);
    return true;
}

bool parse_effect(const Json& value, ResourceEffect* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string resource;
    if (!read_string(value, "resource", &resource, diagnostics)) {
        return false;
    }
    output->resource = StableId{std::move(resource)};
    std::uint32_t access = 0;
    read_u32(value, "access", &access, diagnostics);
    output->access = static_cast<AccessMode>(access);
    read_bool(value, "observable", &output->observable, diagnostics);
    return true;
}

bool parse_definition(const Json& value, NodeDefinition* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string id;
    if (!read_string(value, "id", &id, diagnostics)
        || !read_string(value, "qualified_name", &output->qualified_name, diagnostics))
    {
        return false;
    }
    output->id = StableId{std::move(id)};
    const Json* version = member(value, "version");
    if (version == nullptr || !parse_version(*version, &output->version, diagnostics)) {
        return false;
    }
    const Json* implementation = member(value, "implementation");
    if (implementation != nullptr && implementation->IsObject()) {
        std::uint32_t kind = 0;
        read_u32(*implementation, "kind", &kind, diagnostics);
        output->implementation.kind = static_cast<ImplementationKind>(kind);
        read_string(*implementation, "module", &output->implementation.module, diagnostics, false);
        read_string(*implementation, "function", &output->implementation.function, diagnostics, false);
        std::string subgraph;
        read_string(*implementation, "subgraph", &subgraph, diagnostics, false);
        output->implementation.subgraph = StableId{std::move(subgraph)};
        read_string(*implementation, "runtime",
            &output->implementation.runtime_name, diagnostics, false);
    }
    const Json* inputs = member(value, "inputs");
    if (inputs != nullptr && inputs->IsArray()) {
        for (const auto& item : inputs->GetArray()) {
            Port port;
            if (parse_port(item, &port, diagnostics)) {
                output->inputs.push_back(std::move(port));
            }
        }
    }
    const Json* outputs = member(value, "outputs");
    if (outputs != nullptr && outputs->IsArray()) {
        for (const auto& item : outputs->GetArray()) {
            Port port;
            if (parse_port(item, &port, diagnostics)) {
                output->outputs.push_back(std::move(port));
            }
        }
    }
    const Json* parameters = member(value, "parameters");
    if (parameters != nullptr && parameters->IsArray()) {
        for (const auto& item : parameters->GetArray()) {
            ParameterSpec parameter;
            if (parse_parameter(item, &parameter, diagnostics)) {
                output->parameters.push_back(std::move(parameter));
            }
        }
    }
    const Json* effects = member(value, "effects");
    if (effects != nullptr && effects->IsArray()) {
        for (const auto& item : effects->GetArray()) {
            ResourceEffect effect;
            if (parse_effect(item, &effect, diagnostics)) {
                output->effects.push_back(std::move(effect));
            }
        }
    }
    read_string_array(value, "capabilities", &output->capabilities, diagnostics);
    std::uint32_t inline_policy = 0;
    read_u32(value, "inline_policy", &inline_policy, diagnostics);
    output->inline_policy = static_cast<InlinePolicy>(inline_policy);
    read_string(value, "operation", &output->operation, diagnostics, false);
    read_bool(value, "pure", &output->pure, diagnostics, true);
    read_bool(value, "stateful", &output->stateful, diagnostics, false);
    const Json* provenance = member(value, "provenance");
    if (provenance != nullptr) {
        parse_provenance(*provenance, &output->provenance, diagnostics);
    }
    return true;
}

bool parse_endpoint(const Json& value, Endpoint* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::uint32_t kind = 0;
    std::string owner;
    std::string port;
    if (!read_u32(value, "kind", &kind, diagnostics)
        || !read_string(value, "owner", &owner, diagnostics)
        || !read_string(value, "port", &port, diagnostics))
    {
        return false;
    }
    output->kind = static_cast<EndpointKind>(kind);
    output->owner = StableId{std::move(owner)};
    output->port = StableId{std::move(port)};
    return true;
}

bool parse_mapping(const Json& value, ParameterMapping* output,
    std::vector<Diagnostic>* diagnostics)
{
    if (!read_string(value, "parameter", &output->parameter, diagnostics)) {
        return false;
    }
    std::uint32_t source_kind = 0;
    read_u32(value, "source_kind", &source_kind, diagnostics);
    output->source_kind = static_cast<ParameterSourceKind>(source_kind);
    const Json* source = member(value, "source");
    if (source == nullptr || !parse_endpoint(*source, &output->source, diagnostics)) {
        return false;
    }
    const Json* constant = member(value, "constant");
    if (constant != nullptr && !constant->IsNull()) {
        output->constant = ConstantValue{};
        parse_constant(*constant, &*output->constant, diagnostics);
    }
    read_string(value, "resource_attribute",
        &output->resource_attribute, diagnostics, false);
    read_string(value, "conversion", &output->conversion, diagnostics, false);
    read_bool(value, "compile_time", &output->compile_time, diagnostics);
    const Json* provenance = member(value, "provenance");
    if (provenance != nullptr) {
        parse_provenance(*provenance, &output->provenance, diagnostics);
    }
    return true;
}

bool parse_node(const Json& value, NodeInstance* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string id;
    std::string definition;
    if (!read_string(value, "id", &id, diagnostics)
        || !read_string(value, "definition", &definition, diagnostics)
        || !read_string(value, "name", &output->name, diagnostics))
    {
        return false;
    }
    output->id = StableId{std::move(id)};
    output->definition = StableId{std::move(definition)};
    const Json* parameters = member(value, "parameters");
    if (parameters != nullptr && parameters->IsObject()) {
        for (auto item = parameters->MemberBegin(); item != parameters->MemberEnd(); ++item) {
            ConstantValue constant;
            if (parse_constant(item->value, &constant, diagnostics)) {
                output->parameter_values.emplace(item->name.GetString(), std::move(constant));
            }
        }
    }
    const Json* mappings = member(value, "mappings");
    if (mappings != nullptr && mappings->IsArray()) {
        for (const auto& item : mappings->GetArray()) {
            ParameterMapping mapping;
            if (parse_mapping(item, &mapping, diagnostics)) {
                output->parameter_mappings.push_back(std::move(mapping));
            }
        }
    }
    const Json* provenance = member(value, "provenance");
    if (provenance != nullptr) {
        parse_provenance(*provenance, &output->provenance, diagnostics);
    }
    std::uint32_t inline_policy = 0;
    read_u32(value, "inline_policy", &inline_policy, diagnostics);
    output->inline_policy = static_cast<InlinePolicy>(inline_policy);
    return true;
}

bool parse_interface(const Json& value, InterfacePort* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string id;
    if (!read_string(value, "id", &id, diagnostics)
        || !read_string(value, "name", &output->name, diagnostics))
    {
        return false;
    }
    output->id = StableId{std::move(id)};
    std::uint32_t direction = 0;
    read_u32(value, "direction", &direction, diagnostics);
    output->direction = static_cast<PortDirection>(direction);
    const Json* type = member(value, "type");
    const Json* domain = member(value, "domain");
    const Json* shape = member(value, "shape");
    if (type == nullptr || domain == nullptr || shape == nullptr
        || !parse_type(*type, &output->type, diagnostics)
        || !parse_domain(*domain, &output->domain, diagnostics)
        || !parse_shape(*shape, &output->shape, diagnostics))
    {
        return false;
    }
    read_bool(value, "required", &output->required, diagnostics, true);
    const Json* default_value = member(value, "default");
    if (default_value != nullptr && !default_value->IsNull()) {
        output->default_value = ConstantValue{};
        parse_constant(*default_value, &*output->default_value, diagnostics);
    }
    read_bool(value, "compile_time", &output->compile_time, diagnostics);
    read_string(value, "binding", &output->binding, diagnostics, false);
    read_string(value, "semantic", &output->semantic, diagnostics, false);
    read_string(value, "coordinate_space",
        &output->coordinate_space, diagnostics, false);
    return true;
}

bool parse_resource(const Json& value, Resource* output,
    std::vector<Diagnostic>* diagnostics)
{
    std::string id;
    if (!read_string(value, "id", &id, diagnostics)
        || !read_string(value, "name", &output->name, diagnostics))
    {
        return false;
    }
    output->id = StableId{std::move(id)};
    const Json* type = member(value, "type");
    const Json* domain = member(value, "domain");
    const Json* shape = member(value, "shape");
    if (type == nullptr || domain == nullptr || shape == nullptr
        || !parse_type(*type, &output->type, diagnostics)
        || !parse_domain(*domain, &output->domain, diagnostics)
        || !parse_shape(*shape, &output->shape, diagnostics))
    {
        return false;
    }
    std::uint32_t access = 0;
    read_u32(value, "access", &access, diagnostics);
    output->access = static_cast<AccessMode>(access);
    read_bool(value, "observable", &output->observable, diagnostics);
    read_string(value, "binding", &output->binding, diagnostics, false);
    return true;
}

} // namespace

std::string oro_content_hash(std::string_view canonical_text) {
    return hash_text(canonical_text);
}

OroSerializationResult serialize_oro(const GraphModule& module,
    const NodeRegistry& registry)
{
    OroSerializationResult result;
    rapidjson::Document document;
    document.SetObject();
    auto& allocator = document.GetAllocator();

    Json header(rapidjson::kObjectType);
    add(header, "magic", string_value(kOroMagic, allocator), allocator);
    add(header, "format_version", Json(kOroFormatVersion), allocator);
    add(header, "language_version", string_value(module.language_version, allocator), allocator);
    add(header, "logical_abi_version",
        string_value(module.logical_abi_version, allocator), allocator);
    add(header, "module_id", string_value(module.module_id, allocator), allocator);
    add(header, "content_hash", string_value("", allocator), allocator);
    add(document, "header", std::move(header), allocator);

    Json definitions(rapidjson::kArrayType);
    for (const auto& [_, definition] : registry.definitions()) {
        definitions.PushBack(definition_value(definition, allocator), allocator);
    }
    add(document, "definitions", std::move(definitions), allocator);
    add(document, "graph", graph_value(module, allocator), allocator);

    const std::string base = write_json(document);
    result.content_hash = hash_text(base);
    auto& header_value = document["header"];
    header_value["content_hash"].SetString(
        result.content_hash.data(),
        static_cast<rapidjson::SizeType>(result.content_hash.size()),
        allocator);
    result.text = write_json(document);
    result.ok = true;
    return result;
}

OroDocument deserialize_oro(std::string_view text) {
    OroDocument result;
    rapidjson::Document document;
    document.Parse(text.data(), text.size());
    if (document.HasParseError() || !document.IsObject()) {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_JSON",
            "The .oro payload is not valid JSON", {}, {}, {},
        });
        return result;
    }
    const Json* header = member(document, "header");
    const Json* definitions = member(document, "definitions");
    const Json* graph = member(document, "graph");
    if (header == nullptr || definitions == nullptr || graph == nullptr
        || !header->IsObject() || !definitions->IsArray() || !graph->IsObject())
    {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_INVALID_DOCUMENT",
            "The .oro document is missing required sections", {}, {}, {},
        });
        return result;
    }
    std::string magic;
    if (!read_string(*header, "magic", &magic, &result.diagnostics)
        || magic != kOroMagic)
    {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_BAD_MAGIC",
            "The .oro magic value is invalid", {}, {}, {},
        });
        return result;
    }
    std::uint32_t format_version = 0;
    read_u32(*header, "format_version", &format_version, &result.diagnostics);
    if (format_version != kOroFormatVersion) {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_UNSUPPORTED_FORMAT",
            "Unsupported .oro format version", {}, {}, {},
        });
        return result;
    }
    std::string header_language;
    std::string header_abi;
    read_string(*header, "language_version", &header_language,
        &result.diagnostics, false);
    read_string(*header, "logical_abi_version", &header_abi,
        &result.diagnostics, false);
    if (!header_language.empty() && header_language != kOroLanguageVersion) {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_LANGUAGE_MISMATCH",
            "The .oro language version is not supported: " + header_language,
            {}, {}, {},
        });
    }
    if (!header_abi.empty() && header_abi != kOroLogicalAbiVersion) {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_ABI_MISMATCH",
            "The .oro logical ABI version is not supported: " + header_abi,
            {}, {}, {},
        });
    }
    read_string(*header, "content_hash", &result.content_hash, &result.diagnostics, false);

    for (const auto& item : definitions->GetArray()) {
        NodeDefinition definition;
        if (parse_definition(item, &definition, &result.diagnostics)) {
            std::string error;
            if (!result.registry.register_definition(std::move(definition), &error)) {
                result.diagnostics.push_back({
                    DiagnosticSeverity::Error, "ORLGRAPH_DUPLICATE_DEFINITION",
                    std::move(error), {}, {}, {},
                });
            }
        }
    }

    read_string(*graph, "module_id", &result.module.module_id, &result.diagnostics);
    const Json* version = member(*graph, "version");
    if (version != nullptr) {
        parse_version(*version, &result.module.version, &result.diagnostics);
    }
    read_string(*graph, "language_version",
        &result.module.language_version, &result.diagnostics, false);
    read_string(*graph, "logical_abi_version",
        &result.module.logical_abi_version, &result.diagnostics, false);
    if (result.module.language_version != kOroLanguageVersion) {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_LANGUAGE_MISMATCH",
            "Graph language version is not supported: "
                + result.module.language_version, {}, {}, {},
        });
    }
    if (result.module.logical_abi_version != kOroLogicalAbiVersion) {
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_ABI_MISMATCH",
            "Graph logical ABI version is not supported: "
                + result.module.logical_abi_version, {}, {}, {},
        });
    }

    const Json* inputs = member(*graph, "inputs");
    if (inputs != nullptr && inputs->IsArray()) {
        for (const auto& item : inputs->GetArray()) {
            InterfacePort port;
            if (parse_interface(item, &port, &result.diagnostics)) {
                result.module.add_input(std::move(port));
            }
        }
    }
    const Json* outputs = member(*graph, "outputs");
    if (outputs != nullptr && outputs->IsArray()) {
        for (const auto& item : outputs->GetArray()) {
            InterfacePort port;
            if (parse_interface(item, &port, &result.diagnostics)) {
                result.module.add_output(std::move(port));
            }
        }
    }
    const Json* resources = member(*graph, "resources");
    if (resources != nullptr && resources->IsArray()) {
        for (const auto& item : resources->GetArray()) {
            Resource resource;
            if (parse_resource(item, &resource, &result.diagnostics)) {
                result.module.add_resource(std::move(resource));
            }
        }
    }
    const Json* nodes = member(*graph, "nodes");
    if (nodes != nullptr && nodes->IsArray()) {
        for (const auto& item : nodes->GetArray()) {
            NodeInstance node;
            if (parse_node(item, &node, &result.diagnostics)) {
                result.module.add_node(std::move(node));
            }
        }
    }
    const Json* connections = member(*graph, "connections");
    if (connections != nullptr && connections->IsArray()) {
        for (const auto& item : connections->GetArray()) {
            Connection connection;
            const Json* source = member(item, "source");
            const Json* destination = member(item, "destination");
            if (source == nullptr || destination == nullptr
                || !parse_endpoint(*source, &connection.source, &result.diagnostics)
                || !parse_endpoint(*destination, &connection.destination,
                    &result.diagnostics))
            {
                continue;
            }
            read_string(item, "conversion", &connection.conversion,
                &result.diagnostics, false);
            const Json* shape = member(item, "shape");
            if (shape != nullptr) {
                parse_shape(*shape, &connection.shape, &result.diagnostics);
            }
            read_bool(item, "feedback", &connection.feedback, &result.diagnostics);
            const Json* provenance = member(item, "provenance");
            if (provenance != nullptr) {
                parse_provenance(*provenance, &connection.provenance,
                    &result.diagnostics);
            }
            result.module.add_connection(std::move(connection));
        }
    }

    if (result.diagnostics.empty()) {
        const auto canonical = serialize_oro(result.module, result.registry);
        if (canonical.ok && !result.content_hash.empty()
            && canonical.content_hash != result.content_hash)
        {
            result.diagnostics.push_back({
                DiagnosticSeverity::Error, "ORLGRAPH_HASH_MISMATCH",
                "The .oro content hash does not match its canonical payload",
                {}, {}, {},
            });
        }
        const auto validation = validate(result.module, result.registry);
        result.diagnostics.insert(result.diagnostics.end(),
            validation.diagnostics.begin(), validation.diagnostics.end());
    }
    result.ok = std::none_of(result.diagnostics.begin(), result.diagnostics.end(),
        [](const Diagnostic& diagnostic) {
            return diagnostic.severity == DiagnosticSeverity::Error;
        });
    return result;
}

bool save_oro(const std::string& path, const GraphModule& module,
    const NodeRegistry& registry, std::vector<Diagnostic>* diagnostics)
{
    const OroSerializationResult serialized = serialize_oro(module, registry);
    if (diagnostics != nullptr) {
        *diagnostics = serialized.diagnostics;
    }
    if (!serialized.ok) {
        return false;
    }
    std::ofstream output(path, std::ios::binary);
    if (!output.is_open()) {
        if (diagnostics != nullptr) {
            diagnostics->push_back({
                DiagnosticSeverity::Error, "ORLGRAPH_WRITE_FAILED",
                "Unable to open .oro output file: " + path, {}, {}, {},
            });
        }
        return false;
    }
    output << serialized.text;
    return static_cast<bool>(output);
}

OroDocument load_oro(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        OroDocument result;
        result.diagnostics.push_back({
            DiagnosticSeverity::Error, "ORLGRAPH_READ_FAILED",
            "Unable to open .oro input file: " + path, {}, {}, {},
        });
        return result;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    return deserialize_oro(contents.str());
}

} // namespace orlgraph
