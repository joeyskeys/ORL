#include <catch2/catch_all.hpp>

#include "orl_parser.h"

using namespace orlcomp;

namespace {

void RequireParses(const std::string &source) {
    Parser parser(source);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());
    REQUIRE(parser.Ast() != nullptr);
}

void RequireRejects(const std::string &source) {
    Parser parser(source);
    REQUIRE_FALSE(parser.Parse());
    REQUIRE_FALSE(parser.Errors().empty());
}

} // namespace

TEST_CASE("syntax accepts scalar vector point normal and matrix declarations", "[orl][syntax][declaration]") {
    const std::string src =
        "int declarations() {\n"
        "    int count = 3;\n"
        "    float weight = 0.5;\n"
        "    string label = \"joint_0\";\n"
        "    vector tangent(1, 0, 0);\n"
        "    normal surface_normal(0, 1, 0);\n"
        "    point position(1, 2, 3);\n"
        "    matrix transform(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);\n"
        "    return count;\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts typed mesh and skeleton buffer parameters", "[orl][syntax][buffer]") {
    const std::string src =
        "int deform(point vertices[], matrix bones[], float weights[], int vertex_count) {\n"
        "    int i = 0;\n"
        "    while (i < vertex_count) {\n"
        "        vertices[i] = weights[i] * (bones[0] * vertices[i]);\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return vertex_count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());

    const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(parser.Ast()->items[0].get());
    REQUIRE(function != nullptr);
    REQUIRE(function->parameters.size() == 4);
    REQUIRE(function->parameters[0].type_name == "point");
    REQUIRE(function->parameters[0].is_buffer);
    REQUIRE(function->parameters[1].type_name == "matrix");
    REQUIRE(function->parameters[1].is_buffer);
    REQUIRE(function->parameters[2].type_name == "float");
    REQUIRE(function->parameters[2].is_buffer);
    REQUIRE_FALSE(function->parameters[3].is_buffer);
}

TEST_CASE("syntax accepts vector component access", "[orl][syntax][component]") {
    const std::string src =
        "float components() {\n"
        "    vector direction(1, 2, 3);\n"
        "    vec4 homogeneous(4, 5, 6, 1);\n"
        "    return direction.x + direction.y + direction.z + homogeneous.w;\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts quaternion construction and operations", "[orl][syntax][quaternion]") {
    const std::string src =
        "vector rotate_direction() {\n"
        "    quaternion bind_rotation(0, 0, 0, 1);\n"
        "    quaternion pose_rotation(0, 0.70710678, 0, 0.70710678);\n"
        "    quaternion combined = quat_normalize(quat_mul(bind_rotation, pose_rotation));\n"
        "    quaternion inverse = quat_conjugate(combined);\n"
        "    vector direction(1, 0, 0);\n"
        "    return quat_rotate(inverse, quat_rotate(combined, direction));\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts vector math intrinsic calls", "[orl][syntax][vector][intrinsic]") {
    const std::string src =
        "float vector_math() {\n"
        "    vector a(1, 0, 0);\n"
        "    vector b(0, 1, 0);\n"
        "    vector perpendicular = cross(a, b);\n"
        "    vector unit = normalize(perpendicular);\n"
        "    vector limited = clamp(unit, -0.5, 0.5);\n"
        "    vector halfway = lerp(a, b, 0.5);\n"
        "    return lerp(clamp(length(limited) + dot(a, b), 0.0, 1.0), halfway.x, 0.25);\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts matrix math intrinsic calls", "[orl][syntax][matrix][intrinsic]") {
    const std::string src =
        "vector transform_point() {\n"
        "    matrix identity = mat_identity();\n"
        "    matrix transform(1, 0, 0, 2,\n"
        "                     0, 1, 0, 3,\n"
        "                     0, 0, 1, 4,\n"
        "                     0, 0, 0, 1);\n"
        "    matrix combined = mat_mul(identity, transform);\n"
        "    matrix transposed = mat_transpose(combined);\n"
        "    matrix inverse = mat_inverse(transposed);\n"
        "    point value(1, 2, 3);\n"
        "    return mat_mul(inverse, value);\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts custom struct values fields arrays and buffers", "[orl][syntax][struct]") {
    const std::string src =
        "struct Vertex {\n"
        "    point position;\n"
        "    normal surface_normal;\n"
        "    float weight;\n"
        "}\n"
        "Vertex offset_vertex(Vertex source) {\n"
        "    Vertex copy = source;\n"
        "    copy.position = source.position + vector(1, 0, 0);\n"
        "    return copy;\n"
        "}\n"
        "float deform(Vertex vertices[]) {\n"
        "    Vertex local( point(0, 0, 0), normal(0, 1, 0), 0.5 );\n"
        "    vertices[0].weight = local.weight;\n"
        "    return vertices[0].position.x;\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax rejects duplicate and recursive struct definitions", "[orl][syntax][struct][error]") {
    RequireRejects(
        "struct Invalid {\n"
        "    float weight;\n"
        "    int weight;\n"
        "}\n");
    RequireRejects(
        "struct Recursive {\n"
        "    Recursive next;\n"
        "}\n");
}

TEST_CASE("syntax accepts function calls unary operators and precedence", "[orl][syntax][expression]") {
    const std::string src =
        "float magnitude_squared(vector value) {\n"
        "    return dot(value, value);\n"
        "}\n"
        "int expressions() {\n"
        "    int value = -(2 + 3 * 4) % 5;\n"
        "    if (not (value == 0) and (value < 0 or value > 10)) {\n"
        "        return 1;\n"
        "    }\n"
        "    return 0;\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts if else and all loop forms", "[orl][syntax][control-flow]") {
    const std::string src =
        "int control_flow(int limit) {\n"
        "    int i = 0;\n"
        "    int sum = 0;\n"
        "    for (i = 0; i < limit; i = i + 1) {\n"
        "        if (i % 2 == 0) {\n"
        "            continue;\n"
        "        } else {\n"
        "            sum = sum + i;\n"
        "        }\n"
        "    }\n"
        "    while (sum < 20) {\n"
        "        sum = sum + 1;\n"
        "        if (sum == 17) { break; }\n"
        "    }\n"
        "    do {\n"
        "        sum = sum + 1;\n"
        "    } while (sum < 18);\n"
        "    return sum;\n"
        "}\n";

    RequireParses(src);
}

TEST_CASE("syntax accepts canonical parallel for", "[orl][syntax][parallel]") {
    RequireParses(
        "int transform(int values[], int count) {\n"
        "    parallel for (int index = 0; index < count; index = index + 1) {\n"
        "        values[index] = values[index] + index;\n"
        "    }\n"
        "    return count;\n"
        "}\n");
}

TEST_CASE("syntax rejects noncanonical parallel for", "[orl][syntax][parallel][error]") {
    RequireRejects(
        "int transform(int values[], int count) {\n"
        "    parallel for (int index = 0; index <= count; index = index + 1) { }\n"
        "    return count;\n"
        "}\n");
}

TEST_CASE("syntax rejects incomplete declarations and control flow", "[orl][syntax][error]") {
    const std::string missing_initializer =
        "int invalid() {\n"
        "    float value = ;\n"
        "    return 0;\n"
        "}\n";
    const std::string missing_condition =
        "int invalid() {\n"
        "    if () { return 0; }\n"
        "}\n";

    Parser declaration_parser(missing_initializer);
    REQUIRE_FALSE(declaration_parser.Parse());
    REQUIRE_FALSE(declaration_parser.Errors().empty());

    Parser condition_parser(missing_condition);
    REQUIRE_FALSE(condition_parser.Parse());
    REQUIRE_FALSE(condition_parser.Errors().empty());
}
