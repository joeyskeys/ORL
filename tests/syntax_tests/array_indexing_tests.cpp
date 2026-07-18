#include <catch2/catch_all.hpp>

#include "orl_parser.h"

using namespace orlcomp;

TEST_CASE("syntax accepts fixed arrays with indexed reads and writes", "[orl][syntax][array]") {
    const std::string src =
        "int sum_values() {\n"
        "    int values[3];\n"
        "    values[0] = 4;\n"
        "    values[1] = 5;\n"
        "    values[2] = 6;\n"
        "    int i = 0;\n"
        "    int sum = 0;\n"
        "    while (i < 3) {\n"
        "        sum = sum + values[i];\n"
        "        i = i + 1;\n"
        "    }\n"
        "    return sum;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());
}

TEST_CASE("syntax accepts indexed vector arrays", "[orl][syntax][array]") {
    const std::string src =
        "float first_component_sum() {\n"
        "    vector vertices[2];\n"
        "    vertices[0] = vector(1, 2, 3);\n"
        "    vertices[1] = vector(4, 5, 6);\n"
        "    vector axis(1, 0, 0);\n"
        "    return dot(vertices[0] + vertices[1], axis);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());
}

TEST_CASE("syntax rejects non-literal array sizes", "[orl][syntax][array]") {
    const std::string src =
        "int invalid() {\n"
        "    int size = 3;\n"
        "    int values[size];\n"
        "    return 0;\n"
        "}\n";

    Parser parser(src);
    REQUIRE_FALSE(parser.Parse());
    REQUIRE_FALSE(parser.Errors().empty());
}

TEST_CASE("syntax rejects malformed array indexing", "[orl][syntax][array]") {
    const std::string src =
        "int invalid() {\n"
        "    int values[2];\n"
        "    values[0 = 1;\n"
        "    return 0;\n"
        "}\n";

    Parser parser(src);
    REQUIRE_FALSE(parser.Parse());
    REQUIRE_FALSE(parser.Errors().empty());
}
