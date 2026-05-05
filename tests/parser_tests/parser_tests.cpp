#include <catch2/catch_all.hpp>

#include "orl_parser.h"

using namespace orlcomp;

TEST_CASE("parser accepts basic function body", "[orl][parser]") {
    const std::string src =
        "int main() {\n"
        "    vector v1(1, 0, 0);\n"
        "    vector v2(0, 1, 1);\n"
        "    print(dot(v1, v2));\n"
        "    return 0;\n"
        "}\n";

    Parser parser(src);
    const bool ok = parser.Parse();

    REQUIRE(ok);
    REQUIRE(parser.Errors().empty());
}

TEST_CASE("parser reports syntax error on missing semicolon", "[orl][parser]") {
    const std::string src =
        "int main() {\n"
        "    int x = 3\n"
        "    return x;\n"
        "}\n";

    Parser parser(src);
    const bool ok = parser.Parse();

    REQUIRE_FALSE(ok);
    REQUIRE_FALSE(parser.Errors().empty());
}
