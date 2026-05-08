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
    REQUIRE(parser.Ast() != nullptr);
    REQUIRE(parser.Ast()->items.size() == 1);
    REQUIRE(dynamic_cast<const FunctionDefinitionStatement *>(parser.Ast()->items[0].get()) != nullptr);
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

TEST_CASE("parser builds AST nodes for declarations calls and return", "[orl][parser][ast]") {
    const std::string src =
        "int main() {\n"
        "    vector v1(1, 0, 0);\n"
        "    print(v1);\n"
        "    return 0;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    const Program *program = parser.Ast();
    REQUIRE(program != nullptr);
    REQUIRE(program->items.size() == 1);

    const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(program->items[0].get());
    REQUIRE(function != nullptr);
    REQUIRE(function->name == "main");
    REQUIRE(function->return_type == "int");
    REQUIRE(function->body != nullptr);
    REQUIRE(function->body->statements.size() == 3);

    const auto *declaration = dynamic_cast<const DeclarationStatement *>(function->body->statements[0].get());
    REQUIRE(declaration != nullptr);
    REQUIRE(declaration->type_name == "vector");
    REQUIRE(declaration->variable_name == "v1");
    REQUIRE(declaration->constructor_arguments.size() == 3);

    const auto *call_stmt = dynamic_cast<const ExpressionStatement *>(function->body->statements[1].get());
    REQUIRE(call_stmt != nullptr);
    const auto *call_expr = dynamic_cast<const CallExpression *>(call_stmt->expression.get());
    REQUIRE(call_expr != nullptr);
    REQUIRE(call_expr->arguments.size() == 1);

    const auto *ret = dynamic_cast<const ReturnStatement *>(function->body->statements[2].get());
    REQUIRE(ret != nullptr);
    REQUIRE(ret->value != nullptr);
    const auto *ret_lit = dynamic_cast<const LiteralExpression *>(ret->value.get());
    REQUIRE(ret_lit != nullptr);
    REQUIRE(ret_lit->kind == LiteralKind::Int);
    REQUIRE(ret_lit->int_value == 0);
}

TEST_CASE("parser keeps operator precedence in AST", "[orl][parser][ast]") {
    const std::string src =
        "int main() {\n"
        "    int x = 1 + 2 * 3;\n"
        "    return x;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    const Program *program = parser.Ast();
    REQUIRE(program != nullptr);
    REQUIRE(program->items.size() == 1);

    const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(program->items[0].get());
    REQUIRE(function != nullptr);
    REQUIRE(function->body != nullptr);
    REQUIRE(function->body->statements.size() >= 1);

    const auto *declaration = dynamic_cast<const DeclarationStatement *>(function->body->statements[0].get());
    REQUIRE(declaration != nullptr);
    REQUIRE(declaration->initializer != nullptr);

    const auto *add = dynamic_cast<const BinaryExpression *>(declaration->initializer.get());
    REQUIRE(add != nullptr);
    REQUIRE(add->op == BinaryOp::Add);

    const auto *left = dynamic_cast<const LiteralExpression *>(add->left.get());
    REQUIRE(left != nullptr);
    REQUIRE(left->int_value == 1);

    const auto *mul = dynamic_cast<const BinaryExpression *>(add->right.get());
    REQUIRE(mul != nullptr);
    REQUIRE(mul->op == BinaryOp::Multiply);
}
