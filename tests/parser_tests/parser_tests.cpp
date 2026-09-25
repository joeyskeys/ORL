#include <catch2/catch_all.hpp>

#include "orl_parser.h"

#include <filesystem>
#include <fstream>

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

TEST_CASE("parser tracks implicit solver context usage",
    "[orl][parser][solver_context]")
{
    Parser parser(R"(
        int solve() {
            return solver_context.joint_count
                + solver_context.controller_count;
        }
    )");
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);
    REQUIRE(parser.Ast()->uses_solver_context);
}

TEST_CASE("parser tracks implicit hierarchy context usage",
    "[orl][parser][hierarchy]")
{
    Parser parser(R"(
        int inspect() {
            return hierarchy_context.joint_count
                + hierarchy_data[0];
        }
    )");
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);
    REQUIRE(parser.Ast()->uses_hierarchy_context);
}

TEST_CASE("parser reserves the solver context type and symbol",
    "[orl][parser][solver_context][error]")
{
    Parser struct_parser("struct SolverContext { int value; };\n");
    REQUIRE_FALSE(struct_parser.Parse());

    Parser function_parser("int solver_context() { return 0; }\n");
    REQUIRE_FALSE(function_parser.Parse());
}

TEST_CASE("parser accepts normalized handle unions and is expressions",
    "[orl][parser][handle][union]")
{
    Parser parser(R"(
        handle joint_handle;
        handle locator_handle;
        handle xform_handle = locator_handle | joint_handle;
        int inspect(xform_handle value) {
            if (value is joint_handle) {
                return 1;
            }
            return 0;
        }
    )");
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());
    REQUIRE(parser.Ast()->items.size() == 4);
    const auto* union_definition =
        dynamic_cast<const HandleDefinitionStatement*>(
            parser.Ast()->items[2].get());
    REQUIRE(union_definition != nullptr);
    REQUIRE(union_definition->accepted_handles.size() == 2);
    REQUIRE(union_definition->accepted_handles[0]
        == "<source>::joint_handle");
    REQUIRE(union_definition->accepted_handles[1]
        == "<source>::locator_handle");
    const auto* function =
        dynamic_cast<const FunctionDefinitionStatement*>(
            parser.Ast()->items[3].get());
    REQUIRE(function != nullptr);
    const auto* conditional =
        dynamic_cast<const IfStatement*>(
            function->body->statements.front().get());
    REQUIRE(conditional != nullptr);
    REQUIRE(dynamic_cast<const HandleTestExpression*>(
        conditional->condition.get()) != nullptr);
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

TEST_CASE("parser builds canonical parallel for AST", "[orl][parser][ast][parallel]") {
    const std::string src =
        "int update(int count) {\n"
        "    parallel for (int index = 0; index < count; index = index + 1) {\n"
        "        print(index);\n"
        "    }\n"
        "    return count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());

    const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(parser.Ast()->items[0].get());
    REQUIRE(function != nullptr);
    const auto *parallel_for = dynamic_cast<const ParallelForStatement *>(function->body->statements[0].get());
    REQUIRE(parallel_for != nullptr);
    REQUIRE(parallel_for->index_name == "index");
    const auto *bound = dynamic_cast<const IdentifierExpression *>(parallel_for->bound.get());
    REQUIRE(bound != nullptr);
    REQUIRE(bound->name == "count");
    REQUIRE(parallel_for->body != nullptr);
}

TEST_CASE("parser rejects noncanonical parallel for headers", "[orl][parser][parallel]") {
    const std::string src =
        "int update(int count) {\n"
        "    parallel for (int index = 1; index < count; index = index + 1) { }\n"
        "    return count;\n"
        "}\n";

    Parser parser(src);
    REQUIRE_FALSE(parser.Parse());
    REQUIRE_FALSE(parser.Errors().empty());
}

TEST_CASE("parser builds fixed arrays and indexed assignments", "[orl][parser][ast]") {
    const std::string src =
        "int main() {\n"
        "    float weights[2];\n"
        "    weights[0] = 0.25;\n"
        "    weights[1] = 0.75;\n"
        "    return int(weights[0] * 100 + weights[1] * 100);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());

    const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(parser.Ast()->items[0].get());
    REQUIRE(function != nullptr);
    REQUIRE(function->body->statements.size() == 4);

    const auto *array_declaration = dynamic_cast<const DeclarationStatement *>(function->body->statements[0].get());
    REQUIRE(array_declaration != nullptr);
    REQUIRE(array_declaration->type_name == "float");
    REQUIRE(array_declaration->array_size == 2);

    const auto *first_assignment = dynamic_cast<const ExpressionStatement *>(function->body->statements[1].get());
    REQUIRE(first_assignment != nullptr);
    REQUIRE(dynamic_cast<const IndexAssignmentExpression *>(first_assignment->expression.get()) != nullptr);
}

TEST_CASE("parser accepts GLSL-style vector and matrix type names", "[orl][parser][ast]") {
    const std::string src =
        "dvec3 transform(mat4 m, ivec3 index) {\n"
        "    vec3 p = vec3(1, 2, 3);\n"
        "    dmat4 dm;\n"
        "    return dvec3(1, 2, 3);\n"
        "}\n";

    Parser parser(src);
    REQUIRE(parser.Parse());
    REQUIRE(parser.Errors().empty());

    const Program *program = parser.Ast();
    REQUIRE(program != nullptr);
    REQUIRE(program->items.size() == 1);

    const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(program->items[0].get());
    REQUIRE(function != nullptr);
    REQUIRE(function->return_type == "dvec3");
    REQUIRE(function->parameters.size() == 2);
    REQUIRE(function->parameters[0].type_name == "mat4");
    REQUIRE(function->parameters[1].type_name == "ivec3");
    REQUIRE(function->body != nullptr);
    REQUIRE(function->body->statements.size() == 3);

    const auto *vec_decl = dynamic_cast<const DeclarationStatement *>(function->body->statements[0].get());
    REQUIRE(vec_decl != nullptr);
    REQUIRE(vec_decl->type_name == "vec3");
    REQUIRE(vec_decl->initializer != nullptr);
    REQUIRE(dynamic_cast<const CallExpression *>(vec_decl->initializer.get()) != nullptr);

    const auto *mat_decl = dynamic_cast<const DeclarationStatement *>(function->body->statements[1].get());
    REQUIRE(mat_decl != nullptr);
    REQUIRE(mat_decl->type_name == "dmat4");

    const auto *ret = dynamic_cast<const ReturnStatement *>(function->body->statements[2].get());
    REQUIRE(ret != nullptr);
    REQUIRE(dynamic_cast<const CallExpression *>(ret->value.get()) != nullptr);
}

TEST_CASE("parser recognizes exported functions and OSL-style metadata",
    "[orl][parser][export]")
{
    Parser parser(R"(
        export int solve [[
            string stage = "solver",
            string label = "Solve"
        ]] (int value) {
            return value;
        }

        int helper(int value) {
            return value + 1;
        }
    )");

    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast() != nullptr);
    REQUIRE(parser.Ast()->items.size() == 2);

    const auto* solve = dynamic_cast<const FunctionDefinitionStatement*>(
        parser.Ast()->items[0].get());
    REQUIRE(solve != nullptr);
    REQUIRE(solve->exported);
    REQUIRE(solve->metadata.size() == 2);
    REQUIRE(solve->metadata[0].type_name == "string");
    REQUIRE(solve->metadata[0].name == "stage");
    REQUIRE(solve->metadata[0].value_kind == LiteralKind::String);
    REQUIRE(solve->metadata[0].raw_value == "\"solver\"");

    const auto* helper = dynamic_cast<const FunctionDefinitionStatement*>(
        parser.Ast()->items[1].get());
    REQUIRE(helper != nullptr);
    REQUIRE_FALSE(helper->exported);
    REQUIRE(helper->metadata.empty());
}

TEST_CASE("parser rejects metadata on non-exported functions",
    "[orl][parser][export]")
{
    Parser parser(R"(
        int helper [[ string label = "hidden" ]] (int value) {
            return value;
        }
    )");

    REQUIRE_FALSE(parser.Parse());
    REQUIRE_FALSE(parser.Errors().empty());
}

TEST_CASE("parser retains universal and nominal handle types",
    "[orl][parser][handle]")
{
    Parser parser(R"(
        handle joint_handle;
        joint_handle choose(joint_handle value, handle fallback) {
            joint_handle local = value;
            return local;
        }
    )", "orlrig");
    REQUIRE(parser.Parse());
    REQUIRE(parser.Ast()->items.size() == 2);

    const auto* handle = dynamic_cast<const HandleDefinitionStatement*>(
        parser.Ast()->items[0].get());
    REQUIRE(handle != nullptr);
    REQUIRE(handle->name == "joint_handle");
    REQUIRE(handle->canonical_name == "orlrig::joint_handle");

    const auto* function = dynamic_cast<const FunctionDefinitionStatement*>(
        parser.Ast()->items[1].get());
    REQUIRE(function != nullptr);
    REQUIRE(function->return_type == "joint_handle");
    REQUIRE(function->parameters[1].type_name == "handle");
}

TEST_CASE("parser enforces handle declaration order and scalar storage",
    "[orl][parser][handle][error]")
{
    Parser before_declaration(R"(
        joint_handle invalid(joint_handle value) { return value; }
        handle joint_handle;
    )");
    REQUIRE_FALSE(before_declaration.Parse());

    Parser array(R"(
        handle joint_handle;
        int invalid() { joint_handle values[2]; return 0; }
    )");
    REQUIRE_FALSE(array.Parse());

    Parser buffer(R"(
        handle joint_handle;
        int invalid(joint_handle values[]) { return 0; }
    )");
    REQUIRE_FALSE(buffer.Parse());
}

TEST_CASE("parser rejects duplicate and colliding handle names",
    "[orl][parser][handle][error]")
{
    Parser duplicate("handle joint_handle; handle joint_handle;");
    REQUIRE_FALSE(duplicate.Parse());

    Parser collision(
        "handle joint_handle; struct joint_handle { int value; };");
    REQUIRE_FALSE(collision.Parse());
}

TEST_CASE("parser keeps imported package identity for handles",
    "[orl][parser][handle][use]")
{
    const auto root = std::filesystem::temp_directory_path()
        / "orl_handle_parser_test";
    std::filesystem::create_directories(root / "rig");
    {
        std::ofstream module(root / "rig" / "types.orl");
        REQUIRE(module);
        module << "handle joint_handle;\n";
    }

    Parser parser(R"(
        use rig/types;
        joint_handle identity(joint_handle value) { return value; }
    )");
    parser.AddIncludePath(root.string());
    REQUIRE(parser.Parse());
    const auto* handle = dynamic_cast<const HandleDefinitionStatement*>(
        parser.Ast()->items[0].get());
    REQUIRE(handle != nullptr);
    REQUIRE(handle->canonical_name == "rig::types::joint_handle");
    REQUIRE(handle->source.source == "rig/types");
    REQUIRE(handle->source.line == 1);
    std::filesystem::remove_all(root);
}
