#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace orlcomp {

struct AstNode {
    virtual ~AstNode() = default;
};

struct Expression : AstNode {
    virtual ~Expression() = default;
};

struct Statement : AstNode {
    virtual ~Statement() = default;
};

enum class UnaryOp : std::uint8_t {
    Plus,
    Minus,
    Not
};

enum class BinaryOp : std::uint8_t {
    Add,
    Subtract,
    Multiply,
    Divide,
    Modulo,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
    Equal,
    NotEqual,
    LogicalAnd,
    LogicalOr
};

enum class LiteralKind : std::uint8_t {
    Int,
    Float,
    String
};

struct IdentifierExpression final : Expression {
    std::string name;
};

struct LiteralExpression final : Expression {
    LiteralKind kind = LiteralKind::Int;
    std::string raw_lexeme;
    std::int64_t int_value = 0;
    double float_value = 0.0;
};

struct UnaryExpression final : Expression {
    UnaryOp op = UnaryOp::Plus;
    std::unique_ptr<Expression> operand;
};

struct BinaryExpression final : Expression {
    BinaryOp op = BinaryOp::Add;
    std::unique_ptr<Expression> left;
    std::unique_ptr<Expression> right;
};

struct AssignmentExpression final : Expression {
    std::string target_name;
    std::unique_ptr<Expression> value;
};

struct CallExpression final : Expression {
    std::unique_ptr<Expression> callee;
    std::vector<std::unique_ptr<Expression>> arguments;
};

struct BlockStatement final : Statement {
    std::vector<std::unique_ptr<Statement>> statements;
};

struct ExpressionStatement final : Statement {
    std::unique_ptr<Expression> expression;
};

struct DeclarationStatement final : Statement {
    std::string type_name;
    std::string variable_name;
    std::unique_ptr<Expression> initializer;
    std::vector<std::unique_ptr<Expression>> constructor_arguments;
};

struct ReturnStatement final : Statement {
    std::unique_ptr<Expression> value;
};

struct IfStatement final : Statement {
    std::unique_ptr<Expression> condition;
    std::unique_ptr<Statement> then_branch;
    std::unique_ptr<Statement> else_branch;
};

struct WhileStatement final : Statement {
    std::unique_ptr<Expression> condition;
    std::unique_ptr<Statement> body;
};

struct DoWhileStatement final : Statement {
    std::unique_ptr<Statement> body;
    std::unique_ptr<Expression> condition;
};

struct ForStatement final : Statement {
    std::unique_ptr<Expression> init;
    std::unique_ptr<Expression> condition;
    std::unique_ptr<Expression> increment;
    std::unique_ptr<Statement> body;
};

enum class LoopControlKind : std::uint8_t {
    Break,
    Continue
};

struct LoopControlStatement final : Statement {
    LoopControlKind kind = LoopControlKind::Break;
};

struct Parameter {
    std::string type_name;
    std::string name;
};

struct FunctionDefinitionStatement final : Statement {
    std::string return_type;
    std::string name;
    std::vector<Parameter> parameters;
    std::unique_ptr<BlockStatement> body;
};

struct Program final : AstNode {
    std::vector<std::unique_ptr<Statement>> items;
};

} // namespace orlcomp
