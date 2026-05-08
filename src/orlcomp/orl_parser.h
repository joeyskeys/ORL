#pragma once

#include "orl_ast.h"
#include "orl_lexer.h"

#include <memory>
#include <string>
#include <vector>

namespace orlcomp {

class Parser {
public:
    explicit Parser(std::string source);

    bool Parse();
    const std::vector<std::string> &Errors() const;
    const Program *Ast() const;

private:
    const Token &Peek(std::size_t offset = 0);
    bool IsAtEnd();
    Token Advance();
    bool Match(TokenKind kind);
    bool Expect(TokenKind kind, const char *message);

    bool ParseTopLevel();
    bool ParseFunctionDefinition();
    bool ParseTypeName();
    bool ParseBlock();
    bool ParseStatement();
    bool ParseDeclarationStatement();
    bool ParseReturnStatement();
    bool ParseIfStatement();
    bool ParseWhileStatement();
    bool ParseDoWhileStatement();
    bool ParseForStatement();

    bool ParseExpressionStatement();
    bool ParseExpression();
    bool ParseAssignment();
    bool ParseLogicalOr();
    bool ParseLogicalAnd();
    bool ParseEquality();
    bool ParseComparison();
    bool ParseTerm();
    bool ParseFactor();
    bool ParseUnary();
    bool ParsePostfix();
    bool ParsePrimary();
    bool ParseArgumentList(std::vector<std::unique_ptr<Expression>> *arguments);

    bool IsTypeToken(TokenKind kind) const;
    bool IsUnaryOperator(TokenKind kind) const;
    UnaryOp UnaryOperatorFromToken(TokenKind kind) const;
    BinaryOp BinaryOperatorFromToken(TokenKind kind) const;
    void AddError(const Token &token, const std::string &message);
    void Synchronize();
    std::unique_ptr<Expression> TakeExpression();
    std::unique_ptr<Statement> TakeStatement();

    Lexer lexer_;
    std::vector<Token> buffered_tokens_;
    std::vector<std::string> errors_;
    std::unique_ptr<Program> program_;
    std::unique_ptr<Expression> last_expression_;
    std::unique_ptr<Statement> last_statement_;
};

} // namespace orlcomp
