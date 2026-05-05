#pragma once

#include "orl_lexer.h"

#include <string>
#include <vector>

namespace orlcomp {

class Parser {
public:
    explicit Parser(std::string source);

    bool Parse();
    const std::vector<std::string> &Errors() const;

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
    bool ParseArgumentList();

    bool IsTypeToken(TokenKind kind) const;
    void AddError(const Token &token, const std::string &message);
    void Synchronize();

    // Placeholder hooks for future code generation.
    void EmitPlaceholder(const char *stage);

    Lexer lexer_;
    std::vector<Token> buffered_tokens_;
    std::vector<std::string> errors_;
};

} // namespace orlcomp
