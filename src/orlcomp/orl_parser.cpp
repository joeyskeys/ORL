#include "orl_parser.h"

#include <sstream>

namespace orlcomp {

Parser::Parser(std::string source) : lexer_(std::move(source)) {}

bool Parser::Parse() {
    bool ok = true;
    while (!IsAtEnd()) {
        if (!ParseTopLevel()) {
            ok = false;
            Synchronize();
        }
    }
    return ok && errors_.empty();
}

const std::vector<std::string> &Parser::Errors() const {
    return errors_;
}

const Token &Parser::Peek(std::size_t offset) {
    while (buffered_tokens_.size() <= offset) {
        buffered_tokens_.push_back(lexer_.NextToken());
    }
    return buffered_tokens_[offset];
}

bool Parser::IsAtEnd() {
    return Peek().kind == TokenKind::EndOfFile;
}

Token Parser::Advance() {
    const Token token = Peek();
    buffered_tokens_.erase(buffered_tokens_.begin());
    return token;
}

bool Parser::Match(TokenKind kind) {
    if (Peek().kind != kind) {
        return false;
    }
    Advance();
    return true;
}

bool Parser::Expect(TokenKind kind, const char *message) {
    if (Peek().kind == kind) {
        Advance();
        return true;
    }
    AddError(Peek(), message);
    return false;
}

bool Parser::ParseTopLevel() {
    if (Peek().kind == TokenKind::Invalid) {
        AddError(Peek(), Peek().error_message.empty() ? "Invalid token" : Peek().error_message);
        Advance();
        return false;
    }

    if (IsTypeToken(Peek().kind) && Peek(1).kind == TokenKind::Identifier && Peek(2).kind == TokenKind::LParen) {
        return ParseFunctionDefinition();
    }

    return ParseStatement();
}

bool Parser::ParseFunctionDefinition() {
    if (!ParseTypeName()) {
        return false;
    }
    const Token name = Advance();
    if (name.kind != TokenKind::Identifier) {
        AddError(name, "Expected function name");
        return false;
    }
    if (!Expect(TokenKind::LParen, "Expected '(' after function name")) {
        return false;
    }

    if (Peek().kind != TokenKind::RParen) {
        do {
            if (!ParseTypeName()) {
                return false;
            }
            if (!Expect(TokenKind::Identifier, "Expected parameter name")) {
                return false;
            }
        } while (Match(TokenKind::Comma));
    }

    if (!Expect(TokenKind::RParen, "Expected ')' after function parameters")) {
        return false;
    }

    EmitPlaceholder("function-declare");
    return ParseBlock();
}

bool Parser::ParseTypeName() {
    if (!IsTypeToken(Peek().kind)) {
        AddError(Peek(), "Expected type name");
        return false;
    }
    Advance();
    return true;
}

bool Parser::ParseBlock() {
    if (!Expect(TokenKind::LBrace, "Expected '{' to start block")) {
        return false;
    }

    EmitPlaceholder("block-begin");
    while (!IsAtEnd() && Peek().kind != TokenKind::RBrace) {
        if (!ParseStatement()) {
            return false;
        }
    }

    if (!Expect(TokenKind::RBrace, "Expected '}' to end block")) {
        return false;
    }
    EmitPlaceholder("block-end");
    return true;
}

bool Parser::ParseStatement() {
    switch (Peek().kind) {
    case TokenKind::LBrace:
        return ParseBlock();
    case TokenKind::KwReturn:
        return ParseReturnStatement();
    case TokenKind::KwIf:
        return ParseIfStatement();
    case TokenKind::KwWhile:
        return ParseWhileStatement();
    case TokenKind::KwDo:
        return ParseDoWhileStatement();
    case TokenKind::KwFor:
        return ParseForStatement();
    case TokenKind::KwBreak:
    case TokenKind::KwContinue:
        Advance();
        EmitPlaceholder("loop-control");
        return Expect(TokenKind::Semi, "Expected ';' after loop control statement");
    default:
        if (IsTypeToken(Peek().kind)) {
            return ParseDeclarationStatement();
        }
        return ParseExpressionStatement();
    }
}

bool Parser::ParseDeclarationStatement() {
    if (!ParseTypeName()) {
        return false;
    }

    if (!Expect(TokenKind::Identifier, "Expected variable name")) {
        return false;
    }

    if (Match(TokenKind::Assign)) {
        if (!ParseExpression()) {
            return false;
        }
    } else if (Match(TokenKind::LParen)) {
        if (Peek().kind != TokenKind::RParen && !ParseArgumentList()) {
            return false;
        }
        if (!Expect(TokenKind::RParen, "Expected ')' after constructor arguments")) {
            return false;
        }
    }

    EmitPlaceholder("declare");
    return Expect(TokenKind::Semi, "Expected ';' after declaration");
}

bool Parser::ParseReturnStatement() {
    Advance();
    if (Peek().kind != TokenKind::Semi && !ParseExpression()) {
        return false;
    }
    EmitPlaceholder("return");
    return Expect(TokenKind::Semi, "Expected ';' after return statement");
}

bool Parser::ParseIfStatement() {
    Advance();
    if (!Expect(TokenKind::LParen, "Expected '(' after if")) {
        return false;
    }
    if (!ParseExpression()) {
        return false;
    }
    if (!Expect(TokenKind::RParen, "Expected ')' after if condition")) {
        return false;
    }
    if (!ParseStatement()) {
        return false;
    }
    if (Match(TokenKind::KwElse) && !ParseStatement()) {
        return false;
    }
    EmitPlaceholder("if");
    return true;
}

bool Parser::ParseWhileStatement() {
    Advance();
    if (!Expect(TokenKind::LParen, "Expected '(' after while")) {
        return false;
    }
    if (!ParseExpression()) {
        return false;
    }
    if (!Expect(TokenKind::RParen, "Expected ')' after while condition")) {
        return false;
    }
    EmitPlaceholder("while");
    return ParseStatement();
}

bool Parser::ParseDoWhileStatement() {
    Advance();
    if (!ParseStatement()) {
        return false;
    }
    if (!Expect(TokenKind::KwWhile, "Expected 'while' after do body")) {
        return false;
    }
    if (!Expect(TokenKind::LParen, "Expected '(' after while")) {
        return false;
    }
    if (!ParseExpression()) {
        return false;
    }
    if (!Expect(TokenKind::RParen, "Expected ')' after do-while condition")) {
        return false;
    }
    EmitPlaceholder("do-while");
    return Expect(TokenKind::Semi, "Expected ';' after do-while");
}

bool Parser::ParseForStatement() {
    Advance();
    if (!Expect(TokenKind::LParen, "Expected '(' after for")) {
        return false;
    }

    if (Peek().kind != TokenKind::Semi && !ParseExpression()) {
        return false;
    }
    if (!Expect(TokenKind::Semi, "Expected ';' after for init")) {
        return false;
    }

    if (Peek().kind != TokenKind::Semi && !ParseExpression()) {
        return false;
    }
    if (!Expect(TokenKind::Semi, "Expected ';' after for condition")) {
        return false;
    }

    if (Peek().kind != TokenKind::RParen && !ParseExpression()) {
        return false;
    }
    if (!Expect(TokenKind::RParen, "Expected ')' after for clauses")) {
        return false;
    }

    EmitPlaceholder("for");
    return ParseStatement();
}

bool Parser::ParseExpressionStatement() {
    if (!ParseExpression()) {
        return false;
    }
    EmitPlaceholder("expr");
    return Expect(TokenKind::Semi, "Expected ';' after expression");
}

bool Parser::ParseExpression() {
    return ParseAssignment();
}

bool Parser::ParseAssignment() {
    if (Peek().kind == TokenKind::Identifier && Peek(1).kind == TokenKind::Assign) {
        Advance();
        Advance();
        if (!ParseAssignment()) {
            return false;
        }
        EmitPlaceholder("assign");
        return true;
    }
    return ParseLogicalOr();
}

bool Parser::ParseLogicalOr() {
    if (!ParseLogicalAnd()) {
        return false;
    }
    while (Match(TokenKind::PipePipe) || Match(TokenKind::KwOr)) {
        if (!ParseLogicalAnd()) {
            return false;
        }
    }
    return true;
}

bool Parser::ParseLogicalAnd() {
    if (!ParseEquality()) {
        return false;
    }
    while (Match(TokenKind::AmpAmp) || Match(TokenKind::KwAnd)) {
        if (!ParseEquality()) {
            return false;
        }
    }
    return true;
}

bool Parser::ParseEquality() {
    if (!ParseComparison()) {
        return false;
    }
    while (Match(TokenKind::EqualEqual) || Match(TokenKind::BangEqual)) {
        if (!ParseComparison()) {
            return false;
        }
    }
    return true;
}

bool Parser::ParseComparison() {
    if (!ParseTerm()) {
        return false;
    }
    while (Match(TokenKind::Less) || Match(TokenKind::LessEqual) || Match(TokenKind::Greater) || Match(TokenKind::GreaterEqual)) {
        if (!ParseTerm()) {
            return false;
        }
    }
    return true;
}

bool Parser::ParseTerm() {
    if (!ParseFactor()) {
        return false;
    }
    while (Match(TokenKind::Plus) || Match(TokenKind::Minus)) {
        if (!ParseFactor()) {
            return false;
        }
    }
    return true;
}

bool Parser::ParseFactor() {
    if (!ParseUnary()) {
        return false;
    }
    while (Match(TokenKind::Star) || Match(TokenKind::Slash) || Match(TokenKind::Percent)) {
        if (!ParseUnary()) {
            return false;
        }
    }
    return true;
}

bool Parser::ParseUnary() {
    if (Match(TokenKind::Bang) || Match(TokenKind::KwNot) || Match(TokenKind::Minus) || Match(TokenKind::Plus)) {
        return ParseUnary();
    }
    return ParsePostfix();
}

bool Parser::ParsePostfix() {
    if (!ParsePrimary()) {
        return false;
    }

    while (Match(TokenKind::LParen)) {
        if (Peek().kind != TokenKind::RParen && !ParseArgumentList()) {
            return false;
        }
        if (!Expect(TokenKind::RParen, "Expected ')' after function call arguments")) {
            return false;
        }
        EmitPlaceholder("call");
    }
    return true;
}

bool Parser::ParsePrimary() {
    const TokenKind kind = Peek().kind;
    if (kind == TokenKind::Invalid) {
        AddError(Peek(), Peek().error_message.empty() ? "Invalid token" : Peek().error_message);
        Advance();
        return false;
    }

    if (kind == TokenKind::Identifier ||
        kind == TokenKind::IntLiteral ||
        kind == TokenKind::FloatLiteral ||
        kind == TokenKind::StringLiteral) {
        Advance();
        return true;
    }

    if (Match(TokenKind::LParen)) {
        if (!ParseExpression()) {
            return false;
        }
        return Expect(TokenKind::RParen, "Expected ')' after grouped expression");
    }

    AddError(Peek(), "Expected expression");
    return false;
}

bool Parser::ParseArgumentList() {
    if (!ParseExpression()) {
        return false;
    }
    while (Match(TokenKind::Comma)) {
        if (!ParseExpression()) {
            return false;
        }
    }
    return true;
}

bool Parser::IsTypeToken(TokenKind kind) const {
    return kind == TokenKind::KwInt ||
           kind == TokenKind::KwFloat ||
           kind == TokenKind::KwString ||
           kind == TokenKind::KwVector ||
           kind == TokenKind::KwNormal ||
           kind == TokenKind::KwPoint ||
           kind == TokenKind::KwMatrix;
}

void Parser::AddError(const Token &token, const std::string &message) {
    std::ostringstream oss;
    oss << "line " << token.line << ", col " << token.column << ": " << message;
    if (!token.lexeme.empty()) {
        oss << " near '" << token.lexeme << "'";
    }
    errors_.push_back(oss.str());
}

void Parser::Synchronize() {
    while (!IsAtEnd()) {
        if (Peek().kind == TokenKind::Semi) {
            Advance();
            return;
        }
        if (Peek().kind == TokenKind::RBrace) {
            return;
        }
        Advance();
    }
}

void Parser::EmitPlaceholder(const char *stage) {
    (void)stage;
    // TODO: wire IR/bytecode emission here.
}

} // namespace orlcomp
