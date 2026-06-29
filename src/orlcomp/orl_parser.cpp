#include "orl_parser.h"

#include <sstream>
#include <utility>

namespace orlcomp {

Parser::Parser(std::string source) : lexer_(std::move(source)) {}

bool Parser::Parse() {
    errors_.clear();
    program_ = std::make_unique<Program>();

    bool ok = true;
    while (!IsAtEnd()) {
        if (ParseTopLevel()) {
            program_->items.push_back(TakeStatement());
        } else {
            ok = false;
            Synchronize();
        }
    }
    return ok && errors_.empty();
}

const std::vector<std::string> &Parser::Errors() const {
    return errors_;
}

const Program *Parser::Ast() const {
    return program_.get();
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
    const Token return_type_token = Peek();
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

    std::vector<Parameter> parameters;
    if (Peek().kind != TokenKind::RParen) {
        do {
            const Token parameter_type = Peek();
            if (!ParseTypeName()) {
                return false;
            }

            if (Peek().kind != TokenKind::Identifier) {
                AddError(Peek(), "Expected parameter name");
                return false;
            }

            const Token parameter_name = Advance();
            parameters.push_back(Parameter{parameter_type.lexeme, parameter_name.lexeme});
        } while (Match(TokenKind::Comma));
    }

    if (!Expect(TokenKind::RParen, "Expected ')' after function parameters")) {
        return false;
    }

    if (!ParseBlock()) {
        return false;
    }
    auto body_statement = TakeStatement();
    auto *block = dynamic_cast<BlockStatement *>(body_statement.get());
    if (block == nullptr) {
        AddError(name, "Internal parser error: function body is not a block");
        return false;
    }

    auto function = std::make_unique<FunctionDefinitionStatement>();
    function->return_type = return_type_token.lexeme;
    function->name = name.lexeme;
    function->parameters = std::move(parameters);
    function->body = std::unique_ptr<BlockStatement>(static_cast<BlockStatement *>(body_statement.release()));
    last_statement_ = std::move(function);
    return true;
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

    auto block = std::make_unique<BlockStatement>();
    while (!IsAtEnd() && Peek().kind != TokenKind::RBrace) {
        if (!ParseStatement()) {
            return false;
        }
        block->statements.push_back(TakeStatement());
    }

    if (!Expect(TokenKind::RBrace, "Expected '}' to end block")) {
        return false;
    }

    last_statement_ = std::move(block);
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
    {
        const Token kind = Advance();
        if (!Expect(TokenKind::Semi, "Expected ';' after loop control statement")) {
            return false;
        }
        auto loop_control = std::make_unique<LoopControlStatement>();
        loop_control->kind = kind.kind == TokenKind::KwBreak ? LoopControlKind::Break : LoopControlKind::Continue;
        last_statement_ = std::move(loop_control);
        return true;
    }
    default:
        if (IsTypeToken(Peek().kind)) {
            return ParseDeclarationStatement();
        }
        return ParseExpressionStatement();
    }
}

bool Parser::ParseDeclarationStatement() {
    const Token type_name = Peek();
    if (!ParseTypeName()) {
        return false;
    }

    if (Peek().kind != TokenKind::Identifier) {
        AddError(Peek(), "Expected variable name");
        return false;
    }
    const Token variable_name = Advance();

    auto declaration = std::make_unique<DeclarationStatement>();
    declaration->type_name = type_name.lexeme;
    declaration->variable_name = variable_name.lexeme;

    if (Match(TokenKind::Assign)) {
        if (!ParseExpression()) {
            return false;
        }
        declaration->initializer = TakeExpression();
    } else if (Match(TokenKind::LParen)) {
        if (Peek().kind != TokenKind::RParen && !ParseArgumentList(&declaration->constructor_arguments)) {
            return false;
        }
        if (!Expect(TokenKind::RParen, "Expected ')' after constructor arguments")) {
            return false;
        }
    }

    if (!Expect(TokenKind::Semi, "Expected ';' after declaration")) {
        return false;
    }
    last_statement_ = std::move(declaration);
    return true;
}

bool Parser::ParseReturnStatement() {
    Advance();
    const bool has_value = Peek().kind != TokenKind::Semi;
    auto return_statement = std::make_unique<ReturnStatement>();
    if (has_value && !ParseExpression()) {
        return false;
    }
    if (has_value) {
        return_statement->value = TakeExpression();
    }
    if (!Expect(TokenKind::Semi, "Expected ';' after return statement")) {
        return false;
    }
    last_statement_ = std::move(return_statement);
    return true;
}

bool Parser::ParseIfStatement() {
    Advance();
    if (!Expect(TokenKind::LParen, "Expected '(' after if")) {
        return false;
    }
    if (!ParseExpression()) {
        return false;
    }
    auto condition = TakeExpression();
    if (!Expect(TokenKind::RParen, "Expected ')' after if condition")) {
        return false;
    }
    if (!ParseStatement()) {
        return false;
    }
    auto then_branch = TakeStatement();

    std::unique_ptr<Statement> else_branch;
    if (Match(TokenKind::KwElse)) {
        if (!ParseStatement()) {
            return false;
        }
        else_branch = TakeStatement();
    }

    auto statement = std::make_unique<IfStatement>();
    statement->condition = std::move(condition);
    statement->then_branch = std::move(then_branch);
    statement->else_branch = std::move(else_branch);
    last_statement_ = std::move(statement);
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
    auto condition = TakeExpression();
    if (!Expect(TokenKind::RParen, "Expected ')' after while condition")) {
        return false;
    }
    if (!ParseStatement()) {
        return false;
    }

    auto statement = std::make_unique<WhileStatement>();
    statement->condition = std::move(condition);
    statement->body = TakeStatement();
    last_statement_ = std::move(statement);
    return true;
}

bool Parser::ParseDoWhileStatement() {
    Advance();
    if (!ParseStatement()) {
        return false;
    }
    auto body = TakeStatement();
    if (!Expect(TokenKind::KwWhile, "Expected 'while' after do body")) {
        return false;
    }
    if (!Expect(TokenKind::LParen, "Expected '(' after while")) {
        return false;
    }
    if (!ParseExpression()) {
        return false;
    }
    auto condition = TakeExpression();
    if (!Expect(TokenKind::RParen, "Expected ')' after do-while condition")) {
        return false;
    }
    if (!Expect(TokenKind::Semi, "Expected ';' after do-while")) {
        return false;
    }

    auto statement = std::make_unique<DoWhileStatement>();
    statement->body = std::move(body);
    statement->condition = std::move(condition);
    last_statement_ = std::move(statement);
    return true;
}

bool Parser::ParseForStatement() {
    Advance();
    if (!Expect(TokenKind::LParen, "Expected '(' after for")) {
        return false;
    }

    auto statement = std::make_unique<ForStatement>();
    if (Peek().kind != TokenKind::Semi) {
        if (!ParseExpression()) {
            return false;
        }
        statement->init = TakeExpression();
    }
    if (!Expect(TokenKind::Semi, "Expected ';' after for init")) {
        return false;
    }

    if (Peek().kind != TokenKind::Semi) {
        if (!ParseExpression()) {
            return false;
        }
        statement->condition = TakeExpression();
    }
    if (!Expect(TokenKind::Semi, "Expected ';' after for condition")) {
        return false;
    }

    if (Peek().kind != TokenKind::RParen) {
        if (!ParseExpression()) {
            return false;
        }
        statement->increment = TakeExpression();
    }
    if (!Expect(TokenKind::RParen, "Expected ')' after for clauses")) {
        return false;
    }

    if (!ParseStatement()) {
        return false;
    }
    statement->body = TakeStatement();
    last_statement_ = std::move(statement);
    return true;
}

bool Parser::ParseExpressionStatement() {
    if (!ParseExpression()) {
        return false;
    }
    auto expression = TakeExpression();
    if (!Expect(TokenKind::Semi, "Expected ';' after expression")) {
        return false;
    }
    auto statement = std::make_unique<ExpressionStatement>();
    statement->expression = std::move(expression);
    last_statement_ = std::move(statement);
    return true;
}

bool Parser::ParseExpression() {
    return ParseAssignment();
}

bool Parser::ParseAssignment() {
    if (Peek().kind == TokenKind::Identifier && Peek(1).kind == TokenKind::Assign) {
        const Token identifier = Advance();
        Advance();
        if (!ParseAssignment()) {
            return false;
        }
        auto assignment = std::make_unique<AssignmentExpression>();
        assignment->target_name = identifier.lexeme;
        assignment->value = TakeExpression();
        last_expression_ = std::move(assignment);
        return true;
    }
    return ParseLogicalOr();
}

bool Parser::ParseLogicalOr() {
    if (!ParseLogicalAnd()) {
        return false;
    }
    auto lhs = TakeExpression();
    while (Peek().kind == TokenKind::PipePipe || Peek().kind == TokenKind::KwOr) {
        const TokenKind op = Advance().kind;
        if (!ParseLogicalAnd()) {
            return false;
        }
        auto expression = std::make_unique<BinaryExpression>();
        expression->op = BinaryOperatorFromToken(op);
        expression->left = std::move(lhs);
        expression->right = TakeExpression();
        lhs = std::move(expression);
    }
    last_expression_ = std::move(lhs);
    return true;
}

bool Parser::ParseLogicalAnd() {
    if (!ParseEquality()) {
        return false;
    }
    auto lhs = TakeExpression();
    while (Peek().kind == TokenKind::AmpAmp || Peek().kind == TokenKind::KwAnd) {
        const TokenKind op = Advance().kind;
        if (!ParseEquality()) {
            return false;
        }
        auto expression = std::make_unique<BinaryExpression>();
        expression->op = BinaryOperatorFromToken(op);
        expression->left = std::move(lhs);
        expression->right = TakeExpression();
        lhs = std::move(expression);
    }
    last_expression_ = std::move(lhs);
    return true;
}

bool Parser::ParseEquality() {
    if (!ParseComparison()) {
        return false;
    }
    auto lhs = TakeExpression();
    while (Peek().kind == TokenKind::EqualEqual || Peek().kind == TokenKind::BangEqual) {
        const TokenKind op = Advance().kind;
        if (!ParseComparison()) {
            return false;
        }
        auto expression = std::make_unique<BinaryExpression>();
        expression->op = BinaryOperatorFromToken(op);
        expression->left = std::move(lhs);
        expression->right = TakeExpression();
        lhs = std::move(expression);
    }
    last_expression_ = std::move(lhs);
    return true;
}

bool Parser::ParseComparison() {
    if (!ParseTerm()) {
        return false;
    }
    auto lhs = TakeExpression();
    while (Peek().kind == TokenKind::Less ||
           Peek().kind == TokenKind::LessEqual ||
           Peek().kind == TokenKind::Greater ||
           Peek().kind == TokenKind::GreaterEqual) {
        const TokenKind op = Advance().kind;
        if (!ParseTerm()) {
            return false;
        }
        auto expression = std::make_unique<BinaryExpression>();
        expression->op = BinaryOperatorFromToken(op);
        expression->left = std::move(lhs);
        expression->right = TakeExpression();
        lhs = std::move(expression);
    }
    last_expression_ = std::move(lhs);
    return true;
}

bool Parser::ParseTerm() {
    if (!ParseFactor()) {
        return false;
    }
    auto lhs = TakeExpression();
    while (Peek().kind == TokenKind::Plus || Peek().kind == TokenKind::Minus) {
        const TokenKind op = Advance().kind;
        if (!ParseFactor()) {
            return false;
        }
        auto expression = std::make_unique<BinaryExpression>();
        expression->op = BinaryOperatorFromToken(op);
        expression->left = std::move(lhs);
        expression->right = TakeExpression();
        lhs = std::move(expression);
    }
    last_expression_ = std::move(lhs);
    return true;
}

bool Parser::ParseFactor() {
    if (!ParseUnary()) {
        return false;
    }
    auto lhs = TakeExpression();
    while (Peek().kind == TokenKind::Star || Peek().kind == TokenKind::Slash || Peek().kind == TokenKind::Percent) {
        const TokenKind op = Advance().kind;
        if (!ParseUnary()) {
            return false;
        }
        auto expression = std::make_unique<BinaryExpression>();
        expression->op = BinaryOperatorFromToken(op);
        expression->left = std::move(lhs);
        expression->right = TakeExpression();
        lhs = std::move(expression);
    }
    last_expression_ = std::move(lhs);
    return true;
}

bool Parser::ParseUnary() {
    if (IsUnaryOperator(Peek().kind)) {
        const TokenKind op = Advance().kind;
        if (!ParseUnary()) {
            return false;
        }
        auto expression = std::make_unique<UnaryExpression>();
        expression->op = UnaryOperatorFromToken(op);
        expression->operand = TakeExpression();
        last_expression_ = std::move(expression);
        return true;
    }
    return ParsePostfix();
}

bool Parser::ParsePostfix() {
    if (!ParsePrimary()) {
        return false;
    }
    auto expression = TakeExpression();

    while (Match(TokenKind::LParen)) {
        std::vector<std::unique_ptr<Expression>> arguments;
        if (Peek().kind != TokenKind::RParen && !ParseArgumentList(&arguments)) {
            return false;
        }
        if (!Expect(TokenKind::RParen, "Expected ')' after function call arguments")) {
            return false;
        }
        auto call = std::make_unique<CallExpression>();
        call->callee = std::move(expression);
        call->arguments = std::move(arguments);
        expression = std::move(call);
    }

    last_expression_ = std::move(expression);
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
        IsTypeToken(kind) ||
        kind == TokenKind::IntLiteral ||
        kind == TokenKind::FloatLiteral ||
        kind == TokenKind::StringLiteral) {
        const Token token = Advance();
        if (token.kind == TokenKind::Identifier || IsTypeToken(token.kind)) {
            auto identifier = std::make_unique<IdentifierExpression>();
            identifier->name = token.lexeme;
            last_expression_ = std::move(identifier);
            return true;
        }

        auto literal = std::make_unique<LiteralExpression>();
        literal->raw_lexeme = token.lexeme;
        if (token.kind == TokenKind::IntLiteral) {
            literal->kind = LiteralKind::Int;
            literal->int_value = token.int_value;
        } else if (token.kind == TokenKind::FloatLiteral) {
            literal->kind = LiteralKind::Float;
            literal->float_value = token.float_value;
        } else {
            literal->kind = LiteralKind::String;
        }
        last_expression_ = std::move(literal);
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

bool Parser::ParseArgumentList(std::vector<std::unique_ptr<Expression>> *arguments) {
    if (!ParseExpression()) {
        return false;
    }
    arguments->push_back(TakeExpression());

    while (Match(TokenKind::Comma)) {
        if (!ParseExpression()) {
            return false;
        }
        arguments->push_back(TakeExpression());
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
           kind == TokenKind::KwMatrix ||
           kind == TokenKind::TypeName;
}

bool Parser::IsUnaryOperator(TokenKind kind) const {
    return kind == TokenKind::Bang ||
           kind == TokenKind::KwNot ||
           kind == TokenKind::Minus ||
           kind == TokenKind::Plus;
}

UnaryOp Parser::UnaryOperatorFromToken(TokenKind kind) const {
    switch (kind) {
    case TokenKind::Bang:
    case TokenKind::KwNot:
        return UnaryOp::Not;
    case TokenKind::Minus:
        return UnaryOp::Minus;
    case TokenKind::Plus:
        return UnaryOp::Plus;
    default:
        return UnaryOp::Plus;
    }
}

BinaryOp Parser::BinaryOperatorFromToken(TokenKind kind) const {
    switch (kind) {
    case TokenKind::Plus: return BinaryOp::Add;
    case TokenKind::Minus: return BinaryOp::Subtract;
    case TokenKind::Star: return BinaryOp::Multiply;
    case TokenKind::Slash: return BinaryOp::Divide;
    case TokenKind::Percent: return BinaryOp::Modulo;
    case TokenKind::Less: return BinaryOp::Less;
    case TokenKind::LessEqual: return BinaryOp::LessEqual;
    case TokenKind::Greater: return BinaryOp::Greater;
    case TokenKind::GreaterEqual: return BinaryOp::GreaterEqual;
    case TokenKind::EqualEqual: return BinaryOp::Equal;
    case TokenKind::BangEqual: return BinaryOp::NotEqual;
    case TokenKind::AmpAmp:
    case TokenKind::KwAnd:
        return BinaryOp::LogicalAnd;
    case TokenKind::PipePipe:
    case TokenKind::KwOr:
        return BinaryOp::LogicalOr;
    default:
        return BinaryOp::Add;
    }
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
            // Consume block terminator so error recovery always makes progress.
            Advance();
            return;
        }
        Advance();
    }
}

std::unique_ptr<Expression> Parser::TakeExpression() {
    return std::move(last_expression_);
}

std::unique_ptr<Statement> Parser::TakeStatement() {
    return std::move(last_statement_);
}

} // namespace orlcomp
