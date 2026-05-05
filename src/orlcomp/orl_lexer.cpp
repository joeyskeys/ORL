#include "orl_lexer.h"

#include <cctype>
#include <cstdlib>
#include <unordered_map>

namespace orlcomp {

namespace {

const std::unordered_map<std::string_view, TokenKind> kKeywords = {
    {"if", TokenKind::KwIf},
    {"else", TokenKind::KwElse},
    {"for", TokenKind::KwFor},
    {"while", TokenKind::KwWhile},
    {"do", TokenKind::KwDo},
    {"break", TokenKind::KwBreak},
    {"continue", TokenKind::KwContinue},
    {"or", TokenKind::KwOr},
    {"and", TokenKind::KwAnd},
    {"not", TokenKind::KwNot},
    {"vector", TokenKind::KwVector},
    {"normal", TokenKind::KwNormal},
    {"point", TokenKind::KwPoint},
    {"matrix", TokenKind::KwMatrix},
    {"int", TokenKind::KwInt},
    {"float", TokenKind::KwFloat},
    {"string", TokenKind::KwString},
    {"return", TokenKind::KwReturn},
};

bool IsAlpha(char c) {
    const unsigned char uc = static_cast<unsigned char>(c);
    return std::isalpha(uc) != 0;
}

bool IsDigit(char c) {
    const unsigned char uc = static_cast<unsigned char>(c);
    return std::isdigit(uc) != 0;
}

bool IsHexDigit(char c) {
    const unsigned char uc = static_cast<unsigned char>(c);
    return std::isxdigit(uc) != 0;
}

bool IsIdentifierStart(char c) {
    return IsAlpha(c) || c == '_';
}

bool IsIdentifierPart(char c) {
    return IsIdentifierStart(c) || IsDigit(c);
}

} // namespace

Lexer::Lexer(std::string source) : source_(std::move(source)) {}

Token Lexer::NextToken() {
    SkipIgnored();

    if (IsAtEnd()) {
        return Token{TokenKind::EndOfFile, "", line_, column_, 0, 0.0, ""};
    }

    const std::size_t start = index_;
    const int token_line = line_;
    const int token_column = column_;
    const char c = Peek();

    if (IsIdentifierStart(c)) {
        return ScanIdentifierOrKeyword();
    }

    if (IsDigit(c) || (c == '.' && IsDigit(Peek(1)))) {
        return ScanNumber();
    }

    if (c == '"') {
        return ScanString();
    }

    Advance();
    switch (c) {
    case '+': return MakeToken(TokenKind::Plus, start, index_, token_line, token_column);
    case '-': return MakeToken(TokenKind::Minus, start, index_, token_line, token_column);
    case '*': return MakeToken(TokenKind::Star, start, index_, token_line, token_column);
    case '/': return MakeToken(TokenKind::Slash, start, index_, token_line, token_column);
    case '%': return MakeToken(TokenKind::Percent, start, index_, token_line, token_column);
    case ',': return MakeToken(TokenKind::Comma, start, index_, token_line, token_column);
    case '.': return MakeToken(TokenKind::Dot, start, index_, token_line, token_column);
    case ':': return MakeToken(TokenKind::Colon, start, index_, token_line, token_column);
    case ';': return MakeToken(TokenKind::Semi, start, index_, token_line, token_column);
    case '(': return MakeToken(TokenKind::LParen, start, index_, token_line, token_column);
    case ')': return MakeToken(TokenKind::RParen, start, index_, token_line, token_column);
    case '{': return MakeToken(TokenKind::LBrace, start, index_, token_line, token_column);
    case '}': return MakeToken(TokenKind::RBrace, start, index_, token_line, token_column);
    case '[': return MakeToken(TokenKind::LBracket, start, index_, token_line, token_column);
    case ']': return MakeToken(TokenKind::RBracket, start, index_, token_line, token_column);
    case '=':
        if (Match('=')) {
            return MakeToken(TokenKind::EqualEqual, start, index_, token_line, token_column);
        }
        return MakeToken(TokenKind::Assign, start, index_, token_line, token_column);
    case '!':
        if (Match('=')) {
            return MakeToken(TokenKind::BangEqual, start, index_, token_line, token_column);
        }
        return MakeToken(TokenKind::Bang, start, index_, token_line, token_column);
    case '<':
        if (Match('=')) {
            return MakeToken(TokenKind::LessEqual, start, index_, token_line, token_column);
        }
        return MakeToken(TokenKind::Less, start, index_, token_line, token_column);
    case '>':
        if (Match('=')) {
            return MakeToken(TokenKind::GreaterEqual, start, index_, token_line, token_column);
        }
        return MakeToken(TokenKind::Greater, start, index_, token_line, token_column);
    case '&':
        if (Match('&')) {
            return MakeToken(TokenKind::AmpAmp, start, index_, token_line, token_column);
        }
        return MakeInvalidToken(start, index_, token_line, token_column, "Unexpected '&' (did you mean '&&'?)");
    case '|':
        if (Match('|')) {
            return MakeToken(TokenKind::PipePipe, start, index_, token_line, token_column);
        }
        return MakeInvalidToken(start, index_, token_line, token_column, "Unexpected '|' (did you mean '||'?)");
    default:
        return MakeInvalidToken(start, index_, token_line, token_column, "Unexpected character");
    }
}

Token Lexer::MakeToken(TokenKind kind, std::size_t start, std::size_t end, int line, int column) const {
    Token token;
    token.kind = kind;
    token.lexeme = source_.substr(start, end - start);
    token.line = line;
    token.column = column;
    return token;
}

Token Lexer::MakeInvalidToken(std::size_t start, std::size_t end, int line, int column, std::string message) const {
    Token token = MakeToken(TokenKind::Invalid, start, end, line, column);
    token.error_message = std::move(message);
    return token;
}

void Lexer::SkipIgnored() {
    while (!IsAtEnd()) {
        const char c = Peek();

        if (c == '\\' && Peek(1) == '\n') {
            Advance();
            Advance();
            continue;
        }

        if (c == '/' && Peek(1) == '/') {
            SkipLine();
            continue;
        }

        if (c == '#') {
            if (IsPreprocessorLine()) {
                SkipLine();
                continue;
            }
            break;
        }

        if (c == ' ' || c == '\t' || c == '\v' || c == '\f' || c == '\r' || c == '\n') {
            Advance();
            continue;
        }

        break;
    }
}

bool Lexer::IsPreprocessorLine() const {
    if (Peek() != '#') {
        return false;
    }

    std::size_t i = index_;
    while (i > 0 && source_[i - 1] != '\n') {
        --i;
    }
    while (i < index_) {
        const char c = source_[i];
        if (c != ' ' && c != '\t') {
            return false;
        }
        ++i;
    }
    return true;
}

void Lexer::SkipLine() {
    while (!IsAtEnd() && Peek() != '\n') {
        Advance();
    }
    if (!IsAtEnd() && Peek() == '\n') {
        Advance();
    }
}

char Lexer::Peek(std::size_t lookahead) const {
    const std::size_t position = index_ + lookahead;
    if (position >= source_.size()) {
        return '\0';
    }
    return source_[position];
}

bool Lexer::Match(char expected) {
    if (IsAtEnd() || Peek() != expected) {
        return false;
    }
    Advance();
    return true;
}

char Lexer::Advance() {
    if (IsAtEnd()) {
        return '\0';
    }
    const char c = source_[index_++];
    if (c == '\n') {
        ++line_;
        column_ = 1;
    } else {
        ++column_;
    }
    return c;
}

bool Lexer::IsAtEnd() const {
    return index_ >= source_.size();
}

Token Lexer::ScanIdentifierOrKeyword() {
    const std::size_t start = index_;
    const int token_line = line_;
    const int token_column = column_;

    Advance();
    while (IsIdentifierPart(Peek())) {
        Advance();
    }

    Token token = MakeToken(TokenKind::Identifier, start, index_, token_line, token_column);
    const auto it = kKeywords.find(token.lexeme);
    if (it != kKeywords.end()) {
        token.kind = it->second;
    }
    return token;
}

Token Lexer::ScanNumber() {
    const std::size_t start = index_;
    const int token_line = line_;
    const int token_column = column_;

    bool is_float = false;

    if (Peek() == '0' && (Peek(1) == 'x' || Peek(1) == 'X')) {
        Advance();
        Advance();
        if (!IsHexDigit(Peek())) {
            return MakeInvalidToken(start, index_, token_line, token_column, "Malformed hex literal");
        }
        while (IsHexDigit(Peek())) {
            Advance();
        }
        Token token = MakeToken(TokenKind::IntLiteral, start, index_, token_line, token_column);
        token.int_value = std::strtoll(token.lexeme.c_str(), nullptr, 16);
        return token;
    }

    if (Peek() == '.') {
        is_float = true;
        Advance();
        while (IsDigit(Peek())) {
            Advance();
        }
    } else {
        while (IsDigit(Peek())) {
            Advance();
        }

        if (Peek() == '.') {
            is_float = true;
            Advance();
            while (IsDigit(Peek())) {
                Advance();
            }
        }
    }

    if (Peek() == 'e' || Peek() == 'E') {
        is_float = true;
        const std::size_t e_position = index_;
        Advance();
        if (Peek() == '+' || Peek() == '-') {
            Advance();
        }
        if (!IsDigit(Peek())) {
            return MakeInvalidToken(start, index_, token_line, token_column, "Malformed exponent in float literal");
        }
        while (IsDigit(Peek())) {
            Advance();
        }
        (void)e_position;
    }

    Token token = MakeToken(is_float ? TokenKind::FloatLiteral : TokenKind::IntLiteral, start, index_, token_line, token_column);
    if (is_float) {
        token.float_value = std::strtod(token.lexeme.c_str(), nullptr);
    } else {
        token.int_value = std::strtoll(token.lexeme.c_str(), nullptr, 10);
    }
    return token;
}

Token Lexer::ScanString() {
    const std::size_t start = index_;
    const int token_line = line_;
    const int token_column = column_;

    Advance();
    bool escaped = false;
    while (!IsAtEnd()) {
        const char c = Peek();
        if (c == '\n') {
            return MakeInvalidToken(start, index_, token_line, token_column, "Unterminated string literal");
        }
        Advance();
        if (escaped) {
            escaped = false;
            continue;
        }
        if (c == '\\') {
            escaped = true;
        } else if (c == '"') {
            return MakeToken(TokenKind::StringLiteral, start, index_, token_line, token_column);
        }
    }

    return MakeInvalidToken(start, index_, token_line, token_column, "Unterminated string literal");
}

const char *TokenKindName(TokenKind kind) {
    switch (kind) {
    case TokenKind::EndOfFile: return "EndOfFile";
    case TokenKind::Invalid: return "Invalid";
    case TokenKind::Identifier: return "Identifier";
    case TokenKind::IntLiteral: return "IntLiteral";
    case TokenKind::FloatLiteral: return "FloatLiteral";
    case TokenKind::StringLiteral: return "StringLiteral";
    case TokenKind::KwIf: return "KwIf";
    case TokenKind::KwElse: return "KwElse";
    case TokenKind::KwFor: return "KwFor";
    case TokenKind::KwWhile: return "KwWhile";
    case TokenKind::KwDo: return "KwDo";
    case TokenKind::KwBreak: return "KwBreak";
    case TokenKind::KwContinue: return "KwContinue";
    case TokenKind::KwOr: return "KwOr";
    case TokenKind::KwAnd: return "KwAnd";
    case TokenKind::KwNot: return "KwNot";
    case TokenKind::KwVector: return "KwVector";
    case TokenKind::KwNormal: return "KwNormal";
    case TokenKind::KwPoint: return "KwPoint";
    case TokenKind::KwMatrix: return "KwMatrix";
    case TokenKind::KwInt: return "KwInt";
    case TokenKind::KwFloat: return "KwFloat";
    case TokenKind::KwString: return "KwString";
    case TokenKind::KwReturn: return "KwReturn";
    case TokenKind::Plus: return "Plus";
    case TokenKind::Minus: return "Minus";
    case TokenKind::Star: return "Star";
    case TokenKind::Slash: return "Slash";
    case TokenKind::Percent: return "Percent";
    case TokenKind::Assign: return "Assign";
    case TokenKind::EqualEqual: return "EqualEqual";
    case TokenKind::Bang: return "Bang";
    case TokenKind::BangEqual: return "BangEqual";
    case TokenKind::Less: return "Less";
    case TokenKind::LessEqual: return "LessEqual";
    case TokenKind::Greater: return "Greater";
    case TokenKind::GreaterEqual: return "GreaterEqual";
    case TokenKind::AmpAmp: return "AmpAmp";
    case TokenKind::PipePipe: return "PipePipe";
    case TokenKind::Comma: return "Comma";
    case TokenKind::Dot: return "Dot";
    case TokenKind::Colon: return "Colon";
    case TokenKind::Semi: return "Semi";
    case TokenKind::LParen: return "LParen";
    case TokenKind::RParen: return "RParen";
    case TokenKind::LBrace: return "LBrace";
    case TokenKind::RBrace: return "RBrace";
    case TokenKind::LBracket: return "LBracket";
    case TokenKind::RBracket: return "RBracket";
    }
    return "Unknown";
}

} // namespace orlcomp
