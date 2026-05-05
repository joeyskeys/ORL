#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace orlcomp {

enum class TokenKind : std::uint16_t {
    EndOfFile = 0,
    Invalid,

    Identifier,
    IntLiteral,
    FloatLiteral,
    StringLiteral,

    KwIf,
    KwElse,
    KwFor,
    KwWhile,
    KwDo,
    KwBreak,
    KwContinue,
    KwOr,
    KwAnd,
    KwNot,
    KwVector,
    KwNormal,
    KwPoint,
    KwMatrix,
    KwInt,
    KwFloat,
    KwString,
    KwReturn,

    Plus,
    Minus,
    Star,
    Slash,
    Percent,
    Assign,
    EqualEqual,
    Bang,
    BangEqual,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
    AmpAmp,
    PipePipe,
    Comma,
    Dot,
    Colon,
    Semi,
    LParen,
    RParen,
    LBrace,
    RBrace,
    LBracket,
    RBracket
};

struct Token {
    TokenKind kind = TokenKind::Invalid;
    std::string lexeme;
    int line = 1;
    int column = 1;

    std::int64_t int_value = 0;
    double float_value = 0.0;
    std::string error_message;
};

class Lexer {
public:
    explicit Lexer(std::string source);

    Token NextToken();

private:
    Token MakeToken(TokenKind kind, std::size_t start, std::size_t end, int line, int column) const;
    Token MakeInvalidToken(std::size_t start, std::size_t end, int line, int column, std::string message) const;

    void SkipIgnored();
    bool IsPreprocessorLine() const;
    void SkipLine();

    char Peek(std::size_t lookahead = 0) const;
    bool Match(char expected);
    char Advance();
    bool IsAtEnd() const;

    Token ScanIdentifierOrKeyword();
    Token ScanNumber();
    Token ScanString();

    std::string source_;
    std::size_t index_ = 0;
    int line_ = 1;
    int column_ = 1;
};

const char *TokenKindName(TokenKind kind);

} // namespace orlcomp
