#include <catch2/catch_all.hpp>

#include "orl_lexer.h"

#include <vector>

using namespace orlcomp;

TEST_CASE("lexer tokenizes a basic ORL snippet", "[orl][lexer]") {
    const std::string src =
        "int main() {\n"
        "    vector v1(1, 0, 0);\n"
        "    return 0;\n"
        "}\n";

    Lexer lexer(src);
    std::vector<TokenKind> kinds;

    for (;;) {
        Token t = lexer.NextToken();
        kinds.push_back(t.kind);
        if (t.kind == TokenKind::EndOfFile) {
            break;
        }
    }

    const std::vector<TokenKind> expected = {
        TokenKind::KwInt, TokenKind::Identifier, TokenKind::LParen, TokenKind::RParen, TokenKind::LBrace,
        TokenKind::KwVector, TokenKind::Identifier, TokenKind::LParen, TokenKind::IntLiteral, TokenKind::Comma,
        TokenKind::IntLiteral, TokenKind::Comma, TokenKind::IntLiteral, TokenKind::RParen, TokenKind::Semi,
        TokenKind::KwReturn, TokenKind::IntLiteral, TokenKind::Semi, TokenKind::RBrace, TokenKind::EndOfFile
    };

    REQUIRE(kinds == expected);
}

TEST_CASE("lexer reports malformed token with minimal error info", "[orl][lexer]") {
    const std::string src = "float x = 1e+;";
    Lexer lexer(src);

    Token token;
    do {
        token = lexer.NextToken();
    } while (token.kind != TokenKind::Invalid && token.kind != TokenKind::EndOfFile);

    REQUIRE(token.kind == TokenKind::Invalid);
    REQUIRE_FALSE(token.error_message.empty());
}
