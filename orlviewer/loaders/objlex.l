%option noyywrap
%option nounput
%option noinput
%option prefix="obj"

/* white space, not counting newline */
WHITE           [ \t\v\f\r]+
/* alpha character */
ALPHA           [A-Za-z]
/* numerals */
DIGIT           [0-9]
/* integer literal */
INTEGER         {DIGIT}+
/* floating point literal */
E               [eE][-+]?{DIGIT}+
FLT1            {DIGIT}+\.{DIGIT}*{E}?
FLT2            {DIGIT}*\.{DIGIT}+{E}?
FLT3            {DIGIT}+{E}
FLT             {FLT1}|{FLT2}|{FLT3}

STR             ({ALPHA}|[_])({ALPHA}|{DIGIT}|[_.])*

COMMENT         #.*\n

%{
#include "objgram.hpp"

#include <string>

#define yylval objlval
#define yylloc objlloc

#define SETLINE yylloc.first_line = yylloc.last_line = yylineno
%}

%%

 /* test */
{COMMENT}       { SETLINE; }

{WHITE}         { }

"o"             { SETLINE; return (yylval.i = OBJECT); }
"v"             { SETLINE; return (yylval.i = VERTEX); }
"vt"            { SETLINE; return (yylval.i = UV); }
"vn"            { SETLINE; return (yylval.i = NORMAL); }
"f"             { SETLINE; return (yylval.i = FACE); }

"mtllib"        { SETLINE; return (yylval.i = MATERIAL); }
"usemtl"        { SETLINE; return (yylval.i = USEMAT); }
"s"             { SETLINE; return (yylval.i = SMOOTH); }
"on"            { SETLINE; return (yylval.i = ON); }
"off"           { SETLINE; return (yylval.i = OFF); }

{INTEGER}       { SETLINE;
                  long long lval = std::stoll(yytext);
                  // todo : need overflow handling here
                  yylval.i = static_cast<int>(lval);
                  return INT_LITERAL; }

{FLT}           { SETLINE;            
                  yylval.f = std::stof(yytext);
                  return FLOAT_LITERAL; }

{STR}           { SETLINE;
                  std::string s(yytext);
                  yylval.s = s.c_str();
                  return STRING_LITERAL; }

"/"             { SETLINE; return (yylval.i = SEP); }
"\\\n"  |
[\n]            { SETLINE; return (yylval.i = LINESEP); }

 /* default rule for other single characters */
.               { SETLINE; return (yylval.i = *yytext); }

%%

bool obj_parse_buffer(const std::string obj_buffer, Mesh *m)
{
    extern int objdebug;
    objdebug = 1;
    obj_switch_to_buffer(obj_scan_string(obj_buffer.c_str()));
    objparse(m);
    bool parse_ret = true;
    obj_delete_buffer(YY_CURRENT_BUFFER);

    return parse_ret;
}
