%option noyywrap

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

COMMENT         #.*

%{
#define SETLINE yylloc.first_line = yylloc.last_line = yylineno
%}

%%

 /* test */
{COMMENT}       { SETLINE; }

{WHITE}         { }

{FLT}           { SETLINE; return (yylval.i = FLOAT_LITERAL); }
{INTEGER}       { SETLINE; return (yylval.i = INT_LITERAL); }

"v"             { SETLINE; return (yylval.i = VERTEX); }
"vt"            { SETLINE; return (yylval.i = UV); }
"vn"            { SETLINE; return (yylval.i = NORMAL); }
"f"             { SETLINE; return (yylval.i = FACE); }

"/"             { SETLINE; return (yylval.i = SEP); }
"\\\n"  |
[\n]            { SETLINE; return (yylval.i = LINESEP); }

%%
