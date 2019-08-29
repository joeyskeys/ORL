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
FLT1            DIGIT}+\.{DIGIT}*{E}?
FLT2            {DIGIT}*\.{DIGIT}+{E}?
FLT3            {DIGIT}+{E}

COMMENT         #.*\n

${

%}

%%

/* Lexical matching rules */

/* comments */
{COMMENT}       {}

%%