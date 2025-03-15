%option noyywrap
%option c++

WHITE			[ \t\v\f\r]+
ALPHA			[A-Za-z]
DIGIT			[0-9]
INTEGER			{DIGIT}+
HEXINTEGER		0[xX][0-9a-fA-F]+

E 				[eE][-+]?{DIGIT}+
FLT1			{DIGIT}+\.{DIGIT}*{E}?
FLT2			{DIGIT}*\.{DIGIT}+{E}?
FLT3			{DIGIT}+{E}
FLT 			{FLT1}|{FLT2}|{FLT3}

STR				\"(\\.|[^\\"\n])*\"
INDENT			({ALPHA}|[_])({ALPHA}|{DIGIT}|[_])*

CPP				^[ \t]*#.*\n
CPLUSCOMMENT 	\/\/.*\n

%{
void preprocess (const char* yytext);

#define SETLINE yylloc.first_line = yylloc.last_line = oslcompiler->lineno()
%}

%%

 /* preprocess */
{CPP} 				{ preprocess(yytext); SETLINE; }

 /* comments */
{CPLUSCOMMENT} 		{
						// need to inc line number
						SETLINE;
					}

 /* keywords */
"if"				{ SETLINE; return (yylval.i = IF); }
"else"				{ SETLINE; return (yylval.i = ELSE); }
"for"				{ SETLINE; return (yylval.i = FOR); }
"while"				{ SETLINE; return (yylval.i = WHILE); }
"do"				{ SETLINE; return (yylval.i = DO); }
"break"				{ SETLINE; return (yylval.i = BREAK); }
"continue"			{ SETLINE; return (yylval.i = CONTINUE); }

"or"				{ SETLINE; return (yylval.i = OR); }
"and"				{ SETLINE; return (yylval.i = AND); }
"not"				{ SETLINE; return (yylval.i = NOT); }

"vector"			{ SETLINE; return (yylval.i = VECTOR); }
"normal"			{ SETLINE; return (yylval.i = NORMAL); }
"point"				{ SETLINE; return (yylval.i = POINT); }
"matrix"			{ SETLINE; return (yylval.i = MATRIX); }
"int"				{ SETLINE; return (yylval.i = INT); }
"float"				{ SETLINE; return (yylval.i = FLOAT); }
"string"			{ SETLINE; return (yylval.i = STRING); }

"return"			{ SETLINE; return (yylval.i = RETURN); }


{INTEGER}			{ cout << "int " << YYText() << endl; }
{HEXINTEGER}		{}
{FLT}				{}
{STR}				{}

"\\\n"				|
[\n]				{

					}

{WHITE}				{ }

!					{}
.					{}


%%

void preprocess(const char *yytext)
{
	const char *p = yytext;
	while (*p == ' ' || *p == '\t')
		p++;
	if (*p != '#') {
		
	}
}

int main()
{
	ifstream fs("test.orl", ifstream::in);
	FlexLexer *lexer = new yyFlexLexer(fs, cout);

	lexer->yylex();

	return 0;
}