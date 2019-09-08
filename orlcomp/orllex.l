%option noyywrap
%option c++

WHITE			[ \t\v\f\r]+
ALPHA			[A-Za-z]
DIGIT			[0-9]
INTEGER			{DIGIT}+

%{
#include <iostream>
#include <fstream>
using namespace std;
%}

%%

{INTEGER}	cout << "int " << YYText() << endl;
{DIGIT}		cout << "digit " << YYText() << endl;
{ALPHA}		cout << "alpha " << YYText() << endl;

%%

/*
int main()
{
	ifstream fs("test.orl", ifstream::in);
	FlexLexer *lexer = new yyFlexLexer(fs, cout);

	lexer->yylex();

	return 0;
}
*/