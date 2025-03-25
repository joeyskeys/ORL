%{

#include <iostream>
#include <string>

%}

%union
{
	int			i;
	float 		f;
	const char *s;
}

%token <i> INT_LITERAL
%token <f> FLT_LITERAL

%left <i> '+' '-'
%left <i> '*' '/' '%'

%start rig_file

%%

rig_file
	: expressions
	;

lines
	: lines line
	|								{ $$ = 0; }
	;

line
	: expression ';'
	|								{ $$ = 0; }
	;

expression
	: declare_expr
	| assign_expr
	| arithmetic_expr
	| funcion_call_expr