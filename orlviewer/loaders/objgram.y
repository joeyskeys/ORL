%{
#define YYDEBUG 1
%}

%code requires{
#include "mesh.h"

bool obj_parse_buffer(const std::string obj_buffer, Mesh *m);
}

%code{
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <iostream>

#undef yylex
#define yylex objlex
extern int objlex();
extern FILE *yyin;
void yyerror(Mesh *m, const char *s);
}

%union
{
    int                  i;
    float                f;
    float            v2[2];
    float            v3[3];
    float            v4[4];
    unsigned int    uv3[3];
    unsigned int    uv4[4];
    void                *n;
    const char          *c;
}

%locations

%parse-param { Mesh *mesh }

// Define terminal symbols.
%token <s>      IDENTIFIER STRING_LITERAL
%token <i>      INT_LITERAL
%token <f>      FLOAT_LITERAL
%token <i>      OBJECT VERTEX UV NORMAL FACE
%token <i>      MATERIAL USEMAT SMOOTH
%token <i>      ILLEGAL

// Define the nonterminals.
%type <n>       obj_file
%type <n>       data_lines
%type <n>       data_line
%type <f>       coordinate_component
%type <f>       optional_coordinate_component
%type <n>       vertex_line
%type <v3>      vertex_coordinates
%type <n>       uv_line
%type <v2>      uv_coordinates
%type <v2>      one_component_uv
%type <v2>      two_component_uv
%type <v2>      three_component_uv
%type <n>       normal_line
%type <v3>      normal_coordinates
%type <n>       face_line
%type <n>       face_index
%type <i>       index_component
%type <i>       vert_uv_part
%type <i>       vert_uv_norm_part
%type <i>       vert_null_norm_part
%type <i>       extra_component

// Define precedence of operator.
%right <i>      SEP
%left  <i>      LINESEP

%%

/* Syntax */
obj_file
    : data_lines    { $$ = 0; }
    ;

data_lines
    : data_line
    | data_lines LINESEP data_line  { $$ = 0; }
    ;

data_line
    : vertex_line   { $$ = 0; }
    | uv_line       { $$ = 0; }
    | normal_line   { $$ = 0; }
    | face_line     { $$ = 0; }
    | LINESEP       { $$ = 0; }
    ;

vertex_line
    : VERTEX vertex_coordinates
        {
            mesh->addVertex(glm::make_vec3($2));
            $$ = 0;
        }
    ;

vertex_coordinates
    : coordinate_component coordinate_component coordinate_component optional_coordinate_component
        {
            $$[0] = $1;
            $$[1] = $2;
            $$[2] = $3;
        }
    ;

coordinate_component
    : FLOAT_LITERAL
        {
            $$ = $1;
        }
    ;

optional_coordinate_component
    : FLOAT_LITERAL
        {
            $$ = $1;
        }
    |
        {
            $$ = 0.f;
        }
    ;

uv_line
    : UV uv_coordinates
        {
            $$ = 0;
        }
    ;

uv_coordinates
    : one_component_uv
        {
            $$[0] = $1[0];
            $$[1] = $1[1];
        }
    | two_component_uv
        {
            $$[0] = $1[0];
            $$[1] = $1[1];
        }
    | three_component_uv
        {
            $$[0] = $1[0];
            $$[1] = $1[1];
        }
    ;

one_component_uv
    : coordinate_component
        {
            $$[0] = $1;
            $$[1] = 0;
        }
    ;

two_component_uv
    : coordinate_component coordinate_component
        {
            $$[0] = $1;
            $$[1] = $2;
        }
    ;

three_component_uv
    : coordinate_component coordinate_component coordinate_component
        {
            $$[0] = $1;
            $$[1] = $2;
        }
    ;

normal_line
    : NORMAL normal_coordinates
        {
            mesh->addNormal(glm::make_vec3($2));
            $$ = 0;
        }
    ;

normal_coordinates
    : coordinate_component coordinate_component coordinate_component
        {
            $$[0] = $1;
            $$[1] = $2;
            $$[2] = $3;
        }
    ;

face_line
    : FACE face_index   { $$ = 0; }
    ;

face_index
    : index_component
        {
            mesh->addIndex($1);
            $$ = 0;
        }
    | index_component '/' extra_component
        {
            mesh->addIndex($1);
            $$ = 0;
        }
    ;

extra_component
    : vert_uv_part
        {
            $$ = $1;
        }
    | vert_uv_norm_part
        {
            $$ = $1;
        }
    | vert_null_norm_part
        {
            $$ = $1;
        }
    ;

vert_uv_part
    : index_component
        {
            $$ = $1;
        }
    ;

vert_uv_norm_part
    : index_component '/' index_component
        {
            $$ = $1;
        }
    ;

vert_null_norm_part
    : '/' index_component
        {
            $$ = $2;
        }
    ;

index_component
    : INT_LITERAL
        {
            $$ = $1;
        }
    ;

%%

void yyerror(Mesh *m, const char *s)
{
    std::cout << "Error. " << s << std::endl;
    exit(-1);
}
