%{
#define YYDEBUG 1
%}

%code requires{
#include <string>
#include <iostream>
#include <vector>

#include "mesh.h"

bool obj_parse_buffer(const std::string obj_buffer, Mesh *m);
}

%code{
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>

#undef yylex
#define yylex objlex
extern int objlex();
extern FILE *yyin;
void yyerror(Mesh *m, const char *s);

// Temporary vector to record face index list
std::vector<unsigned int> face_index_list;
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
    const char          *s;
}

%locations

%parse-param { Mesh *mesh }

// Define terminal symbols.
%token <s>      STRING_LITERAL
%token <i>      INT_LITERAL
%token <f>      FLOAT_LITERAL
%token <i>      OBJECT VERTEX UV NORMAL FACE
%token <i>      MATERIAL USEMAT SMOOTH ON OFF
%token <i>      ILLEGAL

// Define the nonterminals.
%type <n>       obj_file
%type <n>       data_lines
%type <n>       data_line
%type <f>       coordinate_component
%type <f>       optional_coordinate_component
%type <v3>      vertex_coordinates
%type <v2>      uv_coordinates
%type <v2>      one_component_uv
%type <v2>      two_component_uv
%type <v2>      three_component_uv
%type <v3>      normal_coordinates
%type <n>       face_index_set
%type <i>       face_index
%type <i>       index_component
%type <i>       vert_uv_part
%type <i>       vert_uv_norm_part
%type <i>       vert_null_norm_part
%type <i>       extra_component
%type <s>       file_name

// Define precedence of operator.
%right <i>      SEP
%left  <i>      LINESEP
%right <i>      '-'

%%

/* Syntax */
obj_file
    : data_lines    { $$ = 0; }
    ;

data_lines
    : data_line
    | data_lines data_line  { $$ = 0; }
    ;

data_line
    : OBJECT STRING_LITERAL LINESEP
        {
            // Currently we only handle one mesh situation
            $$ = 0;
        }
    | VERTEX vertex_coordinates LINESEP
        { 
            mesh->addVertex(glm::make_vec3($2));
            $$ = 0;
        }
    | UV uv_coordinates LINESEP
        { 
            $$ = 0;
        }
    | NORMAL normal_coordinates LINESEP
        {
            mesh->addNormal(glm::make_vec3($2));
            $$ = 0;
        }
    | FACE face_index_set LINESEP
        {
            // Split n-gon mesh into trianles if there's any
            if (face_index_list.size() < 3)
                // Todo : error handling here
                ;
            for (int i = 1, end = face_index_list.size(); i < end - 1; i++)
                mesh->addIndex(glm::uvec3(face_index_list[0], face_index_list[i], face_index_list[i + 1]));
            face_index_list.clear();
            $$ = 0;
        }
    | MATERIAL file_name LINESEP
        {
            $$ = 0;
        }
    | USEMAT STRING_LITERAL LINESEP
        {
            $$ = 0;
        }
    | SMOOTH ON LINESEP
        {
            $$ = 0;
        }
    | SMOOTH OFF LINESEP
        {
            $$ = 0;
        }
    | ILLEGAL LINESEP
        {
            // Bypass all illegal input
            $$ = 0;
        }
    | LINESEP
        {
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
    | '-' FLOAT_LITERAL
        {
            $$ = -$2;
        }
    ;

optional_coordinate_component
    : FLOAT_LITERAL
        {
            $$ = $1;
        }
    | '-' FLOAT_LITERAL
        {
            $$ = -$2;
        }
    |
        {
            $$ = 0.f;
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

normal_coordinates
    : coordinate_component coordinate_component coordinate_component
        {
            $$[0] = $1;
            $$[1] = $2;
            $$[2] = $3;
        }
    ;

face_index_set
    : face_index
        {
            face_index_list.emplace_back($1);
            $$ = 0;
        }
    | face_index_set face_index
        {
            face_index_list.emplace_back($2);
            $$ = 0;
        }
    ;

face_index
    : index_component
        {
            $$ = $1;
        }
    | index_component SEP extra_component
        {
            $$ = $1;
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
    : index_component SEP index_component
        {
            $$ = $1;
        }
    ;

vert_null_norm_part
    : SEP index_component
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

file_name
    : STRING_LITERAL
        {
            $$ = std::string($1).c_str();
        }

%%

void yyerror(Mesh *m, const char *s)
{
    std::cout << "Error. " << s << std::endl;
    exit(-1);
}
