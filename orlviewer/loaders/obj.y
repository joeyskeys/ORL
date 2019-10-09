%{
#include "mesh.h"

#include <string>
%}

%union
{
    int             i;
    float           f;
    glm::vec2      v2;
    glm::vec3      v3;
    glm::vec4      v4;
    glm::uvec3    uv3;
    glm::uvec4    uv4;
    void           *n;
    const char     *c;
}

%locations

%parse-param { Mesh *mesh }

// Define terminal symbols.
%token <s>      IDENTIFIER STRING_LITERAL
%token <i>      INT_LITERAL
%token <f>      FLOAT_LITERAL
%token <i>      VERTEX UV NORMAL FACE

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
    ;

vertex_line
    : VERTEX vertex_coordinates
        {
            mesh->addVertex($2);
            $$ = 0;
        }
    ;

vertex_coordinates
    : coordinate_component coordinate_component coordinate_component optional_coordinate_component
        {
            $$ = new glm::vec3($1, $2, $3);
        }
    ;

coordinate_component
    : FLOAT_LITERAL
        {
            $$ = std::stof($1);
        }
    ;

optional_coordinate_component
    : FLOAT_LITERAL
        {
            $$ = std::stof($1);
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
            $$ = $1;
        }
    | two_component_uv
        {
            $$ = $1;
        }
    | three_component_uv
        {
            $$ = $1;
        }
    ;

one_component_uv
    : coordinate_component
        {
            $$ = new glm::vec2($1, 0);
        }
    ;

two_component_uv
    : coordinate_component coordinate_component
        {
            $$ = new glm::vec2($1, $2);
        }
    ;

three_component_uv
    : coordinate_component coordinate_component coordinate_component
        {
            $$ = new glm::vec2($1, $2);
        }
    ;

normal_line
    : NORMAL normal_coordinates
        {
            mesh->addNormal($2);
            $$ = 0;
        }
    ;

normal_coordinates
    : coordinate_component coordinate_component coordinate_component
        {
            $$ = new glm::vec3($1, $2, $3);
        }
    ;

face_line
    : FACE face_index   { $$ = 0; }
    ;

face_index
    : index_component
        {
            mesh->addIndex($1);
        }
    | index_component '/' extra_component
        {
            mesh->addIndex($1);
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
            $$ = std::stoi($1);
        }
    ;

%%
