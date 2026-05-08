set(ORLCOMP_LEXER_SOURCES
    orl_lexer.cpp
)

set(ORLCOMP_LEXER_HEADERS
    orl_lexer.h
)

set(ORLCOMP_PARSER_SOURCES
    orl_parser.cpp
)

set(ORLCOMP_PARSER_HEADERS
    orl_ast.h
    orl_parser.h
)

add_library(orlcomp_lexer
    ${ORLCOMP_LEXER_HEADERS}
    ${ORLCOMP_LEXER_SOURCES}
)

target_include_directories(orlcomp_lexer
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(orlcomp_parser
    ${ORLCOMP_PARSER_HEADERS}
    ${ORLCOMP_PARSER_SOURCES}
)

target_include_directories(orlcomp_parser
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(orlcomp_parser
    PUBLIC orlcomp_lexer
)

message(STATUS "Configured ORL compiler handwritten lexer and parser")
