set(ORLCOMP_SOURCES
    orl_lexer.cpp
    orl_parser.cpp
)

set(ORLCOMP_HEADERS
    orl_lexer.h
    orl_parser.h
)

add_library(orlcomp_lexer
    ${ORLCOMP_HEADERS}
    ${ORLCOMP_SOURCES}
)

target_include_directories(orlcomp_lexer
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

message(STATUS "Configured ORL compiler handwritten lexer+parser")
