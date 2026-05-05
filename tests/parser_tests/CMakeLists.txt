add_executable(orl_parser_tests
    parser_tests.cpp
)

target_link_libraries(orl_parser_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_lexer
)
