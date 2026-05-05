add_executable(orl_lexer_tests
    lexer_tests.cpp
)

target_link_libraries(orl_lexer_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_lexer
)
