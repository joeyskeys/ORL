add_executable(orl_syntax_tests
    array_indexing_tests.cpp
    language_syntax_tests.cpp
)

target_link_libraries(orl_syntax_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_parser
)
