if (NOT TARGET orlcomp_analysis)
    return()
endif()

add_executable(orl_analysis_tests
    analysis_tests.cpp
)

target_link_libraries(orl_analysis_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlcomp_analysis
)
