if (NOT TARGET orlexec)
    return()
endif()

add_executable(orl_graph_lowering_tests
    graph_lowering_tests.cpp
)

target_link_libraries(orl_graph_lowering_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlexec
)
