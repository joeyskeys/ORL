if (NOT TARGET orlrig)
    return()
endif()

add_executable(orl_rig_graph_tests
    rig_graph_tests.cpp
)

target_link_libraries(orl_rig_graph_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlrig
)
