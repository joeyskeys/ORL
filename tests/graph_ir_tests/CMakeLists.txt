if (NOT TARGET orlgraph)
    return()
endif()

add_executable(orl_graph_ir_tests
    graph_ir_tests.cpp
)

target_link_libraries(orl_graph_ir_tests
    PRIVATE
        Catch2::Catch2WithMain
        orlgraph
)

if(TARGET orlgraph_io)
    target_link_libraries(orl_graph_ir_tests PRIVATE orlgraph_io)
    target_compile_definitions(orl_graph_ir_tests PRIVATE ORLGRAPH_HAS_IO=1)
endif()
