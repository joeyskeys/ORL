add_library(orlgraph
    graph_ids.cpp
    graph_ids.hpp
    graph_types.cpp
    graph_types.hpp
    graph_effects.hpp
    graph_ir.cpp
    graph_ir.hpp
    graph_schedule.cpp
    graph_schedule.hpp
    graph_validation.cpp
    graph_validation.hpp
    orlgraph.hpp
    graph_optimizer.cpp
    graph_optimizer.hpp
    graph_reflection.cpp
    graph_reflection.hpp
)

target_include_directories(orlgraph
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

target_compile_features(orlgraph PUBLIC cxx_std_20)

find_package(RapidJSON CONFIG QUIET)
if(TARGET RapidJSON)
    add_library(orlgraph_io
        graph_serialization.cpp
        graph_serialization.hpp
    )
    target_include_directories(orlgraph_io
        PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(orlgraph_io
        PUBLIC
            orlgraph
            RapidJSON
    )
    target_compile_features(orlgraph_io PUBLIC cxx_std_20)
endif()
