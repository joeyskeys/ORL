include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(viewer_headers
    glfw_application.h
    mesh.h)

set(viewer_srcs
    glfw_application.cpp
    main.cpp)

set(loader_headers
    mesh.h)

set(loader_srcs)

FLEX_BISON(${CMAKE_CURRENT_SOURCE_DIR}/loaders/objlex.l ${CMAKE_CURRENT_SOURCE_DIR}/loaders/objgram.y obj loader_srcs loader_headers)

add_library(loader
    ${loader_headers}
    ${loader_srcs})

add_executable(orlviewer
    ${viewer_headers}
    ${viewer_srcs})

target_link_libraries(orlviewer
    general
        loader
        ${GLFW_LIBRARIES}
        ${Vulkan_LIBRARIES})
