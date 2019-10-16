include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(viewer_headers
    glfw_application.h
    mesh.h)

set(viewer_srcs
    glfw_application.cpp
    mesh.cpp
    main.cpp)

FLEX_BISON(${CMAKE_CURRENT_SOURCE_DIR}/loaders/objlex.l ${CMAKE_CURRENT_SOURCE_DIR}/loaders/objgram.y obj viewer_srcs viewer_headers)

add_executable(orlviewer
    ${viewer_headers}
    ${viewer_srcs})

target_link_libraries(orlviewer
    general
        ${GLFW_LIBRARIES}
        ${Vulkan_LIBRARIES})
