include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(viewer_headers
    glfw_application.h
    mesh.h)

set(viewer_srcs
    glfw_application.cpp
    mesh.cpp
    main.cpp)

FLEX_BISON(${CMAKE_CURRENT_SOURCE_DIR}/loaders/obj.l ${CMAKE_CURRENT_SOURCE_DIR}/loaders/obj.y orl viewer_srcs viewer_headers)

add_executable(oslviewer
    ${viewer_headers}
    ${viewer_srcs})

target_link_libraries(oslviewer
    general
        ${GLFW_LIBRARIES}
        ${Vulkan_LIBRARIES})
