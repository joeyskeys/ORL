include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(oslviewer
    main.cpp
    glfw_application.cpp)

target_link_libraries(oslviewer
    general
        ${GLFW_LIBRARIES}
        ${Vulkan_LIBRARIES})