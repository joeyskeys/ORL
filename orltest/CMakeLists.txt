add_subdirectory("${CMAKE_SOURCE_DIR}/thirdparty/googletest" "${CMAKE_BINARY_DIR}/test")

include_directories(${CMAKE_SOURCE_DIR}/orlviewer)
include_directories(${CMAKE_BINARY_DIR}/orlviewer)

configure_file(vertex_parse.cpp.in vertex_parse.cpp)

macro(add_gtest TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gmock gtest_main loader)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
endmacro()

#add_executable(
#    obj_parse_tests
#    vertex_parse.cpp)

#target_link_libraries(
#    obj_parse_tests
#    gtest_main
#    mesh)

add_gtest(obj_parse_tests ${CMAKE_CURRENT_BINARY_DIR}/vertex_parse.cpp)

