add_subdirectory("${CMAKE_SOURCE_DIR}/thirdparty/googletest" "${CMAKE_BINARY_DIR}/test")

include_directories(${CMAKE_SOURCE_DIR}/orlviewer)
include_directories(${CMAKE_BINARY_DIR}/orlviewer)

message("include ${CMAKE_SOURCE_DIR}/orlviewer")
message("include ${CMAKE_BINARY_DIR}/orlviewer")

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

add_gtest(obj_parse_tests vertex_parse.cpp)

