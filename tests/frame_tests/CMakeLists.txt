add_executable(frame_basic_tests frame_basic_tests.cpp)
target_link_libraries(frame_basic_tests PRIVATE Catch2::Catch2WithMain Eigen3::Eigen)
