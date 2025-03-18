#include <catch2/catch_all.hpp>

#include <ORL/frame.h>

using namespace ORL;
using Catch::Approx;

TEST_CASE("frame tests", "[frame]")
{
    SECTION("getting axis")
    {
        Frame frame;

        REQUIRE(Frame::get_axis_x_index(frame.flag) == Frame::dir_right);
        REQUIRE(Frame::get_axis_y_index(frame.flag) == Frame::dir_up);
        REQUIRE(Frame::get_axis_z_index(frame.flag) == Frame::dir_in);
    }

    SECTION("construct frame")
    {
        Frame frame(Frame::dir_in, Frame::dir_right, Frame::dir_up);

        REQUIRE(Frame::get_axis_x_index(frame.flag) == Frame::dir_in);
        REQUIRE(Frame::get_axis_y_index(frame.flag) == Frame::dir_right);
        REQUIRE(Frame::get_axis_z_index(frame.flag) == Frame::dir_up);
    }

    SECTION("compute base change matrix")
    {
        //Frame from(Frame::dir_right, Frame::dir_up, Frame::dir_in);
        //Frame to(Frame::dir_left, Frame::dir_down, Frame::dir_out);
        Frame from = frame_gl;
        Frame to = frame_dx;

        auto base_change_matrix = Frame::get_base_change_matrix(from, to);

        REQUIRE(base_change_matrix[0] == Vec4f(1, 0, 0, 0));
        REQUIRE(base_change_matrix[1] == Vec4f(0, 1, 0, 0));
        REQUIRE(base_change_matrix[2] == Vec4f(0, 0, -1, 0));
        REQUIRE(base_change_matrix[3] == Vec4f(0, 0, 0, 1));
    }
}