#include <catch2/catch_all.hpp>

#include <ORL/frame.h>

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
}