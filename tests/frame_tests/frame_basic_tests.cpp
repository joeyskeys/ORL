#include <catch2/catch_all.hpp>

#include <ORL/frame.h>

using namespace ORL;
using Catch::Approx;

TEST_CASE("frame tests", "[frame]")
{
    SECTION("getting axis") {
        Frame frame;

        REQUIRE(Frame::get_axis_x_index(frame.flag) == Frame::dir_right);
        REQUIRE(Frame::get_axis_y_index(frame.flag) == Frame::dir_up);
        REQUIRE(Frame::get_axis_z_index(frame.flag) == Frame::dir_in);
    }

    SECTION("construct frame") {
        Frame frame(Frame::dir_in, Frame::dir_right, Frame::dir_up);

        REQUIRE(Frame::get_axis_x_index(frame.flag) == Frame::dir_in);
        REQUIRE(Frame::get_axis_y_index(frame.flag) == Frame::dir_right);
        REQUIRE(Frame::get_axis_z_index(frame.flag) == Frame::dir_up);
    }

    SECTION("compute base change matrix"){
        Frame from = frame_gl;
        Frame to = frame_dx;

        auto base_change_matrix = Frame::get_base_change_matrix(from, to);

        REQUIRE(base_change_matrix[0] == Vec4f(1, 0, 0, 0));
        REQUIRE(base_change_matrix[1] == Vec4f(0, 1, 0, 0));
        REQUIRE(base_change_matrix[2] == Vec4f(0, 0, -1, 0));
        REQUIRE(base_change_matrix[3] == Vec4f(0, 0, 0, 1));

        Vec4f pt(1, 1, 1, 1);
        Vec4f new_pt = base_change_matrix * pt;
        REQUIRE(new_pt == Vec4f(1, 1, -1, 1));
    }

    SECTION("change of basis for transformations") {
        Frame from = frame_gl;
        Frame to = frame_dx;
        auto base_change_matrix = Frame::get_base_change_matrix(from, to);
        auto inverse_base_change_matrix = base_change_matrix.inverse();
        // We're only considering orthogonal basis so we can get inverse of the basis change
        // matrix by just using the transpose
        auto transpose_base_change_matrix = base_change_matrix.transpose();

        Vec4f pt_in_from = {1, 1, 1, 1};
        Mat4f translation_in_from = translate3f(Vec3f(1, 1, 1));
        Vec4f translated_pt_in_to = base_change_matrix * translation_in_from * pt_in_from;
        Vec4f pt_in_to = base_change_matrix * pt_in_from;

        Mat4f translation_in_to_using_inverse = base_change_matrix * translation_in_from * inverse_base_change_matrix;
        Vec4f new_pt_using_inverse = translation_in_to_using_inverse * pt_in_to;
        REQUIRE(new_pt_using_inverse == translated_pt_in_to);

        Mat4f translation_in_to_using_transpose = base_change_matrix * translation_in_from * transpose_base_change_matrix;
        Vec4f new_pt_using_transpose = translation_in_to_using_transpose * pt_in_to;
        REQUIRE(new_pt_using_transpose == translated_pt_in_to);
    }
}