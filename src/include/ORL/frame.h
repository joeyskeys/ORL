#pragma once

#include <array>

#include <yavl/yavl.h>

using namespace yavl;

namespace ORL
{

class Frame {
public:
    Frame() {};
    Frame(const Vec3f& xx, const Vec3f& yy, const Vec3f& zz)
        : x(xx)
        , y(yy)
        , z(zz)
    {
        if (x.cross(y) != z)
            right_handed = false;
    }

public:
    // We use a uint32_t type to record the frame axis info
    // Only the lower 12 bits is used: 0x 0 Z Y X
    // Each four bits records a index indicating the direction of
    // cooresponding axis

    constexpr static uint32_t axis_x_mask = 0x0000 000F;
    constexpr static uint32_t axis_y_mask = 0x0000 00F0;
    constexpr static uint32_t axis_z_mask = 0x0000 0F00;
    constexpr static uint32_t axis_masks[3] = {
        axis_x_mask,
        axis_y_mask,
        axis_z_mask
    };

    template <uint32_t stride>
    inline constexpr static uint32_t get_axis_index(const uint32_t frame_flag) {
        return axis_masks[stride] >> (stride << 2);
    }

    using get_axis_x_index = get_axis_index<0>;
    using get_axis_y_index = get_axis_index<1>;
    using get_axis_z_index = get_axis_index<2>;

    // Index for the six direction of possible axis is encoded
    // in the following graph:
    //
    //              up 0b0100 |    / 0b1000 out
    //                        |   /
    //                        |  /
    //                        | /
    // left 0b0011 ___________|/___________ 0b0010 right
    //                       /|
    //                      / |
    //                     /  |
    //                    /   |
    //         in 0b1001 /    | 0b0101 down
    //
    // A more straightforward explaination will be like this:
    // 0b in/out_bit up/down_bit left/right_bit sign_bit

    constexpr static uint32_t dir_right = 0b0010;
    constexpr static uint32_t dir_left  = 0b0011;
    constexpr static uint32_t dir_up    = 0b0100;
    constexpr static uint32_t dir_down  = 0b0101;
    constexpr static uint32_t dir_out   = 0b1000;
    constexpr static uint32_t dir_in    = 0b1001;
    constexpr static uint32_t orientation_bits_mask = 0b1110;

    #define DIR_RIGHT_B 0010
    #define DIR_LEFT_B  0011
    #define DIR_UP_B    0100
    #define DIR_DOWN_B  0101
    #define DIR_OUT_B   1000
    #define DIR_IN_B    1001

    #define DIR_RIGHT_X 2
    #define DIR_LEFT_X  3
    #define DIR_UP_X    4
    #define DIR_DOWN_X  5
    #define DIR_OUT_X   8
    #define DIR_IN_X    9

    inline constexpr static bool validate_frame(const uint32_t frame_flag) {
        return (get_axis_x_index(frame_flag) | get_axis_y_index(frame_flag) |
            get_axis_z_index(frame_flag)) & orientation_bits_mask == orientation_bits_mask;
    }

    constexpr static Mat4f get_base_change_matrix(const Frame& from, const Frame& to) {
        // The change base matrix is composed by represent to-frame's axes
        // in from-frame
        const auto to_x_axis = get_axis_x_index(to.flag);
        const auto to_y_axis = get_axis_y_index(to.flag);
        const auto to_z_axis = get_axis_z_index(to.flag);
        const std::array<uint32_t, 3> to_axes = {to_x_axis, to_y_axis, to_z_axis};
        const auto from_x_axis = get_axis_x_index(from.flag);
        const auto from_y_axis = get_axis_y_index(from.flag);
        const auto from_z_axis = get_axis_z_index(from.flag);
        const std::array<uint32_t, 3> from_axes = {from_x_axis, from_y_axis, from_z_axis};

        // The matrix to store the result
        Mat4f ret;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                const auto to_axis = to_axes[i];
                const auto from_axis = from_axes[j];
                if (from_axis & orientation_bits_mask == to_axis & orientation_bits_mask) {
                    Vec3f new_axis = 0;
                    new_axis[j] = 1 - 2 * (from_axis & 1) ^ (to_axis & 1);
                    ret[i] = new_axis;
                    break;
                }
            }
        }

        return ret;
    }

public:
    // We take the following frame as the default coordinate system
    // which is the same as OpenGL and Maya
    //
    //              |+y
    //              |
    //              |
    //              |
    //              |___________+x
    //             /
    //            /
    //           /
    //          /
    //         /+z
    //
    bool right_handed = true;

    uint32_t flag = 0x0000 0 DIR_IN_X DIR_UP_X DIR_RIGHT_X;
};

}