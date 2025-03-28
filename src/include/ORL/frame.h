#pragma once

#include <array>

#include "ORL/mathutils.h"

namespace ORL
{

class Frame {
public:
    constexpr Frame() {};
    constexpr Frame(const uint32_t dir_x, const uint32_t dir_y, const uint32_t dir_z) {
        if (validate_dirs(dir_x, dir_y, dir_z))
            flag = create_flag(dir_x, dir_y, dir_z);
    }

public:
    // We use a uint32_t type to record the frame axis info
    // Only the lower 12 bits is used: 0x 0 Z Y X
    // Each four bits records a index indicating the direction of
    // cooresponding axis

    static constexpr uint32_t axis_x_mask = 0x0000000F;
    static constexpr uint32_t axis_y_mask = 0x000000F0;
    static constexpr uint32_t axis_z_mask = 0x00000F00;
    static constexpr uint32_t axis_masks[3] = {
        axis_x_mask,
        axis_y_mask,
        axis_z_mask
    };

    template <uint32_t stride>
    inline static constexpr uint32_t get_axis_index(const uint32_t frame_flag) {
        return (frame_flag & axis_masks[stride]) >> (stride << 2);
    }

    static constexpr auto get_axis_x_index = get_axis_index<0>;
    static constexpr auto get_axis_y_index = get_axis_index<1>;
    static constexpr auto get_axis_z_index = get_axis_index<2>;

    // Index for the six direction of possible axis is encoded
    // in the following graph:
    //
    //              up 0b0100 |   / 0b1000 out
    //                        |  /
    //                        | /
    //                        |/
    // left 0b0011 ___________|___________ 0b0010 right
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

    /*
    inline constexpr static bool validate_frame(const uint32_t frame_flag) {
        return ((get_axis_x_index(frame_flag) | get_axis_y_index(frame_flag) |
            get_axis_z_index(frame_flag)) & orientation_bits_mask) == orientation_bits_mask;
    }
    */

    inline constexpr static bool validate_dirs(const uint32_t dir_x, const uint32_t dir_y, const uint32_t dir_z) {
        return ((dir_x | dir_y | dir_z) & orientation_bits_mask) == orientation_bits_mask;
    }

    inline constexpr static uint32_t create_flag(const uint32_t dir_x, const uint32_t dir_y, const uint32_t dir_z) {
        return (dir_z << 8) | (dir_y << 4) | dir_x;
    }

    const static Mat4f get_base_change_matrix(const Frame& from, const Frame& to) {
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
                // calculation below will need negative numbers
                const int to_axis = to_axes[i];
                const int from_axis = from_axes[j];
                if ((from_axis & orientation_bits_mask) == (to_axis & orientation_bits_mask)) {
                    Vec4f new_axis = 0;
                    new_axis[j] = static_cast<Vec3f::ScalarType>(1 - 2 * (from_axis & 1) ^ (to_axis & 1));
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

    uint32_t flag = 0x00000942;
};

constexpr static Frame frame_gl = Frame();
constexpr static Frame frame_dx = Frame(Frame::dir_right, Frame::dir_up, Frame::dir_out);

}