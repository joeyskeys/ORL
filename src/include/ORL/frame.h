#pragma once

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

    Mat4f get_base_change_matrix(const Frame& from, const Frame& to);

public:
    Vec3f x = { 1, 0, 0 };
    Vec3f y = { 0, 1, 0 };
    Vec3f z = { 0, 0, 1 };

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
    // And other coordinate system is represented using vectors
    bool right_handed = true;
};

}