#include "gtest/gtest.h"

#include "mesh.h"
#include "objgram.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

TEST(ObjParseTests, VertexParse)
{
    Mesh m;
    std::ifstream ifs("/home/joey/Desktop/workspace/repos/self/ORL/orltest/sphere.obj", std::ifstream::in);
    std::string obj_buf;
    if (ifs.good())
    {
        std::ostringstream ss;
        ss << ifs.rdbuf();
        obj_buf = ss.str();
        obj_parse_buffer(obj_buf, &m);
    }

    EXPECT_EQ(m.getVertexAt(0).x, 0.f);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
