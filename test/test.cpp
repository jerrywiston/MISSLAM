#include <iostream>
#include "../src/Config.h"
#include "../src/Utils.h"
#include "gtest/gtest.h"

using namespace misslam;

TEST(trianglate_point, trianglate_point_test)
{
    Matrix4 M1 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::cout<<utils::Triangulate2View(M1.toMat()(cv::Rect{0, 0, 4, 3}), M1.toMat()(cv::Rect{0, 0, 4, 3}), {}, {});
    EXPECT_EQ(1, 1);
}
