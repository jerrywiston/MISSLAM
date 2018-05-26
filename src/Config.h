#ifndef _MISSLAM_CONFIG_H_
#define _MISSLAM_CONFIG_H_
//define USE_DOUBLE
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "Math.h"

namespace misslam {

    using u8 = uint8_t;
    using u32 = uint32_t;
    using i32 = int32_t;

#if defined(USE_DOUBLE)
    using real = double;
    using cvPoint3 = cv::Point3d;
    using cvPoint2 = cv::Point2d;
    using Vector2 = Vector2d;
    using Vector3 = Vector3d;
    using Vector4 = Vector4d;
    using Matrix3 = Matrix3d;
    using Matrix4 = Matrix4d;
    #define CV_REAL CV_64F
    #define CV_REAL_C1 CV_64FC1
    #define CV_REAL_C2 CV_64FC2
#else
    using real = float;
    using cvPoint3 = cv::Point3f;
    using cvPoint2 = cv::Point2f;
    using Vector2 = Vector2f;
    using Vector3 = Vector3f;
    using Vector4 = Vector4f;
    using Matrix3 = Matrix3f;
    using Matrix4 = Matrix4f;
    #define CV_REAL CV_32F
    #define CV_REAL_C1 CV_32FC1
    #define CV_REAL_C2 CV_32FC2
    
#endif

using Point3 = Vector3;
using Point2 = Vector2;

static constexpr real operator "" _r(long double val) {
    return static_cast<real>(val);
}

}

#endif