#ifndef _MISSLAM_CONFIG_H_
#define _MISSLAM_CONFIG_H_

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "Math.h"

namespace misslam {

    using u32 = uint32_t;
    using i32 = int32_t;

#if USE_DOUBLE
    using real = double;
    using cvPoint3 = cv::Point3d;
    using cvPoint2 = cv::Point2d;
    using Vector2 = Vector2d;
    using Vector3 = Vector3d;
    using Matrix3 = Matrix3d;
    #define CV_REAL CV_64F
    #define CV_REAL_C1 CV_64FC1
    #define CV_REAL_C2 CV_64FC2
#else
    using real = float;
    using cvPoint3 = cv::Point3f;
    using cvPoint2 = cv::Point2f;
    using Vector2 = Vector2f;
    using Vector3 = Vector3f;
    using Matrix3 = Matrix3f;
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