#ifndef _MISSLAM_CONFIG_H_
#define _MISSLAM_CONFIG_H_

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "Vector.h"

namespace misslam {

    using u32 = uint32_t;
    using i32 = int32_t;

#if USE_DOUBLE
    using real = double;
    using cvPoint3 = cv::Point3d;
    using cvPoint2 = cv::Point2d;
    #define CV_REAL CV_64F
    #define CV_REAL_C1 CV_64FC1
    #define CV_REAL_C2 CV_64FC2
#else
    using real = float;
    using cvPoint3 = cv::Point3f;
    using cvPoint2 = cv::Point2f;
    #define CV_REAL CV_32F
    #define CV_REAL_C1 CV_32FC1
    #define CV_REAL_C2 CV_32FC2
    
#endif

using Point3 = Vector3<real>;
using Point2 = Vector2<real>;

static constexpr real operator "" _r(long double val) {
    return static_cast<real>(val);
}

static std::ostream &operator<<(std::ostream &out, const misslam::Point3 &p) {
    return out << "Point3(" << p.x << ", " << p.y << ", " << p.z << ")";
}

}

#endif