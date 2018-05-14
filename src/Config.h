#ifndef _MISSLAM_CONFIG_H_
#define _MISSLAM_CONFIG_H_

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace misslam {
    using u32 = uint32_t;
    using i32 = int32_t;

#if USE_DOUBLE
    using real = double;
    using Point3 = cv::Point3d;
    #define CV_REAL CV_64F
    #define CV_REAL_C1 CV_64FC1
    #define CV_REAL_C2 CV_64FC2
#else
    using real = float;
    using Point3 = Eigen::Vector3f;
    using Point2 = Eigen::Vector2f;
    #define CV_REAL CV_32F
    #define CV_REAL_C1 CV_32FC1
    #define CV_REAL_C2 CV_32FC2
    
#endif

constexpr real operator "" _r(long double val) {
    return static_cast<real>(val);
}
}

#endif