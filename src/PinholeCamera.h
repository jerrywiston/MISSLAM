#ifndef _MISSLAM_PINHOLECAMERA_H_
#define _MISSLAM_PINHOLECAMERA_H_

#include <array>
#include <opencv2/opencv.hpp>
#include "Config.h"

namespace misslam {
    namespace cam {
        struct PinholeCamera { 
        public:
            const float fx, fy;
            const float cx, cy;
            const std::array<real, 9> cameraMatrix;
            constexpr PinholeCamera(float fx, float fy, float cx, float cy)
                : fx(fx), fy(fy), cx(cx), cy(cy), cameraMatrix({ fx, 0, cx, 0, fy, cy, 0, 0, 1 })
            {
            }
            const cv::Mat cvCameraMatrix() const;
        };
        static constexpr PinholeCamera KinectCamera = PinholeCamera(525.0_r, 525.0_r, 319.5_r, 239.5_r);
    }
}


#endif