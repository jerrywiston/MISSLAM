#include "PinholeCamera.h"

namespace misslam {
namespace cam {

const cv::Mat PinholeCamera::cvCameraMatrix() const {
    return cv::Mat(3, 3, CV_REAL, const_cast<real *>(cameraMatrix.data()));
}

}
}