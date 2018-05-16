#ifndef _MISSLAM_MAPS_H_
#define _MISSLAM_MAPS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Eigen>
#include "Config.h"
#include "Map.h"

namespace misslam {

namespace map {
struct KeyFrameNode {
    int KeyFrameId;
    std::vector<Point2> keyPoints;
    cv::Mat descriptor;
    /** @brief  */
    cv::Mat mask;
    std::vector<int> keyPointIndices;
};

struct GlobalState {
    std::vector<Eigen::Vector3f> structure;
    std::vector<KeyFrameNode> frameGraph;
};

}
}

#endif
