#ifndef _MISSLAM_MAPS_H_
#define _MISSLAM_MAPS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Eigen>
#include "Config.h"

namespace misslam {

namespace map {
struct KeyFrameNode {
    KeyFrameNode(
        int id, 
        std::vector<cv::KeyPoint> kps, 
        cv::Mat desc, 
        std::vector<int> kpids
    ){
        KeyFrameId = id;
        keyPoints = kps;
        descriptor = desc;
        keyPointIndices = kpids;
    };

    int KeyFrameId;
    std::vector<cv::KeyPoint> keyPoints;
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
