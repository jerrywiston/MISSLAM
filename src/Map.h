#ifndef _MISSLAM_MAPS_H_
#define _MISSLAM_MAPS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <Eigen/Eigen>
#include "Config.h"
#include "Map.h"

namespace misslam {

namespace map {
struct KeyFrameNode {
    int KeyFrameId;
    std::vector<Point2> keyPoints;
    /** @brief  */
    cv::Mat mask;
    std::vector<int> keyPointIndices;
};

template <class T, size_t N>
struct GlobalState {
    std::vector<StructurePoint<T, N>> structure;
    std::vector<KeyFrameNode> frameGraph;
};

struct LocalMap {
    std::vector<u32> pidx; 
    std::vector<std::array<T, N>> descriptors;
};

template <class T, size_t N>
struct StructurePoint {
    Point3 point;
    std::array<T, N> descriptor;
};

}
}

#endif
