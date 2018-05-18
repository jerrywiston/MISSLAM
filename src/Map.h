#ifndef _MISSLAM_MAPS_H_
#define _MISSLAM_MAPS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>
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
    Matrix4 extrinsic;
};

struct ORBPointDescriptor {
    uint8_t data[32];
    ORBPointDescriptor()=default;
    operator cv::Mat() const;
    ORBPointDescriptor(const cv::Mat &mat);
};

struct StructurePoint {
    Point3 point;
    ORBPointDescriptor descriptor;
};

struct GlobalState {
    std::vector<StructurePoint> structure;
    std::list<KeyFrameNode> frameGraph;
    u32 rootIdx;
};

struct LocalMap {
    std::vector<u32> pidx; 
    std::vector<ORBPointDescriptor> descriptors;
};

}
}

#endif
