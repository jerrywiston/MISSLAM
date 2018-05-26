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
    int keyFrameId;
    std::vector<Point2> keyPoints;
    /** @brief  */
    std::vector<int> keyPointIndices;
    Matrix4 extrinsic;
};

template <size_t N>
struct PointDescriptor {
    uint8_t data[N];
    PointDescriptor()=default;
    PointDescriptor(const PointDescriptor<N> &rhs)=default;
    operator cv::Mat() const;
    PointDescriptor(const cv::Mat &mat);
};
template <size_t N>
PointDescriptor<N>::PointDescriptor(const cv::Mat &mat) {
    assert(mat.rows == 1 && mat.cols == sizeof(data));
    std::memcpy(data, mat.ptr<u8>(), sizeof(data));
}
template <size_t N>
PointDescriptor<N>::operator cv::Mat() const
{
    using rettype = decltype(data);

    cv::Mat ret(1, sizeof(data), CV_8UC1, const_cast<rettype *>(&data));
    return ret.clone();
}

using ORBPointDescriptor = PointDescriptor<32>;
using SIFTPointDescriptor = PointDescriptor<128>;

struct StructurePoint {
    Point3 point;
    SIFTPointDescriptor descriptor;
};

struct GlobalState {
    std::vector<StructurePoint> structure;
    std::list<KeyFrameNode> frameGraph;
    u32 rootIdx;
};

struct LocalMap {
    std::vector<u32> pidx; 
    std::vector<SIFTPointDescriptor> descriptors;
};

}
}

#endif
