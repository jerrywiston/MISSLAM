#include "Map.h"

namespace misslam
{
namespace map
{
    /*
    ORBPointDescriptor::ORBPointDescriptor(const cv::Mat &mat) {
        assert(mat.rows == 1 && mat.cols == sizeof(data));
        std::memcpy(data, mat.ptr<u8>(), sizeof(data));
    }

    ORBPointDescriptor::operator cv::Mat() const
    {
        using rettype = decltype(data);

        cv::Mat ret(1, sizeof(data), CV_8UC1, const_cast<rettype *>(&data));
        return ret.clone();
    }

    SIFTPointDescriptor::SIFTPointDescriptor(const cv::Mat &mat) {
        assert(mat.rows == 1 && mat.cols == sizeof(data));
        std::memcpy(data, mat.ptr<u8>(), sizeof(data));
    }

    SIFTPointDescriptor::operator cv::Mat() const {
        using rettype = decltype(data);

        cv::Mat ret(1, sizeof(data), CV_8UC1, const_cast<rettype *>(&data));
        return ret.clone();
    }*/
}
}