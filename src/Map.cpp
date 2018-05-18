#include "Map.h"

namespace misslam
{
namespace map
{
    StructurePoint::StructurePoint(const cv::Mat &mat) {
        assert(mat.rows == 1 && mat.cols == sizeof(descriptor));
        std::memcpy(descriptor.data, mat.ptr<u8>(), sizeof(descriptor));
    }

    StructurePoint::operator cv::Mat() const
    {
        using rettype = decltype(descriptor);

        cv::Mat ret(1, sizeof(descriptor), CV_8UC1, const_cast<rettype *>(&descriptor));
        return ret.clone();
    }
}
}