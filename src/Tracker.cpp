#include "Tracker.h"

namespace misslam
{
namespace tracker
{

Tracker::Tracker(const cv::Mat &cameraMat)
    : cameraMat(cameraMat), R(1._r), T()
{
    
}

}
}