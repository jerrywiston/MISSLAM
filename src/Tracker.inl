#include "Utils.h"

namespace misslam
{
namespace tracker
{

template <class T>
TrackStatus TIndirectTracker<T>::track(const cv::Mat &img, map::GlobalState &gstate){
    // Extract ORB
    /*
    auto detector = T::create();
    std::vector<cv::KeyPoint> kp;
    cv::Mat dp, mask;
    detector->detectAndCompute(img, mask, kp, dp);
    cv::Mat dp_map;
    std::vector<cv::DMatch> matches = utils::FeatureMatch(dp, dp_map);

    // Compute Transform
    cv::Mat mapPoints, imgPoints;
    cv::Mat Rcv, Tcv;
    cv::solvePnPRansac(mapPoints, imgPoints, cameraMat, cv::Mat(), Rcv, Tcv);
    cv::Rodrigues(Rcv, Rcv);
    */
    return TrackStatus::eFail;
}
}
}
