#include "Tracker.h"
#include "Utils.h"

namespace misslam
{
namespace tracker
{

static void ArrangeMatchPoints3D(const std::vector<cvPoint3> &objpoints,
    const std::vector<cv::KeyPoint> &kp,
    const std::vector<cv::DMatch> &matches,
    std::vector<cvPoint3> &query, std::vector<cvPoint2> &train)
{
    query.resize(matches.size());
    train.resize(matches.size());
    
    for(u32 i=0; i<matches.size(); i++) {
        query[i] = objpoints[matches[i].queryIdx];
        train[i] = kp[matches[i].trainIdx].pt;
    }
}

Tracker::Tracker(const cv::Mat &cameraMat)
    : cameraMat(cameraMat), R(1._r), T()
{
    
}

Matrix3 Tracker::getR(){
    return R;
}

Vector3 Tracker::getT(){
    return T;
}
/*
template <>
TrackStatus TIndirectTracker<cv::ORB>::track(const cv::Mat &img, map::GlobalState &gstate){
    // Extract ORB
    auto detector = cv::ORB::create();
    std::vector<cv::KeyPoint> kp;
    cv::Mat dp, mask;
    detector->detectAndCompute(img, mask, kp, dp);
    //cv::Mat dp_map;
    std::vector<cvPoint3> structure;
    std::vector<map::ORBPointDescriptor> dp_map_data;
    structure.reserve(gstate.structure.size());
    dp_map_data.reserve(gstate.structure.size());
    for(u32 i=0; i<gstate.structure.size(); i++) {
        dp_map_data.push_back(gstate.structure[i].descriptor);
        structure.push_back(gstate.structure[i].point);
    }

    cv::Mat dp_map(gstate.structure.size(), sizeof(map::ORBPointDescriptor), CV_8UC1, dp_map_data.data());
    cv::Mat st_map(gstate.structure.size(), 3, CV_REAL_C1, structure.data());
    std::vector<cv::DMatch> matches = utils::FeatureMatch(dp_map, dp);
    std::cout << matches.size() << std::endl;
    // Replace old descriptor
    for(int i=0; i<matches.size(); ++i)
        gstate.structure[matches[i].queryIdx].descriptor = dp.row(matches[i].trainIdx);
    
    std::vector<cvPoint3> q;
    std::vector<cvPoint2> t;
    ArrangeMatchPoints3D(structure, kp, matches, q, t);
        
    cv::Mat Rcv, Tcv;
    cv::Mat RR;
    cv::Mat rmask;
    cv::solvePnPRansac(q, t, cameraMat, cv::Mat(), Rcv, Tcv, false, 100, 8.0, 0.98999, rmask);
    cv::Rodrigues(Rcv, RR);
    
    RR.convertTo(RR, CV_REAL);
    R = Matrix3(RR);
    T = {(real)Tcv.at<double>(0), (real)Tcv.at<double>(1), (real)Tcv.at<double>(2)};
    //std::cout << Rcv.type() << std::endl;
    //std::cout << "GG " << Tcv.at<float>(0) << " " <<  Tcv.at<float>(1) << " " << Tcv.at<float>(2) << std::endl;
    //std::cout << "GG " << Tcv.at<double>(0) << " " <<  Tcv.at<double>(1) << " " << Tcv.at<double>(2) << std::endl;
    //std::cout << Tcv << std::endl;
    //std::cout << T << std::endl;
    return TrackStatus::eFail;
}
*/
template <>
TrackStatus TIndirectTracker<cv::xfeatures2d::SIFT>::track(const cv::Mat &img, map::GlobalState &gstate){
    // Extract ORB
    auto detector = cv::xfeatures2d::SIFT::create();
    std::vector<cv::KeyPoint> kp;
    cv::Mat dp, mask;
    detector->detectAndCompute(img, mask, kp, dp);
    //cv::Mat dp_map;
    std::vector<cvPoint3> structure;
    std::vector<map::SIFTPointDescriptor> dp_map_data;
    structure.reserve(gstate.structure.size());
    dp_map_data.reserve(gstate.structure.size());
    for(u32 i=0; i<gstate.structure.size(); i++) {
        dp_map_data.push_back(gstate.structure[i].descriptor);
        structure.push_back(gstate.structure[i].point);
    }

    cv::Mat dp_map(gstate.structure.size(), sizeof(map::SIFTPointDescriptor), CV_8UC1, dp_map_data.data());
    cv::Mat st_map(gstate.structure.size(), 3, CV_REAL_C1, structure.data());
    std::vector<cv::DMatch> matches = utils::FeatureMatch(dp_map, dp);
    std::cout << matches.size() << std::endl;
    // Replace old descriptor
    for(int i=0; i<matches.size(); ++i)
        gstate.structure[matches[i].queryIdx].descriptor = dp.row(matches[i].trainIdx);
    
    std::vector<cvPoint3> q;
    std::vector<cvPoint2> t;
    ArrangeMatchPoints3D(structure, kp, matches, q, t);
        
    cv::Mat Rcv, Tcv;
    cv::Mat RR;
    cv::Mat rmask;
    cv::solvePnPRansac(q, t, cameraMat, cv::Mat(), Rcv, Tcv, false, 100, 8.0, 0.98999, rmask);
    cv::Rodrigues(Rcv, RR);
    
    RR.convertTo(RR, CV_REAL);
    R = Matrix3(RR);
    T = {(real)Tcv.at<double>(0), (real)Tcv.at<double>(1), (real)Tcv.at<double>(2)};
    //std::cout << Rcv.type() << std::endl;
    //std::cout << "GG " << Tcv.at<float>(0) << " " <<  Tcv.at<float>(1) << " " << Tcv.at<float>(2) << std::endl;
    //std::cout << "GG " << Tcv.at<double>(0) << " " <<  Tcv.at<double>(1) << " " << Tcv.at<double>(2) << std::endl;
    //std::cout << Tcv << std::endl;
    //std::cout << T << std::endl;
    return TrackStatus::eFail;
}


}
}