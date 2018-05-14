#ifndef _MISSLAM_VO_H_
#define _MISSLAM_VO_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Config.h"

namespace misslam {

namespace vo {
    cv::Mat GetFundamentalMatrix(
        std::vector<cv::DMatch> matches, 
        std::vector<cv::KeyPoint> kp1, 
        std::vector<cv::KeyPoint> kp2
    );

    cv::Mat ExtractRT(
        cv::Mat essMat, 
        cv::Mat &R, 
        cv::Mat &T
    );
    
    cv::Mat Triangulate(
        cv::Mat M1, cv::Mat M2, 
        cv::Mat K1, cv::Mat K2,
        std::vector<cv::DMatch> &matches,
        std::vector<cv::KeyPoint> &kp1, std::vector<cv::KeyPoint> &kp2,
        std::vector<Point3> &points3d
    );

    void InitialStructure(
        cv::Mat M1, cv::Mat M2, 
        cv::Mat K1, cv::Mat K2,
        std::vector<cv::DMatch> &matches,
        std::vector<cv::KeyPoint> &kp1, std::vector<cv::KeyPoint> &kp2,
        std::vector<int> &idx1, std::vector<int> &idx2,
        std::vector<Point3> &points3d
    );
}

}

#endif