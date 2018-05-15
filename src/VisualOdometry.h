#ifndef _MISSLAM_VO_H_
#define _MISSLAM_VO_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Config.h"

namespace misslam {

namespace vo {
    /*
    * Compute fundamental matrix from matches
    * @param [in] matches vector of DMatch
    * @param [out] kp1 vector of query keypoints
    * @param [out] kp2 vector of train keypoints
    * @return Fundamental matrix
    */
    cv::Mat GetFundamentalMatrix(
        std::vector<cv::DMatch> matches, 
        std::vector<cv::KeyPoint> kp1, 
        std::vector<cv::KeyPoint> kp2
    );

    /*
    * Extract transform from essential matrix
    * @param [in] essMat essential matrix
    * @param [out] R rotation
    * @param [out] T translate
    * @return Singular value matrix
    */
    cv::Mat ExtractRT(
        cv::Mat essMat, 
        cv::Mat &R, 
        cv::Mat &T
    );

    /*
    * Extract transform from essential matrix
    * @param [in] M1 camera 1 extrinsic
    * @param [in] M2 camera 2 extrinsic
    * @param [in]  p1 normalized image coordinate
    * @param [in]  p2 normalized image coordinate 
    * @return position related to world space
    */
    Point3 Triangulate1Point(
        cv::Mat &M1, cv::Mat &M2,
        cv::Mat &p1, cv::Mat &p2
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