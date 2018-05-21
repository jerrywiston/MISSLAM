#ifndef _MISSLAM_VO_H_
#define _MISSLAM_VO_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Map.h"
#include "Config.h"

namespace misslam {

namespace init {

namespace epipolar {
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
    void ExtractRT(
        const cv::Mat essMat, 
        cv::Mat &R1, cv::Mat &R2,
        cv::Mat &T1, cv::Mat &T2
    );

    u32 Candidate(
        const cv::Mat &R, const cv::Mat &T, 
        const std::vector<Point2> &qpts, const std::vector<Point2> &tpts,
        const int &number
    );

    real InitStructByEssential(
        const cv::Mat img1, const cv::Mat img2, const cv::Mat cameraMat,
        map::KeyFrameNode &kf1, map::KeyFrameNode &kf2,
        std::vector<map::StructurePoint> &initStruct
    );
}

}

}

#endif