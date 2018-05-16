#include <cstdio>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "PinholeCamera.h"
#include "Utils.h"
#include "VisualOdometry.h"
#include "Map.h"
#include <Eigen/Eigen>

#define DATA_PATH "../data/"

using namespace misslam;

//===========================================
// Camera Matrix
const cv::Mat cameraMat = cam::KinectCamera.cvCameraMatrix();
//===========================================

int main(){
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

    Matrix3 m = {1, 2, 3};
    misslam::real temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
    cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
    auto candidate = [&M1] (cv::Mat R, cv::Mat T,  cv::Mat qpts, cv::Mat tpts) -> u32 {
        cv::Mat M2 = utils::ExtrinsicMatrixByRT(R,T);
        u32 passed = 0;
        for (i32 i=0; i<100; i++) {
            passed += static_cast<u32>(vo::Triangulate1Point(M1, M2, qpts, tpts).z > 0);
        }
        return passed;
    };

    for(int i=0; i<500; ++i){
        i = 0;
        // Read Image
        std::string fn1 = std::string(DATA_PATH) + utils::Zfill(i,4) + ".png";
        std::string fn2 = std::string(DATA_PATH) + utils::Zfill(i+3,4) + ".png";
        cv::Mat img1 = cv::imread(fn1, 0);
        cv::Mat img2 = cv::imread(fn2, 0);
        
        // Initialization
        // [Extract ORB Features]
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat dp1, dp2;
        orb->detectAndCompute(img1, cv::Mat(), kp1, dp1);
        orb->detectAndCompute(img2, cv::Mat(), kp2, dp2);
        std::vector<cv::DMatch> matches = utils::ORBMatch(dp1, dp2);

        printf("<Frame %s / %s> Matches: %d\n", utils::Zfill(i,3).c_str(), utils::Zfill(i+1,3).c_str(), (int)matches.size());
        cv::Mat img_matches;
        cv::drawMatches(img1, kp1, img2, kp2, matches, img_matches);

        // [Recover Essential Matrix]
        cv::Mat funMat = vo::GetFundamentalMatrix(matches, kp1, kp2);
        cv::Mat essMat = cameraMat.t() * funMat * cameraMat;
        //cout << essMat << endl;



        // [Transform Extraction]
        cv::Mat R, T;
        vo::ExtractRT(essMat, R, T);
        //cout << R << endl;
        //cout << T << endl;

        // voting
        std::vector<Point2> q, t;
        utils::ArrangeMatchPoints(kp1, kp2, matches, q, t);
        utils::ToNormalizedSpace(cameraMat, q);
        utils::ToNormalizedSpace(cameraMat, t);
        utils::Voter voter;
        
        auto func1 = std::bind(candidate, R, T, cv::Mat(100, 2, CV_REAL_C2, q.data()), cv::Mat(100, 2, CV_REAL_C2, t.data()));
        auto func2 = std::bind(candidate, -R, T, cv::Mat(100, 2, CV_REAL_C2, q.data()), cv::Mat(100, 2, CV_REAL_C2, t.data()));
        auto func3 = std::bind(candidate, -R, -T, cv::Mat(100, 2, CV_REAL_C2, q.data()), cv::Mat(100, 2, CV_REAL_C2, t.data()));
        auto func4 = std::bind(candidate, R, T, cv::Mat(100, 2, CV_REAL_C2, q.data()), cv::Mat(100, 2, CV_REAL_C2, t.data()));
        voter.push(func1);
        voter.push(func2);
        voter.push(func3);
        voter.push(func4);
        auto result = voter.elect();
        printf("result: %d %d\n", result.idx, result.score);
        std::cout<<-R<<std::endl;

        // [Triangulate 3D Structure]
        
        cv::Mat M2 = utils::ExtrinsicMatrixByRT(R,T);
        std::vector<Point3> points3d;
        std::vector<i32> idx1 (kp1.size(), -1);
        std::vector<i32> idx2 (kp2.size(), -1);
        /* 
        vo::InitialStructure(
            M1, M2, cameraMat, cameraMat, matches, 
            kp1, kp2, idx1, idx2, points3d);
        */
        for(i32 i=0; i<points3d.size(); i++)
            std::cout << points3d[i] << std::endl;
        
        // [Insert Keyframes]

        // Show Window
        std::cout << std::endl;
        cv::imshow("test", img_matches);
        cv::waitKey(20);
    }
    return 0;
}