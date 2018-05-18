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

u32 candidate(const cv::Mat &R, const cv::Mat &T, const std::vector<Point2> &qpts, const std::vector<Point2> &tpts)
{
    misslam::real temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
    cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
    cv::Mat M2 = utils::ExtrinsicMatrixByRT(R, T);

    Matrix4 m1(M1);
    Matrix4 m2(M2);

    u32 passed = 0;
    for (i32 i=0; i<100; i++) {
        auto v = Vector4(vo::Triangulate1Point(M1, M2, qpts[i], tpts[i]), 1.0_r);
        //std::cout<<m1*v<<std::endl;
        //std::cout<<m2*v<<std::endl;
        //break;
        passed += static_cast<u32>((m1*v).z > 0 && (m2*v).z > 0);
    }
    //printf("passed: %d\n", passed);
    
    return passed;
};

int main(){
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

    Matrix3 m = {1, 2, 3};
    misslam::real temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
    cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
    

    for(int i=0; i<500; ++i){
        //i = 0;
        // Read Image
        std::string fn1 = std::string(DATA_PATH) + utils::Zfill(i,4) + ".png";
        std::string fn2 = std::string(DATA_PATH) + utils::Zfill(i+1,4) + ".png";
        cv::Mat img1 = cv::imread(fn1, 0);
        cv::Mat img2 = cv::imread(fn2, 0);
        
        // Initialization
        // [Extract ORB Features]
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat dp1, dp2, mask1, mask2;
        orb->detectAndCompute(img1, mask1, kp1, dp1);
        orb->detectAndCompute(img2, mask2, kp2, dp2);
        std::vector<cv::DMatch> matches = utils::ORBMatch(dp1, dp2);

        std::vector<map::ORBPointDescriptor> mydp;
        utils::ConvertDescriptor(dp1, mydp);

        printf("<Frame %s / %s> Matches: %d\n", utils::Zfill(i,3).c_str(), utils::Zfill(i+1,3).c_str(), (int)matches.size());
        cv::Mat img_matches;
        cv::drawMatches(img1, kp1, img2, kp2, matches, img_matches);

        // [Recover Essential Matrix]
        const cv::Mat funMat = vo::GetFundamentalMatrix(matches, kp1, kp2);
        const cv::Mat essMat = cameraMat.t() * funMat * cameraMat;
        //std::cout << essMat << std::endl;

        // [Transform Extraction]
        cv::Mat R1, R2, T1, T2;
        vo::ExtractRT(essMat, R1, R2, T1, T2);
        //std::cout << R1 << std::endl;
        //std::cout << R2 << std::endl;
        //std::cout << T1.t() << std::endl;
        //std::cout << T2.t() << std::endl;

        // voting
        std::vector<Point2> q, t;
        
        utils::ArrangeMatchPoints(kp1, kp2, matches, q, t);
        utils::ToNormalizedSpace(cameraMat, q, -1);
        utils::ToNormalizedSpace(cameraMat, t, -1);
        
        auto func1 = std::bind(candidate, std::ref(R1), std::ref(T1), std::ref(q), std::ref(t));
        auto func2 = std::bind(candidate, std::ref(R1), std::ref(T2), std::ref(q), std::ref(t));
        auto func3 = std::bind(candidate, std::ref(R2), std::ref(T1), std::ref(q), std::ref(t));
        auto func4 = std::bind(candidate, std::ref(R2), std::ref(T2), std::ref(q), std::ref(t));
        
         // Get Correct RT by election
        cv::Mat R;
        cv::Mat T;
        utils::Voter voter;
        voter.push(func1, [&R, &T, R_=R1, T_=T1] () {R=R_; T=T_;});
        voter.push(func2, [&R, &T, R_=R1, T_=T2] () {R=R_; T=T_;});
        voter.push(func3, [&R, &T, R_=R2, T_=T1] () {R=R_; T=T_;});
        voter.push(func4, [&R, &T, R_=R2, T_=T2] () {R=R_; T=T_;});
        auto result = voter.elect();
        printf("result: %d %d\n", result.idx, result.score);

        // [Triangulate 3D Structure]
        cv::Mat M2 = utils::ExtrinsicMatrixByRT(R,T);
        std::vector<Point3> points3d(matches.size());
        for(i32 i=0;i<matches.size();i++)
            points3d[i] = vo::Triangulate1Point(M1, M2, q[i], t[i]);

        for(i32 i=0; i<points3d.size(); i++)
            std::cout << points3d[i] << std::endl;
        
        // [Insert Keyframes]
        return 0;

        // Show Window
        std::cout << std::endl;
        cv::imshow("test", img_matches);
        cv::waitKey(20);
    }
    return 0;
}