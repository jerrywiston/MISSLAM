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
using namespace std;

//===========================================
// Camera Matrix
const cv::Mat cameraMat = cam::KinectCamera.cvCameraMatrix();
//===========================================

int main(){
    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
    for(int i=0; i<500; ++i){
        i = 0;
        // Read Image
        string fn1 = string(DATA_PATH) + utils::Zfill(i,4) + ".png";
        string fn2 = string(DATA_PATH) + utils::Zfill(i+1,4) + ".png";
        cv::Mat img1 = cv::imread(fn1, 0);
        cv::Mat img2 = cv::imread(fn2, 0);
        
        // Initialization
        // [Extract ORB Features]
        cv::ORB orb;
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat dp1, dp2;
        orb(img1, cv::Mat(), kp1, dp1);
        orb(img2, cv::Mat(), kp2, dp2);
        std::vector<cv::DMatch> matches = utils::ORBMatch(dp1, dp2);

        printf("<Frame %s / %s> Matches: %d\n", utils::Zfill(i,3).c_str(), utils::Zfill(i+1,3).c_str(), (int)matches.size());
        cv::Mat img_matches;
        cv::drawMatches(img1, kp1, img2, kp2, matches, img_matches);

        // [Recover Essential Matrix]
        cv::Mat funMat = misslam::vo::GetFundamentalMatrix(matches, kp1, kp2);
        cv::Mat essMat = cameraMat.t() * funMat * cameraMat;
        //cout << essMat << endl;

        // [Transform Extraction]
        cv::Mat R, T;
        misslam::vo::ExtractRT(essMat, R, T);
        //cout << R << endl;
        //cout << T << endl;

        // [Triangulate 3D Structure]
        float temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
        cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
        cv::Mat M2 = utils::ExtrinsicMatrixByRT(R,T);
        std::vector<Point3> points3d;
        std::vector<int> idx1 (kp1.size(), -1);
        std::vector<int> idx2 (kp2.size(), -1);
        misslam::vo::InitialStructure(
            M1, M2, cameraMat, cameraMat, matches, 
            kp1, kp2, idx1, idx2, points3d);
        
        for(int i=0; i<points3d.size(); i++)
            cout << points3d[i] << endl;
        
        // [Insert Keyframes]

        // Show Window
        cout << endl;
        cv::imshow("test", img_matches);
        cv::waitKey(20);
    }
    return 0;
}