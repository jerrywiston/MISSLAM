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
    
    for(int i=0; i<500; ++i){
        //i = 0;
        // Read Image
        std::string fn1 = std::string(DATA_PATH) + utils::Zfill(i,4) + ".png";
        std::string fn2 = std::string(DATA_PATH) + utils::Zfill(i+1,4) + ".png";
        const cv::Mat img1 = cv::imread(fn1, 0);
        const cv::Mat img2 = cv::imread(fn2, 0);
        
        vo::InitialStructure(img1, img2, cameraMat);
        

        // [Insert Keyframes]

        // Show Window
        std::cout << std::endl;
        //
        cv::waitKey(20);
    }
    return 0;
}