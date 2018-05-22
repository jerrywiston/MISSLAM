#include <cstdio>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "PinholeCamera.h"
#include "Utils.h"
#include "Initializer.h"
#include "Map.h"
#include <Eigen/Eigen>
#include "Tracker.h"
#define DATA_PATH "../data/"

using namespace misslam;

//===========================================
// Camera Matrix
const cv::Mat cameraMat = cam::KinectCamera.cvCameraMatrix();
//===========================================

int main(){
    map::GlobalState globalState;

    tracker::ORBTracker otracker(cameraMat);
    otracker.track(cv::Mat(), globalState);
    for(int i=0; i<500; ++i){
        // Read Image
        printf("<Frame %s / %s>\n", utils::Zfill(i,3).c_str(), utils::Zfill(i+1,3).c_str());
        std::string fn1 = std::string(DATA_PATH) + utils::Zfill(i,4) + ".png";
        std::string fn2 = std::string(DATA_PATH) + utils::Zfill(i+1,4) + ".png";
        const cv::Mat img1 = cv::imread(fn1, 0);
        const cv::Mat img2 = cv::imread(fn2, 0);

        // Check Initialize
        if(globalState.structure.size() <= 0){    
            // Initial structure
            map::KeyFrameNode kf1, kf2;
            std::vector<map::StructurePoint> initStruct;
            real rate = init::epipolar::InitStructByEssential(img1, img2, cameraMat, kf1, kf2, initStruct);
            std::cout << rate << std::endl;

            // Insert Keyframes
            if(rate > 0.9){
                globalState.structure.insert(
                    std::end(globalState.structure), 
                    std::begin(initStruct), 
                    std::end(initStruct));
                globalState.frameGraph.push_back(kf1);
                globalState.frameGraph.push_back(kf2);
            }
        }

        // Show Window
        std::cout << globalState.structure.size() << std::endl << std::endl;
        cv::waitKey(20);
        return 0;
    }
    return 0;
}