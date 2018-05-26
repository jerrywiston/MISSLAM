#include <cstdio>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "PinholeCamera.h"
#include "Utils.h"
#include "Initializer.h"
#include "Map.h"
#include <Eigen/Eigen>
#include "Tracker.h"
#include <fstream>
#define DATA_PATH "../data/"

using namespace misslam;

//===========================================
// Camera Matrix
const cv::Mat cameraMat = cam::KinectCamera.cvCameraMatrix();
//===========================================

std::vector<Point3> PointCloudByOptFlow(
    const cv::Mat &img1, const cv::Mat &img2,
    const cv::Mat &M1, const cv::Mat &M2, cv::Mat ess
){
    std::vector<Point3> P;
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(img1, img2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

    const float dist = 100;

    Matrix3 camMat(cameraMat.inv());
    for(int i=0; i<img1.rows; i++){
        for(int j=0; j<img1.cols; j++){
            auto &val = flow.at<cv::Vec2f>(i,j);
            float d = val[0]*val[0] + val[1]*val[1];
            if(d > dist && d < 500){
                //std::cout << val << std::endl;
                Point2 p1 = (camMat * Point3(j,i,1)).xy();
                Point2 p2 = (camMat * Point3(j+val[0],i+val[1],1)).xy();
                cv::Mat a = cv::Mat(Point3(p1, 1)).t() * ess * cv::Mat(Point3(p2, 1));
                //P.push_back(utils::Triangulate2View(M1, M2, p1, p2));
                //std::cout << a << std::endl;
                P.push_back(Point3(j,i,val[0]*val[0] + val[1]*val[1]));
            }
        }
    }
    return P;
}

int main(){
    map::GlobalState globalState;

    //Initialize
    std::string fn1 = std::string(DATA_PATH) + utils::Zfill(0,4) + ".png";
    std::string fn2 = std::string(DATA_PATH) + utils::Zfill(1,4) + ".png";

    const cv::Mat img1 = cv::imread(fn1, 0);
    const cv::Mat img2 = cv::imread(fn2, 0);


    auto sift = cv::xfeatures2d::SIFT::create();
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    sift->detectAndCompute(img1, cv::Mat(), kps, desc);

    map::KeyFrameNode kf1, kf2;
    std::vector<map::StructurePoint> initStruct;
    cv::Mat ess = init::epipolar::InitStructByEssential(img1, img2, cameraMat, kf1, kf2, initStruct);
    std::vector<Point3> P = PointCloudByOptFlow(img1, img2, kf1.extrinsic.toMat()(cv::Rect{0, 0, 4, 3}),
        kf2.extrinsic.toMat()(cv::Rect{0, 0, 4, 3}), ess);

    // Insert Keyframes
    globalState.structure.insert(std::end(globalState.structure), std::begin(initStruct), std::end(initStruct));
    globalState.frameGraph.push_back(kf1);
    globalState.frameGraph.push_back(kf2);
    
    std::fstream file;
    file.open("test.xyz", std::ios::out);
    for(int i=0; i<initStruct.size(); ++i){
        file << initStruct[i].point.x << "\t" << initStruct[i].point.y << "\t" << initStruct[i].point.z << "\n";
    }
    file.close();
    cv::waitKey(0);

    //tracker::ORBTracker otracker(cameraMat);
    /*
    for(int i=2; i<500; ++i){
        // Read Image
        printf("<Frame %s>\n", utils::Zfill(i,3).c_str());
        std::string fn = std::string(DATA_PATH) + utils::Zfill(i,4) + ".png";
        const cv::Mat img = cv::imread(fn, 0);
        
        // Tracking
        otracker.track(img, globalState);
        std::cout << otracker.getR() << std::endl;
        std::cout << otracker.getT() << std::endl;
        
        // Show Window
        cv::namedWindow("Img");
        cv::imshow("Img", img);
        std::cout << globalState.structure.size() << std::endl << std::endl;

        cv::waitKey(0);

    }
    */
    return 0;
}