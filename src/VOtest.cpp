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
#include "Calibrate.h"
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
  //  std::string fn1 = std::string(DATA_PATH) + utils::Zfill(0,4) + ".png";
//    std::string fn2 = std::string(DATA_PATH) + utils::Zfill(4,4) + ".png";

    std::string fn1 = std::string(DATA_PATH) + "LAB0.png";
    std::string fn2 = std::string(DATA_PATH) + "LAB1.png";


    const cv::Mat img1 = cv::imread(fn1);
    const cv::Mat img2 = cv::imread(fn2);


    auto sift = cv::xfeatures2d::SIFT::create();
    std::vector<cv::KeyPoint> kps1;
    cv::Mat desc1;
    sift->detectAndCompute(img1, cv::Mat(), kps1, desc1);

    std::vector<cv::KeyPoint> kps2;
    cv::Mat desc2;
    sift->detectAndCompute(img2, cv::Mat(), kps2, desc2);
    auto matches = utils::FeatureMatch(desc1, desc2);
    std::vector<Point2> q, t;
    utils::ArrangeMatchPoints(kps1, kps2, matches, q, t);
    auto q_ = utils::ConvertInnerType<cv::Point2d>(q);
    auto t_ = utils::ConvertInnerType<cv::Point2d>(t);

    std::vector<uchar> mask;
    cv::findFundamentalMat(q_, t_, mask, cv::RANSAC);
    cv::Mat matchImg;
    auto msk = utils::ConvertInnerType<char>(mask);
    cv::drawMatches(img1, kps1, img2, kps2, matches, matchImg, cv::Scalar_<double>::all(-1), cv::Scalar_<double>::all(-1), msk);
    cv::Mat resized;
    cv::resize(matchImg, resized, cv::Size{matchImg.cols/2, matchImg.rows/2});
    cv::imshow("GGGG", resized);
    cv::waitKey(0);

    std::vector<std::vector<cv::Point2d>> ptss;
    auto count = std::accumulate(mask.cbegin(), mask.cend(), 0);
    std::vector<cv::Point2d> q_masked, t_masked;
    q_masked.reserve(count);
    t_masked.reserve(count);
    for(int i=0; i<matches.size(); i++) {
        if(mask[i]) {
            q_masked.push_back(q_[i]);
            t_masked.push_back(t_[i]);
        }
    }
    
    ptss.push_back(q_masked);
    ptss.push_back(t_masked);

    auto goproCamMat = cam::GoproCameraVideo.cvCameraMatrix();
    double cam_params[] = {goproCamMat.at<real>(0, 0), goproCamMat.at<real>(1, 1), goproCamMat.at<real>(0, 2), goproCamMat.at<real>(1, 2), 
        -0.2773302016457454, 0.12053493308903486, 0.0006533576995293238, -0.0006346971014042673, -0.030872895814480314};
    cv::Mat intrinsic(1, 9, CV_64FC1, cam_params);
    misslam::calib::OneCameraBAProblem ba(2, 10000, intrinsic);
    ba.addCorrespondPoints({0, 1}, ptss);
    ba.lockIntrinsic();
    ba.lockView(0);
    ba.solve();

    auto point3d= ba.getStructure();


    std::fstream file;
    file.open("test.xyz", std::ios::out);
    for(int i=0; i<point3d.size(); ++i){
        auto c = img1.at<cv::Vec3b>(q_masked[i]);
        file << point3d[i].x << "\t" << -point3d[i].y << "\t" << -point3d[i].z << "\t" << c[2]/255.0 << "\t" << c[1]/255.0 << "\t" << c[0]/255.0<< "\n";
    }
    file.close();

    // reproject point cloud....
    for(int i=0; i<point3d.size(); i++) {
        cv::Point2d uv;
        calib::ReprojectCost::project(intrinsic.ptr<double>(), ba.getView(0).ptr<double>(), &point3d[i].x, &uv.x);
        cv::circle(img1, uv, 3, cv::Scalar{255, 0, 0});
    }
    cv::imshow("GG", img1);
    cv::waitKey(0);

    std::cout<<"RT:"<<ba.getView(1);
/*
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
    file.close();*/
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