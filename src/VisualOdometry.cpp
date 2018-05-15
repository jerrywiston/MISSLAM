#include "Utils.h"
#include "VisualOdometry.h"

using namespace std;

namespace misslam{
namespace vo{
    // Essential Matrix Extract (Z,W)
    const real data2[9] = { 0, 1, 0, -1, 0, 0, 0, 0, 0 };
    const cv::Mat ZMat = cv::Mat(3, 3, CV_REAL, (real *)data2);
    const real data3[9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };
    const cv::Mat WMat = cv::Mat(3, 3, CV_REAL, (real *)data3);

    cv::Mat GetFundamentalMatrix(
        std::vector<cv::DMatch> matches, 
        std::vector<cv::KeyPoint> kp1, 
        std::vector<cv::KeyPoint> kp2)
    {
        std::vector<cvPoint2> p1cv(matches.size());
        std::vector<cvPoint2> p2cv(matches.size());

        for(i32 i=0;i<matches.size();i++)
        {              
            p1cv[i] = kp1[matches[i].queryIdx].pt;
            p2cv[i] = kp2[matches[i].trainIdx].pt;
        }

        std::vector<uchar> m_RANSACStatus;
        cv::Mat funMat = cv::findFundamentalMat(p1cv, p2cv, m_RANSACStatus, cv::FM_RANSAC, 2.0, 0.5);
        funMat.convertTo(funMat, CV_REAL);
        
        return funMat;
    }

    cv::Mat ExtractRT(cv::Mat essMat, cv::Mat &R, cv::Mat &T){
        cv::SVD computeSVD(essMat, cv::SVD::FULL_UV);
        cv::Mat U = computeSVD.u;
        cv::Mat W = computeSVD.w;
        cv::Mat VT = computeSVD.vt;
        cv::Mat S = U*ZMat*U.t();
        R = U*WMat*VT;
        real temp[3] = {S.at<real>(2,1), S.at<real>(0,2), S.at<real>(1,0)};
        T = cv::Mat(3, 1, CV_REAL, temp);
        return W;
    }

    Point3 Triangulate1Point(
        cv::Mat &M1, cv::Mat &M2,
        cv::Mat &p1, cv::Mat &p2
    ){
        // Construct matrix
        // assume pi in normalized space
        //cv::Mat p1 = K1 * pp1;
        //cv::Mat p2 = K2 * pp2;
        cv::Mat p1x = misslam::utils::CrossMatrix(p1.at<real>(0), p1.at<real>(1), p1.at<real>(2));
        cv::Mat p2x = misslam::utils::CrossMatrix(p2.at<real>(0), p2.at<real>(1), p2.at<real>(2));
        cv::Mat A;
        cv::vconcat(p1x*M1, p2x*M2, A);
        
        // Matrix decomposition
        cv::SVD computeSVD(A, cv::SVD::FULL_UV);
        cv::Mat U = computeSVD.u;
        cv::Mat W = computeSVD.w;
        cv::Mat VT = computeSVD.vt;
        cv::Mat P = VT.t().col(3);
        P = P / P.at<real>(3);
        return {P.at<real>(0), P.at<real>(1), P.at<real>(2)};
    }
/*
    void InitialStructure(
        cv::Mat M1, cv::Mat M2, 
        cv::Mat K1, cv::Mat K2,
        std::vector<cv::DMatch> &matches,
        std::vector<cv::KeyPoint> &kp1, std::vector<cv::KeyPoint> &kp2,
        std::vector<i32> &idx1, std::vector<i32> &idx2,
        std::vector<Point3> &points3d
    ){     
        idx1 = std::vector<i32>(kp1.size(), -1);
        idx2 = std::vector<i32>(kp2.size(), -1);
        points3d.resize(matches.size());

        i32 count = 0;
        for(i32 i=0;i<matches.size();i++)
        {  
            real data1[3] = {kp1[matches[i].queryIdx].pt.x, kp1[matches[i].queryIdx].pt.y, 1};
            real data2[3] = {kp2[matches[i].trainIdx].pt.x, kp2[matches[i].trainIdx].pt.y, 1};
            cv::Mat p1 = cv::Mat(3,1,CV_REAL,data1);
            cv::Mat p2 = cv::Mat(3,1,CV_REAL,data2);
            cv::Mat P = Triangulate1Point(M1, M2, K1, K2, p1, p2);
            idx1[matches[i].queryIdx] = count;
            idx2[matches[i].trainIdx] = count;
            points3d[i] = Point3(P.at<real>(0), P.at<real>(1), P.at<real>(2));
            ++count;
        }
    }*/

    cv::Mat Init()
    {
        
    }
}
}