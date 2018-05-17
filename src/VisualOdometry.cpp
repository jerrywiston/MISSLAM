#include "Utils.h"
#include "VisualOdometry.h"
#include "Map.h"

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

    void ExtractRT(const cv::Mat essMat, cv::Mat &R1, cv::Mat &R2, cv::Mat &T1, cv::Mat &T2){
        cv::SVD computeSVD(essMat, cv::SVD::FULL_UV);
        cv::Mat U = computeSVD.u;
        cv::Mat W = computeSVD.w;
        cv::Mat VT = computeSVD.vt;

        cv::Mat Tx1 = U*ZMat*U.t();
        R1 = U*WMat*VT;
        real temp1[3] = {Tx1.at<real>(2,1), Tx1.at<real>(0,2), Tx1.at<real>(1,0)};
        T1 = cv::Mat(3, 1, CV_REAL, temp1).clone();

        cv::Mat Tx2 = U*ZMat.t()*U.t();
        R2 = U*WMat.t()*VT;
        real temp2[3] = {Tx2.at<real>(2,1), Tx2.at<real>(0,2), Tx2.at<real>(1,0)};
        T2 = cv::Mat(3, 1, CV_REAL, temp2).clone();
    }

    Point3 Triangulate1Point(
        const cv::Mat &M1, const cv::Mat &M2,
        const Point2 &p1, const Point2 &p2
    ){       
        // Construct matrix
        cv::Mat p1x = utils::CrossMatrix(p1.x, p1.y, 1);
        cv::Mat p2x = utils::CrossMatrix(p2.x, p2.y, 1);
        
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

    std::tuple<misslam::map::KeyFrameNode, misslam::map::KeyFrameNode> InitStructure()
    {
        
    }
}
}