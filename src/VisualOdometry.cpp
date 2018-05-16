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

        cv::Mat S1 = U*ZMat*U.t();
        R1 = U*WMat*VT;
        real temp1[3] = {S1.at<real>(2,1), S1.at<real>(0,2), S1.at<real>(1,0)};
        T1 = cv::Mat(3, 1, CV_REAL, temp1);

        cv::Mat S2 = U*ZMat.t()*U.t();
        R2 = U*WMat.t()*VT;
        real temp2[3] = {S2.at<real>(2,1), S2.at<real>(0,2), S2.at<real>(1,0)};
        T2 = cv::Mat(3, 1, CV_REAL, temp2);
    }

    Point3 Triangulate1Point(
        const cv::Mat &M1, const cv::Mat &M2,
        const Point2 &p1, const Point2 &p2
    ){
        
        // Construct matrix
        cv::Mat p1x = utils::CrossMatrix(p1.x, p1.y, 1);
        cv::Mat p2x = utils::CrossMatrix(p2.x, p2.y, 1);
        
        cv::Mat one = p1x*M1;
        cv::Mat two = p2x*M2;
        cv::Mat A(6,4,CV_REAL);
        return {0,0,1};
        std::memcpy(A.ptr<real>(), one.ptr<real>(), sizeof(real)*12);
        std::memcpy(A.ptr<real>()+12, two.ptr<real>(), sizeof(real)*12);
        
        //cv::vconcat(p1x*M1, p2x*M2, A);
        
        //std::cout << A << std::endl;
        
        // Matrix decomposition
        cv::SVD computeSVD(A, cv::SVD::FULL_UV);
        cv::Mat U = computeSVD.u;
        cv::Mat W = computeSVD.w;
        cv::Mat VT = computeSVD.vt;
        
        cv::Mat P = VT.t().col(3);
        P = P / P.at<real>(3);
        //return {0,0,1};
        return {P.at<real>(0), P.at<real>(1), P.at<real>(2)};
    }

    void VoteRT(const cv::Mat R1, const cv::Mat R2, 
    const cv::Mat T1, const cv::Mat T2, 
    const std::vector<Point2> &qpts, const std::vector<Point2> &tpts)
    {
        std::cout << endl;
        std::cout << T1.t() << std::endl;
        std::cout << T2.t() << std::endl;

        misslam::real temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
        cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
        cv::Mat M2_1 = utils::ExtrinsicMatrixByRT(R1,T1);
        //cv::Mat M2_2 = utils::ExtrinsicMatrixByRT(R1,T2);
        //cv::Mat M2_3 = utils::ExtrinsicMatrixByRT(R2,T1);
        //cv::Mat M2_4 = utils::ExtrinsicMatrixByRT(R2,T2);

        std::cout << endl;
        std::cout << T1.t() << std::endl;
        std::cout << T2.t() << std::endl;

        int count[4] = {0,0,0,0};
        for (i32 i=0; i<100; ++i){
            count[0] += static_cast<u32>(Triangulate1Point(M1, M2_1, qpts[i], tpts[i]).z > 0);
            count[1] += static_cast<u32>(Triangulate1Point(M1, M2_2, qpts[i], tpts[i]).z > 0);
            count[2] += static_cast<u32>(Triangulate1Point(M1, M2_3, qpts[i], tpts[i]).z > 0);
            count[3] += static_cast<u32>(Triangulate1Point(M1, M2_4, qpts[i], tpts[i]).z > 0);
        }

        

        for (i32 i=0; i<4; ++i){
            std::cout << count[i] << std::endl;
        }



    }   

    std::tuple<misslam::map::KeyFrameNode, misslam::map::KeyFrameNode> InitStructure()
    {
        
    }
}
}