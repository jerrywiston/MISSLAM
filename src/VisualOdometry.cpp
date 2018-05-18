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

    u32 candidate(const cv::Mat &R, const cv::Mat &T, const std::vector<Point2> &qpts, const std::vector<Point2> &tpts)
    {
        misslam::real temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
        cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
        cv::Mat M2 = utils::CameraPoseByRT(R, T);

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
        printf("passed: %d\n", passed);
        
        return passed;
    };

    void InitialStructure(const cv::Mat img1, const cv::Mat img2, const cv::Mat cameraMat)
    {
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
        mydp[0] = dp1(cv::Rect{0, 0, 32, 1});

        //printf("<Frame %s / %s> Matches: %d\n", utils::Zfill(i,3).c_str(), utils::Zfill(i+1,3).c_str(), (int)matches.size());
        cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
        cv::Mat img_matches;
        cv::drawMatches(img1, kp1, img2, kp2, matches, img_matches);
        cv::imshow("test", img_matches);

        // [Recover Essential Matrix]
        const cv::Mat funMat = vo::GetFundamentalMatrix(matches, kp1, kp2);
        const cv::Mat essMat = cameraMat.t() * funMat * cameraMat;

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
        
        auto func1 = std::bind(vo::candidate, std::ref(R1), std::ref(T1), std::ref(q), std::ref(t));
        auto func2 = std::bind(vo::candidate, std::ref(R1), std::ref(T2), std::ref(q), std::ref(t));
        auto func3 = std::bind(vo::candidate, std::ref(R2), std::ref(T1), std::ref(q), std::ref(t));
        auto func4 = std::bind(vo::candidate, std::ref(R2), std::ref(T2), std::ref(q), std::ref(t));
        
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
        misslam::real temp2[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
        cv::Mat M1 = cv::Mat(3, 4, CV_REAL, temp2);
        cv::Mat M2 = utils::CameraPoseByRT(R,T);
        std::vector<Point3> points3d(matches.size());
        for(i32 i=0;i<matches.size();i++)
            points3d[i] = vo::Triangulate1Point(M1, M2, q[i], t[i]);

        //for(i32 i=0; i<points3d.size(); i++)
        //    std::cout << points3d[i] << std::endl;
    }
}
}