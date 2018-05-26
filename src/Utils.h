#ifndef _MISSLAM_UTILS_H_
#define _MISSLAM_UTILS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <cstdint>
#include <functional>
#include "Config.h"
#include "Map.h"

namespace misslam {
namespace utils {
    /*
    * Convert integer to string
    * @param [in] i Total number of string, if number of digit is smaller than i, it will fill by '0'
    * @param [in] Fill number
    * @return zfilled string
    */
    std::string Zfill(u32 i, u32 number);
    /*
    * Match two ORB descriptor
    * @param [in] dp1 Query descriptor
    * @param [in] dp2 Train descriptor
    * @return matchings
    */
    std::vector<cv::DMatch> FeatureMatch(cv::Mat dp1, cv::Mat dp2);
    /*
    * Construct 4x4 extrinsic matrix from [R|T]
    * @param [in] R Rotation
    * @param [in] T Translate
    * @return extrinsic matrix
    */
    cv::Mat CameraPoseByRT(cv::Mat R, cv::Mat T);
    cv::Mat CrossMatrix(float a0, float a1, float a2);
    
    u32 ArrangeMatchPoints(const std::vector<cv::KeyPoint> &q, const std::vector<cv::KeyPoint> &t,
        const std::vector<cv::DMatch> &matches,
        std::vector<Point2> &query, std::vector<Point2> &train,
        const std::vector<uchar> &mask={});

    void ToNormalizedSpace(const cv::Mat &K, std::vector<Point2> &image_points, u32 count=-1);

    void ConvertDescriptor(cv::Mat desc, std::vector<map::ORBPointDescriptor> &out);

    Point3 Triangulate2View(
        const cv::Mat &M1, const cv::Mat &M2,
        const Point2 &p1, const Point2 &p2
    );

    cv::Mat Flow2BGR(cv::Mat flow);

    cv::Mat ReconstructImage(const cv::Mat &query, const cv::Mat &flow);

    struct Election{
        i32 idx;
        u32 score;        
    };
    
    class Voter {
    public:
        void push(std::function<u32 ()> evalFunc, std::function<void()> onWin=[](){});
        Election elect() const;
    private:
        std::vector<std::pair<std::function<u32 ()>, std::function<void ()>>> candidates;
    };

    template <class S, class T>
    std::vector<S> ConvertInnerType(const std::vector<T> &v) {
        std::vector<S> ret;
        ret.reserve(v.size());
        for(auto &v_: v) {
            ret.push_back(S{v_});
        }
        return ret;
    }
}
}

#endif