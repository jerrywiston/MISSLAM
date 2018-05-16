#include "Utils.h"

namespace misslam
{
namespace utils
{

std::string Zfill(u32 i, u32 fill){
    int count = std::to_string(i).length();
    std::string out = std::string(fill-count, '0') + std::to_string(i);
    return out;
}

std::vector<cv::DMatch> ORBMatch(cv::Mat dp1, cv::Mat dp2){
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(dp1, dp2, matches);

    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < dp1.rows; i++ )
	{ 
	    float dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

    std::vector< cv::DMatch > good_matches;
	for( int i = 0; i < dp1.rows; i++ )
		if( matches[i].distance < 0.5_r*max_dist )
			good_matches.push_back( matches[i]);
    
    return good_matches;
}

cv::Mat ExtrinsicMatrixByRT(cv::Mat R, cv::Mat T){
    float temp[16] = {
        R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), T.at<float>(0),
        R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), T.at<float>(1),
        R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), T.at<float>(2),
        0, 0, 0, 1
    };

    cv::Mat H = cv::Mat(4, 4, CV_REAL, temp);
    cv::Mat A = H.inv();
    
    float temp2[12] = {
        A.at<float>(0,0), A.at<float>(0,1), A.at<float>(0,2), A.at<float>(0,3),
        A.at<float>(1,0), A.at<float>(1,1), A.at<float>(1,2), A.at<float>(1,3),
        A.at<float>(2,0), A.at<float>(2,1), A.at<float>(2,2), A.at<float>(2,3)
    };

    cv::Mat M = cv::Mat(3, 4, CV_REAL, temp2);
    return M.clone();
}

cv::Mat CrossMatrix(float a0, float a1, float a2){
    float temp[9] = {
        0, -a2, a1,
        a2, 0, -a0,
        -a1, a0, 1,
    };
    cv::Mat out = cv::Mat(3, 3, CV_REAL, temp);
    return out.clone();
}

void ArrangeMatchPoints(const std::vector<cv::KeyPoint> &q, const std::vector<cv::KeyPoint> &t,
    const std::vector<cv::DMatch> &matches,
    std::vector<Point2> &query, std::vector<Point2> &train)
{
    query.resize(matches.size());
    train.resize(matches.size());
    for(u32 i=0; i<matches.size(); i++) {
        query[i] = q[matches[i].queryIdx].pt;
        train[i] = t[matches[i].trainIdx].pt;
    }
}

void ToNormalizedSpace(const cv::Mat &K, std::vector<Point2> &image_points, u32 count)
{
    Matrix3 kinv(K.inv());
    count = (count == -1 ? image_points.size() : std::min<u32>(count, image_points.size()));
    for(u32 i=0; i<count; i++) {
        image_points[i] = (kinv * Vector3(image_points[i].x, image_points[i].y, 1.0_r)).xy();
    }
}

void Voter::push(std::function<u32 ()> func) {
    candidates.push_back(func);
}

Election Voter::elect() const {
    i32 maxone=-1;
    u32 score;
    for(i32 i=0; i<candidates.size(); i++) {
        u32 sc = candidates[i]();
        if (maxone==-1 || score < sc) {
            score = sc;
            maxone = i;
        }
    }
    return {maxone, score};
}

}
}