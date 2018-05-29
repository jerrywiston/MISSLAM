#ifndef _CALIBRATE_H_
#define _CALIBRATE_H_

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include <opencv2/opencv.hpp>

namespace misslam {
    namespace calib {
        struct ReprojectCost
        {
            cv::Point2d observation;

            ReprojectCost(cv::Point2d& observation)
                : observation(observation)
            {
                
            }

            template <typename T>
            static void project(const T* const intrinsic, const T* const extrinsic, const T* const pos3d, T* out)
            {
                const T* r = extrinsic;
                const T* t = &extrinsic[3];

                // RX+T
                T pos_proj[3];
                ceres::AngleAxisRotatePoint(r, pos3d, pos_proj);

                // Apply the camera translation
                pos_proj[0] += t[0];
                pos_proj[1] += t[1];
                pos_proj[2] += t[2];

                const T x = pos_proj[0] / pos_proj[2];
                const T y = pos_proj[1] / pos_proj[2];

                const T l1 = intrinsic[4];  //fx fy cx cy k1 k2 p1 p2 k3
                const T l2 = intrinsic[5];
                const T l3 = intrinsic[8];
                const T p1 = intrinsic[6];
                const T p2 = intrinsic[7];
                const T r2 = x*x+y*y;
                const T r_distortion = l1 * r2 + l2  * r2 * r2 + r2*r2*r2*l3;
                const T t_distortion_x = 2.0*p1*x*y+p2*(r2+2.0*x*x);
                const T t_distortion_y = p1*(r2+2.0*y*y)+2.0*p2*x*y;

                const T fx = intrinsic[0];
                const T fy = intrinsic[1];
                const T cx = intrinsic[2];
                const T cy = intrinsic[3];

                // Apply intrinsic
                const T u = fx*(x + x * r_distortion + t_distortion_x) + cx;
                const T v = fy*(y + y * r_distortion + t_distortion_y) + cy;

                out[0] = u;
                out[1] = v;
            }

            template <typename T>
            bool operator()(const T* const intrinsic, const T* const extrinsic, const T* const pos3d, T* residuals) const
            {
                const T* r = extrinsic;
                const T* t = &extrinsic[3];

                // RX+T
                T pos_proj[3];
                ceres::AngleAxisRotatePoint(r, pos3d, pos_proj);

                // Apply the camera translation
                pos_proj[0] += t[0];
                pos_proj[1] += t[1];
                pos_proj[2] += t[2];

                const T x = pos_proj[0] / pos_proj[2];
                const T y = pos_proj[1] / pos_proj[2];

                const T l1 = intrinsic[4];  //fx fy cx cy k1 k2 p1 p2 k3
                const T l2 = intrinsic[5];
                const T l3 = intrinsic[8];
                const T p1 = intrinsic[6];
                const T p2 = intrinsic[7];
                const T r2 = x*x+y*y;
                const T r_distortion = l1 * r2 + l2  * r2 * r2 + r2*r2*r2*l3;
                const T t_distortion_x = 2.0*p1*x*y+p2*(r2+2.0*x*x);
                const T t_distortion_y = p1*(r2+2.0*y*y)+2.0*p2*x*y;

                const T fx = intrinsic[0];
                const T fy = intrinsic[1];
                const T cx = intrinsic[2];
                const T cy = intrinsic[3];

                // Apply intrinsic
                const T u = fx*(x + x * r_distortion + t_distortion_x) + cx;
                const T v = fy*(y + y * r_distortion + t_distortion_y) + cy;

                residuals[0] = u - T(observation.x);
                residuals[1] = v - T(observation.y);

                return true;
            }


        };


        // Default: Use all matches
        void BundleAdjustment(std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, std::vector<cv::Point3d> &structure, cv::Mat &intrinsic, cv::Mat &extrinsicl, cv::Mat &extrinsicr, int num_matches=-1);


        class OneCameraBAProblem {
        public:
            OneCameraBAProblem(int num_views, int structure_size, cv::Mat intrinsic);
            void addCorrespondPoints(const std::vector<int> &cam_indices, std::vector<std::vector<cv::Point2d>> &ptss, int num_matches=-1);
            void lockIntrinsic();
            void lockView(int index);
            void addView();
            void solve();

            cv::Mat getView(int index)
            { return extrinsic[index]; }

            std::vector<cv::Point3d> &getStructure()
            { return structure; }
        private:
            cv::Mat intrinsic;
            std::vector<cv::Mat> extrinsic;
            std::vector<cv::Point3d> structure;
            ceres::Problem problem;
        };
    }
}

#endif