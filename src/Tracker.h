#ifndef _MISSLAM_TRACKER_H_
#define _MISSLAM_TRACKER_H_

#include <opencv2/opencv.hpp>
#include "Config.h"
#include "Map.h"

namespace misslam {
    namespace tracker {
        enum class TrackStatus {
            eSuccess, eFail
        };


        class Tracker {
        public:
            Tracker(const cv::Mat &cameraMat);

            virtual TrackStatus track(const cv::Mat &img, map::GlobalState &gstate)=0;

            Matrix4 getExtrinsic() const;
        protected:
            cv::Mat cameraMat;
            Matrix3 R;
            Vector3 T;
        };

        template <class T>
        class TIndirectTracker: public Tracker {
        public:
            TIndirectTracker(const cv::Mat &cameraMat)
                : Tracker(cameraMat)
            {
                
            }
            virtual TrackStatus track(const cv::Mat &img, map::GlobalState &gstate) override;
        };

        using ORBTracker = TIndirectTracker<cv::ORB>;
    }
}

#include "Tracker.inl"

#endif