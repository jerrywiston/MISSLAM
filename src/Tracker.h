#ifndef _MISSLAM_TRACKER_H_
#define _MISSLAM_TRACKER_H_

#include <opencv2/opencv.hpp>
#include "Config.h"

namespace misslam {
    namespace tracker {
        class Tracker {
        public:
            virtual void feedFrame(const cv::Mat &frame)=0;
            virtual bool lastFrameIsKeyframe() const;
            Matrix4 getExtrinsic() const;
        private:
            Matrix3 R;
            Vector3 T;
        };
    }
}

#endif