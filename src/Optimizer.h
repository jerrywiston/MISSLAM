#ifndef _MISSLAM_OPTIMIZER_H_
#define _MISSLAM_OPTIMIZER_H_

#include "PinholeCamera.h"

namespace misslam {
    namespace opt {
        class BAProblem {
        public:
            void addCamera(const cam::PinholeCamera &cam);
        private:
            
        };
    }
}

#endif