#ifndef _MISSLAM_MATH_H_
#define _MISSLAM_MATH_H_

#include "Vector.h"
#include "Matrix.h"

namespace misslam{
    namespace math {
        template <class T>
        static inline T Dot(const TVector3<T> &lhs, const TVector3<T> &rhs)
        { return lhs.x*rhs.x+lhs.y*rhs.y+lhs.z*rhs.z;}

        template <class T>
        static inline TMatrix3<T> CrossMatrix(const TVector3<T> &v)
        { return {1.0};}
    }
}
#endif