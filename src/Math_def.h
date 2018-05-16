#ifndef _MISSLAM_MATH_DEF_H_
#define _MISSLAM_MATH_DEF_H_

namespace misslam {
    
namespace math {
    template <class T>
    static inline T Dot(const TVector3<T> &lhs, const TVector3<T> &rhs);
    template <class T>
    static inline T Dot(const TVector4<T> &lhs, const TVector4<T> &rhs);

    template <class T>
    static inline TMatrix3<T> CrossMatrix(const TVector3<T> &v);
}
}

#endif