#ifndef _MISSLAM_VECTOR_H_
#define _MISSLAM_VECTOR_H_

#include <iostream>

namespace misslam {
    template <class T>
    struct TVector2 {
        T x, y, z;
        TVector2()=default;
        TVector2(T x, T y)
            : x(x), y(y)
        {}

        template <class S>
        TVector2<T> operator =(const S &cvpoint) {
            x = cvpoint.x;
            y = cvpoint.y;
            return *this;
        }
    };

    using Vector2f = TVector2<float>;
    using Vector2d = TVector2<double>;

    template <class T>
    struct TVector3 {
        T x, y, z;
        TVector3()=default;
        TVector3(T x, T y, T z)
            : x(x), y(y), z(z)
        {}

        TVector2<T> xy() const {
            return {x, y};
        }

        template <class S>
        TVector3<T> operator =(const S &cvpoint) {
            x = cvpoint.x;
            y = cvpoint.y;
            z = cvpoint.z;
            return *this;
        }
    };
    using Vector3f = TVector3<float>;
    using Vector3d = TVector3<double>;

    template <class T>
    static std::ostream &operator<<(std::ostream &out, const misslam::TVector3<T> &p) {
        return out << "Vector3(" << p.x << ", " << p.y << ", " << p.z << ")";
    }

    template <class T>
    static inline T dot(const TVector3<T> &lhs, const TVector3<T> &rhs)
    { return lhs.x*rhs.x+lhs.y*rhs.y+lhs.z*rhs.z;}
}

#endif