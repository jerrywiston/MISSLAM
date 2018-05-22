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
        TVector3(const TVector2<T> &v, T w)
            : TVector3(v.x, v.y, w)
        {}
        TVector3(T x, T y, T z)
            : x(x), y(y), z(z)
        {}

        TVector2<T> xy() const {
            return {x, y};
        }

        TVector3<T> operator *(T scalar) {
            return {x*scalar, y*scalar, z*scalar};
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
    struct TVector4 {
        T x, y, z, w;
        TVector4()=default;
        TVector4(const TVector3<T> &v, T w)
            : TVector4(v.x, v.y, v.z, w)
        {}
        TVector4(T x, T y, T z, T w)
            : x(x), y(y), z(z), w(w)
        {}

        TVector3<T> xyz() const {
            return {x, y, z};
        }

        TVector2<T> xy() const {
            return {x, y};
        }

        TVector4<T> operator *(T scalar) {
            return {x*scalar, y*scalar, z*scalar, w*scalar};
        }
    };

    using Vector4f = TVector4<float>;
    using Vector4d = TVector4<double>;

    template <class T>
    static std::ostream &operator<<(std::ostream &out, const misslam::TVector2<T> &p) {
        return out << "Vector2(" << p.x << ", " << p.y << ")";
    }

    template <class T>
    static std::ostream &operator<<(std::ostream &out, const misslam::TVector3<T> &p) {
        return out << "Vector3(" << p.x << ", " << p.y << ", " << p.z << ")";
    }

    template <class T>
    static std::ostream &operator<<(std::ostream &out, const misslam::TVector4<T> &p) {
        return out << "Vector4(" << p.x << ", " << p.y << ", " << p.z<<", "<<p.w<< ")";
    }
}

#endif