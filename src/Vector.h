#ifndef _MISSLAM_VECTOR_H_
#define _MISSLAM_VECTOR_H_

namespace misslam {
    template <class T>
    struct Vector3 {
        T x, y, z;
        Vector3()=default;
        Vector3(T x, T y, T z)
            : x(x), y(y), z(z)
        {}
    };

    template <class T>
    struct Vector2 {
        T x, y, z;
        Vector2()=default;
        Vector2(T x, T y)
            : x(x), y(y)
        {}
    };
}

#endif