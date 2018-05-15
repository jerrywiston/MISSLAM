#ifndef _MISSLAM_MATRIX_H_
#define _MISSLAM_MATRIX_H_

#include <cstring>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include "Vector.h"

namespace misslam {
    template <class T>
    class TMatrix3 {
    public:
        TMatrix3()
        : TMatrix3(T(1))
        {

        }
        explicit TMatrix3(T diag)
        {
            memset(m, 0, sizeof(m));
            m[0] = diag;
            m[4] = diag;
            m[8] = diag;
        }
        explicit TMatrix3(T d[9])
        { std::memcpy(m, d, sizeof(m)); }
        explicit TMatrix3(const cv::Mat &mat)
        { std::memcpy(m, mat.ptr<T>(), sizeof(m)); }
        const T *operator[](uint32_t idx) const
        { return &m[idx*3]; }
        T *operator[](uint32_t idx)
        { return &m[idx*3]; }
        const T *data() const
        { return m; }
        T *data()
        { return m; }

        TVector3<T> operator*(const TVector3<T> &rhs) const
        { 
            return {dot(row(0), rhs), dot(row(1), rhs), dot(row(2), rhs)};
        }

        TVector3<T> row(uint32_t idx) const
        { return {m[idx*3], m[idx*3+1], m[idx*3+2]}; }

        TVector3<T> col(uint32_t idx) const
        { return {m[idx], m[idx+3], m[idx+6]}; }
    private:
        T m[9];
    };

    using Matrix3f = TMatrix3<float>;
    using Matrix3d = TMatrix3<double>;
}

#endif