#ifndef _MISSLAM_MATRIX_H_
#define _MISSLAM_MATRIX_H_

#include <cstring>
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace misslam {
    template <class T>
    class TVector3;

    template <class T>
    class TMatrix3 {
    public:
        TMatrix3()
        : TMatrix3(T(1))
        {

        }
        TMatrix3(std::initializer_list<T> l) {
            uint32_t idx = 0;
            for(auto it=l.begin(); it!=l.end(); ++it) {
                m[idx++] = *it;
            }
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

        TVector3<T> operator*(const TVector3<T> &rhs) const;

        TVector3<T> row(uint32_t idx) const
        { return {m[idx*3], m[idx*3+1], m[idx*3+2]}; }

        TVector3<T> col(uint32_t idx) const
        { return {m[idx], m[idx+3], m[idx+6]}; }

        cv::Mat toMat() const {
            // TODO..
            return cv::Mat(3, 3, CV_32FC1, const_cast<T *>(data())).clone();
        }

        operator cv::Mat() const {
            // TODO..
            return toMat();
        }
    private:
        T m[9];
    };

    using Matrix3f = TMatrix3<float>;
    using Matrix3d = TMatrix3<double>;

    template <class T>
    class TMatrix4 {
    public:
        TMatrix4()
        : TMatrix4(T(1))
        {

        }
        TMatrix4(std::initializer_list<T> l) {
            uint32_t idx = 0;
            for(auto it=l.begin(); it!=l.end(); ++it) {
                m[idx++] = *it;
            }
        }
        explicit TMatrix4(T diag)
        {
            memset(m, 0, sizeof(m));
            m[0] = diag;
            m[5] = diag;
            m[10] = diag;
            m[15] = diag;
        }
        explicit TMatrix4(T d[16])
        { std::memcpy(m, d, sizeof(m)); }
        explicit TMatrix4(const cv::Mat &mat)
        {
            if(mat.cols==4 && mat.rows==4)
                std::memcpy(m, mat.ptr<T>(), sizeof(m));
            else if(mat.cols==4 && mat.rows==3) {
                std::memcpy(m, mat.ptr<T>(), sizeof(T)*12);
                m[12] = m[13] = m[14] = T(0);
                m[15] = T(1);
            }
            // TODO: throw
        }
        const T *operator[](uint32_t idx) const
        { return &m[idx*4]; }
        T *operator[](uint32_t idx)
        { return &m[idx*4]; }
        const T *data() const
        { return m; }
        T *data()
        { return m; }

        TVector4<T> row(uint32_t idx) const
        { return {m[idx*4], m[idx*4+1], m[idx*4+2], m[idx*4+3]}; }

        TVector4<T> col(uint32_t idx) const
        { return {m[idx], m[idx+4], m[idx+8], m[idx+12]}; }

        TVector4<T> operator*(const TVector4<T> &rhs) const;

        cv::Mat toMat() const {
            // TODO..
            return cv::Mat(4, 4, CV_32FC1, const_cast<T *>(data())).clone();
        }

        operator cv::Mat() const {
            // TODO..
            return toMat();
        }

    private:
        T m[16];
    };

    using Matrix4f = TMatrix4<float>;
    using Matrix4d = TMatrix4<double>;
}

#include "Matrix.inl"

#endif