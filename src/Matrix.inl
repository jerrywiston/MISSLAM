#include "Math_def.h"
#include <iostream>

namespace misslam {
template<class T>
TVector3<T> TMatrix3<T>::operator*(const TVector3<T> &rhs) const
{ 
    return {math::Dot(row(0), rhs), math::Dot(row(1), rhs), math::Dot(row(2), rhs)};
}

template <class T>
TVector4<T> TMatrix4<T>::operator*(const TVector4<T> &rhs) const
{
    return {math::Dot(row(0), rhs), math::Dot(row(1), rhs), math::Dot(row(2), rhs), math::Dot(row(3), rhs)};
}

template <class T>
std::ostream &operator<<(std::ostream &out, const TMatrix4<T> &m)
{
    out << "[";
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            out << m[i][j] << " ";
        }
        out << ";";
    }
    out << "]";
    return out;
}

}