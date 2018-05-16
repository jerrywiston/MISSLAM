#include "Math_def.h"

namespace misslam {
template<typename T>
TVector3<T> TMatrix3<T>::operator*(const TVector3<T> &rhs) const
{ 
    return {math::Dot(row(0), rhs), math::Dot(row(1), rhs), math::Dot(row(2), rhs)};
}

}