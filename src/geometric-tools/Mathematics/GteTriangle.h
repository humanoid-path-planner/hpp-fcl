// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2018
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once
#ifndef GTE_TRIANGLE_H
#define GTE_TRIANGLE_H

#include <Mathematics/GteVector.h>

// The triangle is represented as an array of three vertices.  The dimension
// N must be 2 or larger.

namespace gte
{

template <int N, typename Real>
class Triangle
{
public:
    // Construction and destruction.  The default constructor sets the
    // vertices to (0,..,0), (1,0,...,0), and (0,1,0,...,0).
    Triangle();
    Triangle(Vector<N, Real> const& v0, Vector<N, Real> const& v1,
        Vector<N, Real> const& v2);
    Triangle(std::array<Vector<N, Real>, 3> const& inV);

    // Public member access.
    std::array<Vector<N, Real>, 3> v;

public:
    // Comparisons to support sorted containers.
    bool operator==(Triangle const& triangle) const;
    bool operator!=(Triangle const& triangle) const;
    bool operator< (Triangle const& triangle) const;
    bool operator<=(Triangle const& triangle) const;
    bool operator> (Triangle const& triangle) const;
    bool operator>=(Triangle const& triangle) const;
};

// Template aliases for convenience.
template <typename Real>
using Triangle2 = Triangle<2, Real>;

template <typename Real>
using Triangle3 = Triangle<3, Real>;


template <int N, typename Real>
Triangle<N, Real>::Triangle()
{
    v[0].MakeZero();
    v[1].MakeUnit(0);
    v[2].MakeUnit(1);
}

template <int N, typename Real>
Triangle<N, Real>::Triangle(Vector<N, Real> const& v0,
    Vector<N, Real> const& v1, Vector<N, Real> const& v2)
{
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
}

template <int N, typename Real>
Triangle<N, Real>::Triangle(std::array<Vector<N, Real>, 3> const& inV)
    :
    v(inV)
{
}

template <int N, typename Real>
bool Triangle<N, Real>::operator==(Triangle const& triangle) const
{
    return v == triangle.v;
}

template <int N, typename Real>
bool Triangle<N, Real>::operator!=(Triangle const& triangle) const
{
    return !operator==(triangle);
}

template <int N, typename Real>
bool Triangle<N, Real>::operator<(Triangle const& triangle) const
{
    return v < triangle.v;
}

template <int N, typename Real>
bool Triangle<N, Real>::operator<=(Triangle const& triangle) const
{
    return operator<(triangle) || operator==(triangle);
}

template <int N, typename Real>
bool Triangle<N, Real>::operator>(Triangle const& triangle) const
{
    return !operator<=(triangle);
}

template <int N, typename Real>
bool Triangle<N, Real>::operator>=(Triangle const& triangle) const
{
    return !operator<(triangle);
}


}
#endif
