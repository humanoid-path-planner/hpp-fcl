// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2018
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.0 (2016/06/19)

#pragma once
#ifndef GTE_LINE_H
#define GTE_LINE_H

#include <Mathematics/GteVector.h>

// The line is represented by P+t*D, where P is an origin point, D is a
// unit-length direction vector, and t is any real number.  The user must
// ensure that D is unit length.

namespace gte
{

template <int N, typename Real>
class Line
{
public:
    // Construction and destruction.  The default constructor sets the origin
    // to (0,...,0) and the line direction to (1,0,...,0).
    Line();
    Line(Vector<N, Real> const& inOrigin, Vector<N, Real> const& inDirection);

    // Public member access.  The direction must be unit length.
    Vector<N, Real> origin, direction;

public:
    // Comparisons to support sorted containers.
    bool operator==(Line const& line) const;
    bool operator!=(Line const& line) const;
    bool operator< (Line const& line) const;
    bool operator<=(Line const& line) const;
    bool operator> (Line const& line) const;
    bool operator>=(Line const& line) const;
};

// Template aliases for convenience.
template <typename Real>
using Line2 = Line<2, Real>;

template <typename Real>
using Line3 = Line<3, Real>;


template <int N, typename Real>
Line<N, Real>::Line()
{
    origin.MakeZero();
    direction.MakeUnit(0);
}

template <int N, typename Real>
Line<N, Real>::Line(Vector<N, Real> const& inOrigin,
    Vector<N, Real> const& inDirection)
    :
    origin(inOrigin),
    direction(inDirection)
{
}

template <int N, typename Real>
bool Line<N, Real>::operator==(Line const& line) const
{
    return origin == line.origin && direction == line.direction;
}

template <int N, typename Real>
bool Line<N, Real>::operator!=(Line const& line) const
{
    return !operator==(line);
}

template <int N, typename Real>
bool Line<N, Real>::operator<(Line const& line) const
{
    if (origin < line.origin)
    {
        return true;
    }

    if (origin > line.origin)
    {
        return false;
    }

    return direction < line.direction;
}

template <int N, typename Real>
bool Line<N, Real>::operator<=(Line const& line) const
{
    return operator<(line) || operator==(line);
}

template <int N, typename Real>
bool Line<N, Real>::operator>(Line const& line) const
{
    return !operator<=(line);
}

template <int N, typename Real>
bool Line<N, Real>::operator>=(Line const& line) const
{
    return !operator<(line);
}


}
#endif
