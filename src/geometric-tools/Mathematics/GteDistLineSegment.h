// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2018
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2018/10/05)

#pragma once
#ifndef GTE_DIST_LINE_SEGMENT_H
#define GTE_DIST_LINE_SEGMENT_H

#include <Mathematics/GteDCPQuery.h>
#include <Mathematics/GteLine.h>
#include <Mathematics/GteSegment.h>

namespace gte
{

template <int N, typename Real>
class DCPQuery<Real, Line<N, Real>, Segment<N, Real>>
{
public:
    struct Result
    {
        Real distance, sqrDistance;
        Real parameter[2];
        Vector<N, Real> closestPoint[2];
    };

    // The centered form of the 'segment' is used.  Thus, parameter[1] of
    // the result is in [-e,e], where e = |segment.p[1] - segment.p[0]|/2.
    Result operator()(Line<N, Real> const& line,
        Segment<N, Real> const& segment);
};

// Template aliases for convenience.
template <int N, typename Real>
using DCPLineSegment = DCPQuery<Real, Line<N, Real>, Segment<N, Real>>;

template <typename Real>
using DCPLine2Segment2 = DCPLineSegment<2, Real>;

template <typename Real>
using DCPLine3Segment3 = DCPLineSegment<3, Real>;


template <int N, typename Real>
typename DCPQuery<Real, Line<N, Real>, Segment<N, Real>>::Result
DCPQuery<Real, Line<N, Real>, Segment<N, Real>>::operator()(
    Line<N, Real> const& line, Segment<N, Real> const& segment)
{
    Result result;

    Vector<N, Real> segCenter, segDirection;
    Real segExtent;
    segment.GetCenteredForm(segCenter, segDirection, segExtent);

    Vector<N, Real> diff = line.origin - segCenter;
    Real a01 = -Dot(line.direction, segDirection);
    Real b0 = Dot(diff, line.direction);
    Real s0, s1;

    if (std::abs(a01) < (Real)1)
    {
        // The line and segment are not parallel.
        Real det = (Real)1 - a01 * a01;
        Real extDet = segExtent * det;
        Real b1 = -Dot(diff, segDirection);
        s1 = a01 * b0 - b1;

        if (s1 >= -extDet)
        {
            if (s1 <= extDet)
            {
                // Two interior points are closest, one on the line and one
                // on the segment.
                s0 = (a01 * b1 - b0) / det;
                s1 /= det;
            }
            else
            {
                // The endpoint e1 of the segment and an interior point of
                // the line are closest.
                s1 = segExtent;
                s0 = -(a01 * s1 + b0);
            }
        }
        else
        {
            // The endpoint e0 of the segment and an interior point of the
            // line are closest.
            s1 = -segExtent;
            s0 = -(a01 * s1 + b0);
        }
    }
    else
    {
        // The line and segment are parallel.  Choose the closest pair so that
        // one point is at segment origin.
        s1 = (Real)0;
        s0 = -b0;
    }

    result.parameter[0] = s0;
    result.parameter[1] = s1;
    result.closestPoint[0] = line.origin + s0 * line.direction;
    result.closestPoint[1] = segCenter + s1 * segDirection;
    diff = result.closestPoint[0] - result.closestPoint[1];
    result.sqrDistance = Dot(diff, diff);
    result.distance = std::sqrt(result.sqrDistance);
    return result;
}


}
#endif
