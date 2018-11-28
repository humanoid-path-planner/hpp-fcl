// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2018
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2018/10/05)

#pragma once
#ifndef GTE_DIST_POINT_TRIANGLE_H
#define GTE_DIST_POINT_TRIANGLE_H

#include <Mathematics/GteVector.h>
#include <Mathematics/GteDCPQuery.h>
#include <Mathematics/GteTriangle.h>

namespace gte
{

template <int N, typename Real>
class DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>
{
public:
    struct Result
    {
        Real distance, sqrDistance;
        Real parameter[3];  // barycentric coordinates for triangle.v[3]
        Vector<N, Real> closest;
    };

    Result operator()(Vector<N, Real> const& point,
        Triangle<N, Real> const& triangle);

private:
    inline void GetMinEdge02(Real const& a11, Real const& b1,
        Vector<2, Real>& p) const;

    inline void GetMinEdge12(Real const& a01, Real const& a11, Real const& b1,
        Real const& f10, Real const& f01, Vector<2, Real>& p) const;

    inline void GetMinInterior(Vector<2, Real> const& p0, Real const& h0,
        Vector<2, Real> const& p1, Real const& h1, Vector<2, Real>& p) const;
};

// Template aliases for convenience.
template <int N, typename Real>
using DCPPointTriangle =
DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>;

template <typename Real>
using DCPPoint2Triangle2 = DCPPointTriangle<2, Real>;

template <typename Real>
using DCPPoint3Triangle3 = DCPPointTriangle<3, Real>;


template <int N, typename Real> inline
void DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>::GetMinEdge02(
    Real const& a11, Real const& b1, Vector<2, Real>& p) const
{
    p[0] = (Real)0;
    if (b1 >= (Real)0)
    {
        p[1] = (Real)0;
    }
    else if (a11 + b1 <= (Real)0)
    {
        p[1] = (Real)1;
    }
    else
    {
        p[1] = -b1 / a11;
    }
}

template <int N, typename Real> inline
void DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>::GetMinEdge12(
    Real const& a01, Real const& a11, Real const& b1, Real const& f10,
    Real const& f01, Vector<2, Real>& p) const
{
    Real h0 = a01 + b1 - f10;
    if (h0 >= (Real)0)
    {
        p[1] = (Real)0;
    }
    else
    {
        Real h1 = a11 + b1 - f01;
        if (h1 <= (Real)0)
        {
            p[1] = (Real)1;
        }
        else
        {
            p[1] = h0 / (h0 - h1);
        }
    }
    p[0] = (Real)1 - p[1];
}

template <int N, typename Real> inline
void DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>::GetMinInterior(
    Vector<2, Real> const& p0, Real const& h0, Vector<2, Real> const& p1,
    Real const& h1, Vector<2, Real>& p) const
{
    Real z = h0 / (h0 - h1);
    p = ((Real)1 - z) * p0 + z * p1;
}

template <int N,typename Real>
typename DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>::Result
DCPQuery<Real, Vector<N, Real>, Triangle<N, Real>>::operator()(
    Vector<N, Real> const& point, Triangle<N, Real> const& triangle)
{
    Vector<N, Real> diff = point - triangle.v[0];
    Vector<N, Real> edge0 = triangle.v[1] - triangle.v[0];
    Vector<N, Real> edge1 = triangle.v[2] - triangle.v[0];
    Real a00 = Dot(edge0, edge0);
    Real a01 = Dot(edge0, edge1);
    Real a11 = Dot(edge1, edge1);
    Real b0 = -Dot(diff, edge0);
    Real b1 = -Dot(diff, edge1);

    Real f00 = b0;
    Real f10 = b0 + a00;
    Real f01 = b0 + a01;

    Vector<2, Real> p0, p1, p;
    Real dt1, h0, h1;

    // Compute the endpoints p0 and p1 of the segment.  The segment is
    // parameterized by L(z) = (1-z)*p0 + z*p1 for z in [0,1] and the
    // directional derivative of half the quadratic on the segment is
    // H(z) = Dot(p1-p0,gradient[Q](L(z))/2), where gradient[Q]/2 = (F,G).
    // By design, F(L(z)) = 0 for cases (2), (4), (5), and (6).  Cases (1) and
    // (3) can correspond to no-intersection or intersection of F = 0 with the
    // triangle.
    if (f00 >= (Real)0)
    {
        if (f01 >= (Real)0)
        {
            // (1) p0 = (0,0), p1 = (0,1), H(z) = G(L(z))
            GetMinEdge02(a11, b1, p);
        }
        else
        {
            // (2) p0 = (0,t10), p1 = (t01,1-t01), H(z) = (t11 - t10)*G(L(z))
            p0[0] = (Real)0;
            p0[1] = f00 / (f00 - f01);
            p1[0] = f01 / (f01 - f10);
            p1[1] = (Real)1 - p1[0];
            dt1 = p1[1] - p0[1];
            h0 = dt1 * (a11 * p0[1] + b1);
            if (h0 >= (Real)0)
            {
                GetMinEdge02(a11, b1, p);
            }
            else
            {
                h1 = dt1 * (a01 * p1[0] + a11 * p1[1] + b1);
                if (h1 <= (Real)0)
                {
                    GetMinEdge12(a01, a11, b1, f10, f01, p);
                }
                else
                {
                    GetMinInterior(p0, h0, p1, h1, p);
                }
            }
        }
    }
    else if (f01 <= (Real)0)
    {
        if (f10 <= (Real)0)
        {
            // (3) p0 = (1,0), p1 = (0,1), H(z) = G(L(z)) - F(L(z))
            GetMinEdge12(a01, a11, b1, f10, f01, p);
        }
        else
        {
            // (4) p0 = (t00,0), p1 = (t01,1-t01), H(z) = t11*G(L(z))
            p0[0] = f00 / (f00 - f10);
            p0[1] = (Real)0;
            p1[0] = f01 / (f01 - f10);
            p1[1] = (Real)1 - p1[0];
            h0 = p1[1] * (a01 * p0[0] + b1);
            if (h0 >= (Real)0)
            {
                p = p0;  // GetMinEdge01
            }
            else
            {
                h1 = p1[1] * (a01 * p1[0] + a11 * p1[1] + b1);
                if (h1 <= (Real)0)
                {
                    GetMinEdge12(a01, a11, b1, f10, f01, p);
                }
                else
                {
                    GetMinInterior(p0, h0, p1, h1, p);
                }
            }
        }
    }
    else if (f10 <= (Real)0)
    {
        // (5) p0 = (0,t10), p1 = (t01,1-t01), H(z) = (t11 - t10)*G(L(z))
        p0[0] = (Real)0;
        p0[1] = f00 / (f00 - f01);
        p1[0] = f01 / (f01 - f10);
        p1[1] = (Real)1 - p1[0];
        dt1 = p1[1] - p0[1];
        h0 = dt1 * (a11 * p0[1] + b1);
        if (h0 >= (Real)0)
        {
            GetMinEdge02(a11, b1, p);
        }
        else
        {
            h1 = dt1 * (a01 * p1[0] + a11 * p1[1] + b1);
            if (h1 <= (Real)0)
            {
                GetMinEdge12(a01, a11, b1, f10, f01, p);
            }
            else
            {
                GetMinInterior(p0, h0, p1, h1, p);
            }
        }
    }
    else
    {
        // (6) p0 = (t00,0), p1 = (0,t11), H(z) = t11*G(L(z))
        p0[0] = f00 / (f00 - f10);
        p0[1] = (Real)0;
        p1[0] = (Real)0;
        p1[1] = f00 / (f00 - f01);
        h0 = p1[1] * (a01 * p0[0] + b1);
        if (h0 >= (Real)0)
        {
            p = p0;  // GetMinEdge01
        }
        else
        {
            h1 = p1[1] * (a11 * p1[1] + b1);
            if (h1 <= (Real)0)
            {
                GetMinEdge02(a11, b1, p);
            }
            else
            {
                GetMinInterior(p0, h0, p1, h1, p);
            }
        }
    }

    Result result;
    result.parameter[0] = (Real)1 - p[0] - p[1];
    result.parameter[1] = p[0];
    result.parameter[2] = p[1];
    result.closest = triangle.v[0] + p[0] * edge0 + p[1] * edge1;
    diff = point - result.closest;
    result.sqrDistance = Dot(diff, diff);
    result.distance = std::sqrt(result.sqrDistance);
    return result;
}


}
#endif
