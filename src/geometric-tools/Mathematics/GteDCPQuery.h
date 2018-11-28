// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2018
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 3.0.1 (2018/10/05)

#pragma once
#ifndef GTE_DCPQUERY_H
#define GTE_DCPQUERY_H

#include <Mathematics/GteMath.h>

namespace gte
{

// Distance and closest-point queries.
template <typename Real, typename Type0, typename Type1>
class DCPQuery
{
public:
    struct Result
    {
        // A DCPQuery-base class B must define a B::Result struct with member
        // 'Real distance'.  A DCPQuery-derived class D must also derive a
        // D::Result from B:Result but may have no members.  The idea is to
        // allow Result to store closest-point information in addition to the
        // distance.  The operator() is non-const to allow DCPQuery to store
        // and modify private state that supports the query.
    };
    Result operator()(Type0 const& primitive0, Type1 const& primitive1);
};

}
#endif
