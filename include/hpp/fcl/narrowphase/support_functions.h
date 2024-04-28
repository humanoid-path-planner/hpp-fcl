/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \authors Jia Pan, Florent Lamiraux, Josef Mirabel, Louis Montaut */

#ifndef HPP_FCL_SUPPORT_FUNCTIONS_H
#define HPP_FCL_SUPPORT_FUNCTIONS_H

#include "hpp/fcl/shape/geometric_shapes.h"
#include "hpp/fcl/math/transform.h"
namespace hpp {
namespace fcl {

namespace details {

/// @brief Options for the computation of support points.
/// `NoSweptSphere` option is used when the support function is called
/// by GJK or EPA. In this case, the swept sphere radius is not taken into
/// account in the support function. It is used by GJK and EPA after they have
/// converged to correct the solution.
/// `WithSweptSphere` option is used when the support function is called
/// directly by the user. In this case, the swept sphere radius is taken into
/// account in the support function.
enum SupportOptions {
  NoSweptSphere = 0,
  WithSweptSphere = 0x1,
};

/// @brief the support function for shape.
/// The output support point is expressed in the frame local of the shape.
/// @return argmax_{v in shape0} v.dot(dir).
/// @param shape the shape.
/// @param dir support direction.
/// @param hint used to initialize the search when shape is a ConvexBase object.
/// @tparam SupportOptions is a value of the SupportOptions enum. If set to
/// `WithSweptSphere`, the support functions take into account the shapes' swept
/// sphere radii. Please see `MinkowskiDiff::set(const ShapeBase*, const
/// ShapeBase*)` for more details.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, int& hint);

/// @brief Stores temporary data for the computation of support points.
struct ShapeSupportData {
  std::vector<int8_t> visited;
  Vec3f last_dir = Vec3f::Zero();
};

/// @brief Triangle Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const TriangleP* triangle, const Vec3f& dir,
                     Vec3f& support, int& /*unused*/,
                     ShapeSupportData* /*unused*/);

/// @brief Box Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
inline void getShapeSupport(const Box* box, const Vec3f& dir, Vec3f& support,
                            int& /*unused*/, ShapeSupportData* /*unused*/);

/// @brief Sphere Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
inline void getShapeSupport(const Sphere* sphere, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData* /*unused*/);

/// @brief Ellipsoid Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
inline void getShapeSupport(const Ellipsoid* ellipsoid, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData* /*unused*/);

/// @brief Capsule Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
inline void getShapeSupport(const Capsule* capsule, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData* /*unused*/);

/// @brief Cone Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Cone* cone, const Vec3f& dir, Vec3f& support,
                     int& /*unused*/, ShapeSupportData* /*unused*/);

/// @brief Cylinder Support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir, Vec3f& support,
                     int& /*unused*/, ShapeSupportData* /*unused*/);

/// @brief ConvexBase Support function.
/// @note See @ref LargeConvex and SmallConvex to see how to optimize
/// ConvexBase's support computation.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupport(const ConvexBase* convex, const Vec3f& dir, Vec3f& support,
                     int& hint, ShapeSupportData* /*unused*/);

/// @brief Cast a `ConvexBase` to a `LargeConvex` to use the log version of
/// `getShapeSupport`. This is **much** faster than the linear version of
/// `getShapeSupport` when a `ConvexBase` has more than a few dozen of vertices.
/// @note WARNING: when using a LargeConvex, the neighbors in `ConvexBase` must
/// have been constructed! Otherwise the support function will segfault.
struct LargeConvex : ShapeBase {};
/// @brief See @ref LargeConvex.
struct SmallConvex : ShapeBase {};

template <int _SupportOptions>
inline void getShapeSupport(const SmallConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint, ShapeSupportData* data);

template <int _SupportOptions>
inline void getShapeSupport(const LargeConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint, ShapeSupportData* data);

}  // namespace details

}  // namespace fcl
}  // namespace hpp

#include "hpp/fcl/narrowphase/support_functions.hxx"

#endif  // HPP_FCL_SUPPORT_FUNCTIONS_H
