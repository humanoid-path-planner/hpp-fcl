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
#include "hpp/fcl/collision_data.h"

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

// ============================================================================
// ============================ SUPPORT FUNCTIONS =============================
// ============================================================================
/// @brief the support function for shape.
/// The output support point is expressed in the local frame of the shape.
/// @return a point which belongs to the set {argmax_{v in shape} v.dot(dir)}.
/// @param shape the shape.
/// @param dir support direction.
/// @param hint used to initialize the search when shape is a ConvexBase object.
/// @tparam SupportOptions is a value of the SupportOptions enum. If set to
/// `WithSweptSphere`, the support functions take into account the shapes' swept
/// sphere radii. Please see `MinkowskiDiff::set(const ShapeBase*, const
/// ShapeBase*)` for more details.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir,
                                int& hint);

/// @brief Stores temporary data for the computation of support points.
struct HPP_FCL_DLLAPI ShapeSupportData {
  std::vector<int8_t> visited;
  Vec3f last_dir = Vec3f::Zero();
};

/// @brief Triangle support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const TriangleP* triangle, const Vec3f& dir,
                                    Vec3f& support, int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief Box support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const Box* box, const Vec3f& dir,
                                    Vec3f& support, int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief Sphere support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const Sphere* sphere, const Vec3f& dir,
                                    Vec3f& support, int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief Ellipsoid support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const Ellipsoid* ellipsoid,
                                    const Vec3f& dir, Vec3f& support,
                                    int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief Capsule support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const Capsule* capsule, const Vec3f& dir,
                                    Vec3f& support, int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief Cone support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const Cone* cone, const Vec3f& dir,
                                    Vec3f& support, int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief Cylinder support function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir,
                                    Vec3f& support, int& /*unused*/,
                                    ShapeSupportData* /*unused*/);

/// @brief ConvexBase support function.
/// @note See @ref LargeConvex and SmallConvex to see how to optimize
/// ConvexBase's support computation.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const ConvexBase* convex, const Vec3f& dir,
                                    Vec3f& support, int& hint,
                                    ShapeSupportData* /*unused*/);

/// @brief Cast a `ConvexBase` to a `LargeConvex` to use the log version of
/// `getShapeSupport`. This is **much** faster than the linear version of
/// `getShapeSupport` when a `ConvexBase` has more than a few dozen of vertices.
/// @note WARNING: when using a LargeConvex, the neighbors in `ConvexBase` must
/// have been constructed! Otherwise the support function will segfault.
struct LargeConvex : ShapeBase {};
/// @brief See @ref LargeConvex.
struct SmallConvex : ShapeBase {};

/// @brief Support function for large ConvexBase (>32 vertices).
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const SmallConvex* convex, const Vec3f& dir,
                                    Vec3f& support, int& hint,
                                    ShapeSupportData* data);

/// @brief Support function for small ConvexBase (<32 vertices).
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupport(const LargeConvex* convex, const Vec3f& dir,
                                    Vec3f& support, int& hint,
                                    ShapeSupportData* data);

/// @brief Templated version of @ref getSupport.
template <typename ShapeType,
          int _SupportOptions = SupportOptions::NoSweptSphere>
Vec3f getSupportTpl(const ShapeBase* shape, const Vec3f& dir, int& hint) {
  const ShapeType* shape_ = static_cast<const ShapeType*>(shape);
  Vec3f support;
  getShapeSupport(shape_, dir, support, hint, nullptr);
  return support;
}

/// @brief Templated version of the getShapeSupport functions.
/// The `getShapeSupport` functions differ from `getSupport` functions in
/// that they take an additional `ShapeSupportData`, used for ConvexBase support
/// set computation, which can be manually allocated by the user.
/// See @ref LargeConvex and @SmallConvex for more info.
template <typename ShapeType,
          int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportTpl(const ShapeBase* shape, const Vec3f& dir,
                        Vec3f& support, int& hint, ShapeSupportData* data) {
  const ShapeType* shape_ = static_cast<const ShapeType*>(shape);
  return getShapeSupport(shape_, dir, support, hint, data);
}

// ============================================================================
// ========================== SUPPORT SET FUNCTIONS ===========================
// ============================================================================
/// @brief the support set function for shape.
/// All the points belonging to the output support set are expressed in the
/// frame c. This function returns an approximation of the support set of shape,
/// in direction dir. The "approximation" depends on the size of the provided
/// input `SupportSet`. If the provided support set has a size of 4 for
/// examples, it can fully capture the support set of a cube in the direction of
/// one of its face. However, if instead we compute the support set of a
/// cylinder in the direction of one of its base, we actually need an infinite
/// amount of points to capture the circle of the base. In such a case, if the
/// provided support set has a size of 4, the support set function smartly
/// selects a subset of this infinite support set.
/// @return an approximation of the set {argmax_{v in shape} v.dot(dir)}.
/// @param[in] shape the shape.
/// @param[in] dir support direction.
/// @param[in] ctfi transform from local frame of shape i to frame c.
/// @param[in] tol given a point v on the shape, if
/// `max_{v in shape}(v.dot(dit)) - v.dot(dir) <= tol` then v is added to the
/// support set.
/// @param[in/out] support_set of shape in direction dir, expressed in the frame
/// c.
/// @param[in] hint used to initialize the search when shape is a ConvexBase
/// object.
/// @tparam SupportOptions is a value of the SupportOptions enum. If set to
/// `WithSweptSphere`, the support functions take into account the shapes' swept
/// sphere radii. Please see `MinkowskiDiff::set(const ShapeBase*, const
/// ShapeBase*)` for more details.
///
/// @note The parameter `tol` can be seen as the "thickness" of the support
/// plane. Any point v which satisfies `max_{v in shape}(v.dot(dit)) -
/// v.dot(dir) <= tol` is tol distant from the support plane.
///
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getSupportSet(const ShapeBase* shape, const Vec3f& dir,
                                  const Transform3f& ctfi, FCL_REAL tol,
                                  SupportSet& support_set, const int hint);

/// @brief Triangle support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const TriangleP* triangle,
                                       const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief Box support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const Box* box, const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief Sphere support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const Sphere* sphere, const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief Ellipsoid support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const Ellipsoid* ellipsoid,
                                       const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief Capsule support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const Capsule* capsule, const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief Cone support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const Cone* cone, const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief Cylinder support set function.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const Cylinder* cylinder,
                                       const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set,
                                       const int /*unused*/,
                                       ShapeSupportData* /*unused*/);

/// @brief ConvexBase support set function.
/// @note See @ref LargeConvex and SmallConvex to see how to optimize
/// ConvexBase's support computation.
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const ConvexBase* convex,
                                       const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set, const int hint,
                                       ShapeSupportData* /*unused*/);

/// @brief Support set function for large ConvexBase (>32 vertices).
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const SmallConvex* convex,
                                       const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set, const int hint,
                                       ShapeSupportData* data);

/// @brief Support set function for small ConvexBase (<32 vertices).
template <int _SupportOptions = SupportOptions::NoSweptSphere>
HPP_FCL_DLLAPI void getShapeSupportSet(const LargeConvex* convex,
                                       const Vec3f& dir,
                                       const Transform3f& ctfi,
                                       SupportSet& support_set, const int hint,
                                       ShapeSupportData* data);

/// @brief Templated support set function, i.e. templated version of @ref
/// getSupportSet.
template <typename ShapeType,
          int _SupportOptions = SupportOptions::NoSweptSphere>
void getSupportSetTpl(const ShapeBase* shape, const Vec3f& dir,
                      const Transform3f& ctfi, SupportSet& support_set,
                      const int hint) {
  const ShapeType* shape_ = static_cast<const ShapeType*>(shape);
  getShapeSupportSet<_SupportOptions>(shape_, dir, ctfi, support_set, hint,
                                      nullptr);
}

/// @brief Templated shape support set functions.
/// The `getShapeSupportSet` functions differ from `getSupportSet` function in
/// that they take an additional `ShapeSupportData`, used for ConvexBase support
/// set computation, which can be manually allocated by the user. See @ref
/// LargeConvex and @SmallConvex for more info.
template <typename ShapeType,
          int _SupportOptions = SupportOptions::NoSweptSphere>
void getShapeSupportSetTpl(const ShapeBase* shape, const Vec3f& dir,
                           const Transform3f& ctfi, SupportSet& support_set,
                           const int hint, ShapeSupportData* data) {
  const ShapeType* shape_ = static_cast<const ShapeType*>(shape);
  getShapeSupportSet<_SupportOptions>(shape_, dir, ctfi, support_set, hint,
                                      data);
}

}  // namespace details

}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_SUPPORT_FUNCTIONS_H
