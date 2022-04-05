/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

/** \author Jia Pan */

#ifndef HPP_FCL_GEOMETRIC_SHAPES_UTILITY_H
#define HPP_FCL_GEOMETRIC_SHAPES_UTILITY_H

#include <vector>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/internal/BV_fitter.h>

namespace hpp {
namespace fcl {

/// @cond IGNORE
namespace details {
/// @brief get the vertices of some convex shape which can bound the given shape
/// in a specific configuration
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const Box& box,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const Sphere& sphere,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const Ellipsoid& ellipsoid,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const Capsule& capsule,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const Cone& cone,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const Cylinder& cylinder,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const ConvexBase& convex,
                                                   const Transform3f& tf);
HPP_FCL_DLLAPI std::vector<Vec3f> getBoundVertices(const TriangleP& triangle,
                                                   const Transform3f& tf);
}  // namespace details
/// @endcond

/// @brief calculate a bounding volume for a shape in a specific configuration
template <typename BV, typename S>
inline void computeBV(const S& s, const Transform3f& tf, BV& bv) {
  std::vector<Vec3f> convex_bound_vertices = details::getBoundVertices(s, tf);
  fit(&convex_bound_vertices[0], (unsigned int)convex_bound_vertices.size(),
      bv);
}

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Box>(const Box& s, const Transform3f& tf,
                                         AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Sphere>(const Sphere& s,
                                            const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Ellipsoid>(const Ellipsoid& e,
                                               const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Capsule>(const Capsule& s,
                                             const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Cone>(const Cone& s, const Transform3f& tf,
                                          AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Cylinder>(const Cylinder& s,
                                              const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, ConvexBase>(const ConvexBase& s,
                                                const Transform3f& tf,
                                                AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, TriangleP>(const TriangleP& s,
                                               const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Halfspace>(const Halfspace& s,
                                               const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<AABB, Plane>(const Plane& s,
                                           const Transform3f& tf, AABB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Box>(const Box& s, const Transform3f& tf,
                                        OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Sphere>(const Sphere& s,
                                           const Transform3f& tf, OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Capsule>(const Capsule& s,
                                            const Transform3f& tf, OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Cone>(const Cone& s, const Transform3f& tf,
                                         OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Cylinder>(const Cylinder& s,
                                             const Transform3f& tf, OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, ConvexBase>(const ConvexBase& s,
                                               const Transform3f& tf, OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Halfspace>(const Halfspace& s,
                                              const Transform3f& tf, OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<RSS, Halfspace>(const Halfspace& s,
                                              const Transform3f& tf, RSS& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBBRSS, Halfspace>(const Halfspace& s,
                                                 const Transform3f& tf,
                                                 OBBRSS& bv);

template <>
HPP_FCL_DLLAPI void computeBV<kIOS, Halfspace>(const Halfspace& s,
                                               const Transform3f& tf, kIOS& bv);

template <>
HPP_FCL_DLLAPI void computeBV<KDOP<16>, Halfspace>(const Halfspace& s,
                                                   const Transform3f& tf,
                                                   KDOP<16>& bv);

template <>
HPP_FCL_DLLAPI void computeBV<KDOP<18>, Halfspace>(const Halfspace& s,
                                                   const Transform3f& tf,
                                                   KDOP<18>& bv);

template <>
HPP_FCL_DLLAPI void computeBV<KDOP<24>, Halfspace>(const Halfspace& s,
                                                   const Transform3f& tf,
                                                   KDOP<24>& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBB, Plane>(const Plane& s, const Transform3f& tf,
                                          OBB& bv);

template <>
HPP_FCL_DLLAPI void computeBV<RSS, Plane>(const Plane& s, const Transform3f& tf,
                                          RSS& bv);

template <>
HPP_FCL_DLLAPI void computeBV<OBBRSS, Plane>(const Plane& s,
                                             const Transform3f& tf, OBBRSS& bv);

template <>
HPP_FCL_DLLAPI void computeBV<kIOS, Plane>(const Plane& s,
                                           const Transform3f& tf, kIOS& bv);

template <>
HPP_FCL_DLLAPI void computeBV<KDOP<16>, Plane>(const Plane& s,
                                               const Transform3f& tf,
                                               KDOP<16>& bv);

template <>
HPP_FCL_DLLAPI void computeBV<KDOP<18>, Plane>(const Plane& s,
                                               const Transform3f& tf,
                                               KDOP<18>& bv);

template <>
HPP_FCL_DLLAPI void computeBV<KDOP<24>, Plane>(const Plane& s,
                                               const Transform3f& tf,
                                               KDOP<24>& bv);

/// @brief construct a box shape (with a configuration) from a given bounding
/// volume
HPP_FCL_DLLAPI void constructBox(const AABB& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const OBB& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const OBBRSS& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const kIOS& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const RSS& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const KDOP<16>& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const KDOP<18>& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const KDOP<24>& bv, Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const AABB& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const OBB& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const OBBRSS& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const kIOS& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const RSS& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const KDOP<16>& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const KDOP<18>& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI void constructBox(const KDOP<24>& bv, const Transform3f& tf_bv,
                                 Box& box, Transform3f& tf);

HPP_FCL_DLLAPI Halfspace transform(const Halfspace& a, const Transform3f& tf);

HPP_FCL_DLLAPI Plane transform(const Plane& a, const Transform3f& tf);

}  // namespace fcl

}  // namespace hpp

#endif
