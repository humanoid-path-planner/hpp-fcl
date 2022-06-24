/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CNRS-LAAS
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Florent Lamiraux */

#ifndef HPP_FCL_INTERNAL_SHAPE_SHAPE_FUNC_H
#define HPP_FCL_INTERNAL_SHAPE_SHAPE_FUNC_H

/// @cond INTERNAL

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_utility.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes_traits.h>

namespace hpp {
namespace fcl {

template <typename T_SH1, typename T_SH2>
HPP_FCL_DLLAPI FCL_REAL ShapeShapeDistance(const CollisionGeometry* o1,
                                           const Transform3f& tf1,
                                           const CollisionGeometry* o2,
                                           const Transform3f& tf2,
                                           const GJKSolver* nsolver,
                                           const DistanceRequest& request,
                                           DistanceResult& result);

template <typename T_SH1, typename T_SH2>
struct ShapeShapeCollider {
  static std::size_t run(const CollisionGeometry* o1, const Transform3f& tf1,
                         const CollisionGeometry* o2, const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();

    DistanceResult distanceResult;
    DistanceRequest distanceRequest(request.enable_contact);
    FCL_REAL distance = ShapeShapeDistance<T_SH1, T_SH2>(
        o1, tf1, o2, tf2, nsolver, distanceRequest, distanceResult);

    size_t num_contacts = 0;
    const Vec3f& p1 = distanceResult.nearest_points[0];
    const Vec3f& p2 = distanceResult.nearest_points[1];
    FCL_REAL distToCollision = distance - request.security_margin;

    internal::updateDistanceLowerBoundFromLeaf(request, result, distToCollision,
                                               p1, p2);
    if (distToCollision <= request.collision_distance_threshold &&
        result.numContacts() < request.num_max_contacts) {
      if (result.numContacts() < request.num_max_contacts) {
        const Vec3f& p1 = distanceResult.nearest_points[0];
        const Vec3f& p2 = distanceResult.nearest_points[1];

        Contact contact(
            o1, o2, distanceResult.b1, distanceResult.b2, (p1 + p2) / 2,
            (distance <= 0 ? distanceResult.normal : (p2 - p1).normalized()),
            -distance);

        result.addContact(contact);
      }
      num_contacts = result.numContacts();
    }

    return num_contacts;
  }
};

template <typename ShapeType1, typename ShapeType2>
std::size_t ShapeShapeCollide(const CollisionGeometry* o1,
                              const Transform3f& tf1,
                              const CollisionGeometry* o2,
                              const Transform3f& tf2, const GJKSolver* nsolver,
                              const CollisionRequest& request,
                              CollisionResult& result) {
  return ShapeShapeCollider<ShapeType1, ShapeType2>::run(
      o1, tf1, o2, tf2, nsolver, request, result);
}

#define SHAPE_SHAPE_DISTANCE_SPECIALIZATION(T1, T2)             \
  template <>                                                   \
  HPP_FCL_DLLAPI FCL_REAL ShapeShapeDistance<T1, T2>(           \
      const CollisionGeometry* o1, const Transform3f& tf1,      \
      const CollisionGeometry* o2, const Transform3f& tf2,      \
      const GJKSolver* nsolver, const DistanceRequest& request, \
      DistanceResult& result);                                  \
  template <>                                                   \
  HPP_FCL_DLLAPI FCL_REAL ShapeShapeDistance<T2, T1>(           \
      const CollisionGeometry* o1, const Transform3f& tf1,      \
      const CollisionGeometry* o2, const Transform3f& tf2,      \
      const GJKSolver* nsolver, const DistanceRequest& request, \
      DistanceResult& result)

SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Sphere);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Capsule);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cone, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cone, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cylinder, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cylinder, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Sphere);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Cylinder);

SHAPE_SHAPE_DISTANCE_SPECIALIZATION(ConvexBase, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(TriangleP, Halfspace);

#undef SHAPE_SHAPE_DISTANCE_SPECIALIZATION

#define SHAPE_SHAPE_COLLIDE_SPECIALIZATION(T1, T2)                         \
  template <>                                                              \
  struct ShapeShapeCollider<T1, T2> {                                      \
    static HPP_FCL_DLLAPI std::size_t run(const CollisionGeometry* o1,     \
                                          const Transform3f& tf1,          \
                                          const CollisionGeometry* o2,     \
                                          const Transform3f& tf2,          \
                                          const GJKSolver* nsolver,        \
                                          const CollisionRequest& request, \
                                          CollisionResult& result);        \
  }

SHAPE_SHAPE_COLLIDE_SPECIALIZATION(Sphere, Sphere);

#undef SHAPE_SHAPE_COLLIDE_SPECIALIZATION
}  // namespace fcl

}  // namespace hpp

/// @endcond

#endif
