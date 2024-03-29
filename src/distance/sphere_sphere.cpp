/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019, CNRS
 *  Author: Florent Lamiraux
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

#include <cmath>
#include <limits>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/internal/shape_shape_func.h>
#include <hpp/fcl/internal/traversal_node_base.h>

// Note that partial specialization of template functions is not allowed.
// Therefore, two implementations with the default narrow phase solvers are
// provided. If another narrow phase solver were to be used, the default
// template implementation would be called, unless the function is also
// specialized for this new type.
//
// One solution would be to make narrow phase solvers derive from an abstract
// class and specialize the template for this abstract class.
namespace hpp {
namespace fcl {
struct GJKSolver;

template <>
FCL_REAL ShapeShapeDistance<Sphere, Sphere>(
    const CollisionGeometry* o1, const Transform3f& tf1,
    const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver*,
    const DistanceRequest&, DistanceResult& result) {
  FCL_REAL epsilon = 1e-7;
  const Sphere* s1 = static_cast<const Sphere*>(o1);
  const Sphere* s2 = static_cast<const Sphere*>(o2);

  // We assume that spheres are centered at the origin of their frame.
  const fcl::Vec3f& center1 = tf1.getTranslation();
  const fcl::Vec3f& center2 = tf2.getTranslation();
  FCL_REAL r1 = s1->radius;
  FCL_REAL r2 = s2->radius;

  result.o1 = o1;
  result.o2 = o2;
  result.b1 = result.b2 = -1;
  Vec3f c1c2 = center2 - center1;
  FCL_REAL dist = c1c2.norm();
  Vec3f unit(0, 0, 0);
  if (dist > epsilon) unit = c1c2 / dist;
  result.min_distance = dist - (r1 + r2);
  result.normal = unit;
  result.nearest_points[0] = center1 + r1 * unit;
  result.nearest_points[1] = center2 - r2 * unit;
  return result.min_distance;
}

std::size_t ShapeShapeCollider<Sphere, Sphere>::run(
    const CollisionGeometry* o1, const Transform3f& tf1,
    const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver*,
    const CollisionRequest& request, CollisionResult& result) {
  FCL_REAL epsilon = 1e-7;
  const Sphere* s1 = static_cast<const Sphere*>(o1);
  const Sphere* s2 = static_cast<const Sphere*>(o2);

  // We assume that spheres are centered at the origin.
  const fcl::Vec3f& center1 = tf1.getTranslation();
  const fcl::Vec3f& center2 = tf2.getTranslation();
  FCL_REAL r1 = s1->radius;
  FCL_REAL r2 = s2->radius;
  FCL_REAL margin = request.security_margin;

  Vec3f c1c2 = center2 - center1;
  FCL_REAL dist = c1c2.norm();
  Vec3f normal(0, 0, 0);
  if (dist > epsilon) normal = c1c2 / dist;
  // Unlike in distance computation, we consider the security margin.
  FCL_REAL distToCollision = dist - (r1 + r2 + margin);

  Vec3f p1 = center1 + normal * r1;
  Vec3f p2 = center2 - normal * r2;
  internal::updateDistanceLowerBoundFromLeaf(request, result, distToCollision,
                                             p1, p2, normal);
  if (distToCollision <= request.collision_distance_threshold) {
    Contact contact(o1, o2, -1, -1, p1, p2, normal, distToCollision + margin);
    result.addContact(contact);
    return 1;
  }
  return 0;
}
}  // namespace fcl

}  // namespace hpp
