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

#include <hpp/fcl/narrowphase/narrowphase.h>

#include <vector>

#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/internal/intersect.h>
#include "details.h"

namespace hpp
{
namespace fcl
{
// Shape intersect algorithms based on:
// - built-in function: 0
// - GJK:               1
//
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | box        |  0  |   0    |    1    |   1  |    1     |   0   |      0     |    1     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   0    |    0    |   1  |    1     |   0   |      0     |    0     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|    1    |   1  |    1     |   0   |      0     |    1     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|/////////|   1  |    1     |   0   |      0     |    1     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|/////////|//////|    1     |   0   |      0     |    1     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|/////////|//////|//////////|   0   |      0     |    0     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|/////////|//////|//////////|///////|      0     |    0     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|/////////|//////|//////////|///////|////////////|    1     |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+


#define SHAPE_INTERSECT_INVERTED(Shape1,Shape2)                                \
  template<>                                                                   \
    bool GJKSolver::shapeIntersect<Shape1, Shape2>                             \
    (const Shape1& s1, const Transform3f& tf1,                                 \
     const Shape2& s2, const Transform3f& tf2,                                 \
     FCL_REAL& distance_lower_bound, bool enable_penetration,                  \
     Vec3f* contact_points, Vec3f* normal) const                               \
  {                                                                            \
    bool res = shapeIntersect (s2, tf2, s1, tf1, distance_lower_bound,         \
        enable_penetration, contact_points, normal);                           \
    (*normal) *= -1.0;                                                         \
    return res;                                                                \
  }

template<>
bool GJKSolver::shapeIntersect<Sphere, Capsule>
 (const Sphere &s1, const Transform3f& tf1,
  const Capsule &s2, const Transform3f& tf2,
  FCL_REAL& distance_lower_bound,
  bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  return details::sphereCapsuleIntersect(s1, tf1, s2, tf2, distance_lower_bound,
      contact_points, normal);
}

SHAPE_INTERSECT_INVERTED(Capsule, Sphere)

template<>
bool GJKSolver::shapeIntersect<Sphere, Sphere>
 (const Sphere& s1, const Transform3f& tf1,
  const Sphere& s2, const Transform3f& tf2,
  FCL_REAL& distance_lower_bound,
  bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  return details::sphereSphereIntersect(s1, tf1, s2, tf2, distance_lower_bound,
      contact_points, normal);
}

template<>
bool GJKSolver::shapeIntersect<Box, Sphere>
 (const Box   & s1, const Transform3f& tf1,
  const Sphere& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f ps, pb, n;
  bool res = details::boxSphereDistance (s1, tf1, s2, tf2, distance, ps, pb, n);
  if (normal)            *normal = n;
  if (contact_points)    *contact_points = pb;
  return res;
}

SHAPE_INTERSECT_INVERTED(Sphere, Box)

/*
template<>
bool GJKSolver::shapeIntersect<Box, Box>(const Box& s1, const Transform3f& tf1,
                                         const Box& s2, const Transform3f& tf2,
                                         FCL_REAL& distance_lower_bound,
                                         bool,
                                         Vec3f* contact_points, Vec3f* normal) const
{
  return details::boxBoxIntersect(s1, tf1, s2, tf2, contact_points, &distance_lower_bound, normal);
}
*/

template<>
bool GJKSolver::shapeIntersect<Sphere, Halfspace>
 (const Sphere& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::sphereHalfspaceIntersect(s1, tf1, s2, tf2, distance, p1,
                                               p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Halfspace, Sphere)

template<>
bool GJKSolver::shapeIntersect<Box, Halfspace>
 (const Box& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::boxHalfspaceIntersect(s1, tf1, s2, tf2, distance, p1,
                                            p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Halfspace, Box)

template<>
bool GJKSolver::shapeIntersect<Capsule, Halfspace>
 (const Capsule& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res =  details::capsuleHalfspaceIntersect
    (s1, tf1, s2, tf2, distance, p1, p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Halfspace, Capsule)

template<>
bool GJKSolver::shapeIntersect<Cylinder, Halfspace>
 (const Cylinder& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res =  details::cylinderHalfspaceIntersect
    (s1, tf1, s2, tf2, distance, p1, p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Halfspace, Cylinder)

template<>
bool GJKSolver::shapeIntersect<Cone, Halfspace>
 (const Cone& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res =  details::coneHalfspaceIntersect
    (s1, tf1, s2, tf2, distance, p1, p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Halfspace, Cone)

template<>
bool GJKSolver::shapeIntersect<Halfspace, Halfspace>
 (const Halfspace& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* /*contact_points*/, Vec3f* /*normal*/) const
{
  Halfspace s;
  Vec3f p, d;
  FCL_REAL depth;
  int ret;
  bool res = details::halfspaceIntersect(s1, tf1, s2, tf2, p, d, s, depth, ret);
  distance = - depth;
  return res;
}

template<>
bool GJKSolver::shapeIntersect<Plane, Halfspace>
 (const Plane& s1, const Transform3f& tf1,
  const Halfspace& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* /*contact_points*/, Vec3f* /*normal*/) const
{
  Plane pl;
  Vec3f p, d;
  FCL_REAL depth;
  int ret;
  bool res = details::planeHalfspaceIntersect(s1, tf1, s2, tf2, pl, p, d, depth, ret);
  distance = - depth;
  return res;
}

SHAPE_INTERSECT_INVERTED(Halfspace, Plane)

template<>
bool GJKSolver::shapeIntersect<Sphere, Plane>
 (const Sphere& s1, const Transform3f& tf1,
  const Plane& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::spherePlaneIntersect(s1, tf1, s2, tf2, distance, p1,
                                           p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Plane, Sphere)

template<>
bool GJKSolver::shapeIntersect<Box, Plane>
 (const Box& s1, const Transform3f& tf1,
  const Plane& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::boxPlaneIntersect
    (s1, tf1, s2, tf2, distance, p1, p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Plane, Box)

template<>
bool GJKSolver::shapeIntersect<Capsule, Plane>
 (const Capsule& s1, const Transform3f& tf1,
  const Plane& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::capsulePlaneIntersect(s1, tf1, s2, tf2, distance, p1,
                                            p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Plane, Capsule)

template<>
bool GJKSolver::shapeIntersect<Cylinder, Plane>
 (const Cylinder& s1, const Transform3f& tf1,
  const Plane& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::cylinderPlaneIntersect
    (s1, tf1, s2, tf2, distance, p1, p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Plane, Cylinder)

template<>
bool GJKSolver::shapeIntersect<Cone, Plane>
 (const Cone& s1, const Transform3f& tf1,
  const Plane& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  Vec3f p1, p2, n;
  bool res = details::conePlaneIntersect
    (s1, tf1, s2, tf2, distance, p1, p2, n);
  if (contact_points) *contact_points = p1;
  if (normal) *normal = n;
  return res;
}

SHAPE_INTERSECT_INVERTED(Plane, Cone)

template<>
bool GJKSolver::shapeIntersect<Plane, Plane>
 (const Plane& s1, const Transform3f& tf1,
  const Plane& s2, const Transform3f& tf2,
  FCL_REAL& distance, bool,
  Vec3f* contact_points, Vec3f* normal) const
{
  return details::planeIntersect(s1, tf1, s2, tf2, contact_points, &distance, normal);
}

template<>
bool GJKSolver::shapeTriangleInteraction
 (const Sphere& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
  const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
  Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::sphereTriangleIntersect
    (s, tf1, tf2.transform(P1), tf2.transform(P2), tf2.transform(P3),
     distance, p1, p2, normal);
}

template<>
bool GJKSolver::shapeTriangleInteraction
 (const Halfspace& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
  const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
  Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::halfspaceTriangleIntersect
    (s, tf1, P1, P2, P3, tf2, distance, p1, p2, normal);
}

template<>
bool GJKSolver::shapeTriangleInteraction
 (const Plane& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
  const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
  Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::planeTriangleIntersect
    (s, tf1, P1, P2, P3, tf2, distance, p1, p2, normal);
}

// Shape distance algorithms not using built-in GJK algorithm
//
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | capsule | cone | cylinder | plane | half-space | triangle |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | box        |     |   O    |         |      |          |       |            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |    O    |      |          |       |            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|    O    |      |          |       |            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|/////////|      |          |       |            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|/////////|//////|          |       |            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|/////////|//////|//////////|       |            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|/////////|//////|//////////|///////|            |          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | triangle   |/////|////////|/////////|//////|//////////|///////|////////////|          |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+

template<>
bool GJKSolver::shapeDistance<Sphere, Capsule>
(const Sphere& s1, const Transform3f& tf1,
 const Capsule& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::sphereCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template<>
bool GJKSolver::shapeDistance<Capsule, Sphere>
(const Capsule& s1, const Transform3f& tf1,
 const Sphere& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::sphereCapsuleDistance(s2, tf2, s1, tf1, dist, p2, p1, normal);
}

template<>
bool GJKSolver::shapeDistance<Box, Sphere>
(const Box   & s1, const Transform3f& tf1,
 const Sphere& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return !details::boxSphereDistance (s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template<>
bool GJKSolver::shapeDistance<Sphere, Box>
(const Sphere& s1, const Transform3f& tf1,
 const Box   & s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  bool collide = details::boxSphereDistance (s2, tf2, s1, tf1, dist, p2, p1, normal);
  normal *= -1;
  return !collide;
}

template<>
bool GJKSolver::shapeDistance<Sphere, Cylinder>
(const Sphere& s1, const Transform3f& tf1,
 const Cylinder& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::sphereCylinderDistance
    (s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template<>
bool GJKSolver::shapeDistance<Cylinder, Sphere>
(const Cylinder& s1, const Transform3f& tf1,
 const Sphere& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::sphereCylinderDistance
    (s2, tf2, s1, tf1, dist, p2, p1, normal);
}

template<>
bool GJKSolver::shapeDistance<Sphere, Sphere>
(const Sphere& s1, const Transform3f& tf1,
 const Sphere& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  return details::sphereSphereDistance(s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template<>
bool GJKSolver::shapeDistance<Capsule, Capsule>
(const Capsule& /*s1*/, const Transform3f& /*tf1*/,
 const Capsule& /*s2*/, const Transform3f& /*tf2*/,
 FCL_REAL& /*dist*/, Vec3f& /*p1*/, Vec3f& /*p2*/, Vec3f& /*normal*/) const
{
  abort ();
}

template<>
bool GJKSolver::shapeDistance<TriangleP, TriangleP>
(const TriangleP& s1, const Transform3f& tf1,
 const TriangleP& s2, const Transform3f& tf2,
 FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
{
  const TriangleP
  t1 (tf1.transform(s1.a), tf1.transform(s1.b), tf1.transform(s1.c)),
  t2 (tf2.transform(s2.a), tf2.transform(s2.b), tf2.transform(s2.c));
  
  Vec3f guess;
  support_func_guess_t support_hint;
  if(enable_cached_guess) {
    guess = cached_guess;
    support_hint = support_func_cached_guess;
  } else {
    support_hint.setZero();
    guess = (t1.a + t1.b + t1.c - t2.a - t2.b - t2.c) / 3;
  }
  bool enable_penetration = true;
  
  details::MinkowskiDiff shape;
  shape.set (&t1, &t2);
  
  details::GJK gjk((unsigned int) gjk_max_iterations, gjk_tolerance);
  details::GJK::Status gjk_status = gjk.evaluate(shape, guess, support_hint);
  if(enable_cached_guess) {
    cached_guess = gjk.getGuessFromSimplex();
    support_func_cached_guess = gjk.support_hint;
  }
  
  gjk.getClosestPoints (shape, p1, p2);
  
  if((gjk_status == details::GJK::Valid) ||
     (gjk_status == details::GJK::Failed))
  {
    // TODO On degenerated case, the closest point may be wrong
    // (i.e. an object face normal is colinear to gjk.ray
    // assert (dist == (w0 - w1).norm());
    dist = gjk.distance;
    
    return true;
  }
  else if (gjk_status == details::GJK::Inside)
  {
    if (enable_penetration) {
      FCL_REAL penetrationDepth = details::computePenetration
      (t1.a, t1.b, t1.c, t2.a, t2.b, t2.c, normal);
      dist = -penetrationDepth;
      assert (dist <= 1e-6);
      // GJK says Inside when below GJK.tolerance. So non intersecting
      // triangle may trigger "Inside" and have no penetration.
      return penetrationDepth < 0;
    }
    dist = 0;
    return false;
  }
  assert (false && "should not reach this point");
  return false;
}
} // fcl

} // namespace hpp
