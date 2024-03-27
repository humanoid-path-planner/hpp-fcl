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

#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/internal/intersect.h>
#include "details.h"

namespace hpp {
namespace fcl {

template <>
bool GJKSolver::shapeTriangleInteraction(
    const Sphere& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
    const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance, bool,
    Vec3f& p1, Vec3f& p2, Vec3f& normal) const {
  return details::sphereTriangleIntersect(s, tf1, tf2.transform(P1),
                                          tf2.transform(P2), tf2.transform(P3),
                                          distance, p1, p2, normal);
}

template <>
bool GJKSolver::shapeTriangleInteraction(
    const Halfspace& s, const Transform3f& tf1, const Vec3f& P1,
    const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
    FCL_REAL& distance, bool, Vec3f& p1, Vec3f& p2, Vec3f& normal) const {
  return details::halfspaceTriangleIntersect(s, tf1, P1, P2, P3, tf2, distance,
                                             p1, p2, normal);
}

template <>
bool GJKSolver::shapeTriangleInteraction(const Plane& s, const Transform3f& tf1,
                                         const Vec3f& P1, const Vec3f& P2,
                                         const Vec3f& P3,
                                         const Transform3f& tf2,
                                         FCL_REAL& distance, bool, Vec3f& p1,
                                         Vec3f& p2, Vec3f& normal) const {
  return details::planeTriangleIntersect(s, tf1, P1, P2, P3, tf2, distance, p1,
                                         p2, normal);
}

// Shape distance algorithms not using built-in GJK algorithm
//
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// |            | box | sphere | capsule | cone | cylinder | plane | half-space
// | triangle |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | box        |     |   O    |         |      |          |       | | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | sphere     |/////|   O    |    O    |      |    O     |       | | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | capsule    |/////|////////|    O    |      |          |       | | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | cone       |/////|////////|/////////|      |          |       | | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | cylinder   |/////|////////|/////////|//////|          |       | | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | plane      |/////|////////|/////////|//////|//////////|       | | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | half-space |/////|////////|/////////|//////|//////////|///////| | |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+
// | triangle |/////|////////|/////////|//////|//////////|///////|////////////|
// |
// +------------+-----+--------+---------+------+----------+-------+------------+----------+

template <>
bool GJKSolver::shapeDistance<Sphere, Capsule>(const Sphere& s1,
                                               const Transform3f& tf1,
                                               const Capsule& s2,
                                               const Transform3f& tf2,
                                               FCL_REAL& dist, bool, Vec3f& p1,
                                               Vec3f& p2, Vec3f& normal) const {
  return details::sphereCapsuleDistance(s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template <>
bool GJKSolver::shapeDistance<Capsule, Sphere>(const Capsule& s1,
                                               const Transform3f& tf1,
                                               const Sphere& s2,
                                               const Transform3f& tf2,
                                               FCL_REAL& dist, bool, Vec3f& p1,
                                               Vec3f& p2, Vec3f& normal) const {
  return details::sphereCapsuleDistance(s2, tf2, s1, tf1, dist, p2, p1, normal);
}

template <>
bool GJKSolver::shapeDistance<Box, Sphere>(const Box& s1,
                                           const Transform3f& tf1,
                                           const Sphere& s2,
                                           const Transform3f& tf2,
                                           FCL_REAL& dist, bool, Vec3f& p1,
                                           Vec3f& p2, Vec3f& normal) const {
  return !details::boxSphereDistance(s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template <>
bool GJKSolver::shapeDistance<Sphere, Box>(const Sphere& s1,
                                           const Transform3f& tf1,
                                           const Box& s2,
                                           const Transform3f& tf2,
                                           FCL_REAL& dist, bool, Vec3f& p1,
                                           Vec3f& p2, Vec3f& normal) const {
  bool collide =
      details::boxSphereDistance(s2, tf2, s1, tf1, dist, p2, p1, normal);
  normal *= -1;
  return !collide;
}

template <>
bool GJKSolver::shapeDistance<Sphere, Cylinder>(
    const Sphere& s1, const Transform3f& tf1, const Cylinder& s2,
    const Transform3f& tf2, FCL_REAL& dist, bool, Vec3f& p1, Vec3f& p2,
    Vec3f& normal) const {
  return details::sphereCylinderDistance(s1, tf1, s2, tf2, dist, p1, p2,
                                         normal);
}

template <>
bool GJKSolver::shapeDistance<Cylinder, Sphere>(
    const Cylinder& s1, const Transform3f& tf1, const Sphere& s2,
    const Transform3f& tf2, FCL_REAL& dist, bool, Vec3f& p1, Vec3f& p2,
    Vec3f& normal) const {
  return details::sphereCylinderDistance(s2, tf2, s1, tf1, dist, p2, p1,
                                         normal);
}

template <>
bool GJKSolver::shapeDistance<Sphere, Sphere>(const Sphere& s1,
                                              const Transform3f& tf1,
                                              const Sphere& s2,
                                              const Transform3f& tf2,
                                              FCL_REAL& dist, bool, Vec3f& p1,
                                              Vec3f& p2, Vec3f& normal) const {
  return details::sphereSphereDistance(s1, tf1, s2, tf2, dist, p1, p2, normal);
}

template <>
bool GJKSolver::shapeDistance<Capsule, Capsule>(
    const Capsule& /*s1*/, const Transform3f& /*tf1*/, const Capsule& /*s2*/,
    const Transform3f& /*tf2*/, FCL_REAL& /*dist*/, bool, Vec3f& /*p1*/,
    Vec3f& /*p2*/, Vec3f& /*normal*/) const {
  abort();
}

template <>
bool GJKSolver::shapeDistance<TriangleP, TriangleP>(
    const TriangleP& s1, const Transform3f& tf1, const TriangleP& s2,
    const Transform3f& tf2, FCL_REAL& dist, bool, Vec3f& p1, Vec3f& p2,
    Vec3f& normal) const {
  const TriangleP t1(tf1.transform(s1.a), tf1.transform(s1.b),
                     tf1.transform(s1.c)),
      t2(tf2.transform(s2.a), tf2.transform(s2.b), tf2.transform(s2.c));

  bool constexpr use_inflated_supports = false;
  minkowski_difference.set<use_inflated_supports>(&t1, &t2);

  // Reset GJK algorithm
  gjk.reset(gjk_max_iterations, gjk_tolerance);

  // Get GJK initial guess
  Vec3f guess;
  Vec3f default_guess = (t1.a + t1.b + t1.c - t2.a - t2.b - t2.c) / 3;
  support_func_guess_t support_hint;
  getGJKInitialGuess(t1, t2, guess, support_hint, default_guess);
  epa.status = details::EPA::DidNotRun;  // EPA is never called in this function

  details::GJK::Status gjk_status =
      gjk.evaluate(minkowski_difference, guess, support_hint);
  if (gjk_initial_guess == GJKInitialGuess::CachedGuess ||
      enable_cached_guess) {
    cached_guess = gjk.getGuessFromSimplex();
    support_func_cached_guess = gjk.support_hint;
  }

  gjk.getWitnessPointsAndNormal(minkowski_difference, p1, p2, normal);

  if (gjk_status != details::GJK::Collision) {
    // TODO On degenerated case, the closest point may be wrong
    // (i.e. an object face normal is colinear to gjk.ray
    // assert (dist == (w0 - w1).norm());
    dist = gjk.distance;
    assert(gjk.ray.norm() > gjk.getTolerance());
    return true;
  } else {
    FCL_REAL penetrationDepth =
        details::computePenetration(t1.a, t1.b, t1.c, t2.a, t2.b, t2.c, normal);
    dist = -penetrationDepth;
    return dist > 0;
  }
  assert(false && "should not reach this point");
  return false;
}
}  // namespace fcl

}  // namespace hpp
