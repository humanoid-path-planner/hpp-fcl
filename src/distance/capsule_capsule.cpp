/*
 * Software License Agreement (BSD License)
 *  Copyright (c) 2015-2021, CNRS-LAAS INRIA
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

/// Clamp num / denom in [0, 1]
FCL_REAL clamp(const FCL_REAL& num, const FCL_REAL& denom) {
  assert(denom >= 0.);
  if (num <= 0.)
    return 0.;
  else if (num >= denom)
    return 1.;
  else
    return num / denom;
}

/// Clamp s=s_n/s_d in [0, 1] and stores a + s * d in a_sd
void clamped_linear(Vec3f& a_sd, const Vec3f& a, const FCL_REAL& s_n,
                    const FCL_REAL& s_d, const Vec3f& d) {
  assert(s_d >= 0.);
  if (s_n <= 0.)
    a_sd = a;
  else if (s_n >= s_d)
    a_sd = a + d;
  else
    a_sd = a + s_n / s_d * d;
}

// Compute the distance between C1 and C2 by computing the distance
// between the two segments supporting the capsules.
// Match algorithm of Real-Time Collision Detection, Christer Ericson - Closest
// Point of Two Line Segments
template <>
FCL_REAL ShapeShapeDistance<Capsule, Capsule>(
    const CollisionGeometry* o1, const Transform3f& tf1,
    const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver*,
    const DistanceRequest& /*request*/, DistanceResult& result) {
  const Capsule* capsule1 = static_cast<const Capsule*>(o1);
  const Capsule* capsule2 = static_cast<const Capsule*>(o2);

  FCL_REAL EPSILON = std::numeric_limits<FCL_REAL>::epsilon() * 100;

  // We assume that capsules are centered at the origin.
  const fcl::Vec3f& c1 = tf1.getTranslation();
  const fcl::Vec3f& c2 = tf2.getTranslation();
  // We assume that capsules are oriented along z-axis.
  FCL_REAL halfLength1 = capsule1->halfLength;
  FCL_REAL halfLength2 = capsule2->halfLength;
  FCL_REAL radius1 = capsule1->radius;
  FCL_REAL radius2 = capsule2->radius;
  // direction of capsules
  // ||d1|| = 2 * halfLength1
  const fcl::Vec3f d1 = 2 * halfLength1 * tf1.getRotation().col(2);
  const fcl::Vec3f d2 = 2 * halfLength2 * tf2.getRotation().col(2);

  // Starting point of the segments
  // p1 + d1 is the end point of the segment
  const fcl::Vec3f p1 = c1 - d1 / 2;
  const fcl::Vec3f p2 = c2 - d2 / 2;
  const fcl::Vec3f r = p1 - p2;
  FCL_REAL a = d1.dot(d1);
  FCL_REAL b = d1.dot(d2);
  FCL_REAL c = d1.dot(r);
  FCL_REAL e = d2.dot(d2);
  FCL_REAL f = d2.dot(r);
  // S1 is parametrized by the equation p1 + s * d1
  // S2 is parametrized by the equation p2 + t * d2

  Vec3f w1, w2;
  if (a <= EPSILON) {
    w1 = p1;
    if (e <= EPSILON)
      // Check if the segments degenerate into points
      w2 = p2;
    else
      // First segment is degenerated
      clamped_linear(w2, p2, f, e, d2);
  } else if (e <= EPSILON) {
    // Second segment is degenerated
    clamped_linear(w1, p1, -c, a, d1);
    w2 = p2;
  } else {
    // Always non-negative, equal 0 if the segments are colinear
    FCL_REAL denom = fmax(a * e - b * b, 0);

    FCL_REAL s;
    FCL_REAL t;
    if (denom > EPSILON) {
      s = clamp((b * f - c * e), denom);
      t = b * s + f;
    } else {
      s = 0.;
      t = f;
    }

    if (t <= 0.0) {
      w2 = p2;
      clamped_linear(w1, p1, -c, a, d1);
    } else if (t >= e) {
      clamped_linear(w1, p1, (b - c), a, d1);
      w2 = p2 + d2;
    } else {
      w1 = p1 + s * d1;
      w2 = p2 + t / e * d2;
    }
  }

  // witness points achieving the distance between the two segments
  FCL_REAL distance = (w1 - w2).norm();

  // capsule spcecific distance computation
  distance = distance - (radius1 + radius2);
  result.min_distance = distance;

  // Normal points from o1 to o2
  result.normal = (w2 - w1).normalized();
  result.nearest_points[0] = w1 + radius1 * result.normal;
  result.nearest_points[1] = w2 - radius2 * result.normal;

  return distance;
}

}  // namespace fcl

}  // namespace hpp
