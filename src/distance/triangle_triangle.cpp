/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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

/** \author Louis Montaut */

#include <hpp/fcl/shape/geometric_shapes.h>

#include <hpp/fcl/internal/shape_shape_func.h>
#include "../narrowphase/details.h"

namespace hpp {
namespace fcl {

template <>
FCL_REAL ShapeShapeDistance<TriangleP, TriangleP>(
    const CollisionGeometry* o1, const Transform3f& tf1,
    const CollisionGeometry* o2, const Transform3f& tf2,
    const GJKSolver* solver, const DistanceRequest&, DistanceResult& result) {
  // Transform the triangles in world frame
  const TriangleP& s1 = static_cast<const TriangleP&>(*o1);
  const TriangleP t1(tf1.transform(s1.a), tf1.transform(s1.b),
                     tf1.transform(s1.c));

  const TriangleP& s2 = static_cast<const TriangleP&>(*o2);
  const TriangleP t2(tf2.transform(s2.a), tf2.transform(s2.b),
                     tf2.transform(s2.c));

  // Reset GJK algorithm
  //   We don't need to take into account swept-sphere radius in GJK iterations;
  //   the result will be corrected after GJK terminates.
  constexpr bool compute_swept_sphere_support = false;
  solver->minkowski_difference.set<compute_swept_sphere_support>(&t1, &t2);
  solver->gjk.reset(solver->gjk_max_iterations, solver->gjk_tolerance);

  // Get GJK initial guess
  Vec3f guess;
  if (solver->gjk_initial_guess == GJKInitialGuess::CachedGuess ||
      solver->enable_cached_guess) {
    guess = solver->cached_guess;
  } else {
    guess = (t1.a + t1.b + t1.c - t2.a - t2.b - t2.c) / 3;
  }
  support_func_guess_t support_hint;
  solver->epa.status =
      details::EPA::DidNotRun;  // EPA is never called in this function

  details::GJK::Status gjk_status =
      solver->gjk.evaluate(solver->minkowski_difference, guess, support_hint);

  if (solver->gjk_initial_guess == GJKInitialGuess::CachedGuess ||
      solver->enable_cached_guess) {
    solver->cached_guess = solver->gjk.getGuessFromSimplex();
    solver->support_func_cached_guess = solver->gjk.support_hint;
  }

  // Retrieve witness points and normal
  solver->gjk.getWitnessPointsAndNormal(
      solver->minkowski_difference, result.nearest_points[0],
      result.nearest_points[1], result.normal);

  if (gjk_status != details::GJK::Collision) {
    // TODO On degenerated case, the closest point may be wrong
    // (i.e. an object face normal is colinear to gjk.ray
    // assert (dist == (w0 - w1).norm());
    result.min_distance = solver->gjk.distance;
    assert(solver->gjk.ray.norm() > solver->gjk.getTolerance());
  } else {
    FCL_REAL penetrationDepth = details::computePenetration(
        t1.a, t1.b, t1.c, t2.a, t2.b, t2.c, result.normal);
    result.min_distance = -penetrationDepth;
  }
  // assert(false && "should not reach this point");
  // return false;

  result.o1 = o1;
  result.o2 = o2;
  result.b1 = -1;
  result.b2 = -1;
  return result.min_distance;
}

}  // namespace fcl
}  // namespace hpp
