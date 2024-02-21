/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Centre National de la Recherche Scientifique
 *  All rights reserved.
 *  Copyright (c) 2021-2024, INRIA
 *  All rights reserved.
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

/** \author Jia Pan, Florent Lamiraux */

#ifndef HPP_FCL_NARROWPHASE_H
#define HPP_FCL_NARROWPHASE_H

#include <limits>

#include <hpp/fcl/narrowphase/gjk.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/narrowphase/narrowphase_defaults.h>
#include <hpp/fcl/logging.h>

namespace hpp {
namespace fcl {

/// @brief collision and distance solver based on GJK algorithm implemented in
/// fcl (rewritten the code from the GJK in bullet)
struct HPP_FCL_DLLAPI GJKSolver {
 public:
  typedef Eigen::Array<FCL_REAL, 1, 2> Array2d;

  /// @brief intersection checking between two shapes.
  /// @return true if the shapes are colliding.
  /// @note the variables `gjk_status` and `epa_status` can be used to
  /// check if GJK or EPA ran successfully.
  /// @note (12 feb, 2024): this function is never called in the
  /// hpp-fcl library.
  /// Either the `shapeDistance` in this file is called or a specialized
  /// function. However, this function is still tested in the test suite,
  /// notably in `test/collision.cpp` and `test/geometric_shapes.cpp`.
  template <typename S1, typename S2>
  bool shapeIntersect(const S1& s1, const Transform3f& tf1, const S2& s2,
                      const Transform3f& tf2, FCL_REAL& distance_lower_bound,
                      bool compute_penetration, Vec3f* contact_points,
                      Vec3f* normal) const {
    minkowski_difference.set(&s1, &s2, tf1, tf2);
    Vec3f p1(Vec3f::Zero()), p2(Vec3f::Zero());
    Vec3f n(Vec3f::Zero());
    FCL_REAL distance((std::numeric_limits<FCL_REAL>::max)());
    bool gjk_and_epa_ran_successfully = runGJKAndEPA(
        s1, tf1, s2, tf2, distance, compute_penetration, p1, p2, n);
    HPP_FCL_UNUSED_VARIABLE(gjk_and_epa_ran_successfully);
    distance_lower_bound = distance;
    if (compute_penetration) {
      if (normal != NULL) *normal = n;
      if (contact_points != NULL) *contact_points = 0.5 * (p1 + p2);
    }
    return (gjk.status == details::GJK::Inside);
  }

  //// @brief intersection checking between one shape and a triangle with
  /// transformation
  /// @return true if the shape are colliding.
  /// The variables `gjk_status` and `epa_status` can be used to
  /// check if GJK or EPA ran successfully.
  template <typename S>
  bool shapeTriangleInteraction(const S& s, const Transform3f& tf1,
                                const Vec3f& P1, const Vec3f& P2,
                                const Vec3f& P3, const Transform3f& tf2,
                                FCL_REAL& distance, bool compute_penetration,
                                Vec3f& p1, Vec3f& p2, Vec3f& normal) const {
    // Express everything in frame 1
    const Transform3f tf_1M2(tf1.inverseTimes(tf2));
    TriangleP tri(tf_1M2.transform(P1), tf_1M2.transform(P2),
                  tf_1M2.transform(P3));

    bool relative_transformation_already_computed = true;
    bool gjk_and_epa_ran_successfully =
        runGJKAndEPA(s, tf1, tri, tf_1M2, distance, compute_penetration, p1, p2,
                     normal, relative_transformation_already_computed);
    HPP_FCL_UNUSED_VARIABLE(gjk_and_epa_ran_successfully);
    return (gjk.status == details::GJK::Inside);
  }

  /// @brief distance computation between two shapes.
  /// @return true if no error occured, false otherwise.
  /// The variables `gjk_status` and `epa_status` can be used to
  /// understand the reason of the failure.
  template <typename S1, typename S2>
  bool shapeDistance(const S1& s1, const Transform3f& tf1, const S2& s2,
                     const Transform3f& tf2, FCL_REAL& distance,
                     bool compute_penetration, Vec3f& p1, Vec3f& p2,
                     Vec3f& normal) const {
    bool gjk_and_epa_ran_successfully = runGJKAndEPA(
        s1, tf1, s2, tf2, distance, compute_penetration, p1, p2, normal);
    return gjk_and_epa_ran_successfully;
  }

 protected:
  /// @brief initialize GJK
  template <typename S1, typename S2>
  void getGJKInitialGuess(const S1& s1, const S2& s2, Vec3f& guess,
                          support_func_guess_t& support_hint) const {
    switch (gjk_initial_guess) {
      case GJKInitialGuess::DefaultGuess:
        guess = Vec3f(1, 0, 0);
        support_hint.setZero();
        break;
      case GJKInitialGuess::CachedGuess:
        guess = cached_guess;
        support_hint = support_func_cached_guess;
        break;
      case GJKInitialGuess::BoundingVolumeGuess:
        if (s1.aabb_local.volume() < 0 || s2.aabb_local.volume() < 0) {
          HPP_FCL_THROW_PRETTY(
              "computeLocalAABB must have been called on the shapes before "
              "using "
              "GJKInitialGuess::BoundingVolumeGuess.",
              std::logic_error);
        }
        guess.noalias() = s1.aabb_local.center() -
                          (minkowski_difference.oR1 * s2.aabb_local.center() +
                           minkowski_difference.ot1);
        support_hint.setZero();
        break;
      default:
        HPP_FCL_THROW_PRETTY("Wrong initial guess for GJK.", std::logic_error);
    }
    // TODO: use gjk_initial_guess instead
    HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
    HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    if (enable_cached_guess) {
      guess = cached_guess;
      support_hint = support_func_cached_guess;
    }
    HPP_FCL_COMPILER_DIAGNOSTIC_POP
  }
  /// @brief Runs the GJK algorithm; if the shapes are in found in collision,
  /// also runs the EPA algorithm.
  /// This function assumes the minkowski difference has been already been set,
  /// i.e. `minkowski_difference.set(&s1, &s2, tf1, tf2);` has been called.
  /// @return true if no error occured, false otherwise.
  template <typename S1, typename S2>
  bool runGJKAndEPA(
      const S1& s1, const Transform3f& tf1, const S2& s2,
      const Transform3f& tf2, FCL_REAL& distance, bool compute_penetration,
      Vec3f& p1, Vec3f& p2, Vec3f& normal,
      bool relative_transformation_already_computed = false) const {
    bool gjk_and_epa_ran_successfully = true;

    // Reset internal state of GJK algorithm
    if (relative_transformation_already_computed)
      minkowski_difference.set(&s1, &s2);
    else
      minkowski_difference.set(&s1, &s2, tf1, tf2);
    gjk.reset(gjk_max_iterations, gjk_tolerance);

    // Get initial guess for GJK: default, cached or bounding volume guess
    Vec3f guess;
    support_func_guess_t support_hint;
    getGJKInitialGuess(*minkowski_difference.shapes[0],
                       *minkowski_difference.shapes[1], guess, support_hint);

    gjk.evaluate(minkowski_difference, guess, support_hint);
    HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
    HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
    if (gjk_initial_guess == GJKInitialGuess::CachedGuess ||
        enable_cached_guess) {
      cached_guess = gjk.getGuessFromSimplex();
      support_func_cached_guess = gjk.support_hint;
    }
    HPP_FCL_COMPILER_DIAGNOSTIC_POP

    switch (gjk.status) {
      case details::GJK::DidNotRun:
        HPP_FCL_ASSERT(false, "GJK did not run. It should have!",
                       std::logic_error);
        distance = -(std::numeric_limits<FCL_REAL>::max)();
        p1 = p2 = normal =
            Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
        gjk_and_epa_ran_successfully = false;
        break;
      case details::GJK::Failed:
        //
        // GJK ran out of iterations.
        HPP_FCL_LOG_WARNING("GJK ran out of iterations.");
        GJKNoCollisionExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                    normal);
        gjk_and_epa_ran_successfully = false;
        break;
      case details::GJK::EarlyStopped:
        //
        // Case where GJK early stopped because the distance was found to be
        // above the `distance_upper_bound`.
        // The two witness points have no meaning.
        GJKEarlyStopExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                  normal);
        HPP_FCL_ASSERT(
            distance >= gjk.distance_upper_bound,
            "The distance should be bigger than GJK's `distance_upper_bound`.",
            std::logic_error);
        break;
      case details::GJK::Valid:
        //
        // Case where GJK converged and proved that the shapes are not in
        // collision, i.e their distance is above GJK's tolerance (default
        // 1e-6).
        GJKNoCollisionExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                    normal);
        HPP_FCL_ASSERT(
            std::abs((p1 - p2).norm() - distance) < gjk.getTolerance(),
            "The distance found by GJK should coincide with the "
            "distance between the closest points.",
            std::logic_error);
        break;
      case details::GJK::Inside:
        //
        // Case where GJK found the shapes to be in collision, i.e. their
        // distance is below GJK's tolerance (default 1e-6).
        if (gjk.hasPenetrationInformation(minkowski_difference)) {
          //
          // Case where the shapes are inflated (sphere or capsule).
          // When the shapes are inflated, the GJK algorithm can provide the
          // witness points and the normal.
          GJKCollisionWithInflationExtractWitnessPointsAndNormal(
              tf1, distance, p1, p2, normal);
          HPP_FCL_ASSERT(distance < gjk.getTolerance(),
                         "The distance found by GJK should be negative (or at )"
                         "least below GJK's tolerance.",
                         std::logic_error);
          // + because the distance is negative.
          HPP_FCL_ASSERT(
              std::abs((p1 - p2).norm() + distance) < gjk.getTolerance(),
              "The distance found by GJK should coincide with the "
              "distance between the closest points.",
              std::logic_error);
        } else {
          if (!compute_penetration) {
            GJKCollisionExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                      normal);
          } else {
            //
            // Case where the shapes are not inflated (box, cylinder, cone,
            // convex meshes etc.). We need to run the EPA algorithm to find the
            // witness points, penetration depth and the normal.

            // Reset EPA algorithm. Potentially allocate memory if
            // `epa_max_face_num` or `epa_max_vertex_num` are bigger than EPA's
            // current storage.
            epa.reset(epa_max_iterations, epa_tolerance);

            // TODO: understand why EPA's performance is so bad on cylinders and
            // cones.
            epa.evaluate(gjk, -guess);

            switch (epa.status) {
              //
              // In the following switch cases, until the "Valid" case,
              // EPA either ran out of iterations, of faces or of vertices.
              // The depth, witness points and the normal are still valid,
              // simply not at the precision of EPA's tolerance.
              // The flag `HPP_FCL_ENABLE_LOGGING` enables feebdack on these
              // cases.
              //
              // TODO: Remove OutOfFaces and OutOfVertices statuses and simply
              // compute the upper bound on max faces and max vertices as a
              // function of the number of iterations.
              case details::EPA::OutOfFaces:
                HPP_FCL_LOG_WARNING("EPA ran out of faces.");
                EPAValidExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                      normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::OutOfVertices:
                HPP_FCL_LOG_WARNING("EPA ran out of vertices.");
                EPAValidExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                      normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::Failed:
                HPP_FCL_LOG_WARNING("EPA ran out of iterations.");
                EPAValidExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                      normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::Valid:
              case details::EPA::AccuracyReached:
                HPP_FCL_ASSERT(
                    -epa.depth <= epa.getTolerance(),
                    "EPA's penetration distance should be negative (or "
                    "at least below EPA's "
                    "tolerance).",
                    std::logic_error);
                EPAValidExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                      normal);
                break;
                //
                // In the following cases, EPA failed to run, in a bad way.
                // The produced witness points, penetration depth and normal
                // may make no sense.
              case details::EPA::DidNotRun:
                HPP_FCL_ASSERT(false, "EPA did not run. It should have!",
                               std::logic_error);
                HPP_FCL_LOG_ERROR("EPA error: did not run. It should have.");
                EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                       normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::Degenerated:
                HPP_FCL_ASSERT(
                    false, "EPA created a polytope with a degenerated face.",
                    std::logic_error);
                HPP_FCL_LOG_ERROR(
                    "EPA error: created a polytope with a degenerated face.");
                EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                       normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::NonConvex:
                HPP_FCL_ASSERT(false, "EPA got called onto non-convex shapes.",
                               std::logic_error);
                HPP_FCL_LOG_ERROR(
                    "EPA error: EPA got called onto non-convex shapes.");
                EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                       normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::InvalidHull:
                HPP_FCL_ASSERT(false, "EPA created an invalid polytope.",
                               std::logic_error);
                HPP_FCL_LOG_ERROR("EPA error: created an invalid polytope.");
                EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                       normal);
                gjk_and_epa_ran_successfully = false;
                break;
              case details::EPA::FallBack:
                HPP_FCL_ASSERT(
                    false,
                    "EPA went into fallback mode. It should never do that.",
                    std::logic_error);
                HPP_FCL_LOG_ERROR("EPA error: FallBack.");
                EPAFailedExtractWitnessPointsAndNormal(tf1, distance, p1, p2,
                                                       normal);
                gjk_and_epa_ran_successfully = false;
                break;
            }
          }
        }
        break;  // End of case details::GJK::Inside
    }
    return gjk_and_epa_ran_successfully;
  }

  void GJKEarlyStopExtractWitnessPointsAndNormal(const Transform3f& tf1,
                                                 FCL_REAL& distance, Vec3f& p1,
                                                 Vec3f& p2,
                                                 Vec3f& normal) const {
    HPP_FCL_UNUSED_VARIABLE(tf1);
    distance = gjk.distance;
    p1 = p2 = normal =
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
    // If we absolutely want to return some witness points, we could use
    // the following code (or simply merge the early stopped case with the
    // valid case below):
    // gjk.getClosestPoints(minkowski_difference, p1, p2);
    // p1 = tf1.transform(p1);
    // p2 = tf1.transform(p2);
    // normal.noalias() = -tf1.getRotation() * gjk.ray;
    // normal.normalize();
  }

  void GJKNoCollisionExtractWitnessPointsAndNormal(const Transform3f& tf1,
                                                   FCL_REAL& distance,
                                                   Vec3f& p1, Vec3f& p2,
                                                   Vec3f& normal) const {
    // Apart from early stopping, there are two cases where GJK says there is no
    // collision:
    // 1. GJK proved the distance is above its tolerance (default 1e-6).
    // 2. GJK ran out of iterations.
    // In any case, `gjk.ray`'s norm is bigger than GJK's tolerance and thus
    // it can safely be normalized.
    distance = gjk.distance;
    HPP_FCL_ASSERT(
        gjk.ray.norm() > gjk.getTolerance(),
        "The norm of GJK's ray should be bigger than GJK's tolerance.",
        std::logic_error);
    normal.noalias() = -tf1.getRotation() * gjk.ray;
    normal.normalize();
    // TODO: On degenerated case, the closest points may be non-unique.
    // (i.e. an object face normal is colinear to `gjk.ray`)
    gjk.getClosestPoints(minkowski_difference, p1, p2);
    p1 = tf1.transform(p1);
    p2 = tf1.transform(p2);
  }

  void GJKCollisionWithInflationExtractWitnessPointsAndNormal(
      const Transform3f& tf1, FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
      Vec3f& normal) const {
    distance = gjk.distance;
    gjk.getClosestPoints(minkowski_difference, p1, p2);
    normal.noalias() = tf1.getRotation() * (p1 - p2);
    normal.normalize();
    p1 = tf1.transform(p1);
    p2 = tf1.transform(p2);
  }

  void GJKCollisionExtractWitnessPointsAndNormal(const Transform3f& tf1,
                                                 FCL_REAL& distance, Vec3f& p1,
                                                 Vec3f& p2,
                                                 Vec3f& normal) const {
    HPP_FCL_ASSERT(gjk.distance <= gjk.getTolerance(),
                   "The distance should be lower than GJK's tolerance.",
                   std::logic_error);
    distance = gjk.distance;
    gjk.getClosestPoints(minkowski_difference, p1, p2);
    p1 = tf1.transform(p1);
    p2 = tf1.transform(p2);
    normal = Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
  }

  void EPAValidExtractWitnessPointsAndNormal(const Transform3f& tf1,
                                             FCL_REAL& distance, Vec3f& p1,
                                             Vec3f& p2, Vec3f& normal) const {
    distance = (std::min)(0., -epa.depth);
    epa.getClosestPoints(minkowski_difference, p1, p2);
    normal.noalias() = tf1.getRotation() * epa.normal;
    p1 = tf1.transform(p1);
    p2 = tf1.transform(p2);
  }

  void EPAFailedExtractWitnessPointsAndNormal(const Transform3f& tf1,
                                              FCL_REAL& distance, Vec3f& p1,
                                              Vec3f& p2, Vec3f& normal) const {
    HPP_FCL_UNUSED_VARIABLE(tf1);
    distance = -(std::numeric_limits<FCL_REAL>::max)();
    p1 = p2 = normal =
        Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN());
  }

 public:
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  /// @brief Default constructor for GJK algorithm
  /// By default, we don't want EPA to allocate memory because
  /// certain functions of the `GJKSolver` class have specializations
  /// which don't use EPA (and/or GJK).
  /// So we give EPA's constructor a max number of iterations of zero.
  /// Only the functions that need EPA will reset the algorithm and allocate
  /// memory if needed.
  GJKSolver()
      : gjk(GJK_DEFAULT_MAX_ITERATIONS, GJK_DEFAULT_TOLERANCE),
        epa(0, EPA_DEFAULT_TOLERANCE) {
    gjk_max_iterations = GJK_DEFAULT_MAX_ITERATIONS;
    gjk_tolerance = GJK_DEFAULT_TOLERANCE;
    epa_max_iterations = EPA_DEFAULT_MAX_ITERATIONS;
    epa_tolerance = EPA_DEFAULT_TOLERANCE;

    gjk_initial_guess = GJKInitialGuess::DefaultGuess;
    enable_cached_guess = false;  // TODO: use gjk_initial_guess instead
    cached_guess = Vec3f(1, 0, 0);
    support_func_cached_guess = support_func_guess_t::Zero();
    distance_upper_bound = (std::numeric_limits<FCL_REAL>::max)();

    // Default settings for GJK algorithm
    gjk.gjk_variant = GJKVariant::DefaultGJK;
    gjk.convergence_criterion = GJKConvergenceCriterion::VDB;
    gjk.convergence_criterion_type = GJKConvergenceCriterionType::Relative;
  }

  /// @brief Constructor from a DistanceRequest
  ///
  /// \param[in] request DistanceRequest input
  ///
  /// See the default constructor; by default, we don't want
  /// EPA to allocate memory so we call EPA's constructor with 0 max
  /// number of iterations.
  GJKSolver(const DistanceRequest& request)
      : gjk(request.gjk_max_iterations, request.gjk_tolerance),
        epa(0, request.epa_tolerance) {
    cached_guess = Vec3f(1, 0, 0);
    support_func_cached_guess = support_func_guess_t::Zero();
    distance_upper_bound = (std::numeric_limits<FCL_REAL>::max)();

    set(request);
  }

  /// @brief setter from a DistanceRequest
  ///
  /// \param[in] request DistanceRequest input
  ///
  void set(const DistanceRequest& request) {
    // ---------------------
    // GJK settings
    gjk_initial_guess = request.gjk_initial_guess;
    enable_cached_guess = request.enable_cached_gjk_guess;
    if (gjk_initial_guess == GJKInitialGuess::CachedGuess ||
        enable_cached_guess) {
      cached_guess = request.cached_gjk_guess;
      support_func_cached_guess = request.cached_support_func_guess;
    }
    gjk_max_iterations = request.gjk_max_iterations;
    gjk_tolerance = request.gjk_tolerance;
    // For distance computation, we don't want GJK to early stop
    gjk.setDistanceEarlyBreak((std::numeric_limits<FCL_REAL>::max)());
    gjk.gjk_variant = request.gjk_variant;
    gjk.convergence_criterion = request.gjk_convergence_criterion;
    gjk.convergence_criterion_type = request.gjk_convergence_criterion_type;
    gjk.status = details::GJK::Status::DidNotRun;

    // ---------------------
    // EPA settings
    epa_max_iterations = request.epa_max_iterations;
    epa_tolerance = request.epa_tolerance;
    epa.status = details::EPA::Status::DidNotRun;

    if (request.gjk_tolerance < GJK_MINIMUM_TOLERANCE) {
      HPP_FCL_LOG_WARNING(
          "WARNING - GJK: using a tolerance ("
          << request.gjk_tolerance
          << ") which is lower than the recommended lowest tolerance ("
          << GJK_DEFAULT_TOLERANCE
          << "). Selecting this tolerance might trigger assertions.\n");
    }
    if (request.epa_tolerance < EPA_MINIMUM_TOLERANCE) {
      HPP_FCL_LOG_WARNING(
          "WARNING - EPA: using a tolerance ("
          << request.epa_tolerance
          << ") which is lower than the recommended lowest tolerance ("
          << EPA_MINIMUM_TOLERANCE
          << "). Selecting this tolerance might trigger assertions.\n");
    }
  }

  /// @brief Constructor from a CollisionRequest
  ///
  /// \param[in] request CollisionRequest input
  ///
  /// See the default constructor; by default, we don't want
  /// EPA to allocate memory so we call EPA's constructor with 0 max
  /// number of iterations.
  GJKSolver(const CollisionRequest& request)
      : gjk(request.gjk_max_iterations, request.gjk_tolerance),
        epa(0, request.epa_tolerance) {
    cached_guess = Vec3f(1, 0, 0);
    support_func_cached_guess = support_func_guess_t::Zero();
    distance_upper_bound = (std::numeric_limits<FCL_REAL>::max)();

    set(request);
  }

  /// @brief setter from a CollisionRequest
  ///
  /// \param[in] request CollisionRequest input
  ///
  void set(const CollisionRequest& request) {
    // ---------------------
    // GJK settings
    gjk_initial_guess = request.gjk_initial_guess;
    // TODO: use gjk_initial_guess instead
    enable_cached_guess = request.enable_cached_gjk_guess;
    if (gjk_initial_guess == GJKInitialGuess::CachedGuess ||
        enable_cached_guess) {
      cached_guess = request.cached_gjk_guess;
      support_func_cached_guess = request.cached_support_func_guess;
    }
    gjk_tolerance = request.gjk_tolerance;
    gjk_max_iterations = request.gjk_max_iterations;
    // The distance upper bound should be at least greater to the requested
    // security margin. Otherwise, we will likely miss some collisions.
    distance_upper_bound = (std::max)(
        0., (std::max)(request.distance_upper_bound, request.security_margin));
    gjk.setDistanceEarlyBreak(distance_upper_bound);
    gjk.gjk_variant = request.gjk_variant;
    gjk.convergence_criterion = request.gjk_convergence_criterion;
    gjk.convergence_criterion_type = request.gjk_convergence_criterion_type;

    // ---------------------
    // EPA settings
    epa_max_iterations = request.epa_max_iterations;
    epa_tolerance = request.epa_tolerance;

    // ---------------------
    // Reset GJK and EPA status
    gjk.status = details::GJK::Status::DidNotRun;
    epa.status = details::EPA::Status::DidNotRun;

    if (request.gjk_tolerance < GJK_MINIMUM_TOLERANCE) {
      HPP_FCL_LOG_WARNING(
          "WARNING - GJK: using a tolerance ("
          << request.gjk_tolerance
          << ") which is lower than the recommended lowest tolerance ("
          << GJK_DEFAULT_TOLERANCE
          << "). Selecting this tolerance might trigger assertions.\n");
    }
    if (request.epa_tolerance < EPA_MINIMUM_TOLERANCE) {
      HPP_FCL_LOG_WARNING(
          "WARNING - EPA: using a tolerance ("
          << request.epa_tolerance
          << ") which is lower than the recommended lowest tolerance ("
          << EPA_MINIMUM_TOLERANCE
          << "). Selecting this tolerance might trigger assertions.\n");
    }
  }

  /// @brief Copy constructor
  GJKSolver(const GJKSolver& other) = default;

  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  bool operator==(const GJKSolver& other) const {
    return enable_cached_guess ==
               other.enable_cached_guess &&  // TODO: use gjk_initial_guess
                                             // instead
           cached_guess == other.cached_guess &&
           gjk_max_iterations == other.gjk_max_iterations &&
           gjk_tolerance == other.gjk_tolerance &&
           epa_max_iterations == other.epa_max_iterations &&
           epa_tolerance == other.epa_tolerance &&
           support_func_cached_guess == other.support_func_cached_guess &&
           distance_upper_bound == other.distance_upper_bound &&
           gjk_initial_guess == other.gjk_initial_guess;
  }
  HPP_FCL_COMPILER_DIAGNOSTIC_POP

  bool operator!=(const GJKSolver& other) const { return !(*this == other); }

  /// @brief Whether smart guess can be provided
  /// @Deprecated Use gjk_initial_guess instead
  HPP_FCL_DEPRECATED_MESSAGE(Use gjk_initial_guess instead)
  bool enable_cached_guess;

  /// @brief smart guess
  mutable Vec3f cached_guess;

  /// @brief which warm start to use for GJK
  GJKInitialGuess gjk_initial_guess;

  /// @brief smart guess for the support function
  mutable support_func_guess_t support_func_cached_guess;

  /// @brief Distance above which the GJK solver stops its computations and
  /// processes to an early stopping.
  ///        The two witness points are incorrect, but with the guaranty that
  ///        the two shapes have a distance greather than distance_upper_bound.
  FCL_REAL distance_upper_bound;

  /// @brief maximum number of iterations of GJK
  size_t gjk_max_iterations;

  /// @brief tolerance of GJK
  FCL_REAL gjk_tolerance;

  /// @brief maximum number of iterations of EPA
  size_t epa_max_iterations;

  /// @brief tolerance of EPA
  FCL_REAL epa_tolerance;

  /// @brief GJK algorithm
  mutable details::GJK gjk;

  /// @brief EPA algorithm
  mutable details::EPA epa;

  /// @brief Minkowski difference used by GJK and EPA algorithms
  mutable details::MinkowskiDiff minkowski_difference;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <>
HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction(
    const Sphere& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
    const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
    bool compute_penetration, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

template <>
HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction(
    const Halfspace& s, const Transform3f& tf1, const Vec3f& P1,
    const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
    FCL_REAL& distance, bool compute_penetration, Vec3f& p1, Vec3f& p2,
    Vec3f& normal) const;

template <>
HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction(
    const Plane& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
    const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
    bool compute_penetration, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

#define SHAPE_INTERSECT_SPECIALIZATION_BASE(S1, S2)                 \
  template <>                                                       \
  HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<S1, S2>(            \
      const S1& s1, const Transform3f& tf1, const S2& s2,           \
      const Transform3f& tf2, FCL_REAL& distance_lower_bound, bool, \
      Vec3f* contact_points, Vec3f* normal) const

#define SHAPE_INTERSECT_SPECIALIZATION(S1, S2) \
  SHAPE_INTERSECT_SPECIALIZATION_BASE(S1, S2); \
  SHAPE_INTERSECT_SPECIALIZATION_BASE(S2, S1)

SHAPE_INTERSECT_SPECIALIZATION(Sphere, Capsule);
SHAPE_INTERSECT_SPECIALIZATION_BASE(Sphere, Sphere);
SHAPE_INTERSECT_SPECIALIZATION(Sphere, Box);
SHAPE_INTERSECT_SPECIALIZATION(Sphere, Halfspace);
SHAPE_INTERSECT_SPECIALIZATION(Sphere, Plane);

SHAPE_INTERSECT_SPECIALIZATION(Halfspace, Box);
SHAPE_INTERSECT_SPECIALIZATION(Halfspace, Capsule);
SHAPE_INTERSECT_SPECIALIZATION(Halfspace, Cylinder);
SHAPE_INTERSECT_SPECIALIZATION(Halfspace, Cone);
SHAPE_INTERSECT_SPECIALIZATION(Halfspace, Plane);

SHAPE_INTERSECT_SPECIALIZATION(Plane, Box);
SHAPE_INTERSECT_SPECIALIZATION(Plane, Capsule);
SHAPE_INTERSECT_SPECIALIZATION(Plane, Cylinder);
SHAPE_INTERSECT_SPECIALIZATION(Plane, Cone);

#undef SHAPE_INTERSECT_SPECIALIZATION
#undef SHAPE_INTERSECT_SPECIALIZATION_BASE

#define SHAPE_DISTANCE_SPECIALIZATION_BASE(S1, S2)                      \
  template <>                                                           \
  HPP_FCL_DLLAPI bool GJKSolver::shapeDistance<S1, S2>(                 \
      const S1& s1, const Transform3f& tf1, const S2& s2,               \
      const Transform3f& tf2, FCL_REAL& dist, bool compute_penetration, \
      Vec3f& p1, Vec3f& p2, Vec3f& normal) const

#define SHAPE_DISTANCE_SPECIALIZATION(S1, S2) \
  SHAPE_DISTANCE_SPECIALIZATION_BASE(S1, S2); \
  SHAPE_DISTANCE_SPECIALIZATION_BASE(S2, S1)

SHAPE_DISTANCE_SPECIALIZATION(Sphere, Capsule);
SHAPE_DISTANCE_SPECIALIZATION(Sphere, Box);
SHAPE_DISTANCE_SPECIALIZATION(Sphere, Cylinder);
SHAPE_DISTANCE_SPECIALIZATION_BASE(Sphere, Sphere);
SHAPE_DISTANCE_SPECIALIZATION_BASE(Capsule, Capsule);
SHAPE_DISTANCE_SPECIALIZATION_BASE(TriangleP, TriangleP);

#undef SHAPE_DISTANCE_SPECIALIZATION
#undef SHAPE_DISTANCE_SPECIALIZATION_BASE

#if !(__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc99-extensions"
#endif
/// \name Shape intersection specializations
/// \{

// param doc is the doxygen detailled description (should be enclosed in /** */
// and contain no dot for some obscure reasons).
#define HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape1, Shape2, doc)      \
  /** @brief Fast implementation for Shape1-Shape2 collision. */  \
  doc template <>                                                 \
  HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<Shape1, Shape2>(  \
      const Shape1& s1, const Transform3f& tf1, const Shape2& s2, \
      const Transform3f& tf2, FCL_REAL& distance_lower_bound,     \
      bool compute_penetration, Vec3f* contact_points, Vec3f* normal) const
#define HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Shape, doc) \
  HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape, Shape, doc)
#define HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Shape1, Shape2, doc) \
  HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape1, Shape2, doc);           \
  HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape2, Shape1, doc)

HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Sphere, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Sphere, Capsule, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Sphere, Halfspace, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Sphere, Plane, );

template <>
HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<Box, Sphere>(
    const Box& s1, const Transform3f& tf1, const Sphere& s2,
    const Transform3f& tf2, FCL_REAL& distance_lower_bound,
    bool compute_penetration, Vec3f* contact_points, Vec3f* normal) const;

#ifdef IS_DOXYGEN  // for doxygen only
/** \todo currently disabled and to re-enable it, API of function
 *  \ref obbDisjointAndLowerBoundDistance should be modified.
 *  */
template <>
HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<Box, Box>(
    const Box& s1, const Transform3f& tf1, const Box& s2,
    const Transform3f& tf2, FCL_REAL& distance_lower_bound,
    bool compute_penetration, Vec3f* contact_points, Vec3f* normal) const;
#endif
// HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Box,);
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Box, Halfspace, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Box, Plane, );

HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Capsule, Halfspace, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Capsule, Plane, );

HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cylinder, Halfspace, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cylinder, Plane, );

HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cone, Halfspace, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cone, Plane, );

HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Halfspace, );

HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Plane, );
HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Plane, Halfspace, );

#undef HPP_FCL_DECLARE_SHAPE_INTERSECT
#undef HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF
#undef HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR

/// \}

/// \name Shape triangle interaction specializations
/// \{

// param doc is the doxygen detailled description (should be enclosed in /** */
// and contain no dot for some obscure reasons).
#define HPP_FCL_DECLARE_SHAPE_TRIANGLE(Shape, doc)                        \
  /** @brief Fast implementation for Shape-Triangle interaction. */       \
  doc template <>                                                         \
  HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction<Shape>(         \
      const Shape& s, const Transform3f& tf1, const Vec3f& P1,            \
      const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,           \
      FCL_REAL& distance, bool compute_penetration, Vec3f& p1, Vec3f& p2, \
      Vec3f& normal) const

HPP_FCL_DECLARE_SHAPE_TRIANGLE(Sphere, );
HPP_FCL_DECLARE_SHAPE_TRIANGLE(Halfspace, );
HPP_FCL_DECLARE_SHAPE_TRIANGLE(Plane, );

#undef HPP_FCL_DECLARE_SHAPE_TRIANGLE

/// \}

/// \name Shape distance specializations
/// \{

// param doc is the doxygen detailled description (should be enclosed in /** */
// and contain no dot for some obscure reasons).
#define HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape1, Shape2, doc)           \
  /** @brief Fast implementation for Shape1-Shape2 distance. */       \
  doc template <>                                                     \
  bool HPP_FCL_DLLAPI GJKSolver::shapeDistance<Shape1, Shape2>(       \
      const Shape1& s1, const Transform3f& tf1, const Shape2& s2,     \
      const Transform3f& tf2, FCL_REAL& dist, bool compute_collision, \
      Vec3f& p1, Vec3f& p2, Vec3f& normal) const
#define HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(Shape, doc) \
  HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape, Shape, doc)
#define HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Shape1, Shape2, doc) \
  HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape1, Shape2, doc);           \
  HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape2, Shape1, doc)

HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Sphere, Box, );
HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Sphere, Capsule, );
HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Sphere, Cylinder, );
HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(Sphere, );

HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(
    Capsule,
    /** Closest points are based on two line-segments. */
);

HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(
    TriangleP,
    /** Do not run EPA algorithm to compute penetration depth. Use a dedicated
       method. */
);

#undef HPP_FCL_DECLARE_SHAPE_DISTANCE
#undef HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF
#undef HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR

/// \}
#if !(__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#pragma GCC diagnostic pop
#endif
}  // namespace fcl

}  // namespace hpp

#endif
