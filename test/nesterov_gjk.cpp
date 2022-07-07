/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, INRIA
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

/** \author Louis Montaut */

#define BOOST_TEST_MODULE FCL_NESTEROV_GJK
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/internal/tools.h>

#include "utility.h"

using hpp::fcl::Box;
using hpp::fcl::Capsule;
using hpp::fcl::constructPolytopeFromEllipsoid;
using hpp::fcl::Convex;
using hpp::fcl::Ellipsoid;
using hpp::fcl::FCL_REAL;
using hpp::fcl::GJKSolver;
using hpp::fcl::GJKVariant;
using hpp::fcl::ShapeBase;
using hpp::fcl::support_func_guess_t;
using hpp::fcl::Transform3f;
using hpp::fcl::Triangle;
using hpp::fcl::Vec3f;
using hpp::fcl::details::GJK;
using hpp::fcl::details::MinkowskiDiff;
using std::size_t;

BOOST_AUTO_TEST_CASE(set_gjk_variant) {
  GJKSolver solver;
  GJK gjk(128, 1e-6);
  MinkowskiDiff shape;

  // Checking defaults
  BOOST_CHECK(solver.gjk_variant == GJKVariant::DefaultGJK);
  BOOST_CHECK(gjk.gjk_variant == GJKVariant::DefaultGJK);
  BOOST_CHECK(shape.normalize_support_direction == false);

  // Checking set
  solver.gjk_variant = GJKVariant::NesterovAcceleration;
  gjk.gjk_variant = GJKVariant::NesterovAcceleration;

  BOOST_CHECK(solver.gjk_variant == GJKVariant::NesterovAcceleration);
  BOOST_CHECK(gjk.gjk_variant == GJKVariant::NesterovAcceleration);
}

BOOST_AUTO_TEST_CASE(need_nesterov_normalize_support_direction) {
  Ellipsoid ellipsoid = Ellipsoid(1, 1, 1);
  Box box = Box(1, 1, 1);
  Convex<Triangle> cvx;

  MinkowskiDiff mink_diff1;
  mink_diff1.set(&ellipsoid, &ellipsoid);
  BOOST_CHECK(mink_diff1.normalize_support_direction == false);

  MinkowskiDiff mink_diff2;
  mink_diff2.set(&ellipsoid, &box);
  BOOST_CHECK(mink_diff2.normalize_support_direction == false);

  MinkowskiDiff mink_diff3;
  mink_diff3.set(&cvx, &cvx);
  BOOST_CHECK(mink_diff3.normalize_support_direction == true);
}

void test_nesterov_gjk(const ShapeBase& shape0, const ShapeBase& shape1) {
  // Solvers
  unsigned int max_iterations = 128;
  FCL_REAL tolerance = 1e-6;
  GJK gjk(max_iterations, tolerance);
  GJK gjk_nesterov(max_iterations, tolerance);
  gjk_nesterov.gjk_variant = GJKVariant::NesterovAcceleration;

  // Minkowski difference
  MinkowskiDiff mink_diff;

  // Generate random transforms
  size_t n = 1000;
  FCL_REAL extents[] = {-3., -3., 0, 3., 3., 3.};
  std::vector<Transform3f> transforms;
  generateRandomTransforms(extents, transforms, n);
  Transform3f identity = Transform3f::Identity();

  // Same init for both solvers
  Vec3f init_guess = Vec3f(1, 0, 0);
  support_func_guess_t init_support_guess;
  init_support_guess.setZero();

  for (size_t i = 0; i < n; ++i) {
    mink_diff.set(&shape0, &shape1, identity, transforms[i]);

    // Evaluate both solvers twice, make sure they give the same solution
    GJK::Status res_gjk_1 =
        gjk.evaluate(mink_diff, init_guess, init_support_guess);
    Vec3f ray_gjk = gjk.ray;
    GJK::Status res_gjk_2 =
        gjk.evaluate(mink_diff, init_guess, init_support_guess);
    BOOST_CHECK(res_gjk_1 == res_gjk_2);
    EIGEN_VECTOR_IS_APPROX(ray_gjk, gjk.ray, 1e-8);

    GJK::Status res_nesterov_gjk_1 =
        gjk_nesterov.evaluate(mink_diff, init_guess, init_support_guess);
    Vec3f ray_nesterov = gjk_nesterov.ray;
    GJK::Status res_nesterov_gjk_2 =
        gjk_nesterov.evaluate(mink_diff, init_guess, init_support_guess);
    BOOST_CHECK(res_nesterov_gjk_1 == res_nesterov_gjk_2);
    EIGEN_VECTOR_IS_APPROX(ray_nesterov, gjk_nesterov.ray, 1e-8);

    // Make sure GJK and Nesterov accelerated GJK find the same distance between
    // the shapes
    BOOST_CHECK(res_nesterov_gjk_1 == res_gjk_1);
    BOOST_CHECK_SMALL(fabs(ray_gjk.norm() - ray_nesterov.norm()), 1e-4);

    // Make sure GJK and Nesterov accelerated GJK converges in a reasonable
    // amount of iterations
    BOOST_CHECK(gjk.getIterations() < max_iterations);
    BOOST_CHECK(gjk_nesterov.getIterations() < max_iterations);
  }
}

BOOST_AUTO_TEST_CASE(ellipsoid_ellipsoid) {
  Ellipsoid ellipsoid0 = Ellipsoid(0.3, 0.4, 0.5);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);

  test_nesterov_gjk(ellipsoid0, ellipsoid1);
  test_nesterov_gjk(ellipsoid0, ellipsoid1);
}

BOOST_AUTO_TEST_CASE(ellipsoid_capsule) {
  Ellipsoid ellipsoid0 = Ellipsoid(0.5, 0.4, 0.3);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);
  Capsule capsule0 = Capsule(0.1, 0.3);
  Capsule capsule1 = Capsule(1.1, 1.3);

  test_nesterov_gjk(ellipsoid0, capsule0);
  test_nesterov_gjk(ellipsoid0, capsule1);
  test_nesterov_gjk(ellipsoid1, capsule0);
  test_nesterov_gjk(ellipsoid1, capsule1);
}

BOOST_AUTO_TEST_CASE(ellipsoid_box) {
  Ellipsoid ellipsoid0 = Ellipsoid(0.5, 0.4, 0.3);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);
  Box box0 = Box(0.1, 0.2, 0.3);
  Box box1 = Box(1.1, 1.2, 1.3);

  test_nesterov_gjk(ellipsoid0, box0);
  test_nesterov_gjk(ellipsoid0, box1);
  test_nesterov_gjk(ellipsoid1, box0);
  test_nesterov_gjk(ellipsoid1, box1);
}

BOOST_AUTO_TEST_CASE(ellipsoid_mesh) {
  Ellipsoid ellipsoid0 = Ellipsoid(0.5, 0.4, 0.3);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);
  Convex<Triangle> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  Convex<Triangle> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);

  test_nesterov_gjk(ellipsoid0, cvx0);
  test_nesterov_gjk(ellipsoid0, cvx1);
  test_nesterov_gjk(ellipsoid1, cvx0);
  test_nesterov_gjk(ellipsoid1, cvx1);
}

BOOST_AUTO_TEST_CASE(capsule_mesh) {
  Ellipsoid ellipsoid0 = Ellipsoid(0.5, 0.4, 0.3);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);
  Convex<Triangle> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  Convex<Triangle> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);
  Capsule capsule0 = Capsule(0.1, 0.3);
  Capsule capsule1 = Capsule(1.1, 1.3);

  test_nesterov_gjk(capsule0, cvx0);
  test_nesterov_gjk(capsule0, cvx1);
  test_nesterov_gjk(capsule1, cvx0);
  test_nesterov_gjk(capsule1, cvx1);
}

BOOST_AUTO_TEST_CASE(capsule_capsule) {
  Capsule capsule0 = Capsule(0.1, 0.3);
  Capsule capsule1 = Capsule(1.1, 1.3);

  test_nesterov_gjk(capsule0, capsule0);
  test_nesterov_gjk(capsule1, capsule1);
  test_nesterov_gjk(capsule0, capsule1);
}

BOOST_AUTO_TEST_CASE(box_box) {
  Box box0 = Box(0.1, 0.2, 0.3);
  Box box1 = Box(1.1, 1.2, 1.3);
  test_nesterov_gjk(box0, box0);
  test_nesterov_gjk(box0, box1);
  test_nesterov_gjk(box1, box1);
}

BOOST_AUTO_TEST_CASE(box_mesh) {
  Box box0 = Box(0.1, 0.2, 0.3);
  Box box1 = Box(1.1, 1.2, 1.3);
  Ellipsoid ellipsoid0 = Ellipsoid(0.5, 0.4, 0.3);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);
  Convex<Triangle> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  Convex<Triangle> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);

  test_nesterov_gjk(box0, cvx0);
  test_nesterov_gjk(box0, cvx1);
  test_nesterov_gjk(box1, cvx0);
  test_nesterov_gjk(box1, cvx1);
}

BOOST_AUTO_TEST_CASE(mesh_mesh) {
  Ellipsoid ellipsoid0 = Ellipsoid(0.5, 0.4, 0.3);
  Ellipsoid ellipsoid1 = Ellipsoid(1.5, 1.4, 1.3);
  Convex<Triangle> cvx0 = constructPolytopeFromEllipsoid(ellipsoid0);
  Convex<Triangle> cvx1 = constructPolytopeFromEllipsoid(ellipsoid1);

  test_nesterov_gjk(cvx0, cvx0);
  test_nesterov_gjk(cvx0, cvx1);
  test_nesterov_gjk(cvx1, cvx1);
}
