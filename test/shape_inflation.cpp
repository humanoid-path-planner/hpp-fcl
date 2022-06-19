/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, INRIA.
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

#define BOOST_TEST_MODULE FCL_SECURITY_MARGIN
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <iostream>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>

#include "utility.h"

using namespace hpp::fcl;
using hpp::fcl::CollisionGeometryPtr_t;
using hpp::fcl::CollisionObject;
using hpp::fcl::CollisionRequest;
using hpp::fcl::CollisionResult;
using hpp::fcl::DistanceRequest;
using hpp::fcl::DistanceResult;
using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;

#define MATH_SQUARED(x) (x * x)

template <typename Shape>
bool isApprox(const Shape &s1, const Shape &s2, const FCL_REAL tol);

bool isApprox(const FCL_REAL &v1, const FCL_REAL &v2, const FCL_REAL tol) {
  typedef Eigen::Matrix<FCL_REAL, 1, 1> ScalarMatrix;
  ScalarMatrix m1;
  m1 << v1;
  ScalarMatrix m2;
  m2 << v2;
  return m1.isApprox(m2, tol);
}

bool isApprox(const Box &s1, const Box &s2, const FCL_REAL tol) {
  return s1.halfSide.isApprox(s2.halfSide, tol);
}

bool isApprox(const Sphere &s1, const Sphere &s2, const FCL_REAL tol) {
  return isApprox(s1.radius, s2.radius, tol);
}

bool isApprox(const Ellipsoid &s1, const Ellipsoid &s2, const FCL_REAL tol) {
  return s1.radii.isApprox(s2.radii, tol);
}

bool isApprox(const Capsule &s1, const Capsule &s2, const FCL_REAL tol) {
  return isApprox(s1.radius, s2.radius, tol) &&
         isApprox(s1.halfLength, s2.halfLength, tol);
}

bool isApprox(const Cylinder &s1, const Cylinder &s2, const FCL_REAL tol) {
  return isApprox(s1.radius, s2.radius, tol) &&
         isApprox(s1.halfLength, s2.halfLength, tol);
}

bool isApprox(const Cone &s1, const Cone &s2, const FCL_REAL tol) {
  return isApprox(s1.radius, s2.radius, tol) &&
         isApprox(s1.halfLength, s2.halfLength, tol);
}

bool isApprox(const TriangleP &s1, const TriangleP &s2, const FCL_REAL tol) {
  return s1.a.isApprox(s2.a, tol) && s1.b.isApprox(s2.b, tol) &&
         s1.c.isApprox(s2.c, tol);
}

bool isApprox(const Halfspace &s1, const Halfspace &s2, const FCL_REAL tol) {
  return isApprox(s1.d, s2.d, tol) && s1.n.isApprox(s2.n, tol);
}

template <typename Shape>
void test(const Shape &original_shape, const FCL_REAL inflation,
          const FCL_REAL tol = 1e-8) {
  // Zero inflation
  {
    const FCL_REAL inflation = 0.;
    const auto &inflation_result = original_shape.inflated(inflation);
    const Transform3f &shift = inflation_result.second;
    const Shape &inflated_shape = inflation_result.first;

    BOOST_CHECK(isApprox(original_shape, inflated_shape, tol));
    BOOST_CHECK(shift.isIdentity(tol));
  }

  // Positive inflation
  {
    const auto &inflation_result = original_shape.inflated(inflation);
    const Shape &inflated_shape = inflation_result.first;
    const Transform3f &inflation_shift = inflation_result.second;

    BOOST_CHECK(!isApprox(original_shape, inflated_shape, tol));

    const auto &deflation_result = inflated_shape.inflated(-inflation);
    const Shape &deflated_shape = deflation_result.first;
    const Transform3f &deflation_shift = deflation_result.second;

    BOOST_CHECK(isApprox(original_shape, deflated_shape, tol));
    BOOST_CHECK((inflation_shift * deflation_shift).isIdentity(tol));
  }

  // Negative inflation
  {
    const auto &inflation_result = original_shape.inflated(-inflation);
    const Shape &inflated_shape = inflation_result.first;
    const Transform3f &inflation_shift = inflation_result.second;

    BOOST_CHECK(!isApprox(original_shape, inflated_shape, tol));

    const auto &deflation_result = inflated_shape.inflated(+inflation);
    const Shape &deflated_shape = deflation_result.first;
    const Transform3f &deflation_shift = deflation_result.second;

    BOOST_CHECK(isApprox(original_shape, deflated_shape, tol));
    BOOST_CHECK((inflation_shift * deflation_shift).isIdentity(tol));
  }
}

template <typename Shape>
void test_throw(const Shape &shape, const FCL_REAL inflation) {
  BOOST_REQUIRE_THROW(shape.inflated(inflation), std::invalid_argument);
}

template <typename Shape>
void test_no_throw(const Shape &shape, const FCL_REAL inflation) {
  BOOST_REQUIRE_NO_THROW(shape.inflated(inflation));
}

BOOST_AUTO_TEST_CASE(test_inflate) {
  const hpp::fcl::Sphere sphere(1);
  test(sphere, 0.01, 1e-8);
  test_throw(sphere, -1.1);
  test_no_throw(sphere, 1.);

  const hpp::fcl::Box box(1, 1, 1);
  test(box, 0.01, 1e-8);
  test_throw(box, -0.6);

  const hpp::fcl::Ellipsoid ellipsoid(1, 2, 3);
  test(ellipsoid, 0.01, 1e-8);
  test_throw(ellipsoid, -1.1);

  const hpp::fcl::Capsule capsule(1., 2.);
  test(capsule, 0.01, 1e-8);
  test_throw(capsule, -1.1);

  const hpp::fcl::Cylinder cylinder(1., 2.);
  test(cylinder, 0.01, 1e-8);
  test_throw(cylinder, -1.1);

  const hpp::fcl::Cone cone(1., 4.);
  test(cone, 0.01, 1e-8);
  test_throw(cone, -1.1);

  const hpp::fcl::Halfspace halfspace(Vec3f::UnitZ(), 0.);
  test(halfspace, 0.01, 1e-8);

  //  const hpp::fcl::TriangleP triangle(Vec3f::UnitX(), Vec3f::UnitY(),
  //                                     Vec3f::UnitZ());
  //  test(triangle, 0.01, 1e-8);
}
