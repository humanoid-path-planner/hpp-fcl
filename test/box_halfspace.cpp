/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Krzysztof Wojciechowski
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

/** \author Krzysztof Wojciechowski <krzy.wojciecho@gmail.com> */

#define BOOST_TEST_MODULE FCL_BOX_HALFSPACE
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include "utility.h"

using hpp::fcl::Box;
using hpp::fcl::CollisionRequest;
using hpp::fcl::CollisionResult;
using hpp::fcl::Halfspace;
using hpp::fcl::Matrix3f;
using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;

BOOST_AUTO_TEST_CASE(box_halfspace_contact) {
  Box box(1.0, 1.0, 1.0);
  Halfspace halfspace({1.0, 0.0, 0.0}, 0.0);

  // Define transforms
  Transform3f T1 = Transform3f::Identity();
  Transform3f T2 = Transform3f::Identity();

  // Compute collision
  CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = 1e-6;
  CollisionResult res;
  hpp::fcl::ComputeCollision collide_functor(&box, &halfspace);

  // Intersection
  T1.setTranslation(Vec3f(0.0, 0.0, 0.0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Far from each other
  T1.setTranslation(Vec3f(2.0, 0.0, 0.0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Touching
  T1.setTranslation(Vec3f(2.0, 0.0, 0.0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Rotated
  Matrix3f R;
  hpp::fcl::eulerToMatrix(M_PI / 4.0, 0.0, 0.0, R);

  // Rotated intersection
  T1.setTranslation(Vec3f(2.0, 0.0, 0.0));
  T1.setRotation(R);
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Rotated collision
  T1.setTranslation(Vec3f(0.5, 0.0, 0.0));
  T1.setRotation(R);
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Rotated touching
  T1.setTranslation(Vec3f(0.5 * sqrt(2.0), 0.0, 0.0));
  T1.setRotation(R);
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);
}

BOOST_AUTO_TEST_CASE(box_halfspace_contact_tilted_halfspace) {
  Box box(1.0, 1.0, 1.0);
  Halfspace halfspace({sqrt(2.0), sqrt(2.0), 0.0}, -0.5);

  // Define transforms
  Transform3f T1 = Transform3f::Identity();
  Transform3f T2 = Transform3f::Identity();

  // Compute collision
  CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = 1e-6;
  CollisionResult res;
  hpp::fcl::ComputeCollision collide_functor(&box, &halfspace);

  // Intersection
  T1.setTranslation(Vec3f(0.0, 0.0, 0.0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Far from each other
  T1.setTranslation(Vec3f(2.0, 0.0, 0.0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Touching
  T1.setTranslation(Vec3f(2.0, 0.0, 0.0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &halfspace, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);
}

hpp::fcl::DistanceResult getDistance(const Vec3f &box_side = Vec3f(1.0, 1.0,
                                                                   1.0),
                                     const Vec3f &halfspace_n = Vec3f(1.0, 0.0,
                                                                      0.0),
                                     const hpp::fcl::FCL_REAL halfspace_d = 0.0,
                                     const Matrix3f R1 = Matrix3f::Identity(),
                                     const Vec3f &t1 = Vec3f(0.0, 0.0, 0.0)) {
  hpp::fcl::CollisionGeometryPtr_t s1(new Box(box_side));
  hpp::fcl::CollisionGeometryPtr_t s2(new Halfspace(halfspace_n, halfspace_d));

  hpp::fcl::CollisionObject o1(s1, R1, t1);
  hpp::fcl::CollisionObject o2(s2, Vec3f(0.0, 0.0, 0.0));

  // Enable computation of nearest points
  hpp::fcl::DistanceRequest distanceRequest(true, 0.0, 0.0);
  hpp::fcl::DistanceResult distanceResult;

  hpp::fcl::distance(&o1, &o2, distanceRequest, distanceResult);
  return distanceResult;
}

BOOST_AUTO_TEST_CASE(box_halfspace_distance) {
  Vec3f box_side(1.0, 1.0, 1.0);
  Vec3f halfspace_n(1.0, 0.0, 0.0);
  hpp::fcl::FCL_REAL halfspace_d = 0.0;
  Matrix3f R1 = Matrix3f::Identity();

  Vec3f t1;
  hpp::fcl::DistanceResult distanceResult;

  // Ortogonal positive x
  t1 = Vec3f(6.0, 0.0, 0.0);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance - 5.5, 1e-12);

  // Ortogonal positive y
  t1 = Vec3f(0.0, 7.0, 0.0);
  halfspace_n = Vec3f(0.0, 1.0, 0.0);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance - 6.5, 1e-12);

  // Ortogonal positive z
  t1 = Vec3f(0.0, 0.0, 8.0);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance - 7.5, 1e-12);

  // Ortogonal negative x
  t1 = Vec3f(-6.0, 0.0, 0.0);
  halfspace_n = Vec3f(1.0, 0.0, 0.0);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance + 6.5, 1e-12);

  // Ortogonal negative y
  t1 = Vec3f(0.0, -7.0, 0.0);
  halfspace_n = Vec3f(0.0, 1.0, 0.0);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance + 7.5, 1e-12);

  // Ortogonal negative z
  t1 = Vec3f(0.0, 0.0, -8.0);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance + 8.5, 1e-12);

  // Rotated front
  t1 = Vec3f(6.0, 0.0, 0.0);
  halfspace_n = Vec3f(1.0, 0.0, 0.0);

  hpp::fcl::eulerToMatrix(M_PI / 4.0, 0.0, 0.0, R1);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance - 6.0 + 0.5 * sqrt(2.0), 1e-12);

  // Rotated back
  t1 = Vec3f(-6.0, 0.0, 0.0);
  halfspace_n = Vec3f(1.0, 0.0, 0.0);
  hpp::fcl::eulerToMatrix(M_PI / 4.0, 0.0, 0.0, R1);
  distanceResult = getDistance(box_side, halfspace_n, halfspace_d, R1, t1);
  BOOST_CHECK_SMALL(distanceResult.min_distance + 6.0 + 0.5 * sqrt(2.0), 1e-12);
}

CollisionResult getCollision(const Vec3f &box_side = Vec3f(1.0, 1.0, 1.0),
                             const Vec3f &halfspace_n = Vec3f(1.0, 0.0, 0.0),
                             const hpp::fcl::FCL_REAL halfspace_d = 0.0,
                             const Matrix3f R1 = Matrix3f::Identity(),
                             const Vec3f &t1 = Vec3f(0.0, 0.0, 0.0)) {
  hpp::fcl::CollisionGeometryPtr_t s1(new Box(box_side));
  hpp::fcl::CollisionGeometryPtr_t s2(new Halfspace(halfspace_n, halfspace_d));

  hpp::fcl::CollisionObject o1(s1, R1, t1);
  hpp::fcl::CollisionObject o2(s2, Transform3f::Identity());

  // Enable computation of nearest points
  CollisionRequest collisionRequest(hpp::fcl::CollisionRequestFlag::CONTACT, 1);
  CollisionResult collisionResult;

  hpp::fcl::collide(&o1, &o2, collisionRequest, collisionResult);
  return collisionResult;
}

BOOST_AUTO_TEST_CASE(box_halfspace_collision) {
  Vec3f box_side(1.0, 1.0, 1.0);
  Vec3f halfspace_n(1.0, 0.0, 0.0);
  hpp::fcl::FCL_REAL halfspace_d = 0.25;
  Matrix3f R1 = Matrix3f::Identity();

  Vec3f t1;
  CollisionResult collisionResult;
  Vec3f np0;
  Vec3f np1;
  hpp::fcl::Contact contact;

  // Ortogonal positive x
  // Expected:
  // - one contact point at the rear side of the Box
  // - contact points equal
  // - contact points at [0.25, 0.0, 0.0]
  // - penetration depth equal to 0.0
  t1 = Vec3f(0.75, 0.0, 0.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check if contact points are in right place
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0.25, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], contact.nearest_points[0],
                         1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, np1, 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.pos, 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, 0, 1e-12);

  // Ortogonal positive y
  // Expected:
  // - one contact point at the right side of the Box
  // - contact points equal
  // - contact points at [0.0, 0.25, 0.0]
  // - penetration depth equal to 0.0
  t1 = Vec3f(0.0, 0.75, 0.0);
  halfspace_n = Vec3f(0.0, 1.0, 0.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check if contact points are in right place
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, 0.25, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], contact.nearest_points[0],
                         1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, np1, 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.pos, 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, 0, 1e-12);

  // Ortogonal positive z
  // Expected:
  // - one contact point at the bottom of the Box
  // - contact points equal
  // - contact points at [0.0, 0.0, 0.25]
  // - penetration depth equal to 0.0
  t1 = Vec3f(0.0, 0.0, 0.75);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check if contact points are in right place
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, 0, 0.25), 1e-12);
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], contact.nearest_points[0],
                         1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, np1, 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.pos, 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, 0, 1e-12);

  // Test for the symetry of the contacts

  // Ortogonal negative x
  // Expected:
  // - one contact point at the front of the Box
  // - contact point of Box is at [-0.75, 0.0, 0.0]
  // - contact point of Halfspace at [0.25, 0.0, 0.0]
  // - penetration depth equal to -1.0
  t1 = Vec3f(-0.25, 0.0, 0.0);
  halfspace_n = Vec3f(1.0, 0.0, 0.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(-0.75, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0.25, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check penetration depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -1.0, 1e-12);

  // Ortogonal negative y
  // Expected:
  // - one contact point at the front of the Box
  // - contact point of Box is at [0.0, -0.75, 0.0]
  // - contact point of Halfspace at [0.0, 0.25, 0.0]
  // - penetration depth equal to -1.0
  t1 = Vec3f(0.0, -0.25, 0.0);
  halfspace_n = Vec3f(0.0, 1.0, 0.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, -0.75, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0, 0.25, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -1.0, 1e-12);

  // Ortogonal negative z
  // Expected:
  // - one contact point at the front of the Box
  // - contact point of Box is at [0.0, -0.75, 0.0]
  // - contact point of Halfspace at [0.0, 0.25, 0.0]
  // - penetration depth equal to -1.0
  t1 = Vec3f(0.0, 0.0, -0.25);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, 0, -0.75), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0, 0, 0.25), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -1.0, 1e-12);

  // Intersecting positive
  // Expected:
  // - one contact point lying on the surface of the Halfspace and
  //   equal to its closest point
  // - closest point of Box on the surface at the point [-0.4, 0.0, 0.0]
  // - closest point of Halfspace on the surface at the point [0.0, 0.0, 0.0]
  // - penetration depth equal to 0.4
  t1 = Vec3f(0.1, 0.0, 0.0);
  halfspace_n = Vec3f(1.0, 0.0, 0.0);
  halfspace_d = 0.0;
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);
  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(-0.4, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -0.4, 1e-12);

  // Intersecting negative
  // Expected:
  // - one contact point lying on the surface of the Halfspace and
  //   equal to its closest point
  // - closest point of Box on the surface at the point [-0.6, 0.0, 0.0]
  // - closest point of Halfspace on the surface at the point [0.0, 0.0, 0.0]
  // - penetration depth equal to -0.6
  t1 = Vec3f(-0.1, 0.0, 0.0);
  halfspace_n = Vec3f(1.0, 0.0, 0.0);
  halfspace_d = 0.0;
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(-0.6, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -0.6, 1e-12);

  // Rotated, stanting on its one edge
  // Expected:
  // - one contact point on the tip of the Box
  // - both contact points at [0.0, 0.0, 0.0]
  // - penetration depth equal to 0.0
  hpp::fcl::eulerToMatrix(0.0, M_PI / 4.0, M_PI / 2.0 - atan(sqrt(2.0)), R1);
  t1 = Vec3f(0.0, 0.0, sqrt(3.0) * 0.5);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check if contact points are in right place
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], contact.nearest_points[0],
                         1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, np1, 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.pos, 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, 0, 1e-12);

  // Rotated, stanting on its one edge, partially intersecting
  // Expected:
  // - contact point of Box at its tip at the point [0.0, 0.0, -0.1]
  // - contact point of Halfspace on the surface at the point [0.0, 0.0, 0.0]
  // - penetration depth equal to -0.1
  hpp::fcl::eulerToMatrix(0.0, M_PI / 4.0, M_PI / 2.0 - atan(sqrt(2.0)), R1);
  t1 = Vec3f(0.0, 0.0, sqrt(3.0) * 0.5 - 0.1);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, 0, -0.1), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -0.1, 1e-12);

  // Rotated, stanting on its one edge, full intersecting
  // Expected:
  // - contact point of Box at its tip at the point [0.0, 0.0, -2.]
  // - contact point of Halfspace on the surface at the point [0.0, 0.0, 0.0]
  // - penetration depth equal to -2.0
  hpp::fcl::eulerToMatrix(0.0, M_PI / 4.0, M_PI / 2.0 - atan(sqrt(2.0)), R1);
  t1 = Vec3f(0.0, 0.0, sqrt(3.0) * 0.5 - 2.0);
  halfspace_n = Vec3f(0.0, 0.0, 1.0);
  collisionResult = getCollision(box_side, halfspace_n, halfspace_d, R1, t1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np0 = collisionResult.nearest_points[0];
  np1 = collisionResult.nearest_points[1];

  contact = collisionResult.getContact(0);

  // Check position of Box closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[0], Vec3f(0, 0, -2.0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np0, contact.nearest_points[0], 1e-12);

  // Check position of Halfspace closest point
  EIGEN_VECTOR_IS_APPROX(contact.nearest_points[1], Vec3f(0, 0, 0), 1e-12);
  EIGEN_VECTOR_IS_APPROX(np1, contact.nearest_points[1], 1e-12);

  // We should have cp = (np0 + np1)/2
  EIGEN_VECTOR_IS_APPROX(
      contact.pos,
      0.5 * (contact.nearest_points[0] + contact.nearest_points[1]), 1e-12);

  // Check depth
  BOOST_CHECK_CLOSE(contact.penetration_depth, -2.0, 1e-12);
}
