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

#define BOOST_TEST_MODULE FCL_BOX_PLANE
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <iostream>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include "utility.h"

BOOST_AUTO_TEST_CASE(box_plane_contact) {
  hpp::fcl::Box box(1, 1, 1);
  hpp::fcl::Halfspace plane({1, 0, 0}, 0);

  // Define transforms
  hpp::fcl::Transform3f T1 = hpp::fcl::Transform3f::Identity();
  hpp::fcl::Transform3f T2 = hpp::fcl::Transform3f::Identity();

  // Compute collision
  hpp::fcl::CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = 1e-6;
  hpp::fcl::CollisionResult res;
  hpp::fcl::ComputeCollision collide_functor(&box, &plane);

  // Intersection
  T1.setTranslation(hpp::fcl::Vec3f(0, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Far from each other
  T1.setTranslation(hpp::fcl::Vec3f(2, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Touching
  T1.setTranslation(hpp::fcl::Vec3f(2, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Rotated
  hpp::fcl::Matrix3f R;
  hpp::fcl::eulerToMatrix(M_PI / 4.0, 0, 0, R);

  // Rotated intersection
  T1.setTranslation(hpp::fcl::Vec3f(2.0, 0, 0));
  T1.setRotation(R);
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Rotated collision
  T1.setTranslation(hpp::fcl::Vec3f(0.5, 0, 0));
  T1.setRotation(R);
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Rotated touching
  T1.setTranslation(hpp::fcl::Vec3f(0.5 * sqrt(2), 0, 0));
  T1.setRotation(R);
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);
}

BOOST_AUTO_TEST_CASE(box_plane_contact_tilted_plane) {
  hpp::fcl::Box box(1, 1, 1);
  hpp::fcl::Halfspace plane({sqrt(2), sqrt(2), 0}, -0.5);

  // Define transforms
  hpp::fcl::Transform3f T1 = hpp::fcl::Transform3f::Identity();
  hpp::fcl::Transform3f T2 = hpp::fcl::Transform3f::Identity();

  // Compute collision
  hpp::fcl::CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = 1e-6;
  hpp::fcl::CollisionResult res;
  hpp::fcl::ComputeCollision collide_functor(&box, &plane);

  // Intersection
  T1.setTranslation(hpp::fcl::Vec3f(0, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == true);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Far from each other
  T1.setTranslation(hpp::fcl::Vec3f(2, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  // Touching
  T1.setTranslation(hpp::fcl::Vec3f(2, 0, 0));
  res.clear();
  BOOST_CHECK(collide(&box, T1, &plane, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);
}

hpp::fcl::DistanceResult getDistance(
    const hpp::fcl::Vec3f &box_side = hpp::fcl::Vec3f(1, 1, 1),
    const hpp::fcl::Vec3f &plane_n = hpp::fcl::Vec3f(1, 0, 0),
    const hpp::fcl::FCL_REAL plane_d = 0.0,
    const hpp::fcl::Matrix3f R1 = hpp::fcl::Matrix3f::Identity(),
    const hpp::fcl::Vec3f &tf1 = hpp::fcl::Vec3f(0, 0, 0)) {
  hpp::fcl::CollisionGeometryPtr_t s1(new hpp::fcl::Box(box_side));
  hpp::fcl::CollisionGeometryPtr_t s2(new hpp::fcl::Plane(plane_n, plane_d));

  hpp::fcl::CollisionObject o1(s1, R1, tf1);
  hpp::fcl::CollisionObject o2(s2, hpp::fcl::Vec3f(0, 0, 0));

  // Enable computation of nearest points
  hpp::fcl::DistanceRequest distanceRequest(true, 0, 0);
  hpp::fcl::DistanceResult distanceResult;

  hpp::fcl::distance(&o1, &o2, distanceRequest, distanceResult);
  return distanceResult;
}

BOOST_AUTO_TEST_CASE(box_plane_distance) {
  hpp::fcl::Vec3f box_side(1, 1, 1);
  hpp::fcl::Vec3f plane_n(1, 0, 0);
  hpp::fcl::FCL_REAL plane_d = 0.0;
  hpp::fcl::Matrix3f R1 = hpp::fcl::Matrix3f::Identity();

  hpp::fcl::Vec3f tf1;
  hpp::fcl::DistanceResult distanceResult;

  // ortogonal positive x
  tf1 = hpp::fcl::Vec3f(6, 0, 0);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 5.5, 1e-12);

  // ortogonal positive y
  tf1 = hpp::fcl::Vec3f(0, 7, 0);
  plane_n = hpp::fcl::Vec3f(0, 1, 0);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 6.5, 1e-12);

  // ortogonal positive z
  tf1 = hpp::fcl::Vec3f(0, 0, 8);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 7.5, 1e-12);

  // ortogonal negative x
  tf1 = hpp::fcl::Vec3f(-6, 0, 0);
  plane_n = hpp::fcl::Vec3f(1, 0, 0);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 5.5, 1e-12);

  // ortogonal negative y
  tf1 = hpp::fcl::Vec3f(0, -7, 0);
  plane_n = hpp::fcl::Vec3f(0, 1, 0);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 6.5, 1e-12);

  // ortogonal negative z
  tf1 = hpp::fcl::Vec3f(0, 0, -8);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 7.5, 1e-12);

  // rotated front
  tf1 = hpp::fcl::Vec3f(6, 0, 0);
  plane_n = hpp::fcl::Vec3f(1, 0, 0);

  hpp::fcl::eulerToMatrix(M_PI / 4.0, 0, 0, R1);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  // this one needs lower precision
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 6 - 0.5 * sqrt(2), 1e-12);

  // rotated back
  tf1 = hpp::fcl::Vec3f(-6, 0, 0);
  plane_n = hpp::fcl::Vec3f(1, 0, 0);
  hpp::fcl::eulerToMatrix(M_PI / 4.0, 0, 0, R1);
  distanceResult = getDistance(box_side, plane_n, plane_d, R1, tf1);
  // this one needs lower precision
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 6 - 0.5 * sqrt(2), 1e-12);
}

hpp::fcl::CollisionResult getCollision(
    const hpp::fcl::Vec3f &box_side = hpp::fcl::Vec3f(1, 1, 1),
    const hpp::fcl::Vec3f &plane_n = hpp::fcl::Vec3f(1, 0, 0),
    const hpp::fcl::FCL_REAL plane_d = 0.0,
    const hpp::fcl::Matrix3f R1 = hpp::fcl::Matrix3f::Identity(),
    const hpp::fcl::Vec3f &tf1 = hpp::fcl::Vec3f(0, 0, 0)) {
  hpp::fcl::CollisionGeometryPtr_t s1(new hpp::fcl::Box(box_side));
  hpp::fcl::CollisionGeometryPtr_t s2(new hpp::fcl::Plane(plane_n, plane_d));

  hpp::fcl::CollisionObject o1(s1, R1, tf1);
  hpp::fcl::CollisionObject o2(s2, hpp::fcl::Vec3f(0, 0, 0));

  // Enable computation of nearest points
  hpp::fcl::CollisionRequest collisionRequest(
      hpp::fcl::CollisionRequestFlag::CONTACT, 1);
  hpp::fcl::CollisionResult collisionResult;

  hpp::fcl::collide(&o1, &o2, collisionRequest, collisionResult);
  return collisionResult;
}

BOOST_AUTO_TEST_CASE(box_plane_collision) {
  hpp::fcl::Vec3f box_side(1, 1, 1);
  hpp::fcl::Vec3f plane_n(1, 0, 0);
  hpp::fcl::FCL_REAL plane_d = 0.25;
  hpp::fcl::Matrix3f R1 = hpp::fcl::Matrix3f::Identity();

  hpp::fcl::Vec3f tf1;
  hpp::fcl::CollisionResult collisionResult;
  hpp::fcl::Vec3f np1;
  hpp::fcl::Vec3f np2;
  hpp::fcl::Contact cp;

  // ortogonal positive x
  tf1 = hpp::fcl::Vec3f(0.75, 0, 0);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  // Check if contact is in right place
  BOOST_CHECK_SMALL(cp.pos[0] - 0.25, 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // ortogonal positive y
  tf1 = hpp::fcl::Vec3f(0, 0.75, 0);
  plane_n = hpp::fcl::Vec3f(0, 1, 0);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1] - 0.25, 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // ortogonal positive z
  tf1 = hpp::fcl::Vec3f(0, 0, 0.75);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2] - 0.25, 1e-12);

  // ortogonal negative x
  tf1 = hpp::fcl::Vec3f(-0.25, 0, 0);
  plane_n = hpp::fcl::Vec3f(1, 0, 0);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0] - 0.25, 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // ortogonal negative y
  tf1 = hpp::fcl::Vec3f(0, -0.25, 0);
  plane_n = hpp::fcl::Vec3f(0, 1, 0);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1] - 0.25, 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // ortogonal negative z
  tf1 = hpp::fcl::Vec3f(0, 0, -0.25);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2] - 0.25, 1e-12);

  // intersecting positive
  tf1 = hpp::fcl::Vec3f(0.1, 0, 0);
  plane_n = hpp::fcl::Vec3f(1, 0, 0);
  plane_d = 0;
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth - 0.4, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // intersecting negative
  tf1 = hpp::fcl::Vec3f(-0.1, 0, 0);
  plane_n = hpp::fcl::Vec3f(1, 0, 0);
  plane_d = 0;
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth - 0.4, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // Rotated, stanting on its one edge with positive Z
  hpp::fcl::eulerToMatrix(0, M_PI / 4, M_PI / 2 - atan(sqrt(2)), R1);
  tf1 = hpp::fcl::Vec3f(0, 0, sqrt(3) * 0.5);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // Rotated, stanting on its one edge with negative Z
  hpp::fcl::eulerToMatrix(0, M_PI / 4, M_PI / 2 - atan(sqrt(2)), R1);
  tf1 = hpp::fcl::Vec3f(0, 0, -sqrt(3) * 0.5);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1], 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);

  // Rotated, stanting on its side with positive Z
  hpp::fcl::eulerToMatrix(0, M_PI / 4, 0, R1);
  tf1 = hpp::fcl::Vec3f(1, 2, -sqrt(2) * 0.5);
  plane_n = hpp::fcl::Vec3f(0, 0, 1);
  collisionResult = getCollision(box_side, plane_n, plane_d, R1, tf1);

  BOOST_CHECK_EQUAL(collisionResult.numContacts(), 1);

  np1 = collisionResult.nearest_points[0];
  np2 = collisionResult.nearest_points[1];

  cp = collisionResult.getContact(0);

  // Check if points equal
  BOOST_CHECK_SMALL((np1 - np2).squaredNorm(), 1e-12);
  BOOST_CHECK_SMALL((np1 - cp.pos).squaredNorm(), 1e-12);

  // Check depth
  BOOST_CHECK_SMALL(cp.penetration_depth, 1e-12);

  BOOST_CHECK_SMALL(cp.pos[0] - 1, 1e-12);
  BOOST_CHECK_SMALL(cp.pos[1] - 2, 1e-12);
  BOOST_CHECK_SMALL(cp.pos[2], 1e-12);
}