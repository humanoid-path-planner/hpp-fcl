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

BOOST_AUTO_TEST_CASE(sphere_sphere) {
  CollisionGeometryPtr_t s1(new hpp::fcl::Sphere(1));
  CollisionGeometryPtr_t s2(new hpp::fcl::Sphere(2));

  const Transform3f tf1;
  const Transform3f tf2_collision(Vec3f(0, 0, 3));

  // No security margin - collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    collide(s1.get(), tf1, s2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, 0, 1e-8);
    BOOST_CHECK_SMALL(-collisionResult.getContact(0).penetration_depth, 1e-8);
  }

  // No security margin - no collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    const double distance = 0.01;
    Transform3f tf2_no_collision(
        Vec3f(tf2_collision.getTranslation() + Vec3f(0, 0, distance)));
    collide(s1.get(), tf1, s2.get(), tf2_no_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(!collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, distance, 1e-8);
  }

  // Positive security margin
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    const double distance = 0.01;
    collisionRequest.security_margin = distance;
    Transform3f tf2_no_collision(
        Vec3f(tf2_collision.getTranslation() + Vec3f(0, 0, distance)));
    collide(s1.get(), tf1, s2.get(), tf2_no_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, 0, 1e-8);
    BOOST_CHECK_CLOSE(-collisionResult.getContact(0).penetration_depth,
                      distance, 1e-8);
  }

  // Negative security margin - collion because the two spheres are in contact
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    const double distance = -0.01;
    collisionRequest.security_margin = distance;
    Transform3f tf2(
        Vec3f(tf2_collision.getTranslation() + Vec3f(0, 0, distance)));
    collide(s1.get(), tf1, s2.get(), tf2, collisionRequest, collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_SMALL(collisionResult.distance_lower_bound, 1e-8);
    BOOST_CHECK_CLOSE(-collisionResult.getContact(0).penetration_depth,
                      distance, 1e-8);
  }

  // Negative security margin - no collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    collisionRequest.security_margin = -0.01;
    collide(s1.get(), tf1, s2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(!collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, 0.01, 1e-8);
  }
}

BOOST_AUTO_TEST_CASE(capsule_capsule) {
  CollisionGeometryPtr_t c1(new hpp::fcl::Capsule(0.5, 1.));
  CollisionGeometryPtr_t c2(new hpp::fcl::Capsule(0.5, 1.));

  const Transform3f tf1;
  const Transform3f tf2_collision(Vec3f(0, 1., 0));

  // No security margin - collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    collide(c1.get(), tf1, c2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_SMALL(collisionResult.distance_lower_bound, 1e-8);
    BOOST_CHECK_SMALL(-collisionResult.getContact(0).penetration_depth, 1e-8);
  }

  // No security margin - no collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    const double distance = 0.01;
    Transform3f tf2_no_collision(
        Vec3f(tf2_collision.getTranslation() + Vec3f(0, distance, 0)));
    collide(c1.get(), tf1, c2.get(), tf2_no_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(!collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, distance, 1e-8);
  }

  // Positive security margin - collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    const double distance = 0.01;
    collisionRequest.security_margin = distance;
    Transform3f tf2_no_collision(
        Vec3f(tf2_collision.getTranslation() + Vec3f(0, distance, 0)));
    collide(c1.get(), tf1, c2.get(), tf2_no_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_SMALL(collisionResult.distance_lower_bound, 1e-8);
    BOOST_CHECK_CLOSE(-collisionResult.getContact(0).penetration_depth,
                      distance, 1e-8);
  }

  // Negative security margin - collion because the two capsules are in contact
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    const double distance = -0.01;
    collisionRequest.security_margin = distance;
    Transform3f tf2(
        Vec3f(tf2_collision.getTranslation() + Vec3f(0, distance, 0)));
    collide(c1.get(), tf1, c2.get(), tf2, collisionRequest, collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_SMALL(collisionResult.distance_lower_bound, 1e-8);
    BOOST_CHECK_CLOSE(-collisionResult.getContact(0).penetration_depth,
                      distance, 1e-8);
  }

  // Negative security margin - no collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    collisionRequest.security_margin = -0.01;
    collide(c1.get(), tf1, c2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(!collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, 0.01, 1e-8);
  }
}

BOOST_AUTO_TEST_CASE(box_box) {
  CollisionGeometryPtr_t b1(new hpp::fcl::Box(1, 1, 1));
  CollisionGeometryPtr_t b2(new hpp::fcl::Box(1, 1, 1));

  const Transform3f tf1;
  const Transform3f tf2_collision(Vec3f(0, 0, 1));

  const double tol = 1e-3;

  // No security margin - collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    CollisionResult collisionResult;
    collide(b1.get(), tf1, b2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_SMALL(collisionResult.distance_lower_bound, tol);
    BOOST_CHECK_SMALL(-collisionResult.getContact(0).penetration_depth, 1e-8);
  }

  // No security margin - no collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    const double distance = 0.01;
    const Transform3f tf2_no_collision(
        (tf2_collision.getTranslation() + Vec3f(0, 0, distance)).eval());

    CollisionResult collisionResult;
    collide(b1.get(), tf1, b2.get(), tf2_no_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(!collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound, distance, tol);
  }

  // Positive security margin - collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    const double distance = 0.01;
    collisionRequest.security_margin = distance;
    CollisionResult collisionResult;
    collide(b1.get(), tf1, b2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound,
                      -collisionRequest.security_margin, tol);
    BOOST_CHECK_SMALL(-collisionResult.getContact(0).penetration_depth, 1e-8);
  }

  // Positive security margin - no collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    collisionRequest.security_margin = -0.01;
    CollisionResult collisionResult;
    collide(b1.get(), tf1, b2.get(), tf2_collision, collisionRequest,
            collisionResult);
    BOOST_CHECK(!collisionResult.isCollision());
    BOOST_CHECK_CLOSE(collisionResult.distance_lower_bound,
                      -collisionRequest.security_margin, tol);
  }

  // Negative security margin - collision
  {
    CollisionRequest collisionRequest(CONTACT, 1);
    collisionRequest.security_margin = -0.01;
    CollisionResult collisionResult;

    const Transform3f tf2((tf2_collision.getTranslation() +
                           Vec3f(0, 0, collisionRequest.security_margin))
                              .eval());
    collide(b1.get(), tf1, b2.get(), tf2, collisionRequest, collisionResult);
    BOOST_CHECK(collisionResult.isCollision());
    BOOST_CHECK_SMALL(collisionResult.distance_lower_bound, tol);
    BOOST_CHECK_CLOSE(-collisionResult.getContact(0).penetration_depth,
                      collisionRequest.security_margin, tol);
  }
}
