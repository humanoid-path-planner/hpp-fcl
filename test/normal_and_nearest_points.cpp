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

#define BOOST_TEST_MODULE NORMAL_AND_NEAREST_POINTS
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision_data.h>

#include "utility.h"

using hpp::fcl::Box;
using hpp::fcl::Capsule;
using hpp::fcl::CollisionRequest;
using hpp::fcl::CollisionResult;
using hpp::fcl::Cone;
using hpp::fcl::constructPolytopeFromEllipsoid;
using hpp::fcl::Contact;
using hpp::fcl::Convex;
using hpp::fcl::Cylinder;
using hpp::fcl::DistanceRequest;
using hpp::fcl::DistanceResult;
using hpp::fcl::Ellipsoid;
using hpp::fcl::FCL_REAL;
using hpp::fcl::Halfspace;
using hpp::fcl::Plane;
using hpp::fcl::shared_ptr;
using hpp::fcl::Sphere;
using hpp::fcl::Transform3f;
using hpp::fcl::Triangle;
using hpp::fcl::Vec3f;

// This test is designed to check if the normal and the nearest points
// are properly handled as defined in DistanceResult::normal.
// Regardless of wether or not the two shapes are in intersection, regardless of
// wether `collide` or `distance` is called:
// --> we denote `dist` the (signed) distance that separates the two shapes
// --> we denote `p1` and `p2` their nearest_points (witness points)
// Thus:
// --> if o1 and o2 are not in collision, translating o2 by vector (abs(dist) -
// eps) * normal should bring them in collision.
// --> if o1 and o2 are in collision, translating o1 by vector (dist + eps)
// * normal should separate them.
// --> finally, if abs(dist) > 0, we should have normal = sign(dist) * (p2 -
// p1).normalized() and p1 = p2 - dist * normal
template <typename ShapeType1, typename ShapeType2>
void test_normal_and_nearest_points(const ShapeType1& o1,
                                    const ShapeType2& o2) {
  // Generate random poses for o2
  size_t n = 10000;
  FCL_REAL extents[] = {-2., -2., -2., 2., 2., 2.};
  std::vector<Transform3f> transforms;
  generateRandomTransforms(extents, transforms, n);
  Transform3f tf1 = Transform3f::Identity();

  CollisionRequest colreq;
  colreq.distance_upper_bound = std::numeric_limits<FCL_REAL>::max();
  CollisionResult colres;
  DistanceRequest distreq;
  DistanceResult distres;

  for (size_t i = 0; i < n; i++) {
    // Run both `distance` and `collide`.
    // Since CollisionRequest::distance_lower_bound is set to infinity,
    // these functions should agree on the results regardless of collision or
    // not.
    Transform3f tf2 = transforms[i];
    colres.clear();
    distres.clear();
    size_t col = collide(&o1, tf1, &o2, tf2, colreq, colres);
    FCL_REAL dist = distance(&o1, tf1, &o2, tf2, distreq, distres);

    if (col) {
      BOOST_CHECK(dist <= 0.);
      BOOST_CHECK_CLOSE(dist, distres.min_distance, 1e-6);
      Contact contact = colres.getContact(0);
      BOOST_CHECK_CLOSE(dist, contact.penetration_depth, 1e-6);

      Vec3f cp1 = contact.nearest_points[0];
      EIGEN_VECTOR_IS_APPROX(cp1, distres.nearest_points[0], 1e-6);

      Vec3f cp2 = contact.nearest_points[1];
      EIGEN_VECTOR_IS_APPROX(cp2, distres.nearest_points[1], 1e-6);
      BOOST_CHECK_CLOSE(contact.penetration_depth, -(cp2 - cp1).norm(), 1e-6);
      EIGEN_VECTOR_IS_APPROX(cp1, cp2 - dist * distres.normal, 1e-6);

      Vec3f separation_vector = contact.penetration_depth * contact.normal;
      EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, 1e-6);

      if (dist < 0) {
        EIGEN_VECTOR_IS_APPROX(contact.normal, -(cp2 - cp1).normalized(), 1e-6);
      }

      // Separate the shapes
      Vec3f t = tf1.getTranslation();
      FCL_REAL eps = 1e-2;
      tf1.setTranslation(t + separation_vector - eps * contact.normal);
      colres.clear();
      distres.clear();
      col = collide(&o1, tf1, &o2, tf2, colreq, colres);
      dist = distance(&o1, tf1, &o2, tf2, distreq, distres);
      BOOST_CHECK(dist > 0);
      BOOST_CHECK(!col);
      BOOST_CHECK_CLOSE(colres.distance_lower_bound, dist, 1e-6);
      BOOST_CHECK(fabs(colres.distance_lower_bound - eps) <= 1e-2);
      cp1 = distres.nearest_points[0];
      cp2 = distres.nearest_points[1];
      BOOST_CHECK_CLOSE(dist, (cp1 - cp2).norm(), 1e-6);
      EIGEN_VECTOR_IS_APPROX(cp1, cp2 - dist * distres.normal, 1e-6);

      separation_vector = dist * distres.normal;
      EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, 1e-6);

      if (dist > 0) {
        EIGEN_VECTOR_IS_APPROX(distres.normal, (cp2 - cp1).normalized(), 1e-6);
      }
    } else {
      BOOST_CHECK(dist >= 0.);
      BOOST_CHECK_CLOSE(distres.min_distance, dist, 1e-6);
      BOOST_CHECK_CLOSE(dist, colres.distance_lower_bound, 1e-6);

      Vec3f cp1 = distres.nearest_points[0];
      Vec3f cp2 = distres.nearest_points[1];
      BOOST_CHECK_CLOSE(dist, (cp1 - cp2).norm(), 1e-6);
      EIGEN_VECTOR_IS_APPROX(cp1, cp2 - dist * distres.normal, 1e-6);

      Vec3f separation_vector = dist * distres.normal;
      EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, 1e-6);

      if (dist > 0) {
        EIGEN_VECTOR_IS_APPROX(distres.normal, (cp2 - cp1).normalized(), 1e-6);
      }

      // Bring the shapes in collision
      Vec3f t = tf1.getTranslation();
      FCL_REAL eps = 1e-2;
      tf1.setTranslation(t + separation_vector + eps * distres.normal);
      colres.clear();
      distres.clear();
      col = collide(&o1, tf1, &o2, tf2, colreq, colres);
      dist = distance(&o1, tf1, &o2, tf2, distreq, distres);
      BOOST_CHECK(dist < 0);
      BOOST_CHECK(col);
      // Contrary to Contact::penetration_depth,
      // CollisionResult::distance_lower_bound is a signed distance like
      // DistanceResult::min_distance
      BOOST_CHECK_CLOSE(colres.distance_lower_bound, dist, 1e-6);
      BOOST_CHECK(fabs(colres.distance_lower_bound - -eps) <= 1e-2);
      Contact contact = colres.getContact(0);
      cp1 = contact.nearest_points[0];
      EIGEN_VECTOR_IS_APPROX(cp1, distres.nearest_points[0], 1e-6);

      cp2 = contact.nearest_points[1];
      EIGEN_VECTOR_IS_APPROX(cp2, distres.nearest_points[1], 1e-6);
      BOOST_CHECK_CLOSE(contact.penetration_depth, -(cp2 - cp1).norm(), 1e-6);
      EIGEN_VECTOR_IS_APPROX(cp1, cp2 - dist * distres.normal, 1e-6);

      separation_vector = contact.penetration_depth * contact.normal;
      EIGEN_VECTOR_IS_APPROX(separation_vector, cp2 - cp1, 1e-6);

      if (dist < 0) {
        EIGEN_VECTOR_IS_APPROX(contact.normal, -(cp2 - cp1).normalized(), 1e-6);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_sphere) {
  FCL_REAL r = 0.5;
  shared_ptr<Sphere> o1(new Sphere(r));
  shared_ptr<Sphere> o2(new Sphere(r));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_capsule) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Sphere> o1(new Sphere(r));
  shared_ptr<Capsule> o2(new Capsule(r, h));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_box) {
  FCL_REAL r = 0.5;
  FCL_REAL rbox = 2 * 0.5;
  shared_ptr<Box> o1(new Box(rbox, rbox, rbox));
  shared_ptr<Sphere> o2(new Sphere(r));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_mesh) {
  FCL_REAL r = 0.5;
  Convex<Triangle> o1_ = constructPolytopeFromEllipsoid(Ellipsoid(r, r, r));
  shared_ptr<Convex<Triangle>> o1(new Convex<Triangle>(
      false, o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
  Convex<Triangle> o2_ = constructPolytopeFromEllipsoid(Ellipsoid(r, r, r));
  shared_ptr<Convex<Triangle>> o2(new Convex<Triangle>(
      false, o2_.points, o2_.num_points, o2_.polygons, o2_.num_polygons));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_box) {
  FCL_REAL r = 0.5;
  FCL_REAL rbox = 2 * 0.5;
  Convex<Triangle> o1_ = constructPolytopeFromEllipsoid(Ellipsoid(r, r, r));
  shared_ptr<Convex<Triangle>> o1(new Convex<Triangle>(
      false, o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
  shared_ptr<Box> o2(new Box(rbox, rbox, rbox));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_ellipsoid) {
  FCL_REAL r = 0.5;
  Convex<Triangle> o1_ = constructPolytopeFromEllipsoid(Ellipsoid(r, r, r));
  shared_ptr<Convex<Triangle>> o1(new Convex<Triangle>(
      false, o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
  shared_ptr<Ellipsoid> o2(new Ellipsoid(0.5 * r, 1.3 * r, 0.8 * r));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_box_plane) {
  FCL_REAL rbox = 1;
  shared_ptr<Box> o1(new Box(rbox, rbox, rbox));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Plane> o2(new Plane(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_box_halfspace) {
  FCL_REAL rbox = 1;
  shared_ptr<Box> o1(new Box(rbox, rbox, rbox));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_capsule_halfspace) {
  FCL_REAL r = 0.5;
  FCL_REAL d = 1.;
  shared_ptr<Capsule> o1(new Capsule(r, d));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_halfspace) {
  FCL_REAL r = 0.5;
  shared_ptr<Sphere> o1(new Sphere(r));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_plane) {
  FCL_REAL r = 0.5;
  shared_ptr<Sphere> o1(new Sphere(r));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Plane> o2(new Plane(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_mesh_halfspace) {
  FCL_REAL r = 0.5;
  Convex<Triangle> o1_ = constructPolytopeFromEllipsoid(Ellipsoid(r, r, r));
  shared_ptr<Convex<Triangle>> o1(new Convex<Triangle>(
      false, o1_.points, o1_.num_points, o1_.polygons, o1_.num_polygons));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cone_halfspace) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Cone> o1(new Cone(r, h));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cylinder_halfspace) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Cylinder> o1(new Cylinder(r, h));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cone_plane) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Cone> o1(new Cone(r, h));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Plane> o2(new Plane(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_cylinder_plane) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Cylinder> o1(new Cylinder(r, h));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Plane> o2(new Plane(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_capsule_plane) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Capsule> o1(new Capsule(r, h));
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Plane> o2(new Plane(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_capsule_capsule) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Capsule> o1(new Capsule(r, h));
  shared_ptr<Capsule> o2(new Capsule(r, h));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_sphere_cylinder) {
  FCL_REAL r = 0.5;
  FCL_REAL h = 1.;
  shared_ptr<Sphere> o1(new Sphere(r));
  shared_ptr<Cylinder> o2(new Cylinder(r, h));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_ellipsoid_halfspace) {
  Vec3f radii(0.3, 0.5, 0.2);
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Ellipsoid> o1(new Ellipsoid(radii));
  shared_ptr<Halfspace> o2(new Halfspace(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}

BOOST_AUTO_TEST_CASE(test_normal_and_nearest_points_ellispoid_plane) {
  Vec3f radii(0.3, 0.5, 0.4);
  FCL_REAL offset = 0.1;
  Vec3f n = Vec3f::Random();
  n.normalize();
  shared_ptr<Ellipsoid> o1(new Ellipsoid(radii));
  shared_ptr<Plane> o2(new Plane(n, offset));

  test_normal_and_nearest_points(*o1.get(), *o2.get());
  test_normal_and_nearest_points(*o2.get(), *o1.get());
}
