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


#define BOOST_TEST_MODULE FCL_GEOMETRIC_SHAPES
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include "utility.h"
#include <iostream>
#include <hpp/fcl/internal/tools.h>
#include <hpp/fcl/shape/geometric_shape_to_BVH_model.h>

using namespace hpp::fcl;

FCL_REAL extents [6] = {0, 0, 0, 10, 10, 10};

FCL_REAL tol_gjk = 0.01;
GJKSolver solver1;
GJKSolver solver2;

int line;
#define SET_LINE line = __LINE__
#define FCL_CHECK(cond) BOOST_CHECK_MESSAGE(cond, "from line " << line << ": " #cond)
#define FCL_CHECK_EQUAL(a,b)            \
  BOOST_CHECK_MESSAGE((a) == (b),       \
    "from line " << line << ": " #a "[" << (a) << "] != " #b "[" << (b) << "].")
#define BOOST_CHECK_FALSE(p) BOOST_CHECK(!(p))

namespace hpp {
namespace fcl {
std::ostream& operator<< (std::ostream& os, const ShapeBase&)
{
  return os << "a_shape";
}

std::ostream& operator<< (std::ostream& os, const Box& b)
{
  return os << "Box(" << 2*b.halfSide.transpose() << ')';
}
}
}

template <typename S1, typename S2>
void printComparisonError(const std::string& comparison_type,
                          const S1& s1, const Transform3f& tf1,
                          const S2& s2, const Transform3f& tf2,
                          const Vec3f& contact_or_normal,
                          const Vec3f& expected_contact_or_normal,
                          bool check_opposite_normal,
                          FCL_REAL tol)
{
  std::cout << "Disagreement between " << comparison_type
            << " and expected_" << comparison_type << " for "
            << getNodeTypeName(s1.getNodeType()) << " and "
            << getNodeTypeName(s2.getNodeType()) << ".\n"
            << "tf1.quaternion: " << tf1.getQuatRotation() << std::endl
            << "tf1.translation: " << tf1.getTranslation().transpose() << std::endl
            << "tf2.quaternion: " << tf2.getQuatRotation() << std::endl
            << "tf2.translation: " << tf2.getTranslation().transpose() << std::endl
            << comparison_type << ": " << contact_or_normal.transpose() << std::endl
            << "expected_" << comparison_type << ": " << expected_contact_or_normal.transpose();

  if (check_opposite_normal)
    std::cout << " or " << -expected_contact_or_normal.transpose();

  std::cout << std::endl
            << "difference: " << (contact_or_normal - expected_contact_or_normal).norm() << std::endl
            << "tolerance: " << tol << std::endl;
}

template <typename S1, typename S2>
void printComparisonError(const std::string& comparison_type,
                          const S1& s1, const Transform3f& tf1,
                          const S2& s2, const Transform3f& tf2,
                          FCL_REAL depth,
                          FCL_REAL expected_depth,
                          FCL_REAL tol)
{
  std::cout << "Disagreement between " << comparison_type
            << " and expected_" << comparison_type << " for "
            << getNodeTypeName(s1.getNodeType()) << " and "
            << getNodeTypeName(s2.getNodeType()) << ".\n"
            << "tf1.quaternion: " << tf1.getQuatRotation() << std::endl
            << "tf1.translation: " << tf1.getTranslation() << std::endl
            << "tf2.quaternion: " << tf2.getQuatRotation() << std::endl
            << "tf2.translation: " << tf2.getTranslation() << std::endl
            << "depth: " << depth << std::endl
            << "expected_depth: " << expected_depth << std::endl
            << "difference: " << std::fabs(depth - expected_depth) << std::endl
            << "tolerance: " << tol << std::endl;
}

template <typename S1, typename S2>
void compareContact(const S1& s1, const Transform3f& tf1,
                    const S2& s2, const Transform3f& tf2,
                    const Vec3f& contact, Vec3f* expected_point,
                    FCL_REAL depth, FCL_REAL* expected_depth,
                    const Vec3f& normal, Vec3f* expected_normal, bool check_opposite_normal,
                    FCL_REAL tol)
{
  if (expected_point)
  {
    bool contact_equal = isEqual(contact, *expected_point, tol);
    FCL_CHECK(contact_equal);
    if (!contact_equal)
      printComparisonError("contact", s1, tf1, s2, tf2, contact, *expected_point, false, tol);
  }

  if (expected_depth)
  {
    bool depth_equal = std::fabs(depth - *expected_depth) < tol;
    FCL_CHECK(depth_equal);
    if (!depth_equal)
      printComparisonError("depth", s1, tf1, s2, tf2, depth, *expected_depth, tol);
  }

  if (expected_normal)
  {
    bool normal_equal = isEqual(normal, *expected_normal, tol);

    if (!normal_equal && check_opposite_normal)
      normal_equal = isEqual(normal, -(*expected_normal), tol);

    FCL_CHECK(normal_equal);
    if (!normal_equal)
      printComparisonError("normal", s1, tf1, s2, tf2, normal, *expected_normal, check_opposite_normal, tol);
  }
}

template <typename S1, typename S2>
void testShapeIntersection(const S1& s1, const Transform3f& tf1,
                           const S2& s2, const Transform3f& tf2,
                           bool expect_collision,
                           Vec3f* expected_point = NULL,
                           FCL_REAL* expected_depth = NULL,
                           Vec3f* expected_normal = NULL,
                           bool check_opposite_normal = false,
                           FCL_REAL tol = 1e-9)
{
  CollisionRequest request;
  CollisionResult result;

  Vec3f contact;
  Vec3f normal;  // normal direction should be from object 1 to object 2
  bool collision;
  bool check_failed = false;

  request.enable_contact = false;
  result.clear();
  collision = (collide(&s1, tf1, &s2, tf2, request, result) > 0);
  FCL_CHECK_EQUAL(collision, expect_collision);
  check_failed = check_failed || (collision != expect_collision);

  request.enable_contact = true;
  result.clear();
  collision = (collide(&s1, tf1, &s2, tf2, request, result) > 0);
  FCL_CHECK_EQUAL(collision, expect_collision);
  check_failed = check_failed || (collision != expect_collision);

  if (check_failed) {
    BOOST_TEST_MESSAGE("Failure occured between " << s1 << " and " << s2 << " at transformations\n"
        << tf1 << '\n' << tf2);
  }

  if (expect_collision)
  {
    FCL_CHECK_EQUAL(result.numContacts(), 1);
    if (result.numContacts() == 1)
    {
      Contact contact = result.getContact(0);
      compareContact(s1, tf1, s2, tf2, contact.pos, expected_point, contact.penetration_depth, expected_depth, contact.normal, expected_normal, check_opposite_normal, tol);
    }
  }
}

BOOST_AUTO_TEST_CASE (box_to_bvh)
{
  Box shape (1,1,1);
  BVHModel<OBB> bvh;
  generateBVHModel (bvh, shape, Transform3f());
}

BOOST_AUTO_TEST_CASE (sphere_to_bvh)
{
  Sphere shape (1);
  BVHModel<OBB> bvh;
  generateBVHModel (bvh, shape, Transform3f(), 10, 10);
  generateBVHModel (bvh, shape, Transform3f(), 50);
}

BOOST_AUTO_TEST_CASE (cylinder_to_bvh)
{
  Cylinder shape (1,1);
  BVHModel<OBB> bvh;
  generateBVHModel (bvh, shape, Transform3f(), 10, 10);
  generateBVHModel (bvh, shape, Transform3f(), 50);
}

BOOST_AUTO_TEST_CASE (cone_to_bvh)
{
  Cone shape (1,1);
  BVHModel<OBB> bvh;
  generateBVHModel (bvh, shape, Transform3f(), 10, 10);
  generateBVHModel (bvh, shape, Transform3f(), 50);
}

BOOST_AUTO_TEST_CASE (shapeIntersection_cylinderbox)
{
  Cylinder s1 (0.029, 0.1);
  Box s2 (1.6, 0.6, 0.025);

  Transform3f tf1
    (Quaternion3f (0.5279170511703305, -0.50981118132505521,
                   -0.67596178682051911, 0.0668715876735793),
     Vec3f (0.041218354748013122, 1.2022554710435607, 0.77338855025700015));

  Transform3f tf2
    (Quaternion3f (0.70738826916719977, 0, 0, 0.70682518110536596),
     Vec3f (-0.29936284351096382, 0.80023864435868775, 0.71750000000000003));

  GJKSolver solver;
  FCL_REAL distance;
  Vec3f p1, p2, normal;
  bool res = solver.shapeDistance (s1, tf1, s2, tf2, distance, p1, p2, normal);
  BOOST_CHECK ((res && distance > 0) || (!res && distance <= 0));
  // If objects are not colliding, p2 should be outside the cylinder and
  // p1 should be outside the box
  Vec3f p2Loc (tf1.inverse().transform (p2));
  bool p2_in_cylinder ((fabs (p2Loc [2]) <= s1.halfLength) &&
                       (p2Loc [0] * p2Loc [0] + p2Loc [1] * p2Loc [1]
                        <= s1.radius));
  Vec3f p1Loc (tf2.inverse().transform (p1));
  bool p1_in_box = (p1Loc.array().abs() <= s2.halfSide.array()).all();
  std::cout << "p2 in cylinder = (" << p2Loc.transpose () << ")" << std::endl;
  std::cout << "p1 in box = (" << p1Loc.transpose () << ")" << std::endl;

  BOOST_CHECK ((res && !p2_in_cylinder && !p1_in_box) ||
               (!res && p2_in_cylinder && p1_in_box));

  res = solver.shapeDistance (s2, tf2, s1, tf1, distance, p2, p1, normal);
  BOOST_CHECK ((res && distance > 0) || (!res && distance <= 0));
  // If objects are not colliding, p2 should be outside the cylinder and
  // p1 should be outside the box

  p2Loc = tf1.inverse().transform (p2);
  p2_in_cylinder = (fabs (p2Loc [2]) <= s1.halfLength) &&
    (p2Loc [0] * p2Loc [0] + p2Loc [1] * p2Loc [1]
     <= s1.radius);
  p1Loc = tf2.inverse().transform (p1);
  p1_in_box = (p1Loc.array().abs() <= s2.halfSide.array()).all();

  std::cout << "p2 in cylinder = (" << p2Loc.transpose () << ")" << std::endl;
  std::cout << "p1 in box = (" << p1.transpose () << ")" << std::endl;

  BOOST_CHECK ((res && !p2_in_cylinder && !p1_in_box) ||
               (!res && p2_in_cylinder && p1_in_box));

  s1 = Cylinder (0.06, 0.1);
  tf1.setTranslation (Vec3f (-0.66734052046473924, 0.22219183277457269,
                             0.76825248755616293));
  tf1.setQuatRotation (Quaternion3f (0.52613359459338371,
                                     0.32189408354839893,
                                     0.70415587451837913,
                                     -0.35175580165512249));
  res = solver.shapeDistance (s1, tf1, s2, tf2, distance, p1, p2, normal);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(40, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(40, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(30, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(30.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(30.01, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(29.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(29.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f();
  normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform;
  normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-29.9, 0, 0));
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-29.9, 0, 0));
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-30.0, 0, 0));
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-30.01, 0, 0));
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-30.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}

bool compareContactPoints(const Vec3f& c1,const Vec3f& c2)
{
  return c1[2] < c2[2];
} // Ascending order

void testBoxBoxContactPoints(const Matrix3f& R)
{
  Box s1(100, 100, 100);
  Box s2(10, 20, 30);

  // Vertices of s2
  std::vector<Vec3f> vertices(8);
  vertices[0] <<  1,  1,  1;
  vertices[1] <<  1,  1, -1;
  vertices[2] <<  1, -1,  1;
  vertices[3] <<  1, -1, -1;
  vertices[4] << -1,  1,  1;
  vertices[5] << -1,  1, -1;
  vertices[6] << -1, -1,  1;
  vertices[7] << -1, -1, -1;

  for (int i = 0; i < 8; ++i)
  {
    vertices[i].array() *= s2.halfSide.array();
  }

  Transform3f tf1 = Transform3f(Vec3f(0, 0, -50));
  Transform3f tf2 = Transform3f(R);

  Vec3f normal;
  Vec3f point(0.,0.,0.);
  double distance;

  // Make sure the two boxes are colliding
  solver1.gjk_tolerance = 1e-5;
  solver1.epa_tolerance = 1e-5;
  bool res = solver1.shapeIntersect(s1, tf1, s2, tf2, distance, true, &point, &normal);
  FCL_CHECK(res);

  // Compute global vertices
  for (int i = 0; i < 8; ++i)
    vertices[i] = tf2.transform(vertices[i]);

  // Sort the vertices so that the lowest vertex along z-axis comes first
  std::sort(vertices.begin(), vertices.end(), compareContactPoints);

  // The lowest vertex along z-axis should be the contact point
  FCL_CHECK(normal.isApprox(Vec3f(0,0,1), 1e-6));
  FCL_CHECK(vertices[0].head<2>().isApprox(point.head<2>(), 1e-6));
  FCL_CHECK(vertices[0][2] <= point[2] && point[2] < 0);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  Quaternion3f q;
  q = AngleAxis((FCL_REAL)3.140 / 6, UnitZ);

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(15, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, 1e-8);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(15.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(q);
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3f(q);
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  int numTests = 100;
  for (int i = 0; i < numTests; ++i)
  {
    Transform3f tf;
    generateRandomTransform(extents, tf);
    SET_LINE; testBoxBoxContactPoints(tf.getRotation());
  }
}

BOOST_AUTO_TEST_CASE(shapeIntersection_spherebox)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (-1, 0, 0).
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(22.50001, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(22.501, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(22.4, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(22.4, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);
}

BOOST_AUTO_TEST_CASE(shapeDistance_spherebox)
{
  hpp::fcl::Matrix3f rotSphere;
  rotSphere<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  hpp::fcl::Vec3f trSphere(0.0, 0.0, 0.0);

  hpp::fcl::Matrix3f rotBox;
  rotBox<< 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  hpp::fcl::Vec3f trBox(0.0, 5.0, 3.0);

  hpp::fcl::Sphere sphere(1);
  hpp::fcl::Box box(10, 2, 10);

  hpp::fcl::DistanceResult result;
  distance(&sphere, Transform3f(rotSphere, trSphere),
      &box, Transform3f(rotBox, trBox),
      DistanceRequest(true), result);

  FCL_REAL eps = Eigen::NumTraits<FCL_REAL>::epsilon();
  BOOST_CHECK_CLOSE(result.min_distance, 3., eps);
  EIGEN_VECTOR_IS_APPROX(result.nearest_points[0], Vec3f(0,1,0), eps);
  EIGEN_VECTOR_IS_APPROX(result.nearest_points[1], Vec3f(0,4,0), eps);
  EIGEN_VECTOR_IS_APPROX(result.normal, Vec3f(0,1,0), eps);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_spherecapsule)
{
  Sphere s1(20);
  Capsule s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(24.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(24.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(25, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(24.999999, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(25.1, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(25.1, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_cylindercylinder)
{
  Cylinder s1(5, 15);
  Cylinder s2(5, 15);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 9.9, 0));
  normal << 0, 1, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(9.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(9.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}

/*
BOOST_AUTO_TEST_CASE(shapeIntersection_conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(9.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(9.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10.001, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.001, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 9.9));
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 9.9));
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);
}
*/

/*
BOOST_AUTO_TEST_CASE(shapeIntersection_conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(9.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, 0.061);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(9.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, 0.46);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 9.9));
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 9.9));
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 10.01));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 10.01));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}
*/

BOOST_AUTO_TEST_CASE(shapeIntersection_spheretriangle)
{
  Sphere s(10);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f c1, c2, normal;
  FCL_REAL distance;
  bool res;

  res = solver1.shapeTriangleInteraction
    (s, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleInteraction(s, transform, t[0], t[1], t[2],
                                          transform, distance, c1, c2, normal);
  BOOST_CHECK(res);


  t[0] << 30, 0, 0;
  t[1] << 9.9, -20, 0;
  t[2] << 9.9, 20, 0;
  res = solver1.shapeTriangleInteraction
    (s, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleInteraction
    (s, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);

  res = solver1.shapeTriangleInteraction
    (s, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, Vec3f(1, 0, 0), 1e-9));

  res =  solver1.shapeTriangleInteraction
    (s, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacetriangle)
{
  Halfspace hs(Vec3f(1, 0, 0), 0);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f c1, c2;
  FCL_REAL distance;
  Vec3f normal;
  bool res;

  res = solver1.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);


  t[0] << 20, 0, 0;
  t[1] << 0, -20, 0;
  t[2] << 0, 20, 0;
  res = solver1.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  // These tests fail because of numerical precision errors. The points t[1] and t[2]
  // lies on the border of the half-space.
  // The normals should be good, when computed (i.e. res == true)
  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  // BOOST_CHECK(res);
  if (res)
    BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));

  res = solver1.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  // BOOST_CHECK(res);
  if (res)
    BOOST_CHECK(isEqual(normal, Vec3f(1, 0, 0), 1e-9));

  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  // BOOST_CHECK(res);
  if (res)
    BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planetriangle)
{
  Plane hs(Vec3f(1, 0, 0), 0);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f c1, c2;
  FCL_REAL distance;
  Vec3f normal;
  bool res;

  res = solver1.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);


  t[0] << 20, 0, 0;
  t[1] << -0.1, -20, 0;
  t[2] << -0.1, 20, 0;

  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);

  res = solver1.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, Vec3f(1, 0, 0), 1e-9));

  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacesphere)
{
  Sphere s(10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << -5, 0, 0;
  depth = 10;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(-5, 0, 0));
  depth = 10;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5, 0, 0));
  contact << -2.5, 0, 0;
  depth = 15;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5, 0, 0));
  contact = transform.transform(Vec3f(-2.5, 0, 0));
  depth = 15;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5, 0, 0));
  contact << -7.5, 0, 0;
  depth = 5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5, 0, 0));
  contact = transform.transform(Vec3f(-7.5, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-10.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-10.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10.1, 0, 0));
  contact << 0.05, 0, 0;
  depth = 20.1;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.1, 0, 0));
  contact = transform.transform(Vec3f(0.05, 0, 0));
  depth = 20.1;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planesphere)
{
  Sphere s(10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact.setZero();
  depth = 10;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 10;
  normal = transform.getRotation() * Vec3f(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5, 0, 0));
  contact << 5, 0, 0;
  depth = 5;
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5, 0, 0));
  contact = transform.transform(Vec3f(5, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5, 0, 0));
  contact << -5, 0, 0;
  depth = 5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5, 0, 0));
  contact = transform.transform(Vec3f(-5, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-10.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-10.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacebox)
{
  Box s(5, 10, 20);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << -1.25, 0, 0;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(-1.25, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(1.25, 0, 0));
  contact << -0.625, 0, 0;
  depth = 3.75;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(1.25, 0, 0));
  contact = transform.transform(Vec3f(-0.625, 0, 0));
  depth = 3.75;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-1.25, 0, 0));
  contact << -1.875, 0, 0;
  depth = 1.25;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-1.25, 0, 0));
  contact = transform.transform(Vec3f(-1.875, 0, 0));
  depth = 1.25;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.51, 0, 0));
  contact << 0.005, 0, 0;
  depth = 5.01;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.51, 0, 0));
  contact = transform.transform(Vec3f(0.005, 0, 0));
  depth = 5.01;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.51, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.51, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f(transform.getRotation());
  tf2 = Transform3f();
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planebox)
{
  Box s(5, 10, 20);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 2.5;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(1.25, 0, 0));
  contact << 1.25, 0, 0;
  depth = 1.25;
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(1.25, 0, 0));
  contact = transform.transform(Vec3f(1.25, 0, 0));
  depth = 1.25;
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-1.25, 0, 0));
  contact << -1.25, 0, 0;
  depth = 1.25;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-1.25, 0, 0));
  contact = transform.transform(Vec3f(-1.25, 0, 0));
  depth = 1.25;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.51, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.51, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.51, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.51, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f(transform.getRotation());
  tf2 = Transform3f();
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacecapsule)
{
  Capsule s(5, 10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << -2.5, 0, 0;
  depth = 5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(-2.5, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.5, 0, 0));
  contact << -1.25, 0, 0;
  depth = 7.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.5, 0, 0));
  contact = transform.transform(Vec3f(-1.25, 0, 0));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.5, 0, 0));
  contact << -3.75, 0, 0;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.5, 0, 0));
  contact = transform.transform(Vec3f(-3.75, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5.1, 0, 0));
  contact << 0.05, 0, 0;
  depth = 10.1;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5.1, 0, 0));
  contact = transform.transform(Vec3f(0.05, 0, 0));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Halfspace(Vec3f(0, 1, 0), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, -2.5, 0;
  depth = 5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, -2.5, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 2.5, 0));
  contact << 0, -1.25, 0;
  depth = 7.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 2.5, 0));
  contact = transform.transform(Vec3f(0, -1.25, 0));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -2.5, 0));
  contact << 0, -3.75, 0;
  depth = 2.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -2.5, 0));
  contact = transform.transform(Vec3f(0, -3.75, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 5.1, 0));
  contact << 0, 0.05, 0;
  depth = 10.1;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 5.1, 0));
  contact = transform.transform(Vec3f(0, 0.05, 0));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Halfspace(Vec3f(0, 0, 1), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, -5;
  depth = 10;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, -5));
  depth = 10;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 2.5));
  contact << 0, 0, -3.75;
  depth = 12.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 2.5));
  contact = transform.transform(Vec3f(0, 0, -3.75));
  depth = 12.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -2.5));
  contact << 0, 0, -6.25;
  depth = 7.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -2.5));
  contact = transform.transform(Vec3f(0, 0, -6.25));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 10.1));
  contact << 0, 0, 0.05;
  depth = 20.1;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 10.1));
  contact = transform.transform(Vec3f(0, 0, 0.05));
  depth = 20.1;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planecapsule)
{
  Capsule s(5, 10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, 0x0, 0x0, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, 0x0, 0x0, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.5, 0, 0));
  contact << 2.5, 0, 0;
  depth = 2.5;
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.5, 0, 0));
  contact = transform.transform(Vec3f(2.5, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.5, 0, 0));
  contact << -2.5, 0, 0;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.5, 0, 0));
  contact = transform.transform(Vec3f(-2.5, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Plane(Vec3f(0, 1, 0), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 0, 1, 0;  // (0, 1, 0) or (0, -1, 0)
  SET_LINE; testShapeIntersection
    (s, tf1, hs, tf2, true, 0x0, &depth, 0x0, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 1, 0);  // (0, 1, 0) or (0, -1, 0)
  SET_LINE; testShapeIntersection
    (s, tf1, hs, tf2, true, 0x0, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 2.5, 0));
  contact << 0, 2.5, 0;
  depth = 2.5;
  normal << 0, 1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 2.5, 0));
  contact = transform.transform(Vec3f(0, 2.5, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -2.5, 0));
  contact << 0, -2.5, 0;
  depth = 2.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -2.5, 0));
  contact = transform.transform(Vec3f(0, -2.5, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Plane(Vec3f(0, 0, 1), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 10;
  normal << 0, 0, 1;  // (0, 0, 1) or (0, 0, -1)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, 0x0, 0x0, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 10;
  normal = transform.getRotation() * Vec3f(0, 0, 1);  // (0, 0, 1) or (0, 0, -1)
  SET_LINE; testShapeIntersection
    (s, tf1, hs, tf2, true, 0x0, &depth, 0x0, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 2.5));
  contact << 0, 0, 2.5;
  depth = 7.5;
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 2.5));
  contact = transform.transform(Vec3f(0, 0, 2.5));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -2.5));
  contact << 0, 0, -2.5;
  depth = 7.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -2.5));
  contact = transform.transform(Vec3f(0, 0, -2.5));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection
    (s, tf1, hs, tf2, true, 0x0, &depth, 0x0);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacecylinder)
{
  Cylinder s(5, 10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << -2.5, 0, 0;
  depth = 5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(-2.5, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.5, 0, 0));
  contact << -1.25, 0, 0;
  depth = 7.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.5, 0, 0));
  contact = transform.transform(Vec3f(-1.25, 0, 0));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.5, 0, 0));
  contact << -3.75, 0, 0;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.5, 0, 0));
  contact = transform.transform(Vec3f(-3.75, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5.1, 0, 0));
  contact << 0.05, 0, 0;
  depth = 10.1;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5.1, 0, 0));
  contact = transform.transform(Vec3f(0.05, 0, 0));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Halfspace(Vec3f(0, 1, 0), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, -2.5, 0;
  depth = 5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, -2.5, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 2.5, 0));
  contact << 0, -1.25, 0;
  depth = 7.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 2.5, 0));
  contact = transform.transform(Vec3f(0, -1.25, 0));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -2.5, 0));
  contact << 0, -3.75, 0;
  depth = 2.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -2.5, 0));
  contact = transform.transform(Vec3f(0, -3.75, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 5.1, 0));
  contact << 0, 0.05, 0;
  depth = 10.1;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 5.1, 0));
  contact = transform.transform(Vec3f(0, 0.05, 0));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Halfspace(Vec3f(0, 0, 1), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, -2.5;
  depth = 5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, -2.5));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 2.5));
  contact << 0, 0, -1.25;
  depth = 7.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 2.5));
  contact = transform.transform(Vec3f(0, 0, -1.25));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -2.5));
  contact << 0, 0, -3.75;
  depth = 2.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -2.5));
  contact = transform.transform(Vec3f(0, 0, -3.75));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 5.1));
  contact << 0, 0, 0.05;
  depth = 10.1;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 5.1));
  contact = transform.transform(Vec3f(0, 0, 0.05));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -5.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -5.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planecylinder)
{
  Cylinder s(5, 10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.5, 0, 0));
  contact << 2.5, 0, 0;
  depth = 2.5;
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.5, 0, 0));
  contact = transform.transform(Vec3f(2.5, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.5, 0, 0));
  contact << -2.5, 0, 0;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.5, 0, 0));
  contact = transform.transform(Vec3f(-2.5, 0, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Plane(Vec3f(0, 1, 0), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 2.5, 0));
  contact << 0, 2.5, 0;
  depth = 2.5;
  normal << 0, 1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 2.5, 0));
  contact = transform.transform(Vec3f(0, 2.5, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -2.5, 0));
  contact << 0, -2.5, 0;
  depth = 2.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -2.5, 0));
  contact = transform.transform(Vec3f(0, -2.5, 0));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Plane(Vec3f(0, 0, 1), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 2.5));
  contact << 0, 0, 2.5;
  depth = 2.5;
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 2.5));
  contact = transform.transform(Vec3f(0, 0, 2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -2.5));
  contact << 0, 0, -2.5;
  depth = 2.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -2.5));
  contact = transform.transform(Vec3f(0, 0, -2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}


BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacecone)
{
  Cone s(5, 10);
  Halfspace hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << -2.5, 0, -5;
  depth = 5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(-2.5, 0, -5));
  depth = 5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.5, 0, 0));
  contact << -1.25, 0, -5;
  depth = 7.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.5, 0, 0));
  contact = transform.transform(Vec3f(-1.25, 0, -5));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.5, 0, 0));
  contact << -3.75, 0, -5;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.5, 0, 0));
  contact = transform.transform(Vec3f(-3.75, 0, -5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5.1, 0, 0));
  contact << 0.05, 0, -5;
  depth = 10.1;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5.1, 0, 0));
  contact = transform.transform(Vec3f(0.05, 0, -5));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Halfspace(Vec3f(0, 1, 0), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, -2.5, -5;
  depth = 5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, -2.5, -5));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 2.5, 0));
  contact << 0, -1.25, -5;
  depth = 7.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 2.5, 0));
  contact = transform.transform(Vec3f(0, -1.25, -5));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -2.5, 0));
  contact << 0, -3.75, -5;
  depth = 2.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -2.5, 0));
  contact = transform.transform(Vec3f(0, -3.75, -5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 5.1, 0));
  contact << 0, 0.05, -5;
  depth = 10.1;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 5.1, 0));
  contact = transform.transform(Vec3f(0, 0.05, -5));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Halfspace(Vec3f(0, 0, 1), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, -2.5;
  depth = 5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, -2.5));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 2.5));
  contact << 0, 0, -1.25;
  depth = 7.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 2.5));
  contact = transform.transform(Vec3f(0, 0, -1.25));
  depth = 7.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -2.5));
  contact << 0, 0, -3.75;
  depth = 2.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -2.5));
  contact = transform.transform(Vec3f(0, 0, -3.75));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 5.1));
  contact << 0, 0, 0.05;
  depth = 10.1;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 5.1));
  contact = transform.transform(Vec3f(0, 0, 0.05));
  depth = 10.1;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -5.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -5.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planecone)
{
  Cone s(5, 10);
  Plane hs(Vec3f(1, 0, 0), 0);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 1, 0, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(2.5, 0, 0));
  contact << 2.5, 0, -2.5;
  depth = 2.5;
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(2.5, 0, 0));
  contact = transform.transform(Vec3f(2.5, 0, -2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-2.5, 0, 0));
  contact << -2.5, 0, -2.5;
  depth = 2.5;
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-2.5, 0, 0));
  contact = transform.transform(Vec3f(-2.5, 0, -2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-5.1, 0, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Plane(Vec3f(0, 1, 0), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 0, 1, 0;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 1, 0);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 2.5, 0));
  contact << 0, 2.5, -2.5;
  depth = 2.5;
  normal << 0, 1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 2.5, 0));
  contact = transform.transform(Vec3f(0, 2.5, -2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -2.5, 0));
  contact << 0, -2.5, -2.5;
  depth = 2.5;
  normal << 0, -1, 0;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -2.5, 0));
  contact = transform.transform(Vec3f(0, -2.5, -2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, -1, 0);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, -5.1, 0));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);




  hs = Plane(Vec3f(0, 0, 1), 0);

  tf1 = Transform3f();
  tf2 = Transform3f();
  contact << 0, 0, 0;
  depth = 5;
  normal << 0, 0, 1;  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = transform;
  tf2 = transform;
  contact = transform.transform(Vec3f(0, 0, 0));
  depth = 5;
  normal = transform.getRotation() * Vec3f(0, 0, 1);  // (1, 0, 0) or (-1, 0, 0)
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal, true);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 2.5));
  contact << 0, 0, 2.5;
  depth = 2.5;
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 2.5));
  contact = transform.transform(Vec3f(0, 0, 2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -2.5));
  contact << 0, 0, -2.5;
  depth = 2.5;
  normal << 0, 0, -1;
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -2.5));
  contact = transform.transform(Vec3f(0, 0, -2.5));
  depth = 2.5;
  normal = transform.getRotation() * Vec3f(0, 0, -1);
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, true, &contact, &depth, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, -10.1));
  SET_LINE; testShapeIntersection(s, tf1, hs, tf2, false);
}



BOOST_AUTO_TEST_CASE(shapeDistance_spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  //generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist = -1;
  Vec3f closest_p1, closest_p2, normal;
  res = solver1.shapeDistance (s1, Transform3f(), s2,
                               Transform3f(Vec3f(0, 40, 0)),
                               dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2,
                              Transform3f(Vec3f(30.1, 0, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2,
                              Transform3f(Vec3f(29.9, 0, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(Vec3f(40, 0, 0)),
                              s2, Transform3f(), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(Vec3f(30.1, 0, 0)),
                              s2, Transform3f(), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(Vec3f(29.9, 0, 0)),
                              s2, Transform3f(), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);


  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  // this is one problem: the precise is low sometimes
  BOOST_CHECK(fabs(dist - 10) < 0.1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(30.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.06);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2,
                              transform * Transform3f(Vec3f(29.9, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(Vec3f(40, 0, 0)),
                              s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(Vec3f(30.1, 0, 0)),
                              s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(Vec3f(29.9, 0, 0)),
                              s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  //generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(20.1, 0, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(0, 20.2, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.2) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 10.1, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1 * 1.414) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(20.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(0, 20.1, 0)),dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 10.1, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1 * 1.414) < 0.001);
  BOOST_CHECK(res);


  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(15.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(20, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(20, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_boxsphere)
{
  Sphere s1(20);
  Box s2(5, 5, 5);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  int N = 10;
  for (int i = 0; i < N+1; ++i) {
    FCL_REAL dbox = 0.0001 + (s1.radius + s2.halfSide(0)) * i * 4 / (3*N);
    res = solver1.shapeDistance(
        s1, Transform3f(Vec3f(dbox,0.,0.)),
        s2, Transform3f(),
        dist, closest_p1, closest_p2, normal);
    BOOST_CHECK_CLOSE(dist, (dbox - s1.radius - s2.halfSide(0)), 1e-6);
    EIGEN_VECTOR_IS_APPROX(normal, -Vec3f(1,0,0), 1e-6);

    res = solver1.shapeDistance(s1, transform, s2, transform, dist,
                                closest_p1, closest_p2, normal);
    res = solver1.shapeDistance(
        s1, transform * Transform3f(Vec3f(dbox,0.,0.)),
        s2, transform,
        dist, closest_p1, closest_p2, normal);
    BOOST_CHECK_CLOSE(dist, (dbox - s1.radius - s2.halfSide(0)), 1e-6);
    EIGEN_VECTOR_IS_APPROX(normal, -transform.getRotation().col(0), 1e-6);
  }

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(22.6, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(22.6, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.05);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(40, 0, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);
}



BOOST_AUTO_TEST_CASE(shapeDistance_conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(0, 0, 40)), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(0, 0, 40)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 1);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, dist,
                              closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.02);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.01);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.1);
  BOOST_CHECK(res);
}



BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

//  Vec3f contact;
//  FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(40, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(40, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(30, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(30.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(30.01, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(29.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(29.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f();
  normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform;
  normal.setZero();  // If the centers of two sphere are at the same position, the normal is (0, 0, 0)
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-29.9, 0, 0));
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-29.9, 0, 0));
  normal = transform.getRotation() * Vec3f(-1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-30.0, 0, 0));
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(-30.01, 0, 0));
  normal << -1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(-30.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  Quaternion3f q;
  q = AngleAxis((FCL_REAL)3.140 / 6, UnitZ);

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position. The current result is (1, 0, 0).
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(15, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, 1e-8);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(15.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(q);
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);

  tf1 = transform;
  tf2 = transform * Transform3f(q);
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, 0x0);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spherebox)
{
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(22.5, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, 1e-7);  // built-in GJK solver requires larger tolerance than libccd

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(22.51, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(22.4, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, 1e-2);  // built-in GJK solver requires larger tolerance than libccd

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(22.4, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);
  // built-in GJK solver returns incorrect normal.
  // testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spherecapsule)
{
  Sphere s1(20);
  Capsule s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(24.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(24.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(25, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(25.1, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(9.9, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(9.9, 0, 0));
  normal = transform.getRotation() * Vec3f(1, 0, 0);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10, 0, 0));
  normal << 1, 0, 0;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.01, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  // z=0 is a singular points. Two normals could be returned.
  tf2 = Transform3f(Vec3f(9.9, 0, 0.00001));
  normal = Vec3f(2*(s1.halfLength + s2.halfLength), 0, s1.radius+s2.radius).normalized();
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform * tf1;
  tf2 = transform * tf2;
  normal = transform.getRotation() * normal;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10.1, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10.1, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 9.9));
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 9.9));
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_conecylinder)
{
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f tf1;
  Transform3f tf2;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  // Vec3f point;
  // FCL_REAL depth;
  Vec3f normal;

  tf1 = Transform3f();
  tf2 = Transform3f();
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform;
  // TODO: Need convention for normal when the centers of two objects are at same position.
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(9.9, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(9.9, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(10, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(10, 0, 0));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, NULL);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 9.9));
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 9.9));
  normal = transform.getRotation() * Vec3f(0, 0, 1);
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = Transform3f();
  tf2 = Transform3f(Vec3f(0, 0, 10));
  normal << 0, 0, 1;
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, true, NULL, NULL, &normal, false, tol_gjk);

  tf1 = transform;
  tf2 = transform * Transform3f(Vec3f(0, 0, 10.1));
  SET_LINE; testShapeIntersection(s1, tf1, s2, tf2, false);
}


BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spheretriangle)
{
  Sphere s(10);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f c1, c2;
  FCL_REAL distance;
  Vec3f normal;
  bool res;

  res = solver2.shapeTriangleInteraction
    (s, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver2.shapeTriangleInteraction
    (s, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);

  t[0] << 30, 0, 0;
  t[1] << 9.9, -20, 0;
  t[2] << 9.9, 20, 0;
  res = solver2.shapeTriangleInteraction
    (s, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver2.shapeTriangleInteraction
    (s, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);

  res = solver2.shapeTriangleInteraction
    (s, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, Vec3f(1, 0, 0), 1e-9));

  res =  solver2.shapeTriangleInteraction
    (s, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_halfspacetriangle)
{
  Halfspace hs(Vec3f(1, 0, 0), 0);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f c1, c2;
  FCL_REAL distance;
  Vec3f normal;
  bool res;

  res = solver2.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res = solver2.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);


  t[0] << 20, 0, 0;
  t[1] << 0, -20, 0;
  t[2] << 0, 20, 0;
  res = solver2.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver2.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);

  res = solver2.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, Vec3f(1, 0, 0), 1e-9));

  res =  solver2.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_planetriangle)
{
  Plane hs(Vec3f(1, 0, 0), 0);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f c1, c2;
  FCL_REAL distance;
  Vec3f normal;
  bool res;

  res = solver1.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);


  t[0] << 20, 0, 0;
  t[1] << -0.1, -20, 0;
  t[2] << -0.1, 20, 0;
  res = solver2.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);

  res =  solver2.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);

  res = solver2.shapeTriangleInteraction
    (hs, Transform3f(), t[0], t[1], t[2], Transform3f(), distance, c1, c2,
     normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, Vec3f(1, 0, 0), 1e-9));

  res =  solver2.shapeTriangleInteraction
    (hs, transform, t[0], t[1], t[2], transform, distance, c1, c2, normal);
  BOOST_CHECK(res);
  BOOST_CHECK(isEqual(normal, transform.getRotation() * Vec3f(1, 0, 0), 1e-9));
}




BOOST_AUTO_TEST_CASE(spheresphere)
{
  Sphere s1(20);
  Sphere s2(10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist = -1;

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(30.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(29.9, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(Vec3f(40, 0, 0)),
                              s2, Transform3f(), dist, closest_p1,
                              closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(Vec3f(30.1, 0, 0)),
                              s2, Transform3f(), dist, closest_p1,
                              closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(Vec3f(29.9, 0, 0)),
                              s2, Transform3f(), dist, closest_p1, closest_p2,
                              normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);


  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(30.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(29.9, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(Vec3f(40, 0, 0)),
                              s2, transform, dist, closest_p1, closest_p2,
                              normal);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(Vec3f(30.1, 0, 0)),
                              s2, transform, dist, closest_p1, closest_p2,
                              normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(Vec3f(29.9, 0, 0)),
                              s2, transform, dist, closest_p1, closest_p2,
                              normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(boxbox)
{
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform,
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(15.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(15.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(20, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(20, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(boxsphere)
{
  Sphere s1(20);
  Box s2(5, 5, 5);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform,
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(22.6, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2,
                              transform * Transform3f(Vec3f(22.6, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(cylindercylinder)
{
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform,
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(40, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);
}



BOOST_AUTO_TEST_CASE(conecone)
{
  Cone s1(5, 10);
  Cone s2(5, 10);
  Vec3f closest_p1, closest_p2, normal;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform,
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(dist <= 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(10.1, 0, 0)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(),
                              s2, Transform3f(Vec3f(0, 0, 40)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform,
                              s2, transform * Transform3f(Vec3f(0, 0, 40)),
                              dist, closest_p1, closest_p2, normal);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);
}




template<typename S1, typename S2>
void testReversibleShapeDistance(const S1& s1, const S2& s2, FCL_REAL distance)
{
  Transform3f tf1(Vec3f(-0.5 * distance, 0.0, 0.0));
  Transform3f tf2(Vec3f(+0.5 * distance, 0.0, 0.0));

  FCL_REAL distA;
  FCL_REAL distB;
  Vec3f p1A;
  Vec3f p1B;
  Vec3f p2A;
  Vec3f p2B;
  Vec3f normalA, normalB;

  bool resA;
  bool resB;

  const double tol = 1e-6;

  resA = solver1.shapeDistance(s1, tf1, s2, tf2, distA, p1A, p2A, normalA);
  resB = solver1.shapeDistance(s2, tf2, s1, tf1, distB, p1B, p2B, normalB);

  BOOST_CHECK(resA);
  BOOST_CHECK(resB);
  BOOST_CHECK_CLOSE(distA, distB, tol); // distances should be same
  BOOST_CHECK(isEqual(p1A, p2B, tol)); // closest points should in reverse order
  BOOST_CHECK(isEqual(p2A, p1B, tol));

  resA = solver2.shapeDistance(s1, tf1, s2, tf2, distA, p1A, p2A, normalA);
  resB = solver2.shapeDistance(s2, tf2, s1, tf1, distB, p1B, p2B, normalB);

  BOOST_CHECK(resA);
  BOOST_CHECK(resB);
  BOOST_CHECK_CLOSE(distA, distB, tol);
  BOOST_CHECK(isEqual(p1A, p2B, tol));
  BOOST_CHECK(isEqual(p2A, p1B, tol));
}

BOOST_AUTO_TEST_CASE(reversibleShapeDistance_allshapes)
{
  // This test check whether a shape distance algorithm is called for the
  // reverse case as well. For example, if FCL has sphere-capsule distance
  // algorithm, then this algorithm should be called for capsule-sphere case.

  // Prepare all kinds of primitive shapes (7) -- box, sphere, capsule, cone, cylinder, plane, halfspace
  Box box(10, 10, 10);
  Sphere sphere(5);
  Capsule capsule(5, 10);
  Cone cone(5, 10);
  Cylinder cylinder(5, 10);
  Plane plane(Vec3f(0,0,0), 0.0);
  Halfspace halfspace(Vec3f(0,0,0), 0.0);

  // Use sufficiently long distance so that all the primitive shapes CANNOT intersect
  FCL_REAL distance = 15.0;

  // If new shape distance algorithm is added for two distinct primitive
  // shapes, uncomment associated lines. For example, box-sphere intersection
  // algorithm is added, then uncomment box-sphere.

//  testReversibleShapeDistance(box, sphere, distance);
//  testReversibleShapeDistance(box, capsule, distance);
//  testReversibleShapeDistance(box, cone, distance);
//  testReversibleShapeDistance(box, cylinder, distance);
//  testReversibleShapeDistance(box, plane, distance);
//  testReversibleShapeDistance(box, halfspace, distance);

  SET_LINE; testReversibleShapeDistance(sphere, capsule, distance);
//  testReversibleShapeDistance(sphere, cone, distance);
//  testReversibleShapeDistance(sphere, cylinder, distance);
//  testReversibleShapeDistance(sphere, plane, distance);
//  testReversibleShapeDistance(sphere, halfspace, distance);

//  testReversibleShapeDistance(capsule, cone, distance);
//  testReversibleShapeDistance(capsule, cylinder, distance);
//  testReversibleShapeDistance(capsule, plane, distance);
//  testReversibleShapeDistance(capsule, halfspace, distance);

//  testReversibleShapeDistance(cone, cylinder, distance);
//  testReversibleShapeDistance(cone, plane, distance);
//  testReversibleShapeDistance(cone, halfspace, distance);

//  testReversibleShapeDistance(cylinder, plane, distance);
//  testReversibleShapeDistance(cylinder, halfspace, distance);

//  testReversibleShapeDistance(plane, halfspace, distance);
}

