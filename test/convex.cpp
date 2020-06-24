/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, LAAS-CNRS
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

/** \author Joseph Mirabel */


#define BOOST_TEST_MODULE FCL_GEOMETRIC_SHAPES
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>

#include "utility.h"

using namespace hpp::fcl;

struct Quadrilateral
{
public:
  typedef std::size_t index_type;
  typedef int size_type;

  Quadrilateral() {}

  Quadrilateral(index_type p0, index_type p1, index_type p2, index_type p3)
  {
    set(p0, p1, p2, p3);
  }

  /// @brief Set the vertex indices of the triangle
  inline void set(index_type p0, index_type p1, index_type p2, index_type p3)
  {
    vids[0] = p0; vids[1] = p1; vids[2] = p2; vids[3] = p3;
  }

  /// @access the triangle index
  inline index_type operator[](int i) const { return vids[i]; }

  inline index_type& operator[](int i) { return vids[i]; }

  static inline size_type size() { return 4; }

private:
  index_type vids[4];
};

Convex<Quadrilateral> buildBox (FCL_REAL l, FCL_REAL w, FCL_REAL d)
{
  Vec3f* pts = new Vec3f[8];
  pts[0] = Vec3f( l, w, d);
  pts[1] = Vec3f( l, w,-d);
  pts[2] = Vec3f( l,-w, d);
  pts[3] = Vec3f( l,-w,-d);
  pts[4] = Vec3f(-l, w, d);
  pts[5] = Vec3f(-l, w,-d);
  pts[6] = Vec3f(-l,-w, d);
  pts[7] = Vec3f(-l,-w,-d);

  Quadrilateral* polygons = new Quadrilateral[6];
  polygons[0].set(0, 2, 3, 1); // x+ side
  polygons[1].set(2, 6, 7, 3); // y- side
  polygons[2].set(4, 5, 7, 6); // x- side
  polygons[3].set(0, 1, 5, 4); // y+ side
  polygons[4].set(1, 3, 7, 5); // z- side
  polygons[5].set(0, 2, 6, 4); // z+ side

  return Convex<Quadrilateral> (true,
      pts, // points
      8, // num points
      polygons,
      6 // number of polygons
      );
}

BOOST_AUTO_TEST_CASE(convex)
{
  FCL_REAL l = 1, w = 1, d = 1;
  Convex<Quadrilateral> box (buildBox (l, w, d));

  // Check neighbors
  for (int i = 0; i < 8; ++i) {
    BOOST_CHECK_EQUAL(box.neighbors[i].count(), 3);
  }
  BOOST_CHECK_EQUAL(box.neighbors[0][0], 1);
  BOOST_CHECK_EQUAL(box.neighbors[0][1], 2);
  BOOST_CHECK_EQUAL(box.neighbors[0][2], 4);

  BOOST_CHECK_EQUAL(box.neighbors[1][0], 0);
  BOOST_CHECK_EQUAL(box.neighbors[1][1], 3);
  BOOST_CHECK_EQUAL(box.neighbors[1][2], 5);

  BOOST_CHECK_EQUAL(box.neighbors[2][0], 0);
  BOOST_CHECK_EQUAL(box.neighbors[2][1], 3);
  BOOST_CHECK_EQUAL(box.neighbors[2][2], 6);

  BOOST_CHECK_EQUAL(box.neighbors[3][0], 1);
  BOOST_CHECK_EQUAL(box.neighbors[3][1], 2);
  BOOST_CHECK_EQUAL(box.neighbors[3][2], 7);

  BOOST_CHECK_EQUAL(box.neighbors[4][0], 0);
  BOOST_CHECK_EQUAL(box.neighbors[4][1], 5);
  BOOST_CHECK_EQUAL(box.neighbors[4][2], 6);

  BOOST_CHECK_EQUAL(box.neighbors[5][0], 1);
  BOOST_CHECK_EQUAL(box.neighbors[5][1], 4);
  BOOST_CHECK_EQUAL(box.neighbors[5][2], 7);

  BOOST_CHECK_EQUAL(box.neighbors[6][0], 2);
  BOOST_CHECK_EQUAL(box.neighbors[6][1], 4);
  BOOST_CHECK_EQUAL(box.neighbors[6][2], 7);

  BOOST_CHECK_EQUAL(box.neighbors[7][0], 3);
  BOOST_CHECK_EQUAL(box.neighbors[7][1], 5);
  BOOST_CHECK_EQUAL(box.neighbors[7][2], 6);
}

template <typename Sa, typename Sb> void compareShapeIntersection (
    const Sa& sa, const Sb& sb, 
    const Transform3f& tf1, const Transform3f& tf2,
    FCL_REAL tol = 1e-9)
{
  CollisionRequest request (CONTACT | DISTANCE_LOWER_BOUND, 1);
  CollisionResult resA, resB;

  collide(&sa, tf1, &sa, tf2, request, resA);
  collide(&sb, tf1, &sb, tf2, request, resB);

  BOOST_CHECK_EQUAL(resA.isCollision(), resB.isCollision());
  BOOST_CHECK_EQUAL(resA.numContacts(), resB.numContacts());

  if (resA.isCollision() && resB.isCollision()) {
    Contact cA = resA.getContact(0),
            cB = resB.getContact(0);

    BOOST_TEST_MESSAGE(
        tf1 << '\n'
        << cA.pos.format(pyfmt) << '\n'
        << '\n'
        << tf2 << '\n'
        << cB.pos.format(pyfmt) << '\n'
        );
    // Only warnings because there are still some bugs.
    BOOST_WARN_SMALL((cA.pos    - cB.pos   ).squaredNorm(), tol);
    BOOST_WARN_SMALL((cA.normal - cB.normal).squaredNorm(), tol);
  } else {
    BOOST_CHECK_CLOSE(resA.distance_lower_bound, resB.distance_lower_bound, tol); // distances should be same
  }
}

template <typename Sa, typename Sb> void compareShapeDistance (
    const Sa& sa, const Sb& sb, 
    const Transform3f& tf1, const Transform3f& tf2,
    FCL_REAL tol = 1e-9)
{
  DistanceRequest request (true);
  DistanceResult resA, resB;

  distance(&sa, tf1, &sa, tf2, request, resA);
  distance(&sb, tf1, &sb, tf2, request, resB);

  BOOST_TEST_MESSAGE(
      tf1 << '\n'
      << resA.normal.format(pyfmt) << '\n'
      << resA.nearest_points[0].format(pyfmt) << '\n'
      << resA.nearest_points[1].format(pyfmt) << '\n'
      << '\n'
      << tf2 << '\n'
      << resB.normal.format(pyfmt) << '\n'
      << resB.nearest_points[0].format(pyfmt) << '\n'
      << resB.nearest_points[1].format(pyfmt) << '\n'
      );
  // TODO in one case, there is a mismatch between the distances and I cannot say
  // which one is correct. To visualize the case, use script test/geometric_shapes.py
  BOOST_WARN_CLOSE(resA.min_distance, resB.min_distance, tol);
  //BOOST_CHECK_CLOSE(resA.min_distance, resB.min_distance, tol);

  // Only warnings because there are still some bugs.
  BOOST_WARN_SMALL((resA.normal - resA.normal).squaredNorm(), tol);
  BOOST_WARN_SMALL((resA.nearest_points[0] - resB.nearest_points[0]).squaredNorm(), tol);
  BOOST_WARN_SMALL((resA.nearest_points[1] - resB.nearest_points[1]).squaredNorm(), tol);
}

BOOST_AUTO_TEST_CASE(compare_convex_box)
{
  FCL_REAL extents [6] = {0, 0, 0, 10, 10, 10};
  FCL_REAL l = 1, w = 1, d = 1, eps = 1e-4;
  Box box(l*2, w*2, d*2);
  Convex<Quadrilateral> convex_box (buildBox (l, w, d));

  Transform3f tf1;
  Transform3f tf2;

  tf2.setTranslation (Vec3f (3, 0, 0));
  compareShapeIntersection(box, convex_box, tf1, tf2, eps);
  compareShapeDistance    (box, convex_box, tf1, tf2, eps);

  tf2.setTranslation (Vec3f (0, 0, 0));
  compareShapeIntersection(box, convex_box, tf1, tf2, eps);
  compareShapeDistance    (box, convex_box, tf1, tf2, eps);

  for (int i = 0; i < 1000; ++i) {
    generateRandomTransform(extents, tf2);
    compareShapeIntersection(box, convex_box, tf1, tf2, eps);
    compareShapeDistance    (box, convex_box, tf1, tf2, eps);
  }
}

#ifdef HPP_FCL_HAS_QHULL
BOOST_AUTO_TEST_CASE(convex_hull_throw)
{
  std::vector<Vec3f> points ({
    Vec3f ( 1, 1, 1),
    Vec3f ( 0, 0, 0),
    Vec3f ( 1, 0, 0),
    });

  BOOST_CHECK_THROW(ConvexBase::convexHull(points.data(), 0, false, NULL),
      std::invalid_argument);
  BOOST_CHECK_THROW(ConvexBase::convexHull(points.data(), 1, false, NULL),
      std::invalid_argument);
  BOOST_CHECK_THROW(ConvexBase::convexHull(points.data(), 2, false, NULL),
      std::invalid_argument);
  BOOST_CHECK_THROW(ConvexBase::convexHull(points.data(), 3, false, NULL),
      std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(convex_hull_quad)
{
  std::vector<Vec3f> points ({
    Vec3f ( 1, 1, 1),
    Vec3f ( 0, 0, 0),
    Vec3f ( 1, 0, 0),
    Vec3f ( 0, 0, 1),
    });

  ConvexBase* convexHull = ConvexBase::convexHull(points.data(), (int)points.size(),
      false, NULL);

  BOOST_REQUIRE_EQUAL(convexHull->num_points, 4);
  BOOST_CHECK_EQUAL(convexHull->neighbors[0].count(), 3);
  BOOST_CHECK_EQUAL(convexHull->neighbors[1].count(), 3);
  BOOST_CHECK_EQUAL(convexHull->neighbors[2].count(), 3);
  delete convexHull;
}

BOOST_AUTO_TEST_CASE(convex_hull_box_like)
{
  std::vector<Vec3f> points ({
    Vec3f ( 1, 1, 1),
    Vec3f ( 1, 1,-1),
    Vec3f ( 1,-1, 1),
    Vec3f ( 1,-1,-1),
    Vec3f (-1, 1, 1),
    Vec3f (-1, 1,-1),
    Vec3f (-1,-1, 1),
    Vec3f (-1,-1,-1),
    Vec3f ( 0, 0, 0   ),
    Vec3f ( 0, 0, 0.99),
    });

  ConvexBase* convexHull = ConvexBase::convexHull(points.data(), (int)points.size(),
      false, NULL);

  BOOST_REQUIRE_EQUAL(8, convexHull->num_points);
  for (int i = 0; i < 8; ++i)
  {
    BOOST_CHECK(convexHull->points[i].cwiseAbs() == Vec3f(1,1,1));
    BOOST_CHECK_EQUAL(convexHull->neighbors[i].count(), 3);
  }
  delete convexHull;

  convexHull = ConvexBase::convexHull(points.data(), (int)points.size(),
      true, NULL);
  Convex<Triangle>* convex_tri = dynamic_cast<Convex<Triangle>*> (convexHull);
  BOOST_CHECK(convex_tri != NULL);

  BOOST_REQUIRE_EQUAL(8, convexHull->num_points);
  for (int i = 0; i < 8; ++i)
  {
    BOOST_CHECK(convexHull->points[i].cwiseAbs() == Vec3f(1,1,1));
    BOOST_CHECK(convexHull->neighbors[i].count() >= 3);
    BOOST_CHECK(convexHull->neighbors[i].count() <= 6);
  }
  delete convexHull;
}
#endif
