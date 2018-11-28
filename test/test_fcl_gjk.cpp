/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, CNRS-LAAS.
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

/** \author Florent Lamiraux <florent@laas.fr> */

#define BOOST_TEST_MODULE FCL_GJK
#define BOOST_TEST_DYN_LINK

#include <time.h>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <Mathematics/GteDistTriangle3Triangle3.h>

using fcl::GJKSolver_indep;
using fcl::TriangleP;
using fcl::Vec3f;
using fcl::Transform3f;
using fcl::Matrix3f;
using fcl::FCL_REAL;
using gte::DCPQuery;
using gte::Triangle3;
using gte::Vector3;

typedef DCPQuery <FCL_REAL, Triangle3 <FCL_REAL>, Triangle3 <FCL_REAL> >
Query_t;

struct Result
{
  bool collision;
  clock_t timeGjk;
  clock_t timeGte;
}; // struct benchmark

typedef std::vector <Result> Results_t;

BOOST_AUTO_TEST_CASE(distance_triangle_triangle_1)
{
  std::size_t N = 10000;
  GJKSolver_indep solver;
  Transform3f tf1, tf2;
  Vec3f p1, p2, a1, a2;
  Matrix3f M;
  FCL_REAL distance (sqrt (-1));
  clock_t start, end;

  std::size_t nCol = 0, nDiff = 0;
  FCL_REAL eps = 1e-7;
  Results_t results (N);
  for (std::size_t i=0; i<N; ++i) {
    Vec3f P1 (Vec3f::Random ()), P2 (Vec3f::Random ()), P3 (Vec3f::Random ());
    Vec3f Q1 (Vec3f::Random ()), Q2 (Vec3f::Random ()), Q3 (Vec3f::Random ());
    TriangleP tri1 (P1, P2, P3);
    TriangleP tri2 (Q1, Q2, Q3);

    start = clock ();
    bool res = solver.shapeDistance (tri1, tf1, tri2, tf2, &distance, &p1, &p2);
    end = clock ();
    results [i].timeGjk = end - start;
    results [i].collision = !res;
    assert (res != (distance < 0));
    if (distance >= 0) {
      // Compute vectors between vertices
      Vec3f u1 (P2 - P1);
      Vec3f v1 (P3 - P1);
      Vec3f w1 (u1.cross (v1));
      Vec3f u2 (Q2 - Q1);
      Vec3f v2 (Q3 - Q1);
      Vec3f w2 (u2.cross (v2));
      // Check that closest points are on triangles
      // Compute a1 such that p1 = P1 + a11 u1 + b11 v1 + c11 u1 x v1
      assert (w1.squaredNorm () > eps*eps);
      M.col (0) = u1; M.col (1) = v1; M.col (2) = w1;
      a1 = M.inverse () * (p1 - P1);
      assert (fabs (a1 [2]) < eps);
      assert (a1 [0] >= -eps);
      assert (a1 [1] >= -eps);
      assert (a1 [0] + a1 [1] <= 1+eps);
      assert (w2.squaredNorm () > eps*eps);
      M.col (0) = u2; M.col (1) = v2; M.col (2) = w2;
      a2 = M.inverse () * (p2 - Q1);
      assert (fabs (a2 [2]) < eps);
      assert (a2 [0] >= -eps);
      assert (a2 [1] >= -eps);
      assert (a2 [0] + a2 [1] <= 1+eps);
    } else {
      ++nCol;
    }
    // Using geometric tools
    Query_t query;
    Vector3 <FCL_REAL> gteP1 = {P1 [0], P1 [1], P1 [2]};
    Vector3 <FCL_REAL> gteP2 = {P2 [0], P2 [1], P2 [2]};
    Vector3 <FCL_REAL> gteP3 = {P3 [0], P3 [1], P3 [2]};
    Vector3 <FCL_REAL> gteQ1 = {Q1 [0], Q1 [1], Q1 [2]};
    Vector3 <FCL_REAL> gteQ2 = {Q2 [0], Q2 [1], Q2 [2]};
    Vector3 <FCL_REAL> gteQ3 = {Q3 [0], Q3 [1], Q3 [2]};
    Triangle3 <FCL_REAL> gteTri1 (gteP1, gteP2, gteP3);
    Triangle3 <FCL_REAL> gteTri2 (gteQ1, gteQ2, gteQ3);
    start = clock ();
    Query_t::Result gteRes = query (gteTri1, gteTri2);
    end = clock ();
    results [i].timeGte = end - start;

    Vector3 <FCL_REAL> gte_p1 = gteRes.closestPoint [0];
    Vector3 <FCL_REAL> gte_p2 = gteRes.closestPoint [1];
    Vector3 <FCL_REAL> gte_p2_p1 = gte_p2 - gte_p1;
    // assert (((distance < 0) && (gteRes.distance == 0)) ||
    //         ((distance > 0) && (gteRes.distance > 0)));
    assert (fabs (gteRes.sqrDistance - Dot (gte_p2_p1, gte_p2_p1)) < eps);
    if ((distance >= 0) && (fabs (distance - gteRes.distance) >= eps)) {
      ++nDiff;
    } else {
      assert ((distance < 0) ||
              ((fabs (gte_p1 [0] - p1 [0]) < eps) &&
               (fabs (gte_p1 [1] - p1 [1]) < eps) &&
               (fabs (gte_p1 [2] - p1 [2]) < eps) &&
               (fabs (gte_p2 [0] - p2 [0]) < eps) &&
               (fabs (gte_p2 [1] - p2 [1]) < eps) &&
               (fabs (gte_p2 [2] - p2 [2]) < eps)) ||
              ((fabs (gte_p1 [0] - p2 [0]) < eps) &&
               (fabs (gte_p1 [1] - p2 [1]) < eps) &&
               (fabs (gte_p1 [2] - p2 [2]) < eps) &&
               (fabs (gte_p2 [0] - p1 [0]) < eps) &&
               (fabs (gte_p2 [1] - p1 [1]) < eps) &&
               (fabs (gte_p2 [2] - p1 [2]) < eps)));
    }
#if 0
    // Compute vectors between vertices
    Vector3 <FCL_REAL> gte_u1 (gteP2 - gteP1);
    Vector3 <FCL_REAL> gte_v1 (gteP3 - gteP1);
    Vector3 <FCL_REAL> gte_w1 (Cross (gte_u1, gte_v1));
    Vector3 <FCL_REAL> gte_u2 (gteQ2 - gteQ1);
    Vector3 <FCL_REAL> gte_v2 (gteQ3 - gteQ1);
    Vector3 <FCL_REAL> gte_w2 (Cross (gte_u2, gte_v2));
    // Check that closest points are on triangles
    // Compute a1 such that p1 = P1 + a11 u1 + b11 v1 + c11 u1 x v1
    assert (std::sqrt(Dot(gte_w1, gte_w1)) > eps*eps);
    M (0,0) = gte_u1 [0]; M (1,0) = gte_u1 [1]; M (2,0) = gte_u1 [2];
    M (0,1) = gte_v1 [0]; M (1,1) = gte_v1 [1]; M (2,1) = gte_v1 [2];
    M (0,2) = gte_w1 [0]; M (1,2) = gte_w1 [1]; M (2,2) = gte_w1 [2];
    p1 [0] = gte_p1 [0]; p1 [1] = gte_p1 [1]; p1 [2] = gte_p1 [2];
    a1 = M.inverse () * (p1 - P1);
    assert (fabs (a1 [2]) < eps);
    assert (a1 [0] >= -eps);
    assert (a1 [1] >= -eps);
    assert (a1 [0] + a1 [1] <= 1+eps);
    // Check whether closest point is on triangle edges
    if ((a1 [0] > eps) && (a1 [1] > eps) && (a1 [0] + a1 [1] < 1 - eps)) {
      std::cerr << "a1 = " << a1.transpose () << std::endl;
    }
    assert (std::sqrt(Dot(gte_w2, gte_w2)) > eps*eps);
    M (0,0) = gte_u2 [0]; M (1,0) = gte_u2 [1]; M (2,0) = gte_u2 [2];
    M (0,1) = gte_v2 [0]; M (1,1) = gte_v2 [1]; M (2,1) = gte_v2 [2];
    M (0,2) = gte_w2 [0]; M (1,2) = gte_w2 [1]; M (2,2) = gte_w2 [2];
    p2 [0] = gte_p2 [0]; p2 [1] = gte_p2 [1]; p2 [2] = gte_p2 [2];
    a2 = M.inverse () * (p2 - Q1);
    assert (fabs (a2 [2]) < eps);
    assert (a2 [0] >= -eps);
    assert (a2 [1] >= -eps);
    assert (a2 [0] + a2 [1] <= 1+eps);
    // Check whether closest point is on triangle edges
    if ((a2 [0] > eps) && (a2 [1] > eps) && (a2 [0] + a2 [1] < 1 - eps)) {
      std::cerr << "a2 = " << a2.transpose () << std::endl;
    }
#endif
  }
  std::cerr << "nCol = " << nCol << std::endl;
  std::cerr << "nDiff = " << nDiff << std::endl;
  // statistics
  clock_t totalTimeGjk = 0, totalTimeGte = 0;
  clock_t totalTimeGjkColl = 0, totalTimeGteColl = 0;
  clock_t totalTimeGjkNoColl = 0, totalTimeGteNoColl = 0;
  std::size_t nGjkBetterThanGteColl = 0;
  std::size_t nGjkBetterThanGteNoColl = 0;
  for (std::size_t i=0; i<N; ++i) {
    if (results [i].collision) {
      totalTimeGjkColl += results [i].timeGjk;
      totalTimeGteColl += results [i].timeGte;
      if ((results [i].timeGjk < results [i].timeGte)) {
        ++ nGjkBetterThanGteColl;
      }
    } else {
      totalTimeGjkNoColl += results [i].timeGjk;
      totalTimeGteNoColl += results [i].timeGte;
      if ((results [i].timeGjk < results [i].timeGte)) {
        ++ nGjkBetterThanGteNoColl;
      }
    }
  }
  std::cerr << "Total time gjk: " << totalTimeGjkNoColl + totalTimeGjkColl
            << std::endl;
  std::cerr << "Total time gte: " << totalTimeGteNoColl + totalTimeGteColl
            << std::endl;
  std::cerr << "Number times GJK better than Geometric tools: "
            << nGjkBetterThanGteNoColl + nGjkBetterThanGteColl << std::endl;
  std::cerr << "-- Collisions -------------------------" << std::endl;
  std::cerr << "Total time gjk: " << totalTimeGjkColl << std::endl;
  std::cerr << "Total time gte: " << totalTimeGteColl << std::endl;
  std::cerr << "Number times GJK better than Geometric tools: "
            << nGjkBetterThanGteColl << std::endl;
  std::cerr << "-- No collisions -------------------------" << std::endl;
  std::cerr << "Total time gjk: " << totalTimeGjkNoColl << std::endl;
  std::cerr << "Total time gte: " << totalTimeGteNoColl << std::endl;
  std::cerr << "Number times GJK better than Geometric tools: "
            << nGjkBetterThanGteNoColl << std::endl;

}
