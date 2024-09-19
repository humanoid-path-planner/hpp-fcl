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

#define BOOST_TEST_MODULE COAL_DISTANCE
#include <chrono>

#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "coal/internal/traversal_node_bvhs.h"
#include "coal/internal/traversal_node_setup.h"
#include "../src/collision_node.h"
#include "coal/internal/BV_splitter.h"

#include "utility.h"
#include "fcl_resources/config.h"

using namespace coal;
namespace utf = boost::unit_test::framework;

bool verbose = false;
CoalScalar DELTA = 0.001;

template <typename BV>
void distance_Test(const Transform3s& tf, const std::vector<Vec3s>& vertices1,
                   const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3s>& vertices2,
                   const std::vector<Triangle>& triangles2,
                   SplitMethodType split_method, unsigned int qsize,
                   DistanceRes& distance_result, bool verbose = true);

bool collide_Test_OBB(const Transform3s& tf,
                      const std::vector<Vec3s>& vertices1,
                      const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3s>& vertices2,
                      const std::vector<Triangle>& triangles2,
                      SplitMethodType split_method, bool verbose);

template <typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3s& tf,
                            const std::vector<Vec3s>& vertices1,
                            const std::vector<Triangle>& triangles1,
                            const std::vector<Vec3s>& vertices2,
                            const std::vector<Triangle>& triangles2,
                            SplitMethodType split_method, unsigned int qsize,
                            DistanceRes& distance_result, bool verbose = true);

inline bool nearlyEqual(const Vec3s& a, const Vec3s& b) {
  if (fabs(a[0] - b[0]) > DELTA) return false;
  if (fabs(a[1] - b[1]) > DELTA) return false;
  if (fabs(a[2] - b[2]) > DELTA) return false;
  return true;
}

BOOST_AUTO_TEST_CASE(mesh_distance) {
  std::vector<Vec3s> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  std::vector<Transform3s> transforms;  // t0
  CoalScalar extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#ifndef NDEBUG  // if debug mode
  std::size_t n = 1;
#else
  std::size_t n = 10;
#endif
  n = getNbRun(utf::master_test_suite().argc, utf::master_test_suite().argv, n);

  generateRandomTransforms(extents, transforms, n);

  double dis_time = 0;
  double col_time = 0;

  DistanceRes res, res_now;
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    auto timer_col = std::chrono::high_resolution_clock::now();
    collide_Test_OBB(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
    col_time += std::chrono::duration_cast<std::chrono::duration<double>>(
                    std::chrono::high_resolution_clock::now() - timer_col)
                    .count();

    auto timer_dist = std::chrono::high_resolution_clock::now();
    distance_Test_Oriented<RSS, MeshDistanceTraversalNodeRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res, verbose);
    dis_time += std::chrono::duration_cast<std::chrono::duration<double>>(
                    std::chrono::high_resolution_clock::now() - timer_dist)
                    .count();

    distance_Test_Oriented<RSS, MeshDistanceTraversalNodeRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS, MeshDistanceTraversalNodeRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS, MeshDistanceTraversalNodeRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS, MeshDistanceTraversalNodeRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<RSS, MeshDistanceTraversalNodeRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS, MeshDistanceTraversalNodekIOS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS, MeshDistanceTraversalNodekIOS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS, MeshDistanceTraversalNodekIOS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS, MeshDistanceTraversalNodekIOS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS, MeshDistanceTraversalNodekIOS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<kIOS, MeshDistanceTraversalNodekIOS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test_Oriented<OBBRSS, MeshDistanceTraversalNodeOBBRSS>(
        transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 20, res_now,
        verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER,
                       20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, 2,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER,
                       20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<RSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20,
                       res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2,
                        res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2,
                        res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER,
                        2, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20,
                        res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 20,
                        res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<kIOS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER,
                        20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 2,
                          res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, 2,
                          res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER,
                          2, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, 20,
                          res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEDIAN,
                          20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));

    distance_Test<OBBRSS>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER,
                          20, res_now, verbose);

    BOOST_CHECK(fabs(res.distance - res_now.distance) < DELTA);
    BOOST_CHECK(fabs(res.distance) < DELTA ||
                (res.distance > 0 && nearlyEqual(res.p1, res_now.p1) &&
                 nearlyEqual(res.p2, res_now.p2)));
  }

  BOOST_TEST_MESSAGE("distance timing: " << dis_time << " sec");
  BOOST_TEST_MESSAGE("collision timing: " << col_time << " sec");
}

template <typename BV, typename TraversalNode>
void distance_Test_Oriented(const Transform3s& tf,
                            const std::vector<Vec3s>& vertices1,
                            const std::vector<Triangle>& triangles1,
                            const std::vector<Vec3s>& vertices2,
                            const std::vector<Triangle>& triangles2,
                            SplitMethodType split_method, unsigned int qsize,
                            DistanceRes& distance_result, bool verbose) {
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  DistanceResult local_result;
  TraversalNode node;
  if (!initialize(node, (const BVHModel<BV>&)m1, tf, (const BVHModel<BV>&)m2,
                  Transform3s(), DistanceRequest(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, NULL, qsize);

  // points are in local coordinate, to global coordinate
  Vec3s p1 = local_result.nearest_points[0];
  Vec3s p2 = local_result.nearest_points[1];

  distance_result.distance = local_result.min_distance;
  distance_result.p1 = p1;
  distance_result.p2 = p2;

  if (verbose) {
    std::cout << "distance " << local_result.min_distance << std::endl;

    std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
    std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

template <typename BV>
void distance_Test(const Transform3s& tf, const std::vector<Vec3s>& vertices1,
                   const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3s>& vertices2,
                   const std::vector<Triangle>& triangles2,
                   SplitMethodType split_method, unsigned int qsize,
                   DistanceRes& distance_result, bool verbose) {
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3s pose1(tf), pose2;

  DistanceResult local_result;
  MeshDistanceTraversalNode<BV> node;

  if (!initialize<BV>(node, m1, pose1, m2, pose2, DistanceRequest(true),
                      local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, NULL, qsize);

  distance_result.distance = local_result.min_distance;
  distance_result.p1 = local_result.nearest_points[0];
  distance_result.p2 = local_result.nearest_points[1];

  if (verbose) {
    std::cout << "distance " << local_result.min_distance << std::endl;

    std::cout << local_result.nearest_points[0][0] << " "
              << local_result.nearest_points[0][1] << " "
              << local_result.nearest_points[0][2] << std::endl;
    std::cout << local_result.nearest_points[1][0] << " "
              << local_result.nearest_points[1][1] << " "
              << local_result.nearest_points[1][2] << std::endl;
    std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  }
}

bool collide_Test_OBB(const Transform3s& tf,
                      const std::vector<Vec3s>& vertices1,
                      const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3s>& vertices2,
                      const std::vector<Triangle>& triangles2,
                      SplitMethodType split_method, bool verbose) {
  BVHModel<OBB> m1;
  BVHModel<OBB> m2;
  m1.bv_splitter.reset(new BVSplitter<OBB>(split_method));
  m2.bv_splitter.reset(new BVSplitter<OBB>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  CollisionResult local_result;
  CollisionRequest request(CONTACT | DISTANCE_LOWER_BOUND, 1);
  MeshCollisionTraversalNodeOBB node(request);
  bool success(initialize(node, (const BVHModel<OBB>&)m1, tf,
                          (const BVHModel<OBB>&)m2, Transform3s(),
                          local_result));
  BOOST_REQUIRE(success);

  node.enable_statistics = verbose;

  collide(&node, request, local_result);

  if (local_result.numContacts() > 0)
    return true;
  else
    return false;
}
