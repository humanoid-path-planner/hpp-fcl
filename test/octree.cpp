/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS-LAAS.
 *  Copyright (c) 2023, INRIA.
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

#define BOOST_TEST_MODULE FCL_OCTREE
#include <fstream>
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/hfield.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/internal/BV_splitter.h>

#include "utility.h"
#include "fcl_resources/config.h"

namespace utf = boost::unit_test::framework;

using namespace hpp::fcl;

void makeMesh(const std::vector<Vec3f>& vertices,
              const std::vector<Triangle>& triangles, BVHModel<OBBRSS>& model) {
  hpp::fcl::SplitMethodType split_method(hpp::fcl::SPLIT_METHOD_MEAN);
  model.bv_splitter.reset(new BVSplitter<OBBRSS>(split_method));
  model.bv_splitter.reset(new BVSplitter<OBBRSS>(split_method));

  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

hpp::fcl::OcTree makeOctree(const BVHModel<OBBRSS>& mesh,
                            const FCL_REAL& resolution) {
  Vec3f m(mesh.aabb_local.min_);
  Vec3f M(mesh.aabb_local.max_);
  hpp::fcl::Box box(resolution, resolution, resolution);
  CollisionRequest request(hpp::fcl::CONTACT | hpp::fcl::DISTANCE_LOWER_BOUND,
                           1);
  CollisionResult result;
  Transform3f tfBox;
  octomap::OcTreePtr_t octree(new octomap::OcTree(resolution));

  for (FCL_REAL x = resolution * floor(m[0] / resolution); x <= M[0];
       x += resolution) {
    for (FCL_REAL y = resolution * floor(m[1] / resolution); y <= M[1];
         y += resolution) {
      for (FCL_REAL z = resolution * floor(m[2] / resolution); z <= M[2];
           z += resolution) {
        Vec3f center(x + .5 * resolution, y + .5 * resolution,
                     z + .5 * resolution);
        tfBox.setTranslation(center);
        hpp::fcl::collide(&box, tfBox, &mesh, Transform3f(), request, result);
        if (result.isCollision()) {
          octomap::point3d p((float)center[0], (float)center[1],
                             (float)center[2]);
          octree->updateNode(p, true);
          result.clear();
        }
      }
    }
  }

  octree->updateInnerOccupancy();
  octree->writeBinary("./env.octree");
  return OcTree(octree);
}

BOOST_AUTO_TEST_CASE(octree_mesh) {
  Eigen::IOFormat tuple(Eigen::FullPrecision, Eigen::DontAlignCols, "", ", ",
                        "", "", "(", ")");
  FCL_REAL resolution(10.);
  std::vector<Vec3f> pRob, pEnv;
  std::vector<Triangle> tRob, tEnv;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "rob.obj").string().c_str(), pRob, tRob);
  loadOBJFile((path / "env.obj").string().c_str(), pEnv, tEnv);

  BVHModel<OBBRSS> robMesh, envMesh;
  // Build meshes with robot and environment
  makeMesh(pRob, tRob, robMesh);
  makeMesh(pEnv, tEnv, envMesh);
  // Build octomap with environment
  envMesh.computeLocalAABB();
  // Load octree built from envMesh by makeOctree(envMesh, resolution)
  OcTree envOctree(
      hpp::fcl::loadOctreeFile((path / "env.octree").string(), resolution));

  std::cout << "Finished loading octree." << std::endl;

  // Test operator==
  {
    BOOST_CHECK(envOctree == envOctree);
    BOOST_CHECK(envOctree == OcTree(envOctree));

    const OcTree envOctree_from_tree(envOctree.getTree());
    BOOST_CHECK(envOctree == envOctree_from_tree);
  }

  // Test tobytes()
  {
    const std::vector<uint8_t> bytes = envOctree.tobytes();
    BOOST_CHECK(bytes.size() > 0 && bytes.size() <= envOctree.toBoxes().size() *
                                                        3 * sizeof(FCL_REAL));
  }

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-2000, -2000, 0, 2000, 2000, 2000};
#ifndef NDEBUG  // if debug mode
  std::size_t N = 100;
#else
  std::size_t N = 10000;
#endif
  N = hpp::fcl::getNbRun(utf::master_test_suite().argc,
                         utf::master_test_suite().argv, N);

  generateRandomTransforms(extents, transforms, 2 * N);

  CollisionRequest request(hpp::fcl::CONTACT | hpp::fcl::DISTANCE_LOWER_BOUND,
                           1);
  for (std::size_t i = 0; i < N; ++i) {
    CollisionResult resultMesh;
    CollisionResult resultOctree;
    Transform3f tf1(transforms[2 * i]);
    Transform3f tf2(transforms[2 * i + 1]);
    ;
    // Test collision between meshes with random transform for robot.
    hpp::fcl::collide(&robMesh, tf1, &envMesh, tf2, request, resultMesh);
    // Test collision between mesh and octree for the same transform.
    hpp::fcl::collide(&robMesh, tf1, &envOctree, tf2, request, resultOctree);
    bool resMesh(resultMesh.isCollision());
    bool resOctree(resultOctree.isCollision());
    BOOST_CHECK(!resMesh || resOctree);
    if (!resMesh && resOctree) {
      hpp::fcl::DistanceRequest dreq;
      hpp::fcl::DistanceResult dres;
      hpp::fcl::distance(&robMesh, tf1, &envMesh, tf2, dreq, dres);
      std::cout << "distance mesh mesh: " << dres.min_distance << std::endl;
      BOOST_CHECK(dres.min_distance < sqrt(2.) * resolution);
    }
  }
}

BOOST_AUTO_TEST_CASE(octree_height_field) {
  Eigen::IOFormat tuple(Eigen::FullPrecision, Eigen::DontAlignCols, "", ", ",
                        "", "", "(", ")");
  FCL_REAL resolution(10.);
  std::vector<Vec3f> pEnv;
  std::vector<Triangle> tEnv;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "env.obj").string().c_str(), pEnv, tEnv);

  BVHModel<OBBRSS> envMesh;
  // Build meshes with robot and environment
  makeMesh(pEnv, tEnv, envMesh);
  // Build octomap with environment
  envMesh.computeLocalAABB();
  // Load octree built from envMesh by makeOctree(envMesh, resolution)
  OcTree envOctree(
      hpp::fcl::loadOctreeFile((path / "env.octree").string(), resolution));

  std::cout << "Finished loading octree." << std::endl;

  // Building hfield
  const FCL_REAL x_dim = 10, y_dim = 20;
  const int nx = 100, ny = 100;
  const FCL_REAL max_altitude = 1., min_altitude = 0.;
  const MatrixXf heights = MatrixXf::Constant(ny, nx, max_altitude);

  HeightField<AABB> hfield(x_dim, y_dim, heights, min_altitude);
  hfield.computeLocalAABB();

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-2000, -2000, 0, 2000, 2000, 2000};
#ifndef NDEBUG  // if debug mode
  std::size_t N = 1000;
#else
  std::size_t N = 100000;
#endif
  N = hpp::fcl::getNbRun(utf::master_test_suite().argc,
                         utf::master_test_suite().argv, N);

  generateRandomTransforms(extents, transforms, 2 * N);

  CollisionRequest request(hpp::fcl::CONTACT | hpp::fcl::DISTANCE_LOWER_BOUND,
                           1);
  for (std::size_t i = 0; i < N; ++i) {
    CollisionResult resultBox;
    CollisionResult resultHfield1, resultHfield2;
    Transform3f tf1(transforms[2 * i]);
    Transform3f tf2(transforms[2 * i + 1]);

    Box box;
    Transform3f box_tf;
    constructBox(hfield.aabb_local, tf2, box, box_tf);

    // Test collision between octree and equivalent box.
    hpp::fcl::collide(&envOctree, tf1, &box, box_tf, request, resultBox);
    // Test collision between octree and hfield.
    hpp::fcl::collide(&envOctree, tf1, &hfield, tf2, request, resultHfield1);
    hpp::fcl::collide(&hfield, tf2, &envOctree, tf1, request, resultHfield2);

    bool resBox(resultBox.isCollision());
    bool resHfield(resultHfield1.isCollision());
    BOOST_CHECK(resBox == resHfield);
    BOOST_CHECK(resultHfield1.isCollision() == resultHfield2.isCollision());
    if (!resBox && resHfield) {
      hpp::fcl::DistanceRequest dreq;
      hpp::fcl::DistanceResult dres;
      hpp::fcl::distance(&envMesh, tf1, &box, box_tf, dreq, dres);
      std::cout << "distance mesh box: " << dres.min_distance << std::endl;
      BOOST_CHECK(dres.min_distance < sqrt(2.) * resolution);
    }
  }
}
