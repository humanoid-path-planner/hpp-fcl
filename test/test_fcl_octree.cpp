/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS-LAAS.
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
#define BOOST_TEST_DYN_LINK
#include <fstream>
#include <boost/test/unit_test.hpp>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include "test_fcl_utility.h"
#include "fcl_resources/config.h"
#include <boost/filesystem.hpp>

using fcl::Vec3f;
using fcl::Triangle;
using fcl::OBBRSS;
using fcl::BVHModel;
using fcl::BVSplitter;
using fcl::OcTree;
using fcl::FCL_REAL;
using fcl::Transform3f;
using fcl::CollisionRequest;
using fcl::CollisionResult;

void makeMesh (const std::vector<Vec3f>& vertices,
               const std::vector<Triangle>& triangles, BVHModel<OBBRSS>& model)
{
  fcl::SplitMethodType split_method (fcl::SPLIT_METHOD_MEAN);
  model.bv_splitter.reset(new BVSplitter<OBBRSS>(split_method));
  model.bv_splitter.reset(new BVSplitter<OBBRSS>(split_method));

  model.beginModel();
  model.addSubModel(vertices, triangles);
  model.endModel();
}

fcl::OcTree makeOctree (const BVHModel <OBBRSS>& mesh,
                        const FCL_REAL& resolution)
{
  std::ofstream file;
  file.open ("./env.octree");
  Eigen::IOFormat csv (Eigen::FullPrecision, Eigen::DontAlignCols, "",
                        ", ", "", "", "", "");
  Vec3f m (mesh.aabb_local.min_);
  Vec3f M (mesh.aabb_local.max_);
  fcl::Box box (resolution, resolution, resolution);
  CollisionRequest request (true, 1, true);
  CollisionResult result;
  Transform3f tfBox;
  octomap::OcTreePtr_t octree (new octomap::OcTree (resolution));

  for (FCL_REAL x = m [0]; x + resolution <= M [0]; x += resolution) {
    for (FCL_REAL y = m [1]; y + resolution <= M [1]; y += resolution) {
      for (FCL_REAL z = m [2]; z + resolution <= M [2]; z += resolution) {
        Vec3f center (x + .5*resolution, y + .5*resolution, z + .5*resolution);
        tfBox.setTranslation (center);
        fcl::collide (&box, tfBox, &mesh, Transform3f (), request, result);
        if (result.isCollision ()) {
          octomap::point3d p
            ((float) center [0], (float) center [1], (float) center [2]);
          octree->updateNode (p, true);
          file << center.format (csv) << std::endl;
          result.clear ();
        }
      }
    }
  }
  octree->updateInnerOccupancy();
  file.close ();
  return OcTree (octree);
}

BOOST_AUTO_TEST_CASE (OCTREE)
{
  Eigen::IOFormat tuple (Eigen::FullPrecision, Eigen::DontAlignCols, "",
                         ", ", "", "", "(", ")");
  FCL_REAL resolution (10.);
  std::vector<Vec3f> pRob, pEnv;
  std::vector<Triangle> tRob, tEnv;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  loadOBJFile((path / "rob.obj").string().c_str(), pRob, tRob);
  loadOBJFile((path / "env.obj").string().c_str(), pEnv, tEnv);

  BVHModel <OBBRSS> robMesh, envMesh;
  // Build meshes with robot and environment
  makeMesh (pRob, tRob, robMesh);
  makeMesh (pEnv, tEnv, envMesh);
  // Build octomap with environment
  envMesh.computeLocalAABB ();
  // Load octree built from envMesh by makeOctree
  OcTree envOctree
    (fcl::loadOctreeFile ((path / "env.octree").string().c_str(), resolution));

  std::cout << "Finished loading octree." << std::endl;

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-2000, -2000, 0, 2000, 2000, 2000};
  std::size_t N = 10000;

  generateRandomTransforms(extents, transforms, 2*N);

  CollisionRequest request (true, 1, true);
  for (std::size_t i=0; i<N; ++i) {
    CollisionResult resultMesh;
    CollisionResult resultOctree;
    Transform3f tf1 (transforms [2*i]);
    Transform3f tf2 (transforms [2*i+1]);;
    // Test collision between meshes with random transform for robot.
    fcl::collide (&robMesh, tf1, &envMesh, tf2, request, resultMesh);
    // Test collision between mesh and octree for the same transform.
    fcl::collide (&robMesh, tf1, &envOctree, tf2, request, resultOctree);
    bool resMesh (resultMesh.isCollision ());
    bool resOctree (resultOctree.isCollision ());
    if (resMesh && !resOctree) {
      // std::cerr << "q1=" << tf1.getTranslation ().format (tuple)
      //           << "+" << tf1.getQuatRotation ().coeffs ().format (tuple)
      //           << std::endl;
      // std::cerr << "q2=" << tf2.getTranslation ().format (tuple)
      //           << "+" << tf2.getQuatRotation ().coeffs ().format (tuple)
      //           << std::endl;
      fcl::DistanceRequest dreq;
      fcl::DistanceResult  dres;
      fcl::distance (&robMesh, tf1, &envOctree, tf2, dreq, dres);
      std::cout << "distance mesh octree: " << dres.min_distance << std::endl;
      BOOST_CHECK (dres.min_distance < sqrt (2.) * resolution);
    }
    if (!resMesh && resOctree) {
      fcl::DistanceRequest dreq;
      fcl::DistanceResult  dres;
      fcl::distance (&robMesh, tf1, &envMesh, tf2, dreq, dres);
      std::cout << "distance mesh mesh: " << dres.min_distance
                << std::endl;
      BOOST_CHECK (dres.min_distance < sqrt (2.) * resolution);
    }
  }
}
