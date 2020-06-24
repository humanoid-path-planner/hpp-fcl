/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS
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
 *   * Neither the name of CNRS-LAAS nor the names of its
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

#define BOOST_TEST_MODULE FCL_DISTANCE_LOWER_BOUND
#include <boost/test/included/unit_test.hpp>
# include <boost/filesystem.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
# include "utility.h"
# include "fcl_resources/config.h"

using hpp::fcl::Transform3f;
using hpp::fcl::Vec3f;
using hpp::fcl::Triangle;
using hpp::fcl::OBBRSS;
using hpp::fcl::BVHModel;
using hpp::fcl::CollisionResult;
using hpp::fcl::CollisionRequest;
using hpp::fcl::DistanceRequest;
using hpp::fcl::DistanceResult;
using hpp::fcl::CollisionObject;
using hpp::fcl::CollisionGeometryPtr_t;
using hpp::fcl::FCL_REAL;

bool testDistanceLowerBound (const Transform3f& tf,
			     const CollisionGeometryPtr_t& m1,
			     const CollisionGeometryPtr_t& m2,
			     FCL_REAL& distance)
{
  Transform3f pose1(tf), pose2;

  CollisionRequest request (hpp::fcl::NO_REQUEST, 1);
  request.enable_distance_lower_bound = true;

  CollisionResult result;
  CollisionObject co1 (m1, pose1);
  CollisionObject co2 (m2, pose2);

  hpp::fcl::collide(&co1, &co2, request, result);
  distance = result.distance_lower_bound;

  return result.isCollision ();
}

bool testCollide (const Transform3f& tf, const CollisionGeometryPtr_t& m1,
		  const CollisionGeometryPtr_t& m2)
{
  Transform3f pose1(tf), pose2;

  CollisionRequest request (hpp::fcl::NO_REQUEST, 1);
  request.enable_distance_lower_bound = false;

  CollisionResult result;
  CollisionObject co1 (m1, pose1);
  CollisionObject co2 (m2, pose2);

  hpp::fcl::collide(&co1, &co2, request, result);
  return result.isCollision ();
}

bool testDistance (const Transform3f& tf, const CollisionGeometryPtr_t& m1,
		   const CollisionGeometryPtr_t& m2, FCL_REAL& distance)
{
  Transform3f pose1(tf), pose2;

  DistanceRequest request;
  DistanceResult result;
  CollisionObject co1 (m1, pose1);
  CollisionObject co2 (m2, pose2);

  hpp::fcl::distance (&co1, &co2, request, result);
  distance = result.min_distance;

  if(result.min_distance <= 0) {
    return true;
  }
  else {
    return false;
  }
}

BOOST_AUTO_TEST_CASE(mesh_mesh)
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  boost::shared_ptr < BVHModel <OBBRSS> > m1 (new BVHModel <OBBRSS>);
  boost::shared_ptr < BVHModel <OBBRSS> > m2 (new BVHModel <OBBRSS>);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  m2->beginModel();
  m2->addSubModel(p2, t2);
  m2->endModel();

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  std::size_t n = 100;

  generateRandomTransforms(extents, transforms, n);

  // collision
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    FCL_REAL distanceLowerBound, distance;
    bool col1, col2, col3;
    col1 = testDistanceLowerBound (transforms[i], m1, m2, distanceLowerBound);
    col2 = testDistance (transforms[i], m1, m2, distance);
    col3 = testCollide (transforms[i], m1, m2);
    std::cout << "col1 = " << col1 << ", col2 = " << col2 
	      << ", col3 = " << col3 << std::endl;
    std::cout << "distance lower bound = " << distanceLowerBound << std::endl;
    std::cout << "distance             = " << distance << std::endl;
    BOOST_CHECK (col1 == col3);
    if (!col1) {
      BOOST_CHECK_MESSAGE (distanceLowerBound <= distance,
			   "distance = " << distance << ", lower bound = "
			   << distanceLowerBound);
    }
  }
}

BOOST_AUTO_TEST_CASE(box_mesh)
{
  std::vector<Vec3f> p1;
  std::vector<Triangle> t1;
  boost::filesystem::path path(TEST_RESOURCES_DIR);

  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);

  boost::shared_ptr < BVHModel <OBBRSS> > m1 (new BVHModel <OBBRSS>);
  boost::shared_ptr < hpp::fcl::Box > m2 (new hpp::fcl::Box (500, 200, 150));

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  std::vector<Transform3f> transforms;
  FCL_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  std::size_t n = 100;

  generateRandomTransforms(extents, transforms, n);

  // collision
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    FCL_REAL distanceLowerBound, distance;
    bool col1, col2;
    col1 = testDistanceLowerBound (transforms[i], m1, m2, distanceLowerBound);
    col2 = testDistance (transforms[i], m1, m2, distance);
    BOOST_CHECK (col1 == col2);
    if (!col1) {
      BOOST_CHECK_MESSAGE (distanceLowerBound <= distance,
			   "distance = " << distance << ", lower bound = "
			   << distanceLowerBound);
    }
  }
}
