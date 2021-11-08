/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, INRIA
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

/** \author Justin Carpentier */


#define BOOST_TEST_MODULE FCL_HEIGHT_FIELDS
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "fcl_resources/config.h"

#include <hpp/fcl/collision.h>
#include <hpp/fcl/hfield.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <hpp/fcl/mesh_loader/loader.h>

#include <hpp/fcl/collision.h>

#include "utility.h"
#include <iostream>

using namespace hpp::fcl;

template<typename BV>
void test_constant_hfields(const Eigen::DenseIndex nx, const Eigen::DenseIndex ny,
                           const FCL_REAL min_altitude, const FCL_REAL max_altitude)
{
  const FCL_REAL x_dim = 1., y_dim = 2.;
  const MatrixXf heights = MatrixXf::Constant(ny, nx, max_altitude);
  
  HeightField<BV> hfield(x_dim,y_dim,heights,min_altitude);

  BOOST_CHECK(hfield.getXDim() == x_dim);
  BOOST_CHECK(hfield.getYDim() == y_dim);

  const VecXf & x_grid = hfield.getXGrid();
  BOOST_CHECK(x_grid[0] == -x_dim/2.);
  BOOST_CHECK(x_grid[nx-1] == x_dim/2.);

  const VecXf & y_grid = hfield.getYGrid();
  BOOST_CHECK(y_grid[0] == y_dim/2.);
  BOOST_CHECK(y_grid[ny-1] == -y_dim/2.);
  
  // Test local AABB
  hfield.computeLocalAABB();
  
  for(Eigen::DenseIndex i = 0; i < nx; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < ny; ++j)
    {
      Vec3f point(x_grid[i],y_grid[j],heights(j,i));
      BOOST_CHECK(hfield.aabb_local.contain(point));
    }
  }
  
//  const OBBRSS & bv = reinterpret_cast<const OBBRSS &>(hfield.getBV(0).bv);
//  std::cout << "bv_center: " << bv.center().transpose() << std::endl;
//  std::cout << "bv_extent: " << bv.obb.extent.transpose() << std::endl;
//  std::cout << "bv_To: " << bv.obb.To.transpose() << std::endl;
  
  // Test clone
  {
    HeightField<BV> * hfield_clone = hfield.clone();
    hfield_clone->computeLocalAABB();
    BOOST_CHECK(*hfield_clone == hfield);
    
    delete hfield_clone;
  }
  
  // Build equivalent object
  const Box equivalent_box(x_dim,y_dim,max_altitude - min_altitude);
  const Transform3f box_placement(Matrix3f::Identity(),
                                  Vec3f(0.,0.,(max_altitude + min_altitude)/2.));
  
  // Test collision
  const Sphere sphere(1.);
  static const Transform3f IdTransform;
  
  const Box box(Vec3f::Ones());
  
  Transform3f M_sphere, M_box;
  
  // No collision case
  {
    const FCL_REAL eps_no_collision = +0.1*(max_altitude - min_altitude);
    M_sphere.setTranslation(Vec3f(0.,0.,max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(Vec3f(0.,0.,max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;
    
    CollisionResult result;
    collide(&hfield,IdTransform,
            &sphere,M_sphere,
            request,result);
    
    BOOST_CHECK(!result.isCollision());
    
    CollisionResult result_check_sphere;
    collide(&equivalent_box,IdTransform*box_placement,
            &sphere,M_sphere,
            request,result_check_sphere);
    
    BOOST_CHECK(!result_check_sphere.isCollision());
    
    CollisionResult result_check_box;
    collide(&equivalent_box,IdTransform*box_placement,
            &box,M_box,
            request,result_check_box);
    
    BOOST_CHECK(!result_check_box.isCollision());
  }
  
  // Collision case
  {
    const FCL_REAL eps_collision = -0.1*(max_altitude - min_altitude);
    M_sphere.setTranslation(Vec3f(0.,0.,max_altitude + sphere.radius + eps_collision));
    M_box.setTranslation(Vec3f(0.,0.,max_altitude + box.halfSide[2] + eps_collision));
    CollisionRequest request; //(CONTACT | DISTANCE_LOWER_BOUND, (size_t)((nx-1)*(ny-1)));
    
    CollisionResult result;
    collide(&hfield,IdTransform,
            &sphere,M_sphere,
            request,result);

    BOOST_CHECK(result.isCollision());
    
    CollisionResult result_check;
    collide(&equivalent_box,IdTransform*box_placement,
            &sphere,M_sphere,
            request,result_check);
    
    BOOST_CHECK(result_check.isCollision());
    
    CollisionResult result_check_box;
    collide(&equivalent_box,IdTransform*box_placement,
            &box,M_box,
            request,result_check_box);
    
    BOOST_CHECK(result_check_box.isCollision());
  }
  
  // Update height
  hfield.updateHeights(MatrixXf::Constant(ny, nx, max_altitude/2.)); // We change nothing
  
  // No collision case
  {
    const FCL_REAL eps_no_collision = +0.1*(max_altitude - min_altitude);
    M_sphere.setTranslation(Vec3f(0.,0.,max_altitude + sphere.radius + eps_no_collision));
    M_box.setTranslation(Vec3f(0.,0.,max_altitude + box.halfSide[2] + eps_no_collision));
    CollisionRequest request;
    
    CollisionResult result;
    collide(&hfield,IdTransform,
            &sphere,M_sphere,
            request,result);
    
    BOOST_CHECK(!result.isCollision());
    
    CollisionResult result_check_sphere;
    collide(&equivalent_box,IdTransform*box_placement,
            &sphere,M_sphere,
            request,result_check_sphere);
    
    BOOST_CHECK(!result_check_sphere.isCollision());
    
    CollisionResult result_check_box;
    collide(&equivalent_box,IdTransform*box_placement,
            &box,M_box,
            request,result_check_box);
    
    BOOST_CHECK(!result_check_box.isCollision());
  }
  
  // Collision case
  {
    const FCL_REAL eps_collision = -0.1*(max_altitude - min_altitude);
    M_sphere.setTranslation(Vec3f(0.,0.,max_altitude + sphere.radius + eps_collision));
    M_box.setTranslation(Vec3f(0.,0.,max_altitude + box.halfSide[2] + eps_collision));
    CollisionRequest request; //(CONTACT | DISTANCE_LOWER_BOUND, (size_t)((nx-1)*(ny-1)));
    
    CollisionResult result;
    collide(&hfield,IdTransform,
            &sphere,M_sphere,
            request,result);

    BOOST_CHECK(!result.isCollision());
    
    CollisionResult result_check;
    collide(&equivalent_box,IdTransform*box_placement,
            &sphere,M_sphere,
            request,result_check);
    
    BOOST_CHECK(result_check.isCollision());
    
    CollisionResult result_check_box;
    collide(&equivalent_box,IdTransform*box_placement,
            &box,M_box,
            request,result_check_box);
    
    BOOST_CHECK(result_check_box.isCollision());
  }
  
  // Restore height
  hfield.updateHeights(MatrixXf::Constant(ny, nx, max_altitude)); // We change nothing
  
  // Collision case
  {
    const FCL_REAL eps_collision = -0.1*(max_altitude - min_altitude);
    M_sphere.setTranslation(Vec3f(0.,0.,max_altitude + sphere.radius + eps_collision));
    M_box.setTranslation(Vec3f(0.,0.,max_altitude + box.halfSide[2] + eps_collision));
    CollisionRequest request; //(CONTACT | DISTANCE_LOWER_BOUND, (size_t)((nx-1)*(ny-1)));
    
    CollisionResult result;
    collide(&hfield,IdTransform,
            &sphere,M_sphere,
            request,result);

    BOOST_CHECK(result.isCollision());
    
    CollisionResult result_check;
    collide(&equivalent_box,IdTransform*box_placement,
            &sphere,M_sphere,
            request,result_check);
    
    BOOST_CHECK(result_check.isCollision());
    
    CollisionResult result_check_box;
    collide(&equivalent_box,IdTransform*box_placement,
            &box,M_box,
            request,result_check_box);
    
    BOOST_CHECK(result_check_box.isCollision());
  }
  
}

BOOST_AUTO_TEST_CASE(building_constant_hfields)
{
  const FCL_REAL max_altitude = 1., min_altitude = 0.;
  
  test_constant_hfields<OBBRSS>(2,2,min_altitude,max_altitude); // Simple case
  test_constant_hfields<OBBRSS>(20,2,min_altitude,max_altitude);
  test_constant_hfields<OBBRSS>(100,100,min_altitude,max_altitude);
//  test_constant_hfields<OBBRSS>(1000,1000,min_altitude,max_altitude);
  
  test_constant_hfields<AABB>(2,2,min_altitude,max_altitude); // Simple case
}

