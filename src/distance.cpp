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

#include <hpp/fcl/distance.h>
#include <hpp/fcl/distance_func_matrix.h>
#include <hpp/fcl/narrowphase/narrowphase.h>

#include <iostream>

namespace hpp
{
namespace fcl
{

DistanceFunctionMatrix& getDistanceFunctionLookTable()
{
  static DistanceFunctionMatrix table;
  return table;
}

FCL_REAL distance(const CollisionObject* o1, const CollisionObject* o2, const DistanceRequest& request, DistanceResult& result)
{
  return distance(
      o1->collisionGeometry().get(), o1->getTransform(),
      o2->collisionGeometry().get(), o2->getTransform(),
      request, result);
}

FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1,
                  const CollisionGeometry* o2, const Transform3f& tf2,
                  const DistanceRequest& request, DistanceResult& result)
{
  GJKSolver solver;
  solver.enable_cached_guess = request.enable_cached_gjk_guess;
  if (solver.enable_cached_guess) {
    solver.cached_guess = request.cached_gjk_guess;
    solver.support_func_cached_guess = request.cached_support_func_guess;
  }

  const DistanceFunctionMatrix& looktable = getDistanceFunctionLookTable();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  FCL_REAL res = std::numeric_limits<FCL_REAL>::max();

  if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
  {
    if(!looktable.distance_matrix[node_type2][node_type1])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported" << std::endl;
    }
    else
    {
      res = looktable.distance_matrix[node_type2][node_type1](o2, tf2, o1, tf1, &solver, request, result);
      // If closest points are requested, switch object 1 and 2
      if (request.enable_nearest_points) {
	const CollisionGeometry *tmpo = result.o1;
	result.o1 = result.o2;
	result.o2 = tmpo;
	Vec3f tmpn (result.nearest_points [0]);
	result.nearest_points [0] = result.nearest_points [1];
	result.nearest_points [1] = tmpn;
      }
    }
  }
  else
  {
    if(!looktable.distance_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported" << std::endl;
    }
    else
    {
      res = looktable.distance_matrix[node_type1][node_type2](o1, tf1, o2, tf2, &solver, request, result);    
    }
  }
  if (solver.enable_cached_guess) {
    result.cached_gjk_guess = solver.cached_guess;
    result.cached_support_func_guess = solver.support_func_cached_guess;
  }

  return res;
}

ComputeDistance::ComputeDistance(const CollisionGeometry* o1,
    const CollisionGeometry* o2)
  : o1(o1), o2(o2)
{
  const DistanceFunctionMatrix& looktable = getDistanceFunctionLookTable();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  swap_geoms = object_type1 == OT_GEOM && object_type2 == OT_BVH;

  if(   ( swap_geoms && !looktable.distance_matrix[node_type2][node_type1])
     || (!swap_geoms && !looktable.distance_matrix[node_type1][node_type2]))
  {
    std::ostringstream oss;
    oss << "Warning: distance function between node type " << node_type1 <<
      " and node type " << node_type2 << " is not supported";
    throw std::invalid_argument(oss.str());
  }
  if (swap_geoms)
    func = looktable.distance_matrix[node_type2][node_type1];
  else
    func = looktable.distance_matrix[node_type1][node_type2];
}

} // namespace fcl
} // namespace hpp
