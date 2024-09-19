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

#include "coal/distance.h"
#include "coal/collision_utility.h"
#include "coal/distance_func_matrix.h"
#include "coal/narrowphase/narrowphase.h"

#include <iostream>

namespace coal {

DistanceFunctionMatrix& getDistanceFunctionLookTable() {
  static DistanceFunctionMatrix table;
  return table;
}

CoalScalar distance(const CollisionObject* o1, const CollisionObject* o2,
                    const DistanceRequest& request, DistanceResult& result) {
  return distance(o1->collisionGeometryPtr(), o1->getTransform(),
                  o2->collisionGeometryPtr(), o2->getTransform(), request,
                  result);
}

CoalScalar distance(const CollisionGeometry* o1, const Transform3s& tf1,
                    const CollisionGeometry* o2, const Transform3s& tf2,
                    const DistanceRequest& request, DistanceResult& result) {
  GJKSolver solver(request);

  const DistanceFunctionMatrix& looktable = getDistanceFunctionLookTable();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  CoalScalar res = (std::numeric_limits<CoalScalar>::max)();

  if (object_type1 == OT_GEOM &&
      (object_type2 == OT_BVH || object_type2 == OT_HFIELD)) {
    if (!looktable.distance_matrix[node_type2][node_type1]) {
      COAL_THROW_PRETTY("Distance function between node type "
                            << std::string(get_node_type_name(node_type1))
                            << " and node type "
                            << std::string(get_node_type_name(node_type2))
                            << " is not yet supported.",
                        std::invalid_argument);
    } else {
      res = looktable.distance_matrix[node_type2][node_type1](
          o2, tf2, o1, tf1, &solver, request, result);
      std::swap(result.o1, result.o2);
      result.nearest_points[0].swap(result.nearest_points[1]);
      result.normal *= -1;
    }
  } else {
    if (!looktable.distance_matrix[node_type1][node_type2]) {
      COAL_THROW_PRETTY("Distance function between node type "
                            << std::string(get_node_type_name(node_type1))
                            << " and node type "
                            << std::string(get_node_type_name(node_type2))
                            << " is not yet supported.",
                        std::invalid_argument);
    } else {
      res = looktable.distance_matrix[node_type1][node_type2](
          o1, tf1, o2, tf2, &solver, request, result);
    }
  }
  // Cache narrow phase solver result. If the option in the request is selected,
  // also store the solver result in the request for the next call.
  result.cached_gjk_guess = solver.cached_guess;
  result.cached_support_func_guess = solver.support_func_cached_guess;
  request.updateGuess(result);
  return res;
}

ComputeDistance::ComputeDistance(const CollisionGeometry* o1,
                                 const CollisionGeometry* o2)
    : o1(o1), o2(o2) {
  const DistanceFunctionMatrix& looktable = getDistanceFunctionLookTable();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  swap_geoms = object_type1 == OT_GEOM &&
               (object_type2 == OT_BVH || object_type2 == OT_HFIELD);

  if ((swap_geoms && !looktable.distance_matrix[node_type2][node_type1]) ||
      (!swap_geoms && !looktable.distance_matrix[node_type1][node_type2])) {
    COAL_THROW_PRETTY("Distance function between node type "
                          << std::string(get_node_type_name(node_type1))
                          << " and node type "
                          << std::string(get_node_type_name(node_type2))
                          << " is not yet supported.",
                      std::invalid_argument);
  }
  if (swap_geoms)
    func = looktable.distance_matrix[node_type2][node_type1];
  else
    func = looktable.distance_matrix[node_type1][node_type2];
}

CoalScalar ComputeDistance::run(const Transform3s& tf1, const Transform3s& tf2,
                                const DistanceRequest& request,
                                DistanceResult& result) const {
  CoalScalar res;

  if (swap_geoms) {
    res = func(o2, tf2, o1, tf1, &solver, request, result);
    std::swap(result.o1, result.o2);
    result.nearest_points[0].swap(result.nearest_points[1]);
    result.normal *= -1;
  } else {
    res = func(o1, tf1, o2, tf2, &solver, request, result);
  }
  // Cache narrow phase solver result. If the option in the request is selected,
  // also store the solver result in the request for the next call.
  result.cached_gjk_guess = solver.cached_guess;
  result.cached_support_func_guess = solver.support_func_cached_guess;
  request.updateGuess(result);
  return res;
}

CoalScalar ComputeDistance::operator()(const Transform3s& tf1,
                                       const Transform3s& tf2,
                                       const DistanceRequest& request,
                                       DistanceResult& result) const {
  solver.set(request);

  CoalScalar res;
  if (request.enable_timings) {
    Timer timer;
    res = run(tf1, tf2, request, result);
    result.timings = timer.elapsed();
  } else
    res = run(tf1, tf2, request, result);
  return res;
}

}  // namespace coal
