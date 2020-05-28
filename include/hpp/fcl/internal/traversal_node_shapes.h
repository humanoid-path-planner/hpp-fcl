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

#ifndef HPP_FCL_TRAVERSAL_NODE_SHAPES_H
#define HPP_FCL_TRAVERSAL_NODE_SHAPES_H

/// @cond INTERNAL

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/shape/geometric_shapes_utility.h>
#include <hpp/fcl/internal/traversal_node_base.h>

namespace hpp
{
namespace fcl
{

/// @addtogroup Traversal_For_Collision
/// @{

/// @brief Traversal node for collision between two shapes
template<typename S1, typename S2>
class HPP_FCL_DLLAPI ShapeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  ShapeCollisionTraversalNode(const CollisionRequest& request) :
  CollisionTraversalNodeBase(request)
  {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  bool BVDisjoints(int, int) const
  {
    return false;
  }

  /// @brief BV culling test in one BVTT node
  bool BVDisjoints(int, int, FCL_REAL&) const
  {
    throw std::runtime_error ("Not implemented");
  }

  /// @brief Intersection testing between leaves (two shapes)
  void leafCollides(int, int, FCL_REAL&) const
  {
    bool is_collision = false;
    FCL_REAL distance;
    if(request.enable_contact && request.num_max_contacts > result->numContacts())
    {
      Vec3f contact_point, normal;
      if(nsolver->shapeIntersect(*model1, tf1, *model2, tf2, distance, true,
                                 &contact_point, &normal))
      {
        is_collision = true;
        result->addContact(Contact(model1, model2, Contact::NONE,
                                   Contact::NONE, contact_point,
                                   normal, distance));
      }
    }
    else
    {
      bool res = nsolver->shapeIntersect(*model1, tf1, *model2, tf2, distance,
          request.enable_distance_lower_bound, NULL, NULL);
      if (request.enable_distance_lower_bound)
        result->updateDistanceLowerBound (distance);
      if(res)
      {
        is_collision = true;
        if(request.num_max_contacts > result->numContacts())
          result->addContact(Contact(model1, model2, Contact::NONE,
                                     Contact::NONE));
      }
    }
  }

  const S1* model1;
  const S2* model2;

  const GJKSolver* nsolver;
};

/// @}

/// @addtogroup Traversal_For_Distance
/// @{

/// @brief Traversal node for distance between two shapes
template<typename S1, typename S2>
class HPP_FCL_DLLAPI ShapeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  ShapeDistanceTraversalNode() : DistanceTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    nsolver = NULL;
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVDistanceLowerBound(int, int) const
  {
    return -1; // should not be used 
  }

  /// @brief Distance testing between leaves (two shapes)
  void leafComputeDistance(int, int) const
  {
    FCL_REAL distance;
    Vec3f closest_p1, closest_p2, normal;
    nsolver->shapeDistance(*model1, tf1, *model2, tf2, distance, closest_p1,
                           closest_p2, normal);
    result->update(distance, model1, model2, DistanceResult::NONE,
                   DistanceResult::NONE, closest_p1, closest_p2, normal);
  }

  const S1* model1;
  const S2* model2;

  const GJKSolver* nsolver;
};

/// @}

}

} // namespace hpp

/// @endcond

#endif
