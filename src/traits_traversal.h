/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CNRS-LAAS
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

/** \author Florent Lamiraux */


#include <hpp/fcl/collision_func_matrix.h>

#include "traversal/traversal_node_setup.h"
#include <../src/collision_node.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include "distance_func_matrix.h"

namespace hpp
{
namespace fcl
{
template <typename TypeA, typename TypeB>
struct TraversalTraits
{
};

template <typename T_SH>
struct TraversalTraits <T_SH, OcTree>
{
  typedef ShapeOcTreeCollisionTraversalNode<T_SH, GJKSolver> CollisionTraversal_t;
};

template <typename T_SH>
struct TraversalTraits <OcTree, T_SH>
{
  typedef OcTreeShapeCollisionTraversalNode<T_SH, GJKSolver> CollisionTraversal_t;
};

template <>
struct TraversalTraits <OcTree, OcTree>
{
  typedef OcTreeCollisionTraversalNode<GJKSolver> CollisionTraversal_t;
};

template <typename T_BVH>
struct TraversalTraits <OcTree, BVHModel<T_BVH>>
{
  typedef OcTreeMeshCollisionTraversalNode<T_BVH, GJKSolver> CollisionTraversal_t;
};

template <typename T_BVH>
struct TraversalTraits <BVHModel<T_BVH>, OcTree>
{
  typedef MeshOcTreeCollisionTraversalNode<T_BVH, GJKSolver> CollisionTraversal_t;
};












}

} //hpp