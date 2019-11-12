/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CNRS-LAAS
 *  All rights reserved.
 */

/** \author Lucile Remigy, Joseph Mirabel */

#include <hpp/fcl/collision_func_matrix.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <../src/collision_node.h>
#include <hpp/fcl/internal/traversal_node_setup.h>
#include <../src/distance_func_matrix.h>

namespace hpp
{
namespace fcl
{

#ifdef HPP_FCL_HAVE_OCTOMAP

// TraversalTraitsCollision for collision_func_matrix.cpp

template <typename TypeA, typename TypeB>
struct TraversalTraitsCollision
{
};

template <typename T_SH>
struct TraversalTraitsCollision <T_SH, OcTree>
{
  typedef ShapeOcTreeCollisionTraversalNode<T_SH> CollisionTraversal_t;
};

template <typename T_SH>
struct TraversalTraitsCollision <OcTree, T_SH>
{
  typedef OcTreeShapeCollisionTraversalNode<T_SH> CollisionTraversal_t;
};

template <>
struct TraversalTraitsCollision <OcTree, OcTree>
{
  typedef OcTreeCollisionTraversalNode CollisionTraversal_t;
};

template <typename T_BVH>
struct TraversalTraitsCollision <OcTree, BVHModel<T_BVH> >
{
  typedef OcTreeMeshCollisionTraversalNode<T_BVH> CollisionTraversal_t;
};

template <typename T_BVH>
struct TraversalTraitsCollision <BVHModel<T_BVH>, OcTree>
{
  typedef MeshOcTreeCollisionTraversalNode<T_BVH> CollisionTraversal_t;
};

// TraversalTraitsDistance for distance_func_matrix.cpp

template <typename TypeA, typename TypeB>
struct TraversalTraitsDistance
{
};

template <typename T_SH>
struct TraversalTraitsDistance <T_SH, OcTree>
{
  typedef ShapeOcTreeDistanceTraversalNode<T_SH> CollisionTraversal_t;
};

template <typename T_SH>
struct TraversalTraitsDistance <OcTree, T_SH>
{
  typedef OcTreeShapeDistanceTraversalNode<T_SH> CollisionTraversal_t;
};

template <>
struct TraversalTraitsDistance <OcTree, OcTree>
{
  typedef OcTreeDistanceTraversalNode CollisionTraversal_t;
};

template <typename T_BVH>
struct TraversalTraitsDistance <OcTree, BVHModel<T_BVH> >
{
  typedef OcTreeMeshDistanceTraversalNode<T_BVH> CollisionTraversal_t;
};

template <typename T_BVH>
struct TraversalTraitsDistance <BVHModel<T_BVH>, OcTree>
{
  typedef MeshOcTreeDistanceTraversalNode<T_BVH> CollisionTraversal_t;
};

#endif

}

} //hpp