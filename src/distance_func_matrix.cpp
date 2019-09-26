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

#include <hpp/fcl/distance_func_matrix.h>

#include <../src/collision_node.h>
#include "traversal/traversal_node_setup.h"
#include <hpp/fcl/narrowphase/narrowphase.h>

namespace hpp
{
namespace fcl
{

#ifdef HPP_FCL_HAVE_OCTOMAP
template<typename T_SH, typename GJKSolver>
FCL_REAL ShapeOcTreeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                             const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeOcTreeDistanceTraversalNode<T_SH, GJKSolver> node;
  const T_SH* obj1 = static_cast<const T_SH*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<GJKSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);
  
  return result.min_distance;
}

template<typename T_SH, typename GJKSolver>
FCL_REAL OcTreeShapeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                             const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeShapeDistanceTraversalNode<T_SH, GJKSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);
  OcTreeSolver<GJKSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);
  
  return result.min_distance;
}

template<typename GJKSolver>
FCL_REAL OcTreeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                        const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeDistanceTraversalNode<GJKSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<GJKSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template<typename T_BVH, typename GJKSolver>
FCL_REAL BVHOcTreeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  MeshOcTreeDistanceTraversalNode<T_BVH, GJKSolver> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>*>(o1);
  const OcTree* obj2 = static_cast<const OcTree*>(o2);
  OcTreeSolver<GJKSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template<typename T_BVH, typename GJKSolver>
FCL_REAL OcTreeBVHDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                       const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OcTreeMeshDistanceTraversalNode<T_BVH, GJKSolver> node;
  const OcTree* obj1 = static_cast<const OcTree*>(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>*>(o2);
  OcTreeSolver<GJKSolver> otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, request, result);
  distance(&node);

  return result.min_distance;
}

#endif

template<typename T_SH1, typename T_SH2, typename GJKSolver>
FCL_REAL ShapeShapeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                        const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  ShapeDistanceTraversalNode<T_SH1, T_SH2, GJKSolver> node;
  const T_SH1* obj1 = static_cast<const T_SH1*>(o1);
  const T_SH2* obj2 = static_cast<const T_SH2*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  distance(&node);

  return result.min_distance;
}

template<typename T_BVH, typename T_SH, typename GJKSolver>
struct BVHShapeDistancer
{
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshShapeDistanceTraversalNode<T_BVH, T_SH, GJKSolver> node;
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3f tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, request, result);
    fcl::distance(&node);
    
    delete obj1_tmp;
    return result.min_distance;
  }
};

namespace details
{

template<typename OrientedMeshShapeDistanceTraversalNode, typename T_BVH, typename T_SH, typename GJKSolver>
FCL_REAL orientedBVHShapeDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                                  const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshShapeDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const T_SH* obj2 = static_cast<const T_SH*>(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, nsolver, request, result);
  fcl::distance(&node);

  return result.min_distance;  
}

}

template<typename T_SH, typename GJKSolver>
struct BVHShapeDistancer<RSS, T_SH, GJKSolver>
{
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeRSS<T_SH, GJKSolver>, RSS, T_SH, GJKSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template<typename T_SH, typename GJKSolver>
struct BVHShapeDistancer<kIOS, T_SH, GJKSolver>
{
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                       const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodekIOS<T_SH, GJKSolver>, kIOS, T_SH, GJKSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};

template<typename T_SH, typename GJKSolver>
struct BVHShapeDistancer<OBBRSS, T_SH, GJKSolver>
{
  static FCL_REAL distance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver* nsolver,
                           const DistanceRequest& request, DistanceResult& result)
  {
    return details::orientedBVHShapeDistance<MeshShapeDistanceTraversalNodeOBBRSS<T_SH, GJKSolver>, OBBRSS, T_SH, GJKSolver>(o1, tf1, o2, tf2, nsolver, request, result);
  }
};


template<typename T_BVH>
FCL_REAL BVHDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                     const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  MeshDistanceTraversalNode<T_BVH> node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3f tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3f tf2_tmp = tf2;

  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
  distance(&node);
  delete obj1_tmp;
  delete obj2_tmp;
  
  return result.min_distance;
}

namespace details
{
template<typename OrientedMeshDistanceTraversalNode, typename T_BVH>
FCL_REAL orientedMeshDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                              const DistanceRequest& request, DistanceResult& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshDistanceTraversalNode node;
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  distance(&node);

  return result.min_distance;
}

}

template<>
FCL_REAL BVHDistance<RSS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                          const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeRSS, RSS>(o1, tf1, o2, tf2, request, result);
}

template<>
FCL_REAL BVHDistance<kIOS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                           const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodekIOS, kIOS>(o1, tf1, o2, tf2, request, result);
}


template<>
FCL_REAL BVHDistance<OBBRSS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                             const DistanceRequest& request, DistanceResult& result)
{
  return details::orientedMeshDistance<MeshDistanceTraversalNodeOBBRSS, OBBRSS>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH, typename GJKSolver>
FCL_REAL BVHDistance(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                     const GJKSolver* /*nsolver*/,
                     const DistanceRequest& request, DistanceResult& result)
{
  return BVHDistance<T_BVH>(o1, tf1, o2, tf2, request, result);
}

template<typename GJKSolver>
DistanceFunctionMatrix<GJKSolver>::DistanceFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      distance_matrix[i][j] = NULL;
  }

  distance_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeDistance<Box, Box, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeDistance<Box, Sphere, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeDistance<Box, Capsule, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeDistance<Box, Cone, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeDistance<Box, Cylinder, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeDistance<Box, Convex, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeDistance<Box, Plane, GJKSolver>;
  distance_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeDistance<Box, Halfspace, GJKSolver>;

  distance_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeDistance<Sphere, Box, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeDistance<Sphere, Sphere, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeDistance<Sphere, Capsule, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeDistance<Sphere, Cone, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeDistance<Sphere, Cylinder, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeDistance<Sphere, Convex, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeDistance<Sphere, Plane, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeDistance<Sphere, Halfspace, GJKSolver>;

  distance_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeDistance<Capsule, Box, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeDistance<Capsule, Sphere, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeDistance<Capsule, Capsule, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeDistance<Capsule, Cone, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeDistance<Capsule, Cylinder, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeDistance<Capsule, Convex, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeDistance<Capsule, Plane, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeDistance<Capsule, Halfspace, GJKSolver>;

  distance_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeDistance<Cone, Box, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeDistance<Cone, Sphere, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeDistance<Cone, Capsule, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeDistance<Cone, Cone, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeDistance<Cone, Cylinder, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeDistance<Cone, Convex, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeDistance<Cone, Plane, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeDistance<Cone, Halfspace, GJKSolver>;

  distance_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeDistance<Cylinder, Box, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeDistance<Cylinder, Sphere, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeDistance<Cylinder, Capsule, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeDistance<Cylinder, Cone, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeDistance<Cylinder, Cylinder, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeDistance<Cylinder, Convex, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeDistance<Cylinder, Plane, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeDistance<Cylinder, Halfspace, GJKSolver>;

  distance_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeDistance<Convex, Box, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeDistance<Convex, Sphere, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeDistance<Convex, Capsule, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeDistance<Convex, Cone, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeDistance<Convex, Cylinder, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeDistance<Convex, Convex, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeDistance<Convex, Plane, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeDistance<Convex, Halfspace, GJKSolver>;

  distance_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeDistance<Plane, Box, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeDistance<Plane, Sphere, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeDistance<Plane, Capsule, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeDistance<Plane, Cone, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeDistance<Plane, Cylinder, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeDistance<Plane, Convex, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeDistance<Plane, Plane, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeDistance<Plane, Halfspace, GJKSolver>;

  distance_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeDistance<Halfspace, Box, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeDistance<Halfspace, Sphere, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeDistance<Halfspace, Capsule, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeDistance<Halfspace, Cone, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeDistance<Halfspace, Cylinder, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeDistance<Halfspace, Convex, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeDistance<Halfspace, Plane, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeDistance<Halfspace, Halfspace, GJKSolver>;

  /* AABB distance not implemented */
  /*
  distance_matrix[BV_AABB][GEOM_BOX] = &BVHShapeDistancer<AABB, Box, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeDistancer<AABB, Sphere, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeDistancer<AABB, Capsule, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONE] = &BVHShapeDistancer<AABB, Cone, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeDistancer<AABB, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeDistancer<AABB, Convex, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeDistancer<AABB, Plane, GJKSolver>::distance;
  distance_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeDistancer<AABB, Halfspace, GJKSolver>::distance;
  */

  distance_matrix[BV_OBB][GEOM_BOX] = &BVHShapeDistancer<OBB, Box, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeDistancer<OBB, Sphere, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeDistancer<OBB, Capsule, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONE] = &BVHShapeDistancer<OBB, Cone, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeDistancer<OBB, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeDistancer<OBB, Convex, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeDistancer<OBB, Plane, GJKSolver>::distance;
  distance_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeDistancer<OBB, Halfspace, GJKSolver>::distance;

  distance_matrix[BV_RSS][GEOM_BOX] = &BVHShapeDistancer<RSS, Box, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeDistancer<RSS, Sphere, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeDistancer<RSS, Capsule, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONE] = &BVHShapeDistancer<RSS, Cone, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeDistancer<RSS, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeDistancer<RSS, Convex, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeDistancer<RSS, Plane, GJKSolver>::distance;
  distance_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeDistancer<RSS, Halfspace, GJKSolver>::distance;

  /* KDOP distance not implemented */
  /*
  distance_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeDistancer<KDOP<16>, Box, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<16>, Sphere, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<16>, Capsule, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeDistancer<KDOP<16>, Cone, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<16>, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<16>, Convex, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeDistancer<KDOP<16>, Plane, GJKSolver>::distance;
  distance_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<16>, Halfspace, GJKSolver>::distance;

  distance_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeDistancer<KDOP<18>, Box, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<18>, Sphere, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<18>, Capsule, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeDistancer<KDOP<18>, Cone, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<18>, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<18>, Convex, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeDistancer<KDOP<18>, Plane, GJKSolver>::distance;
  distance_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<18>, Halfspace, GJKSolver>::distance;

  distance_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeDistancer<KDOP<24>, Box, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeDistancer<KDOP<24>, Sphere, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeDistancer<KDOP<24>, Capsule, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeDistancer<KDOP<24>, Cone, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeDistancer<KDOP<24>, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeDistancer<KDOP<24>, Convex, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeDistancer<KDOP<24>, Plane, GJKSolver>::distance;
  distance_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeDistancer<KDOP<24>, Halfspace, GJKSolver>::distance;
  */

  distance_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeDistancer<kIOS, Box, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeDistancer<kIOS, Sphere, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeDistancer<kIOS, Capsule, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeDistancer<kIOS, Cone, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeDistancer<kIOS, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeDistancer<kIOS, Convex, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeDistancer<kIOS, Plane, GJKSolver>::distance;
  distance_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeDistancer<kIOS, Halfspace, GJKSolver>::distance;

  distance_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeDistancer<OBBRSS, Box, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeDistancer<OBBRSS, Sphere, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeDistancer<OBBRSS, Capsule, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeDistancer<OBBRSS, Cone, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeDistancer<OBBRSS, Cylinder, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeDistancer<OBBRSS, Convex, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeDistancer<OBBRSS, Plane, GJKSolver>::distance;
  distance_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeDistancer<OBBRSS, Halfspace, GJKSolver>::distance;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB, GJKSolver>;
  distance_matrix[BV_OBB][BV_OBB] = &BVHDistance<OBB, GJKSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS, GJKSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS, GJKSolver>;
  distance_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHDistance<OBBRSS, GJKSolver>;

#ifdef HPP_FCL_HAVE_OCTOMAP
  distance_matrix[GEOM_OCTREE][GEOM_BOX] = &OcTreeShapeDistance<Box, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_SPHERE] = &OcTreeShapeDistance<Sphere, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &OcTreeShapeDistance<Capsule, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONE] = &OcTreeShapeDistance<Cone, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &OcTreeShapeDistance<Cylinder, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_CONVEX] = &OcTreeShapeDistance<Convex, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_PLANE] = &OcTreeShapeDistance<Plane, GJKSolver>;
  distance_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &OcTreeShapeDistance<Halfspace, GJKSolver>;

  distance_matrix[GEOM_BOX][GEOM_OCTREE] = &ShapeOcTreeDistance<Box, GJKSolver>;
  distance_matrix[GEOM_SPHERE][GEOM_OCTREE] = &ShapeOcTreeDistance<Sphere, GJKSolver>;
  distance_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &ShapeOcTreeDistance<Capsule, GJKSolver>;
  distance_matrix[GEOM_CONE][GEOM_OCTREE] = &ShapeOcTreeDistance<Cone, GJKSolver>;
  distance_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &ShapeOcTreeDistance<Cylinder, GJKSolver>;
  distance_matrix[GEOM_CONVEX][GEOM_OCTREE] = &ShapeOcTreeDistance<Convex, GJKSolver>;
  distance_matrix[GEOM_PLANE][GEOM_OCTREE] = &ShapeOcTreeDistance<Plane, GJKSolver>;
  distance_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &ShapeOcTreeDistance<Halfspace, GJKSolver>;

  distance_matrix[GEOM_OCTREE][GEOM_OCTREE] = &OcTreeDistance<GJKSolver>;

  distance_matrix[GEOM_OCTREE][BV_AABB] = &OcTreeBVHDistance<AABB, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBB] = &OcTreeBVHDistance<OBB, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_RSS] = &OcTreeBVHDistance<RSS, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_OBBRSS] = &OcTreeBVHDistance<OBBRSS, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_kIOS] = &OcTreeBVHDistance<kIOS, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP16] = &OcTreeBVHDistance<KDOP<16>, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP18] = &OcTreeBVHDistance<KDOP<18>, GJKSolver>;
  distance_matrix[GEOM_OCTREE][BV_KDOP24] = &OcTreeBVHDistance<KDOP<24>, GJKSolver>;

  distance_matrix[BV_AABB][GEOM_OCTREE] = &BVHOcTreeDistance<AABB, GJKSolver>;
  distance_matrix[BV_OBB][GEOM_OCTREE] = &BVHOcTreeDistance<OBB, GJKSolver>;
  distance_matrix[BV_RSS][GEOM_OCTREE] = &BVHOcTreeDistance<RSS, GJKSolver>;
  distance_matrix[BV_OBBRSS][GEOM_OCTREE] = &BVHOcTreeDistance<OBBRSS, GJKSolver>;
  distance_matrix[BV_kIOS][GEOM_OCTREE] = &BVHOcTreeDistance<kIOS, GJKSolver>;
  distance_matrix[BV_KDOP16][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<16>, GJKSolver>;
  distance_matrix[BV_KDOP18][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<18>, GJKSolver>;
  distance_matrix[BV_KDOP24][GEOM_OCTREE] = &BVHOcTreeDistance<KDOP<24>, GJKSolver>;
#endif


}
template struct DistanceFunctionMatrix<GJKSolver>;
}

} // namespace hpp
