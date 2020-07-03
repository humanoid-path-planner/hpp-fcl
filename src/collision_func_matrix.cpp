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


#include <hpp/fcl/collision_func_matrix.h>

#include <hpp/fcl/internal/traversal_node_setup.h>
#include <../src/collision_node.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <../src/distance_func_matrix.h>
#include <../src/traits_traversal.h>

namespace hpp
{
namespace fcl
{

#ifdef HPP_FCL_HAVE_OCTOMAP

template<typename TypeA, typename TypeB>
std::size_t Collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2,
                               const GJKSolver* nsolver,
                               const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  typename TraversalTraitsCollision<TypeA, TypeB>::CollisionTraversal_t node (request);
  const TypeA* obj1 = dynamic_cast<const TypeA*>(o1);
  const TypeB* obj2 = dynamic_cast<const TypeB*>(o2);
  OcTreeSolver otsolver(nsolver);

  initialize(node, *obj1, tf1, *obj2, tf2, &otsolver, result);
  collide(&node, request, result);

  return result.numContacts();
}
#endif

template<typename T_SH1, typename T_SH2>
std::size_t ShapeShapeCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                              const GJKSolver* nsolver,
                              const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  DistanceResult distanceResult;
  DistanceRequest distanceRequest (request.enable_contact);
  FCL_REAL distance = ShapeShapeDistance <T_SH1, T_SH2>
    (o1, tf1, o2, tf2, nsolver, distanceRequest, distanceResult);

  if (distance <= 0) {
    if (result.numContacts () < request.num_max_contacts) {
      const Vec3f& p1 = distanceResult.nearest_points [0];
      const Vec3f& p2 = distanceResult.nearest_points [1];

      Contact contact (o1, o2, distanceResult.b1, distanceResult.b2,
          (p1+p2)/2,
          distanceResult.normal,
          -distance+request.security_margin);

      result.addContact (contact);
    }
    return 1;
  }
  if (distance <= request.security_margin) {
    if (result.numContacts () < request.num_max_contacts) {
      const Vec3f& p1 = distanceResult.nearest_points [0];
      const Vec3f& p2 = distanceResult.nearest_points [1];

      Contact contact (o1, o2, distanceResult.b1, distanceResult.b2,
          .5 * (p1 + p2),
          (p2-p1).normalized (),
          -distance+request.security_margin);
      result.addContact (contact);
    }
    return 1;
  }
  result.updateDistanceLowerBound (distance);
  return 0;
}

namespace details
{
  template<typename T_BVH, typename T_SH> struct bvh_shape_traits
  {
    enum { Options = RelativeTransformationIsIdentity };
  };
#define BVH_SHAPE_DEFAULT_TO_ORIENTED(bv)                                      \
  template<typename T_SH> struct bvh_shape_traits<bv, T_SH>                    \
  { enum { Options = 0 }; }
  BVH_SHAPE_DEFAULT_TO_ORIENTED(OBB);
  BVH_SHAPE_DEFAULT_TO_ORIENTED(RSS);
  BVH_SHAPE_DEFAULT_TO_ORIENTED(kIOS);
  BVH_SHAPE_DEFAULT_TO_ORIENTED(OBBRSS);
#undef BVH_SHAPE_DEFAULT_TO_ORIENTED
}

/// \tparam _Options takes two values.
///         - RelativeTransformationIsIdentity if object 1 should be moved the
///           into the frame of object 2 before computing collisions.
///         - 0 if the query should be made with non-aligned object frames.
template<typename T_BVH, typename T_SH,
  int _Options = details::bvh_shape_traits<T_BVH, T_SH>::Options>
struct HPP_FCL_LOCAL BVHShapeCollider
{
  static std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const GJKSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    if (_Options & RelativeTransformationIsIdentity)
      return aligned(o1, tf1, o2, tf2, nsolver, request, result);
    else
      return oriented(o1, tf1, o2, tf2, nsolver, request, result);
  }

  static std::size_t aligned(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                             const GJKSolver* nsolver,
                             const CollisionRequest& request, CollisionResult& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    MeshShapeCollisionTraversalNode<T_BVH, T_SH, RelativeTransformationIsIdentity> node (request);
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
    Transform3f tf1_tmp = tf1;
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1_tmp, tf1_tmp, *obj2, tf2, nsolver, result);
    fcl::collide(&node, request, result);

    delete obj1_tmp;
    return result.numContacts();
  }

  static std::size_t oriented(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
      const GJKSolver* nsolver,
      const CollisionRequest& request, CollisionResult& result)
  {
    if(request.isSatisfied(result)) return result.numContacts();

    MeshShapeCollisionTraversalNode<T_BVH, T_SH, 0> node (request);
    const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
    const T_SH* obj2 = static_cast<const T_SH*>(o2);

    initialize(node, *obj1, tf1, *obj2, tf2, nsolver, result);
    fcl::collide(&node, request, result);
    return result.numContacts();
  }

};

namespace details
{
template<typename OrientedMeshCollisionTraversalNode, typename T_BVH>
std::size_t orientedMeshCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();

  OrientedMeshCollisionTraversalNode node (request);
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, result);
  collide(&node, request, result);

  return result.numContacts();
}

}

template<typename T_BVH>
std::size_t BVHCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  if(request.isSatisfied(result)) return result.numContacts();
  
  MeshCollisionTraversalNode<T_BVH> node (request);
  const BVHModel<T_BVH>* obj1 = static_cast<const BVHModel<T_BVH>* >(o1);
  const BVHModel<T_BVH>* obj2 = static_cast<const BVHModel<T_BVH>* >(o2);
  BVHModel<T_BVH>* obj1_tmp = new BVHModel<T_BVH>(*obj1);
  Transform3f tf1_tmp = tf1;
  BVHModel<T_BVH>* obj2_tmp = new BVHModel<T_BVH>(*obj2);
  Transform3f tf2_tmp = tf2;
  
  initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, result);
  fcl::collide(&node, request, result);

  delete obj1_tmp;
  delete obj2_tmp;

  return result.numContacts();
}

template<>
std::size_t BVHCollide<OBB>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBB, OBB>(o1, tf1, o2, tf2, request, result);
}

template<>
std::size_t BVHCollide<OBBRSS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodeOBBRSS, OBBRSS>(o1, tf1, o2, tf2, request, result);
}


template<>
std::size_t BVHCollide<kIOS>(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, const CollisionRequest& request, CollisionResult& result)
{
  return details::orientedMeshCollide<MeshCollisionTraversalNodekIOS, kIOS>(o1, tf1, o2, tf2, request, result);
}


template<typename T_BVH>
std::size_t BVHCollide(const CollisionGeometry* o1, const Transform3f& tf1, const CollisionGeometry* o2, const Transform3f& tf2, 
                       const GJKSolver* /*nsolver*/,
                       const CollisionRequest& request, CollisionResult& result)
{
  return BVHCollide<T_BVH>(o1, tf1, o2, tf2, request, result);
}


CollisionFunctionMatrix::CollisionFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      collision_matrix[i][j] = NULL;
  }

  collision_matrix[GEOM_BOX][GEOM_BOX] = &ShapeShapeCollide<Box, Box>;
  collision_matrix[GEOM_BOX][GEOM_SPHERE] = &ShapeShapeCollide<Box, Sphere>;
  collision_matrix[GEOM_BOX][GEOM_CAPSULE] = &ShapeShapeCollide<Box, Capsule>;
  collision_matrix[GEOM_BOX][GEOM_CONE] = &ShapeShapeCollide<Box, Cone>;
  collision_matrix[GEOM_BOX][GEOM_CYLINDER] = &ShapeShapeCollide<Box, Cylinder>;
  collision_matrix[GEOM_BOX][GEOM_CONVEX] = &ShapeShapeCollide<Box, ConvexBase>;
  collision_matrix[GEOM_BOX][GEOM_PLANE] = &ShapeShapeCollide<Box, Plane>;
  collision_matrix[GEOM_BOX][GEOM_HALFSPACE] = &ShapeShapeCollide<Box, Halfspace>;

  collision_matrix[GEOM_SPHERE][GEOM_BOX] = &ShapeShapeCollide<Sphere, Box>;
  collision_matrix[GEOM_SPHERE][GEOM_SPHERE] = &ShapeShapeCollide<Sphere, Sphere>;
  collision_matrix[GEOM_SPHERE][GEOM_CAPSULE] = &ShapeShapeCollide<Sphere, Capsule>;
  collision_matrix[GEOM_SPHERE][GEOM_CONE] = &ShapeShapeCollide<Sphere, Cone>;
  collision_matrix[GEOM_SPHERE][GEOM_CYLINDER] = &ShapeShapeCollide<Sphere, Cylinder>;
  collision_matrix[GEOM_SPHERE][GEOM_CONVEX] = &ShapeShapeCollide<Sphere, ConvexBase>;
  collision_matrix[GEOM_SPHERE][GEOM_PLANE] = &ShapeShapeCollide<Sphere, Plane>;
  collision_matrix[GEOM_SPHERE][GEOM_HALFSPACE] = &ShapeShapeCollide<Sphere, Halfspace>;

  collision_matrix[GEOM_CAPSULE][GEOM_BOX] = &ShapeShapeCollide<Capsule, Box>;
  collision_matrix[GEOM_CAPSULE][GEOM_SPHERE] = &ShapeShapeCollide<Capsule, Sphere>;
  collision_matrix[GEOM_CAPSULE][GEOM_CAPSULE] = &ShapeShapeCollide<Capsule, Capsule>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONE] = &ShapeShapeCollide<Capsule, Cone>;
  collision_matrix[GEOM_CAPSULE][GEOM_CYLINDER] = &ShapeShapeCollide<Capsule, Cylinder>;
  collision_matrix[GEOM_CAPSULE][GEOM_CONVEX] = &ShapeShapeCollide<Capsule, ConvexBase>;
  collision_matrix[GEOM_CAPSULE][GEOM_PLANE] = &ShapeShapeCollide<Capsule, Plane>;
  collision_matrix[GEOM_CAPSULE][GEOM_HALFSPACE] = &ShapeShapeCollide<Capsule, Halfspace>;

  collision_matrix[GEOM_CONE][GEOM_BOX] = &ShapeShapeCollide<Cone, Box>;
  collision_matrix[GEOM_CONE][GEOM_SPHERE] = &ShapeShapeCollide<Cone, Sphere>;
  collision_matrix[GEOM_CONE][GEOM_CAPSULE] = &ShapeShapeCollide<Cone, Capsule>;
  collision_matrix[GEOM_CONE][GEOM_CONE] = &ShapeShapeCollide<Cone, Cone>;
  collision_matrix[GEOM_CONE][GEOM_CYLINDER] = &ShapeShapeCollide<Cone, Cylinder>;
  collision_matrix[GEOM_CONE][GEOM_CONVEX] = &ShapeShapeCollide<Cone, ConvexBase>;
  collision_matrix[GEOM_CONE][GEOM_PLANE] = &ShapeShapeCollide<Cone, Plane>;
  collision_matrix[GEOM_CONE][GEOM_HALFSPACE] = &ShapeShapeCollide<Cone, Halfspace>;

  collision_matrix[GEOM_CYLINDER][GEOM_BOX] = &ShapeShapeCollide<Cylinder, Box>;
  collision_matrix[GEOM_CYLINDER][GEOM_SPHERE] = &ShapeShapeCollide<Cylinder, Sphere>;
  collision_matrix[GEOM_CYLINDER][GEOM_CAPSULE] = &ShapeShapeCollide<Cylinder, Capsule>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONE] = &ShapeShapeCollide<Cylinder, Cone>;
  collision_matrix[GEOM_CYLINDER][GEOM_CYLINDER] = &ShapeShapeCollide<Cylinder, Cylinder>;
  collision_matrix[GEOM_CYLINDER][GEOM_CONVEX] = &ShapeShapeCollide<Cylinder, ConvexBase>;
  collision_matrix[GEOM_CYLINDER][GEOM_PLANE] = &ShapeShapeCollide<Cylinder, Plane>;
  collision_matrix[GEOM_CYLINDER][GEOM_HALFSPACE] = &ShapeShapeCollide<Cylinder, Halfspace>;

  collision_matrix[GEOM_CONVEX][GEOM_BOX] = &ShapeShapeCollide<ConvexBase, Box>;
  collision_matrix[GEOM_CONVEX][GEOM_SPHERE] = &ShapeShapeCollide<ConvexBase, Sphere>;
  collision_matrix[GEOM_CONVEX][GEOM_CAPSULE] = &ShapeShapeCollide<ConvexBase, Capsule>;
  collision_matrix[GEOM_CONVEX][GEOM_CONE] = &ShapeShapeCollide<ConvexBase, Cone>;
  collision_matrix[GEOM_CONVEX][GEOM_CYLINDER] = &ShapeShapeCollide<ConvexBase, Cylinder>;
  collision_matrix[GEOM_CONVEX][GEOM_CONVEX] = &ShapeShapeCollide<ConvexBase, ConvexBase>;
  collision_matrix[GEOM_CONVEX][GEOM_PLANE] = &ShapeShapeCollide<ConvexBase, Plane>;
  collision_matrix[GEOM_CONVEX][GEOM_HALFSPACE] = &ShapeShapeCollide<ConvexBase, Halfspace>;

  collision_matrix[GEOM_PLANE][GEOM_BOX] = &ShapeShapeCollide<Plane, Box>;
  collision_matrix[GEOM_PLANE][GEOM_SPHERE] = &ShapeShapeCollide<Plane, Sphere>;
  collision_matrix[GEOM_PLANE][GEOM_CAPSULE] = &ShapeShapeCollide<Plane, Capsule>;
  collision_matrix[GEOM_PLANE][GEOM_CONE] = &ShapeShapeCollide<Plane, Cone>;
  collision_matrix[GEOM_PLANE][GEOM_CYLINDER] = &ShapeShapeCollide<Plane, Cylinder>;
  collision_matrix[GEOM_PLANE][GEOM_CONVEX] = &ShapeShapeCollide<Plane, ConvexBase>;
  collision_matrix[GEOM_PLANE][GEOM_PLANE] = &ShapeShapeCollide<Plane, Plane>;
  collision_matrix[GEOM_PLANE][GEOM_HALFSPACE] = &ShapeShapeCollide<Plane, Halfspace>;

  collision_matrix[GEOM_HALFSPACE][GEOM_BOX] = &ShapeShapeCollide<Halfspace, Box>;
  collision_matrix[GEOM_HALFSPACE][GEOM_SPHERE] = &ShapeShapeCollide<Halfspace, Sphere>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CAPSULE] = &ShapeShapeCollide<Halfspace, Capsule>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONE] = &ShapeShapeCollide<Halfspace, Cone>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CYLINDER] = &ShapeShapeCollide<Halfspace, Cylinder>;
  collision_matrix[GEOM_HALFSPACE][GEOM_CONVEX] = &ShapeShapeCollide<Halfspace, ConvexBase>;
  collision_matrix[GEOM_HALFSPACE][GEOM_PLANE] = &ShapeShapeCollide<Halfspace, Plane>;
  collision_matrix[GEOM_HALFSPACE][GEOM_HALFSPACE] = &ShapeShapeCollide<Halfspace, Halfspace>;

  collision_matrix[BV_AABB][GEOM_BOX] = &BVHShapeCollider<AABB, Box>::collide;
  collision_matrix[BV_AABB][GEOM_SPHERE] = &BVHShapeCollider<AABB, Sphere>::collide;
  collision_matrix[BV_AABB][GEOM_CAPSULE] = &BVHShapeCollider<AABB, Capsule>::collide;
  collision_matrix[BV_AABB][GEOM_CONE] = &BVHShapeCollider<AABB, Cone>::collide;
  collision_matrix[BV_AABB][GEOM_CYLINDER] = &BVHShapeCollider<AABB, Cylinder>::collide;
  collision_matrix[BV_AABB][GEOM_CONVEX] = &BVHShapeCollider<AABB, ConvexBase>::collide;
  collision_matrix[BV_AABB][GEOM_PLANE] = &BVHShapeCollider<AABB, Plane>::collide;
  collision_matrix[BV_AABB][GEOM_HALFSPACE] = &BVHShapeCollider<AABB, Halfspace>::collide;

  collision_matrix[BV_OBB][GEOM_BOX] = &BVHShapeCollider<OBB, Box>::collide;
  collision_matrix[BV_OBB][GEOM_SPHERE] = &BVHShapeCollider<OBB, Sphere>::collide;
  collision_matrix[BV_OBB][GEOM_CAPSULE] = &BVHShapeCollider<OBB, Capsule>::collide;
  collision_matrix[BV_OBB][GEOM_CONE] = &BVHShapeCollider<OBB, Cone>::collide;
  collision_matrix[BV_OBB][GEOM_CYLINDER] = &BVHShapeCollider<OBB, Cylinder>::collide;
  collision_matrix[BV_OBB][GEOM_CONVEX] = &BVHShapeCollider<OBB, ConvexBase>::collide;
  collision_matrix[BV_OBB][GEOM_PLANE] = &BVHShapeCollider<OBB, Plane>::collide;
  collision_matrix[BV_OBB][GEOM_HALFSPACE] = &BVHShapeCollider<OBB, Halfspace>::collide;

  collision_matrix[BV_RSS][GEOM_BOX] = &BVHShapeCollider<RSS, Box>::collide;
  collision_matrix[BV_RSS][GEOM_SPHERE] = &BVHShapeCollider<RSS, Sphere>::collide;
  collision_matrix[BV_RSS][GEOM_CAPSULE] = &BVHShapeCollider<RSS, Capsule>::collide;
  collision_matrix[BV_RSS][GEOM_CONE] = &BVHShapeCollider<RSS, Cone>::collide;
  collision_matrix[BV_RSS][GEOM_CYLINDER] = &BVHShapeCollider<RSS, Cylinder>::collide;
  collision_matrix[BV_RSS][GEOM_CONVEX] = &BVHShapeCollider<RSS, ConvexBase>::collide;
  collision_matrix[BV_RSS][GEOM_PLANE] = &BVHShapeCollider<RSS, Plane>::collide;
  collision_matrix[BV_RSS][GEOM_HALFSPACE] = &BVHShapeCollider<RSS, Halfspace>::collide;

  collision_matrix[BV_KDOP16][GEOM_BOX] = &BVHShapeCollider<KDOP<16>, Box>::collide;
  collision_matrix[BV_KDOP16][GEOM_SPHERE] = &BVHShapeCollider<KDOP<16>, Sphere>::collide;
  collision_matrix[BV_KDOP16][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<16>, Capsule>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONE] = &BVHShapeCollider<KDOP<16>, Cone>::collide;
  collision_matrix[BV_KDOP16][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<16>, Cylinder>::collide;
  collision_matrix[BV_KDOP16][GEOM_CONVEX] = &BVHShapeCollider<KDOP<16>, ConvexBase>::collide;
  collision_matrix[BV_KDOP16][GEOM_PLANE] = &BVHShapeCollider<KDOP<16>, Plane>::collide;
  collision_matrix[BV_KDOP16][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<16>, Halfspace>::collide;

  collision_matrix[BV_KDOP18][GEOM_BOX] = &BVHShapeCollider<KDOP<18>, Box>::collide;
  collision_matrix[BV_KDOP18][GEOM_SPHERE] = &BVHShapeCollider<KDOP<18>, Sphere>::collide;
  collision_matrix[BV_KDOP18][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<18>, Capsule>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONE] = &BVHShapeCollider<KDOP<18>, Cone>::collide;
  collision_matrix[BV_KDOP18][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<18>, Cylinder>::collide;
  collision_matrix[BV_KDOP18][GEOM_CONVEX] = &BVHShapeCollider<KDOP<18>, ConvexBase>::collide;
  collision_matrix[BV_KDOP18][GEOM_PLANE] = &BVHShapeCollider<KDOP<18>, Plane>::collide;
  collision_matrix[BV_KDOP18][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<18>, Halfspace>::collide;

  collision_matrix[BV_KDOP24][GEOM_BOX] = &BVHShapeCollider<KDOP<24>, Box>::collide;
  collision_matrix[BV_KDOP24][GEOM_SPHERE] = &BVHShapeCollider<KDOP<24>, Sphere>::collide;
  collision_matrix[BV_KDOP24][GEOM_CAPSULE] = &BVHShapeCollider<KDOP<24>, Capsule>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONE] = &BVHShapeCollider<KDOP<24>, Cone>::collide;
  collision_matrix[BV_KDOP24][GEOM_CYLINDER] = &BVHShapeCollider<KDOP<24>, Cylinder>::collide;
  collision_matrix[BV_KDOP24][GEOM_CONVEX] = &BVHShapeCollider<KDOP<24>, ConvexBase>::collide;
  collision_matrix[BV_KDOP24][GEOM_PLANE] = &BVHShapeCollider<KDOP<24>, Plane>::collide;
  collision_matrix[BV_KDOP24][GEOM_HALFSPACE] = &BVHShapeCollider<KDOP<24>, Halfspace>::collide;

  collision_matrix[BV_kIOS][GEOM_BOX] = &BVHShapeCollider<kIOS, Box>::collide;
  collision_matrix[BV_kIOS][GEOM_SPHERE] = &BVHShapeCollider<kIOS, Sphere>::collide;
  collision_matrix[BV_kIOS][GEOM_CAPSULE] = &BVHShapeCollider<kIOS, Capsule>::collide;
  collision_matrix[BV_kIOS][GEOM_CONE] = &BVHShapeCollider<kIOS, Cone>::collide;
  collision_matrix[BV_kIOS][GEOM_CYLINDER] = &BVHShapeCollider<kIOS, Cylinder>::collide;
  collision_matrix[BV_kIOS][GEOM_CONVEX] = &BVHShapeCollider<kIOS, ConvexBase>::collide;
  collision_matrix[BV_kIOS][GEOM_PLANE] = &BVHShapeCollider<kIOS, Plane>::collide;
  collision_matrix[BV_kIOS][GEOM_HALFSPACE] = &BVHShapeCollider<kIOS, Halfspace>::collide;

  collision_matrix[BV_OBBRSS][GEOM_BOX] = &BVHShapeCollider<OBBRSS, Box>::collide;
  collision_matrix[BV_OBBRSS][GEOM_SPHERE] = &BVHShapeCollider<OBBRSS, Sphere>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CAPSULE] = &BVHShapeCollider<OBBRSS, Capsule>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONE] = &BVHShapeCollider<OBBRSS, Cone>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CYLINDER] = &BVHShapeCollider<OBBRSS, Cylinder>::collide;
  collision_matrix[BV_OBBRSS][GEOM_CONVEX] = &BVHShapeCollider<OBBRSS, ConvexBase>::collide;
  collision_matrix[BV_OBBRSS][GEOM_PLANE] = &BVHShapeCollider<OBBRSS, Plane>::collide;
  collision_matrix[BV_OBBRSS][GEOM_HALFSPACE] = &BVHShapeCollider<OBBRSS, Halfspace>::collide;

  collision_matrix[BV_AABB][BV_AABB] = &BVHCollide<AABB>;
  collision_matrix[BV_OBB][BV_OBB] = &BVHCollide<OBB>;
  collision_matrix[BV_RSS][BV_RSS] = &BVHCollide<RSS>;
  collision_matrix[BV_KDOP16][BV_KDOP16] = &BVHCollide<KDOP<16> >;
  collision_matrix[BV_KDOP18][BV_KDOP18] = &BVHCollide<KDOP<18> >;
  collision_matrix[BV_KDOP24][BV_KDOP24] = &BVHCollide<KDOP<24> >;
  collision_matrix[BV_kIOS][BV_kIOS] = &BVHCollide<kIOS>;
  collision_matrix[BV_OBBRSS][BV_OBBRSS] = &BVHCollide<OBBRSS>;

#ifdef HPP_FCL_HAVE_OCTOMAP
  collision_matrix[GEOM_OCTREE][GEOM_BOX] = &Collide<OcTree, Box>;
  collision_matrix[GEOM_OCTREE][GEOM_SPHERE] = &Collide<OcTree, Sphere>;
  collision_matrix[GEOM_OCTREE][GEOM_CAPSULE] = &Collide<OcTree, Capsule>;
  collision_matrix[GEOM_OCTREE][GEOM_CONE] = &Collide<OcTree, Cone>;
  collision_matrix[GEOM_OCTREE][GEOM_CYLINDER] = &Collide<OcTree, Cylinder>;
  collision_matrix[GEOM_OCTREE][GEOM_CONVEX] = &Collide<OcTree, ConvexBase>;
  collision_matrix[GEOM_OCTREE][GEOM_PLANE] = &Collide<OcTree, Plane>;
  collision_matrix[GEOM_OCTREE][GEOM_HALFSPACE] = &Collide<OcTree, Halfspace>;

  collision_matrix[GEOM_BOX][GEOM_OCTREE] = &Collide<Box, OcTree>;
  collision_matrix[GEOM_SPHERE][GEOM_OCTREE] = &Collide<Sphere, OcTree>;
  collision_matrix[GEOM_CAPSULE][GEOM_OCTREE] = &Collide<Capsule, OcTree>;
  collision_matrix[GEOM_CONE][GEOM_OCTREE] = &Collide<Cone, OcTree>;
  collision_matrix[GEOM_CYLINDER][GEOM_OCTREE] = &Collide<Cylinder, OcTree>;
  collision_matrix[GEOM_CONVEX][GEOM_OCTREE] = &Collide<ConvexBase, OcTree>;
  collision_matrix[GEOM_PLANE][GEOM_OCTREE] = &Collide<Plane, OcTree>;
  collision_matrix[GEOM_HALFSPACE][GEOM_OCTREE] = &Collide<Halfspace, OcTree>;

  collision_matrix[GEOM_OCTREE][GEOM_OCTREE] = &Collide<OcTree, OcTree>;

  collision_matrix[GEOM_OCTREE][BV_AABB  ] = &Collide<OcTree, BVHModel<AABB     > >;
  collision_matrix[GEOM_OCTREE][BV_OBB   ] = &Collide<OcTree, BVHModel<OBB      > >;
  collision_matrix[GEOM_OCTREE][BV_RSS   ] = &Collide<OcTree, BVHModel<RSS      > >;
  collision_matrix[GEOM_OCTREE][BV_OBBRSS] = &Collide<OcTree, BVHModel<OBBRSS   > >;
  collision_matrix[GEOM_OCTREE][BV_kIOS  ] = &Collide<OcTree, BVHModel<kIOS     > >;
  collision_matrix[GEOM_OCTREE][BV_KDOP16] = &Collide<OcTree, BVHModel<KDOP<16> > >;
  collision_matrix[GEOM_OCTREE][BV_KDOP18] = &Collide<OcTree, BVHModel<KDOP<18> > >;
  collision_matrix[GEOM_OCTREE][BV_KDOP24] = &Collide<OcTree, BVHModel<KDOP<24> > >;

  collision_matrix[BV_AABB  ][GEOM_OCTREE] = &Collide<BVHModel<AABB     >, OcTree>;
  collision_matrix[BV_OBB   ][GEOM_OCTREE] = &Collide<BVHModel<OBB      >, OcTree>;
  collision_matrix[BV_RSS   ][GEOM_OCTREE] = &Collide<BVHModel<RSS      >, OcTree>;
  collision_matrix[BV_OBBRSS][GEOM_OCTREE] = &Collide<BVHModel<OBBRSS   >, OcTree>;
  collision_matrix[BV_kIOS  ][GEOM_OCTREE] = &Collide<BVHModel<kIOS     >, OcTree>;
  collision_matrix[BV_KDOP16][GEOM_OCTREE] = &Collide<BVHModel<KDOP<16> >, OcTree>;
  collision_matrix[BV_KDOP18][GEOM_OCTREE] = &Collide<BVHModel<KDOP<18> >, OcTree>;
  collision_matrix[BV_KDOP24][GEOM_OCTREE] = &Collide<BVHModel<KDOP<24> >, OcTree>;
#endif
}
//template struct CollisionFunctionMatrix;
}

} // namespace hpp
