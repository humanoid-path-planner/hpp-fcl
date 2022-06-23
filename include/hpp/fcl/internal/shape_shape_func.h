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

#ifndef HPP_FCL_INTERNAL_SHAPE_SHAPE_FUNC_H
#define HPP_FCL_INTERNAL_SHAPE_SHAPE_FUNC_H

/// @cond INTERNAL

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_utility.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/shape/geometric_shapes_traits.h>

namespace hpp {
namespace fcl {

template <typename T_SH1, typename T_SH2>
HPP_FCL_DLLAPI FCL_REAL ShapeShapeDistance(const CollisionGeometry* o1,
                                           const Transform3f& tf1,
                                           const CollisionGeometry* o2,
                                           const Transform3f& tf2,
                                           const GJKSolver* nsolver,
                                           const DistanceRequest& request,
                                           DistanceResult& result);

template <typename T_SH1, typename T_SH2>
struct ShapeShapeCollider
{
  static std::size_t run(const CollisionGeometry* o1,
                         const Transform3f& tf1,
                         const CollisionGeometry* o2,
                         const Transform3f& tf2, const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result) {
    if (request.isSatisfied(result)) return result.numContacts();
    
    DistanceResult distanceResult;
    DistanceRequest distanceRequest(request.enable_contact);
    FCL_REAL distance = ShapeShapeDistance<T_SH1, T_SH2>(
                                                         o1, tf1, o2, tf2, nsolver, distanceRequest, distanceResult);
    
    size_t num_contacts = 0;
    const Vec3f& p1 = distanceResult.nearest_points[0];
    const Vec3f& p2 = distanceResult.nearest_points[1];
    FCL_REAL distToCollision = distance - request.security_margin;
    
    internal::updateDistanceLowerBoundFromLeaf(request, result, distToCollision,
                                               p1, p2);
    if (distToCollision <= request.collision_distance_threshold &&
        result.numContacts() < request.num_max_contacts) {
      if (result.numContacts() < request.num_max_contacts) {
        const Vec3f& p1 = distanceResult.nearest_points[0];
        const Vec3f& p2 = distanceResult.nearest_points[1];
        
        Contact contact(
                        o1, o2, distanceResult.b1, distanceResult.b2, (p1 + p2) / 2,
                        (distance <= 0 ? distanceResult.normal : (p2 - p1).normalized()),
                        -distance);
        
        result.addContact(contact);
      }
      num_contacts = result.numContacts();
    }
    
    return num_contacts;
  }
};

/// @cond DEV
namespace details
{

// Forward declaration
template<typename ShapeType1, typename ShapeType2, bool shape1_is_inflatable = shape_traits<ShapeType1>::IsInflatable, bool shape1_has_inflated_support_function = shape_traits<ShapeType1>::HasInflatedSupportFunction, bool shape2_is_inflatable = shape_traits<ShapeType2>::IsInflatable, bool shape2_has_inflated_support_function =  shape_traits<ShapeType2>::HasInflatedSupportFunction>
struct shape_shape_collide_negative_security_margin;

// Handle case where both shapes are inflatable
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, true,false,true,false>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const FCL_REAL min_inflation1 = o1.minInflationValue();
    const FCL_REAL min_inflation2 = o2.minInflationValue();
    const FCL_REAL min_inflation = min_inflation1 + min_inflation2;
    const FCL_REAL security_margin = request.security_margin;
    
    if(security_margin < min_inflation)
      HPP_FCL_THROW_PRETTY("The request security margin: "
                           << security_margin
                           << " is below the minimal security margin authorised by the pair of two shapes"
                           << "(" << std::string(get_node_type_name(o1.getNodeType()))
                           << "," << std::string(get_node_type_name(o2.getNodeType()))
                           << "): "
                           << min_inflation
                           << ".\n Please consider increasing the requested security margin.",
                           std::invalid_argument);
    
    if(security_margin > min_inflation1)
    {
      const auto & deflated_result = o1.inflated(security_margin);
      const ShapeType1 & deflated_o1 = deflated_result.first;
      const Transform3f deflated_tf1 = tf1 * deflated_result.second;
      CollisionRequest deflated_request(request);
      deflated_request.security_margin = 0;
      return ShapeShapeCollider<ShapeType1,ShapeType2>::run(&deflated_o1, deflated_tf1, &o2, tf2, nsolver, deflated_request, result);
    }
    else if(security_margin > min_inflation2)
    {
      const auto & deflated_result = o2.inflated(security_margin);
      const ShapeType2 & deflated_o2 = deflated_result.first;
      const Transform3f deflated_tf2 = tf2 * deflated_result.second;
      CollisionRequest deflated_request(request);
      deflated_request.security_margin = 0;
      return ShapeShapeCollider<ShapeType1,ShapeType2>::run(&o1, tf1, &deflated_o2, deflated_tf2, nsolver, deflated_request, result);
    }
    else // deflate both
    {
      const FCL_REAL half_security_margin = 0.5 * security_margin;
      const auto & deflated_result1 = o1.inflated(half_security_margin);
      const ShapeType1 & deflated_o1 = deflated_result1.first;
      const Transform3f deflated_tf1 = tf1 * deflated_result1.second;
      const auto & deflated_result2 = o2.inflated(half_security_margin);
      const ShapeType2 & deflated_o2 = deflated_result2.first;
      const Transform3f deflated_tf2 = tf2 * deflated_result2.second;
      CollisionRequest deflated_request(request);
      deflated_request.security_margin = 0;
      return ShapeShapeCollider<ShapeType1,ShapeType2>::run(&deflated_o1, deflated_tf1, &deflated_o2, deflated_tf2, nsolver, deflated_request, result);
    }
  }
};

// Handle case where only the first shape is inflatable
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, true,false,false,false>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const FCL_REAL min_inflation = o1.minInflationValue();
    const FCL_REAL security_margin = request.security_margin;
    
    if(security_margin < min_inflation)
      HPP_FCL_THROW_PRETTY("The request security margin: "
                           << security_margin
                           << " is below the minimal security margin authorised by the pair of two shapes"
                           << "(" << std::string(get_node_type_name(o1.getNodeType()))
                           << "," << std::string(get_node_type_name(o2.getNodeType()))
                           << "): "
                           << min_inflation
                           << ".\n Please consider increasing the requested security margin.",
                           std::invalid_argument);
    
    const auto & deflated_result = o1.inflated(security_margin);
    const ShapeType1 & deflated_o1 = deflated_result.first;
    const Transform3f deflated_tf1 = tf1 * deflated_result.second;
    CollisionRequest deflated_request(request);
    deflated_request.security_margin = 0;
    return ShapeShapeCollider<ShapeType1,ShapeType2>::run(&deflated_o1, deflated_tf1, &o2, tf2, nsolver, deflated_request, result);
  }
};

// Handle case where only the second shape is inflatable
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, false,false,true,false>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const std::size_t res = shape_shape_collide_negative_security_margin<ShapeType2,ShapeType1,true,false,false,false>::run(o2, tf2, o1, tf1, nsolver, request, result);
    result.swapObjects();
    return res;
  }
};

// Handle case where the first shape is inflatable and the second shape has an inflated support function
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, true,false,false,true>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const FCL_REAL min_inflation = o1.minInflationValue();
    const FCL_REAL security_margin = request.security_margin;
    
    if(min_inflation < security_margin)
    {
      const auto & deflated_result = o1.inflated(security_margin);
      const ShapeType1 & deflated_o1 = deflated_result.first;
      const Transform3f deflated_tf1 = tf1 * deflated_result.second;
      CollisionRequest deflated_request(request);
      deflated_request.security_margin = 0;
      return ShapeShapeCollider<ShapeType1,ShapeType2>::run(&deflated_o1, deflated_tf1, &o2, tf2, nsolver, deflated_request, result);
    }
    else
    {
      const FCL_REAL half_security_margin = 0.5 * security_margin;
      const auto & deflated_result = o1.inflated(half_security_margin);
      const ShapeType1 & deflated_o1 = deflated_result.first;
      const Transform3f deflated_tf1 = tf1 * deflated_result.second;
      CollisionRequest deflated_request(request);
      deflated_request.security_margin = 0;
      assert(nsolver != NULL && "The GJK solver is not defined.");
      nsolver->setShapeDeflation(0, half_security_margin);
      
      std::size_t res = ShapeShapeCollider<ShapeType1,ShapeType2>::run(&deflated_o1, deflated_tf1, &o2, tf2, nsolver, deflated_request, result);
      
      nsolver->resetShapeDeflation();
      return res;
    }
  }
};

// Handle case where the second shape is inflatable and the first shape has an inflated support function
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, false,true,true,false>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const std::size_t res = shape_shape_collide_negative_security_margin<ShapeType2,ShapeType1,true,false,false,true>::run(o2, tf2, o1, tf1, nsolver, request, result);
    result.swapObjects();
    return res;
  }
};

// Handle case where the two shapes have inflated support function
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, false,true,false,true>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const FCL_REAL security_margin = request.security_margin;
    const FCL_REAL half_security_margin = 0.5 * security_margin;
    CollisionRequest deflated_request(request);
    deflated_request.security_margin = 0;
    
    assert(nsolver != NULL && "The GJK solver is not defined.");
    nsolver->setShapeDeflation(half_security_margin, half_security_margin);
    
    std::size_t res = ShapeShapeCollider<ShapeType1,ShapeType2>::run(&o1, tf1, &o2, tf2, nsolver, deflated_request, result);
    
    nsolver->resetShapeDeflation();
    return res;
  }
};

// Handle case where only the first shape has inflated support function
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, false,true,false,false>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const FCL_REAL security_margin = request.security_margin;
    CollisionRequest deflated_request(request);
    deflated_request.security_margin = 0;
    
    assert(nsolver != NULL && "The GJK solver is not defined.");
    nsolver->setShapeDeflation(security_margin, 0);
    
    std::size_t res = ShapeShapeCollider<ShapeType1,ShapeType2>::run(&o1, tf1, &o2, tf2, nsolver, deflated_request, result);
    
    nsolver->resetShapeDeflation();
    return res;
  }
};

// Handle case where only the second shape has inflated support function
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, false,false,false,true>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& tf1,
                         const ShapeType2 & o2,
                         const Transform3f& tf2,
                         const GJKSolver* nsolver,
                         const CollisionRequest& request,
                         CollisionResult& result)
  {
    const std::size_t res = shape_shape_collide_negative_security_margin<ShapeType2,ShapeType1,false,true,false,false>::run(o2, tf2, o1, tf1, nsolver, request, result);
    result.swapObjects();
    return res;
  }
};

// Handle case where the shapes info are not provided.
template<typename ShapeType1, typename ShapeType2>
struct shape_shape_collide_negative_security_margin<ShapeType1, ShapeType2, false,false,false,false>
{
  static std::size_t run(const ShapeType1 & o1,
                         const Transform3f& /*tf1*/,
                         const ShapeType2 & o2,
                         const Transform3f& /*tf2*/,
                         const GJKSolver* /*nsolver*/,
                         const CollisionRequest& /*request*/,
                         CollisionResult& /*result*/)
  {
    HPP_FCL_THROW_PRETTY("Negative security margin between node type "
                         << std::string(get_node_type_name(o1.getNodeType()))
                         << " and node type "
                         << std::string(get_node_type_name(o2.getNodeType()))
                         << " is not supported.",
                         std::invalid_argument);
    return 0;
  }
};
} // namespace details

/// \endcond

template <typename ShapeType1, typename ShapeType2>
HPP_FCL_DLLAPI std::size_t ShapeShapeCollide(const CollisionGeometry* o1,
                                             const Transform3f& tf1,
                                             const CollisionGeometry* o2,
                                             const Transform3f& tf2,
                                             const GJKSolver* nsolver,
                                             const CollisionRequest& request,
                                             CollisionResult& result)
{
  if(request.security_margin < 0)
  {
    const ShapeType1 & shape1 = static_cast<const ShapeType1&>(*o1);
    const ShapeType2 & shape2 = static_cast<const ShapeType2&>(*o2);
    
    return details::shape_shape_collide_negative_security_margin<ShapeType1,ShapeType2>::run(shape1, tf1, shape2, tf2, nsolver, request, result);
  }
  else
  {
    return ShapeShapeCollider<ShapeType1,ShapeType2>::run(o1, tf1, o2, tf2, nsolver, request, result);
  }
}

#define SHAPE_SHAPE_DISTANCE_SPECIALIZATION(T1, T2)             \
  template <>                                                   \
  HPP_FCL_DLLAPI FCL_REAL ShapeShapeDistance<T1, T2>(           \
      const CollisionGeometry* o1, const Transform3f& tf1,      \
      const CollisionGeometry* o2, const Transform3f& tf2,      \
      const GJKSolver* nsolver, const DistanceRequest& request, \
      DistanceResult& result);                                  \
  template <>                                                   \
  HPP_FCL_DLLAPI FCL_REAL ShapeShapeDistance<T2, T1>(           \
      const CollisionGeometry* o1, const Transform3f& tf1,      \
      const CollisionGeometry* o2, const Transform3f& tf2,      \
      const GJKSolver* nsolver, const DistanceRequest& request, \
      DistanceResult& result)

SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Box, Sphere);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Capsule);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Capsule, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cone, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cone, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cylinder, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Cylinder, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Plane);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Sphere);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(Sphere, Cylinder);

SHAPE_SHAPE_DISTANCE_SPECIALIZATION(ConvexBase, Halfspace);
SHAPE_SHAPE_DISTANCE_SPECIALIZATION(TriangleP, Halfspace);

#undef SHAPE_SHAPE_DISTANCE_SPECIALIZATION

#define SHAPE_SHAPE_COLLIDE_SPECIALIZATION(T1, T2)               \
  template <>                                                    \
struct ShapeShapeCollider<T1, T2> {         \
      static HPP_FCL_DLLAPI std::size_t run( \
      const CollisionGeometry* o1, const Transform3f& tf1,       \
      const CollisionGeometry* o2, const Transform3f& tf2,       \
      const GJKSolver* nsolver, const CollisionRequest& request, \
  CollisionResult& result); \
}

SHAPE_SHAPE_COLLIDE_SPECIALIZATION(Sphere, Sphere);

#undef SHAPE_SHAPE_COLLIDE_SPECIALIZATION
}  // namespace fcl

}  // namespace hpp

/// @endcond

#endif
