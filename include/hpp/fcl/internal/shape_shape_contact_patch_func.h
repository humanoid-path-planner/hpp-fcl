/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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

/** \author Louis Montaut */

#ifndef HPP_FCL_INTERNAL_SHAPE_SHAPE_CONTACT_PATCH_FUNC_H
#define HPP_FCL_INTERNAL_SHAPE_SHAPE_CONTACT_PATCH_FUNC_H

#include "hpp/fcl/collision_data.h"
#include "hpp/fcl/collision_utility.h"
#include "hpp/fcl/narrowphase/narrowphase.h"
#include "hpp/fcl/contact_patch/contact_patch_solver.h"
#include "hpp/fcl/shape/geometric_shapes_traits.h"

namespace hpp {
namespace fcl {

/// @brief Shape-shape contact patch computation.
/// Assumes that `csolver` and the `ContactPatchResult` have already been set up
/// by the `ContactPatchRequest`.
template <typename ShapeType1, typename ShapeType2>
struct ComputeShapeShapeContactPatch {
  static void run(const CollisionGeometry* o1, const Transform3f& tf1,
                  const CollisionGeometry* o2, const Transform3f& tf2,
                  const CollisionResult& collision_result,
                  const ContactPatchSolver* csolver,
                  const ContactPatchRequest& request,
                  ContactPatchResult& result) {
    // TODO(louis): don't forget about swept-sphere radius
    // TODO(louis): deal with pairs with strictly convex shapes
    if (!collision_result.isCollision()) {
      return;
    }
    HPP_FCL_ASSERT(
        result.check(request),
        "The contact patch result and request are incompatible (issue of "
        "contact patch size or maximum number of contact patches). Make sure "
        "result is initialized with request.",
        std::logic_error);

    const ShapeType1& s1 = static_cast<const ShapeType1&>(*o1);
    const ShapeType2& s2 = static_cast<const ShapeType2&>(*o2);
    for (size_t i = 0; i < collision_result.numContacts(); ++i) {
      if (i >= request.getMaxNumContactPatch() ||
          i >= result.maxNumContactPatches()) {
        break;
      }
      csolver->setSupportGuess(collision_result.cached_support_func_guess);
      const Contact& contact = collision_result.getContact(i);
      ContactPatch& contact_patch = result.getUnusedContactPatch();
      csolver->computePatch(s1, tf1, s2, tf2, contact, contact_patch);
    }
  }
};

template <typename ShapeType1, typename ShapeType2>
void ShapeShapeContactPatch(const CollisionGeometry* o1, const Transform3f& tf1,
                            const CollisionGeometry* o2, const Transform3f& tf2,
                            const CollisionResult& collision_result,
                            const ContactPatchSolver* csolver,
                            const ContactPatchRequest& request,
                            ContactPatchResult& result) {
  return ComputeShapeShapeContactPatch<ShapeType1, ShapeType2>::run(
      o1, tf1, o2, tf2, collision_result, csolver, request, result);
}

inline void capsuleBoxMCP(const Capsule& s1, const Transform3f& tf1,
                          const Box& s2, const Transform3f& tf2,
                          const Vec3f& p1, const Vec3f& p2, const Vec3f& normal,
                          const FCL_REAL distance,
                          const CollisionRequest& request,
                          CollisionResult& result) {
  HPP_FCL_UNUSED_VARIABLE(s2);
  HPP_FCL_UNUSED_VARIABLE(tf2);
  HPP_FCL_UNUSED_VARIABLE(p2);
  HPP_FCL_UNUSED_VARIABLE(request);

  // Transform normal and p1 into capsule frame
  // Note: "loc" stands for "local"
  const Vec3f n1_loc = tf1.rotation().transpose() * normal;
  const Vec3f p1_loc = tf1.inverseTransform(p1);

  const Vec3f capsule_tip_a = Vec3f(0, 0, s1.halfLength) + s1.radius * n1_loc;
  const bool tip_a_is_support =
      (std::abs((p1_loc - capsule_tip_a).dot(n1_loc)) < 1e-3);
  const Vec3f capsule_tip_b = Vec3f(0, 0, -s1.halfLength) + s1.radius * n1_loc;
  const bool tip_b_is_support =
      (std::abs((p1_loc - capsule_tip_b).dot(n1_loc)) < 1e-3);

  if (tip_a_is_support && tip_b_is_support) {
    // Add 2 contact points
    const Vec3f pa = capsule_tip_a + 0.5 * distance * n1_loc;
    result.addContact(Contact(&s1, &s2, Contact::NONE, Contact::NONE,
                              tf1.transform(pa), normal, distance));

    const Vec3f pb = capsule_tip_b + 0.5 * distance * n1_loc;
    result.addContact(Contact(&s1, &s2, Contact::NONE, Contact::NONE,
                              tf1.transform(pb), normal, distance));
  } else {
    Contact contact(&s1, &s2, Contact::NONE, Contact::NONE, p1, p2, normal,
                    distance);
    result.addContact(contact);
  }
}

// template <>
// inline void extractContactPoints<Capsule, Box>(
//     const CollisionGeometry* o1, const Transform3f& tf1,
//     const CollisionGeometry* o2, const Transform3f& tf2, const Vec3f& p1,
//     const Vec3f& p2, const Vec3f& normal, const FCL_REAL distance,
//     const CollisionRequest& request, CollisionResult& result) {
//   const Capsule& s1 = static_cast<const Capsule&>(*o1);
//   const Box& s2 = static_cast<const Box&>(*o2);
//   capsuleBoxMCP(s1, tf1, s2, tf2, p1, p2, normal, distance, request,
//   result);
// }

// template <>
// inline void extractContactPoints<Box, Capsule>(
//     const CollisionGeometry* o1, const Transform3f& tf1,
//     const CollisionGeometry* o2, const Transform3f& tf2, const Vec3f& p1,
//     const Vec3f& p2, const Vec3f& normal, const FCL_REAL distance,
//     const CollisionRequest& request, CollisionResult& result) {
//   const Box& s1 = static_cast<const Box&>(*o1);
//   const Capsule& s2 = static_cast<const Capsule&>(*o2);
//   capsuleBoxMCP(s2, tf2, s1, tf1, p2, p1, -normal, distance, request,
//   result);
// }

}  // namespace fcl
}  // namespace hpp

#endif
