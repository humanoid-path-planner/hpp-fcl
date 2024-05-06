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
 *   * Neither the name of INRIA nor the names of its
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

#include "hpp/fcl/contact_patch.h"
#include "hpp/fcl/collision_utility.h"

namespace hpp {
namespace fcl {

ContactPatchFunctionMatrix& getContactPatchFunctionLookTable() {
  static ContactPatchFunctionMatrix table;
  return table;
}

void computeContactPatch(const CollisionGeometry* o1, const Transform3f& tf1,
                         const CollisionGeometry* o2, const Transform3f& tf2,
                         const CollisionResult& collision_result,
                         const ContactPatchRequest& request,
                         ContactPatchResult& result) {
  if (!collision_result.isCollision() || request.max_num_patch == 0) {
    // do nothing
    return;
  }

  OBJECT_TYPE object_type1 = o1->getObjectType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  NODE_TYPE node_type2 = o2->getNodeType();
  // TODO(louis): add support for BVH (leaf is a triangle) and Hfield (leaf is a
  // convex)
  if (object_type1 != OBJECT_TYPE::OT_GEOM ||
      object_type2 != OBJECT_TYPE::OT_GEOM) {
    HPP_FCL_THROW_PRETTY("Computing contact patches between node type "
                             << std::string(get_node_type_name(node_type1))
                             << " and node type "
                             << std::string(get_node_type_name(node_type2))
                             << " is not yet supported. Only primitive shapes "
                                "are supported for now.",
                         std::invalid_argument);
    return;
  }

  const ContactPatchFunctionMatrix& looktable =
      getContactPatchFunctionLookTable();
  if (!looktable.contact_patch_matrix[node_type1][node_type2]) {
    HPP_FCL_THROW_PRETTY("Contact patch computation between node type "
                             << std::string(get_node_type_name(node_type1))
                             << " and node type "
                             << std::string(get_node_type_name(node_type2))
                             << " is not yet supported.",
                         std::invalid_argument);
  }

  // Before doing any computation, we initialize and clear the input result.
  result.set(request);
  ContactPatchSolver csolver(request);
  return looktable.contact_patch_matrix[node_type1][node_type2](
      o1, tf1, o2, tf2, collision_result, &csolver, request, result);
}

ComputeContactPatch::ComputeContactPatch(const CollisionGeometry* o1,
                                         const CollisionGeometry* o2)
    : o1(o1), o2(o2) {
  const ContactPatchFunctionMatrix& looktable =
      getContactPatchFunctionLookTable();

  OBJECT_TYPE object_type1 = this->o1->getObjectType();
  NODE_TYPE node_type1 = this->o1->getNodeType();
  OBJECT_TYPE object_type2 = this->o2->getObjectType();
  NODE_TYPE node_type2 = this->o2->getNodeType();

  if (object_type1 != OBJECT_TYPE::OT_GEOM ||
      object_type2 != OBJECT_TYPE::OT_GEOM) {
    HPP_FCL_THROW_PRETTY("Computing contact patches between node type "
                             << std::string(get_node_type_name(node_type1))
                             << " and node type "
                             << std::string(get_node_type_name(node_type2))
                             << " is not yet supported. Only primitive shapes "
                                "are supported for now.",
                         std::invalid_argument);
  }

  if (!looktable.contact_patch_matrix[node_type1][node_type2]) {
    HPP_FCL_THROW_PRETTY("Contact patch computation between node type "
                             << std::string(get_node_type_name(node_type1))
                             << " and node type "
                             << std::string(get_node_type_name(node_type2))
                             << " is not yet supported.",
                         std::invalid_argument);
  }

  this->func = looktable.contact_patch_matrix[node_type1][node_type2];
}

void ComputeContactPatch::run(const Transform3f& tf1, const Transform3f& tf2,
                              const CollisionResult& collision_result,
                              const ContactPatchRequest& request,
                              ContactPatchResult& result) const {
  if (!collision_result.isCollision() || request.max_num_patch == 0) {
    // do nothing
    return;
  }

  // Before doing any computation, we initialize and clear the input result.
  result.set(request);
  this->func(this->o1, tf1, this->o2, tf2, collision_result, &(this->csolver),
             request, result);
}

void ComputeContactPatch::operator()(const Transform3f& tf1,
                                     const Transform3f& tf2,
                                     const CollisionResult& collision_result,
                                     const ContactPatchRequest& request,
                                     ContactPatchResult& result) const

{
  this->csolver.set(request);
  this->run(tf1, tf2, collision_result, request, result);
}

}  // namespace fcl
}  // namespace hpp
