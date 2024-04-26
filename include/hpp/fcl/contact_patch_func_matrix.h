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

#ifndef HPP_FCL_CONTACT_PATCH_FUNC_MATRIX_H
#define HPP_FCL_CONTACT_PATCH_FUNC_MATRIX_H

#include <hpp/fcl/collision_data.h>

struct GJKSolver;
struct ContactPatchSolver;

namespace hpp {
namespace fcl {

/// @brief The contact patch matrix stores the functions for contact patches
/// computation between different types of objects and provides a uniform call
/// interface
struct HPP_FCL_DLLAPI ContactPatchFunctionMatrix {
  /// @brief the uniform call interface for computing contact patches: we need
  /// know
  /// 1. two objects o1 and o2 and their configuration in world coordinate tf1
  ///    and tf2;
  /// 2. the collision result that generated contact patches candidates
  ///    (`hpp::fcl::Contact`), from which contact patches will be expanded;
  /// 3. the narrow phase solver that was used to compute the collision result;
  /// 4. the solver for computation of contact patches;
  /// 5. the request setting for contact patches (e.g. maximum amoung and size
  ///    of contact patches);
  /// 6. the structure to return contact patches.
  ///
  /// Note: we pass a GJKSolver, because it allows to reuse internal computation
  /// that was made during the narrow phase. It also allows to experiment with
  /// different ways to compute contact patches. We could, for example, perturb
  /// tf1 and tf2 and make multiple calls to the GJKSolver (although this is not
  /// the approach done by default).
  typedef std::size_t (*ContactPatchFunc)(
      const CollisionGeometry* o1, const Transform3f& tf1,
      const CollisionGeometry* o2, const Transform3f& tf2,
      const CollisionResult& collision_result, const GJKSolver* nsolver,
      const ContactPatchSolver* csolver, const ContactPatchRequest& request,
      ContactPatchResult& result);

  /// @brief Each item in the contact patch matrix is a function to handle
  /// contact patch computation between objects of type1 and type2.
  ContactPatchFunc contact_patch_matrix[NODE_COUNT][NODE_COUNT];

  ContactPatchFunctionMatrix();
};

}  // namespace fcl
}  // namespace hpp

#endif
