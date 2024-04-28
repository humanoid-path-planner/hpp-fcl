/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
 *  All rights reserved.
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

#ifndef HPP_FCL_CONTACT_PATCH_SOLVER_H
#define HPP_FCL_CONTACT_PATCH_SOLVER_H

#include "hpp/fcl/collision_data.h"
#include "hpp/fcl/logging.h"
#include "hpp/fcl/narrowphase/gjk.h"

namespace hpp {
namespace fcl {

/// @brief Construct a frame from the contact's normal.
/// This frame is expressed w.r.t the world frame.
/// The origin of the frame is `contact.pos` and the z-axis of the frame
/// is `contact.normal`.
inline void constructContactPatchFrame(const Contact& contact,
                                       ContactPatch& contact_patch);

/// @brief Solver to compute contact patches, i.e. the intersection between two
/// contact surfaces projected onto the shapes' separating plane.
struct HPP_FCL_DLLAPI ContactPatchSolver {
 public:
  using Index = ContactPatch::Index;
  using ContactPoint = ContactPatch::ContactPoint;
  using ReferenceFrame = ContactPatch::ReferenceFrame;

 private:
  /// @brief Minkowski difference used to compute support function of the
  /// considered shapes.
  mutable details::MinkowskiDiff m_minkowski_difference;

  /// @brief Two sets of points, which are the projections of the shapes
  /// supports onto the separating plane. These sets of points may not be
  /// convex. The shapes supports all belong to the support sets of the shapes,
  /// in the direction of the `Contact`'s normal.
  /// The resulting contact surface that the `ContactPatchSolver` computes is
  /// the intersection of the convex-hull of these two sets of points.
  /// @note Because these are 2D points, we use the convenient `ContactPatch`
  /// struct to represent these two sets of points.
  mutable std::array<ContactPatch, 2> m_projected_shapes_supports;

  /// @brief Contact patches used for internal computation.
  /// @note The `computePatch` algorithm starts by constructing two 2D
  /// convex-hulls (the convex-hulls of the `m_projected_shapes_supports`). It
  /// then uses the first convex-hull to clip the second one, effectively
  /// computing the intersection between the two convex-hulls.
  /// Why have 3 contact patches then? Because the algorithm works by
  /// successively clipping the first conve-hull. So the first two contact
  /// patches represent the current and previous iteration of the algorithm and
  /// the third contact patch represents the convex-hull of the second shape.
  mutable std::array<ContactPatch, 3> m_contact_patches;

  /// @brief Tracks the current iterate of the algorithm.
  mutable size_t m_id_current{0};

 public:
  /// @brief Number of vectors to pre-allocate in the `shapes_supports` vectors.
  static constexpr size_t default_num_preallocated_supports = 16;

  /// @brief Default constructor.
  explicit ContactPatchSolver() {
    const size_t num_contact_patch = 1;
    const size_t size_contact_patch = ContactPatch::default_max_size;
    const ContactPatchRequest request(num_contact_patch, size_contact_patch);
    this->set(request);
  }

  /// @brief Construct the solver with a `ContactPatchRequest`.
  explicit ContactPatchSolver(const ContactPatchRequest& request) {
    this->set(request);
  }

  /// @brief Set up the solver using a `ContactPatchRequest`.
  void set(const ContactPatchRequest& request);

  /// @brief Main API of the solver: compute a contact patch from a contact
  /// between shapes s1 and s2.
  /// The contact patch is the (triple) intersection between the separating
  /// plane passing (by `contact.pos` and supported by `contact.normal`) and the
  /// shapes s1 and s2.
  template <typename ShapeType1, typename ShapeType2>
  void computePatch(const ShapeType1& s1, const Transform3f& tf1,
                    const ShapeType2& s2, const Transform3f& tf2,
                    const Contact& contact, ContactPatch& contact_patch) const;

  /// @return true if p inside a clipping region defined by a and b, false
  /// otherwise.
  /// @param p point to check
  /// @param a, b the vertices forming the edge of the clipping region.
  /// @note the clipping ray points from a to b. Points on the right of the ray
  /// are outside the clipping region; points on the left are inside.
  template <typename Vector2dLike>
  static bool pointIsInsideClippingRegion(
      const Eigen::MatrixBase<Vector2dLike>& p,
      const Eigen::MatrixBase<Vector2dLike>& a,
      const Eigen::MatrixBase<Vector2dLike>& b);

  /// @return the intersecting point between line defined by ray (a, b) and
  /// the segment [c, d].
  /// @note we make the following hypothesis:
  /// 1) c != d (should be when creating initial polytopes)
  /// 2) (c, d) is not parallel to ray -> if so, we return d.
  template <typename Vector2dLike>
  static ContactPoint computeLineSegmentIntersection(
      const Eigen::MatrixBase<Vector2dLike>& a,
      const Eigen::MatrixBase<Vector2dLike>& b,
      const Eigen::MatrixBase<Vector2dLike>& c,
      const Eigen::MatrixBase<Vector2dLike>& d);

 private:
  /// @brief Reset the internal quantities of the solver.
  void reset() const;

  /// @brief Getter for current iterate.
  ContactPatch& current() {
    return this->m_contact_patches[this->m_id_current];
  }

  /// @brief Const getter for current iterate.
  const ContactPatch& current() const {
    return this->m_contact_patches[this->m_id_current];
  }

  /// @brief Getter for previous iterate.
  ContactPatch& previous() {
    return this->m_contact_patches[1 - this->m_id_current];
  }

  /// @brief Const getter for previous iterate.
  const ContactPatch& previous() const {
    return this->m_contact_patches[1 - this->m_id_current];
  }

  /// @brief Getter for the patch used to clip the other one.
  ContactPatch& clipper() { return this->m_contact_patches[2]; }

  /// @brief Const getter for the patch used to clip the other one.
  const ContactPatch& clipper() const { return this->m_contact_patches[2]; }
};

}  // namespace fcl
}  // namespace hpp

#include "hpp/fcl/contact_patch/contact_patch_solver.hxx"

#endif  // HPP_FCL_CONTACT_PATCH_SOLVER_H
