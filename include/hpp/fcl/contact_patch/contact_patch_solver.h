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

/// @brief Solver to compute contact patches, i.e. the intersection between two
/// contact surfaces projected onto the shapes' separating plane.
/// Otherwise said, a contact patch is simply the intersection between two
/// support sets: the support set of shape S1 in direction `n` and the support
/// set of shape S2 in direction `-n`, where `n` is the contact normal
/// (satisfying the optimality conditions of GJK/EPA).
/// @note A contact patch is **not** the support set of the Minkowski Difference
/// in the direction of the normal.
/// A contact patch is actually the support set of the Minkowski difference in
/// the direction of the normal, i.e. the instersection of the shapes support
/// sets as mentioned above.
///
/// TODO(louis): algo improvement:
/// - The clipping algo is currently n1 * n2; it can be done in n1 + n2.
struct HPP_FCL_DLLAPI ContactPatchSolver {
 public:
  // Note: `ContactPatch` is an alias for `SupportSet`.
  // The two can be used interchangeably.
  using ShapeSupportData = details::ShapeSupportData;
  using SupportSetDirection = SupportSet::PatchDirection;

  /// @brief Support set function for shape si.
  /// @param[in] shape the shape.
  /// @param[in/out] support_set a support set of the shape. A support set is
  /// attached to a frame. All the points of the set computed by this function
  /// will be expressed in the local frame of the support set. The support set
  /// is computed in the direction of the positive z-axis if its direction is
  /// DEFAULT, negative z-axis if its direction is INVERTED.
  /// @param[in/out] hint for the support computation of ConvexBase shapes. Gets
  /// updated after calling the function onto ConvexBase shapes.
  /// @param[in/out] support_data for the support computation of ConvexBase
  /// shapes. Gets updated with visited vertices after calling the function onto
  /// ConvexBase shapes.
  /// @param[in] num_sampled_supports for shapes like cone or cylinders which
  /// have smooth non-strictly convex sides (their bases are circles), we need
  /// to know how many supports we sample from these sides. For any other shape,
  /// this parameter is not used.
  /// @param[in] tol the "thickness" of the support plane. Any point v which
  /// satisfies `max_{x in shape}(x.dot(dir)) - v.dot(dir) <= tol` is tol
  /// distant from the support plane and is added to the support set.
  typedef void (*SupportSetFunction)(const ShapeBase* shape,
                                     SupportSet& support_set, int& hint,
                                     ShapeSupportData& support_data,
                                     size_t num_sampled_supports, FCL_REAL tol);

  /// @brief Number of vectors to pre-allocate in the `m_clipping_sets` vectors.
  static constexpr size_t default_num_preallocated_supports = 16;

  /// @brief Number of points of the contact patch to keep once the
  /// Sutherland-Hodgman algo has run.
  /// See @ref ContactPatchRequest::m_max_patch_size for more details.
  size_t max_patch_size;

  /// @brief Number of points sampled for Cone and Cylinder when the normal is
  /// orthogonal to the shapes' basis.
  /// See @ref ContactPatchRequest::m_num_samples_curved_shapes for more
  /// details.
  size_t num_samples_curved_shapes;

  /// @brief Tolerance below which points are added to the shapes support sets.
  /// See @ref ContactPatchRequest::m_patch_tolerance for more details.
  FCL_REAL patch_tolerance;

  /// @brief Support set function for shape s1.
  mutable SupportSetFunction supportFuncShape1;

  /// @brief Support set function for shape s2.
  mutable SupportSetFunction supportFuncShape2;

  /// @brief Temporary data to compute the support sets on each shape.
  mutable std::array<ShapeSupportData, 2> supports_data;

  /// @brief Guess for the support sets computation.
  mutable support_func_guess_t support_guess;

 protected:
  /// @brief Support sets used for internal computation.
  /// @note The `computePatch` algorithm starts by constructing two 2D
  /// convex-hulls (the convex-hulls of the `m_projected_shapes_supports`). It
  /// then uses the first convex-hull to clip the second one, effectively
  /// computing the intersection between the two convex-hulls.
  /// Why have 3 support sets then? Because the algorithm works by
  /// successively clipping the first conve-hull. So the first two support
  /// sets represent the current and previous iteration of the algorithm and
  /// the third set represents the convex-hull of the second shape's support
  /// set.
  mutable std::array<SupportSet, 3> m_clipping_sets;

  /// @brief Tracks the current iterate of the algorithm.
  mutable size_t m_id_current{0};

  /// @brief Tracks which point of the Sutherland-Hodgman result have been added
  /// to the contact patch. Only used if the post-processing step occurs, i.e.
  /// if the result of Sutherland-Hodgman has a size bigger than
  /// `max_patch_size`.
  mutable std::vector<bool> m_added_to_patch;

 public:
  /// @brief Default constructor.
  explicit ContactPatchSolver() {
    const size_t num_contact_patch = 1;
    const size_t preallocated_patch_size =
        ContactPatch::default_preallocated_size;
    const FCL_REAL patch_tolerance = 1e-3;
    const ContactPatchRequest request(num_contact_patch,
                                      preallocated_patch_size,
                                      preallocated_patch_size, patch_tolerance);
    this->set(request);
  }

  /// @brief Construct the solver with a `ContactPatchRequest`.
  explicit ContactPatchSolver(const ContactPatchRequest& request) {
    this->set(request);
  }

  /// @brief Set up the solver using a `ContactPatchRequest`.
  void set(const ContactPatchRequest& request);

  /// @brief Sets the support guess used during support set computation of
  /// shapes s1 and s2.
  void setSupportGuess(const support_func_guess_t guess) const {
    this->support_guess = guess;
  }

  /// @brief Main API of the solver: compute a contact patch from a contact
  /// between shapes s1 and s2.
  /// The contact patch is the (triple) intersection between the separating
  /// plane passing (by `contact.pos` and supported by `contact.normal`) and the
  /// shapes s1 and s2.
  template <typename ShapeType1, typename ShapeType2>
  void computePatch(const ShapeType1& s1, const Transform3f& tf1,
                    const ShapeType2& s2, const Transform3f& tf2,
                    const Contact& contact, ContactPatch& contact_patch) const;

  /// @brief Reset the internal quantities of the solver.
  template <typename ShapeType1, typename ShapeType2>
  void reset(const ShapeType1& shape1, const Transform3f& tf1,
             const ShapeType2& shape2, const Transform3f& tf2,
             const ContactPatch& contact_patch) const;

  /// @brief Getter for current iterate.
  SupportSet& current() { return this->m_clipping_sets[this->m_id_current]; }

  /// @brief Const getter for current iterate.
  const SupportSet& current() const {
    return this->m_clipping_sets[this->m_id_current];
  }

  /// @brief Getter for previous iterate.
  SupportSet& previous() {
    return this->m_clipping_sets[1 - this->m_id_current];
  }

  /// @brief Const getter for previous iterate.
  const SupportSet& previous() const {
    return this->m_clipping_sets[1 - this->m_id_current];
  }

  /// @brief Getter for the set used to clip the other one.
  SupportSet& clipper() { return this->m_clipping_sets[2]; }

  /// @brief Const getter for the set used to clip the other one.
  const SupportSet& clipper() const { return this->m_clipping_sets[2]; }

  /// @brief Retrieve result from `this->current()`.
  void getResult(ContactPatch& contact_patch) const;

  /// @return true if p inside a clipping region defined by a and b, false
  /// otherwise.
  /// @param p point to check
  /// @param a, b the vertices forming the edge of the clipping region.
  /// @note the clipping ray points from a to b. Points on the right of the ray
  /// are outside the clipping region; points on the left are inside.
  static bool pointIsInsideClippingRegion(const Vec2f& p, const Vec2f& a,
                                          const Vec2f& b);

  /// @return the intersecting point between line defined by ray (a, b) and
  /// the segment [c, d].
  /// @note we make the following hypothesis:
  /// 1) c != d (should be when creating initial polytopes)
  /// 2) (c, d) is not parallel to ray -> if so, we return d.
  static Vec2f computeLineSegmentIntersection(const Vec2f& a, const Vec2f& b,
                                              const Vec2f& c, const Vec2f& d);

  /// @brief Construct support set function for shape.
  static SupportSetFunction makeSupportSetFunction(
      const ShapeBase* shape, ShapeSupportData& support_data);

  bool operator==(const ContactPatchSolver& other) const {
    return this->max_patch_size ==
               other.max_patch_size &&  // use gjk_initial_guess instead
           this->num_samples_curved_shapes == other.num_samples_curved_shapes &&
           this->patch_tolerance == other.patch_tolerance &&
           this->support_guess == other.support_guess &&
           this->current() == other.current() &&
           this->previous() == other.previous() &&
           this->clipper() == other.clipper() &&
           this->supportFuncShape1 == other.supportFuncShape1 &&
           this->supportFuncShape2 == other.supportFuncShape2;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fcl
}  // namespace hpp

#include "hpp/fcl/contact_patch/contact_patch_solver.hxx"

#endif  // HPP_FCL_CONTACT_PATCH_SOLVER_H
