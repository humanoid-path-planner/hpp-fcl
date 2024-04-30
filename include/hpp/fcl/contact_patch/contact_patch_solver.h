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
struct HPP_FCL_DLLAPI ContactPatchSolver {
 public:
  // Note: `ContactPatch` is an alias for `SupportSet`.
  // The two can be used interchangeably.
  using Index = SupportSet::Index;
  using ReferenceFrame = SupportSet::ReferenceFrame;
  using ShapeSupportData = details::ShapeSupportData;

  /// @brief Support set function for shape si, expressed in the reference frame
  /// c.
  /// @param[in] shape the shape.
  /// @param[in] dir support direction, expressed in the frame c.
  /// @param[in] ctfi transform from shape si to frame c.
  /// @param[out] projected_support_set a support set in the direction dir,
  /// @param[in] hint for the support computation of ConvexBase shapes.
  /// @param[in] support_data for the support computation of ConvexBase shapes.
  /// projected onto the plane supported by the z-axis of ctfi and passing by
  /// the origin of ctf1. All the points in this ouput set are expressed in the
  /// frame c.
  typedef void (*SupportSetFunction)(const ShapeBase* shape, const Vec3f& dir,
                                     const Transform3f& ctfi,
                                     SupportSet& projected_support_set,
                                     const int hint,
                                     ShapeSupportData* support_data);

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
  mutable std::array<SupportSet, 2> m_projected_shapes_supports;

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
  mutable std::array<SupportSet, 3> m_support_sets;

  /// @brief Tracks the current iterate of the algorithm.
  mutable size_t m_id_current{0};

  /// @brief Transform from shape1 contact frame.
  mutable Transform3f m_ctf1;

  /// @brief Transform from shape2 contact frame.
  mutable Transform3f m_ctf2;

  /// @brief Temporary data to compute the support sets on each shape.
  mutable std::array<ShapeSupportData, 2> m_supports_data;

  /// @brief Support set function for shape s1.
  mutable SupportSetFunction m_supportFuncShape1;

  /// @brief Support set function for shape s2.
  mutable SupportSetFunction m_supportFuncShape2;

  /// @brief Pointers to shapes s1 and s2.
  mutable const ShapeBase* m_shapes[2];

  /// @brief Guess for the support sets computation.
  mutable support_func_guess_t m_support_guess;

 public:
  /// @brief Number of vectors to pre-allocate in the `shapes_supports`
  /// vectors.
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

  /// @brief Sets the support guess used during support set computation of
  /// shapes s1 and s2.
  void setSupportGuess(const support_func_guess_t guess) const {
    this->m_support_guess = guess;
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

  /// @brief Compute support set of shape s1.
  void computeSupportSetShape1(SupportSet& projected_support_set) const {
    // Note: the support direction must be expressed in the frame of the support
    // set with which `reset` was called. Because of that, the support direction
    // is always (0, 0, 1), which corresponds to the normal of the
    // output contact patch, expressed in the frame of the contact patch, i.e.
    // the z-axis.
    this->m_supportFuncShape1(
        this->m_shapes[0], Vec3f(0, 0, 1), this->m_ctf1, projected_support_set,
        this->m_support_guess[0],
        const_cast<ShapeSupportData*>(&(this->m_supports_data[0])));
  }

  /// @brief Compute support set of shape s2.
  void computeSupportSetShape2(SupportSet& projected_support_set) const {
    // See `computeSupportSetShape1` for explanation on why Vec3f(0, 0, -1).
    // The -1 comes from the fact that the support set of shape s2 is in the
    // opposite direction to the support set of shape s1.
    this->m_supportFuncShape1(
        this->m_shapes[1], Vec3f(0, 0, -1), this->m_ctf2, projected_support_set,
        this->m_support_guess[1],
        const_cast<ShapeSupportData*>(&(this->m_supports_data[1])));
  }

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

 private:
  /// @brief Reset the internal quantities of the solver.
  template <typename ShapeType1, typename ShapeType2>
  void reset(const ShapeType1& shape1, const Transform3f& tf1,
             const ShapeType2& shape2, const Transform3f& tf2,
             const ContactPatch& contact_patch) const;

  /// @brief Getter for current iterate.
  SupportSet& current() { return this->m_support_sets[this->m_id_current]; }

  /// @brief Const getter for current iterate.
  const SupportSet& current() const {
    return this->m_support_sets[this->m_id_current];
  }

  /// @brief Getter for previous iterate.
  SupportSet& previous() {
    return this->m_support_sets[1 - this->m_id_current];
  }

  /// @brief Const getter for previous iterate.
  const SupportSet& previous() const {
    return this->m_support_sets[1 - this->m_id_current];
  }

  /// @brief Getter for the set used to clip the other one.
  SupportSet& clipper() { return this->m_support_sets[2]; }

  /// @brief Const getter for the set used to clip the other one.
  const SupportSet& clipper() const { return this->m_support_sets[2]; }
};

namespace details {

/// @brief Construct support set function for shape, w.r.t reference frame c.
ContactPatchSolver::SupportSetFunction makeSupportSetFunction(
    const ShapeBase* shape, const Transform3f& ctfi,
    ShapeSupportData* support_data);

}  // namespace details

}  // namespace fcl
}  // namespace hpp

#include "hpp/fcl/contact_patch/contact_patch_solver.hxx"

#endif  // HPP_FCL_CONTACT_PATCH_SOLVER_H
