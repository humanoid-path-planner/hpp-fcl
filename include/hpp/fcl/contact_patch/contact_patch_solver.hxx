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

#ifndef HPP_FCL_CONTACT_PATCH_SOLVER_HXX
#define HPP_FCL_CONTACT_PATCH_SOLVER_HXX

#include "hpp/fcl/data_types.h"
#include "hpp/fcl/shape/geometric_shapes_traits.h"

namespace hpp {
namespace fcl {

// ============================================================================
inline void ContactPatchSolver::set(const ContactPatchRequest& request) {
  // Note: it's important for the number of pre-allocated Vec3f in
  // `m_clipping_sets` to be larger than `request.max_size_patch`
  // because we don't know in advance how many supports will be discarded to
  // form the convex-hulls of the shapes supports which will serve as the
  // input of the Sutherland-Hodgman algorithm.
  size_t num_preallocated_supports = default_num_preallocated_supports;
  if (num_preallocated_supports < 2 * request.getNumSamplesCurvedShapes()) {
    num_preallocated_supports = 2 * request.getNumSamplesCurvedShapes();
  }

  // Used for support set computation of shape1 and for the first iterate of the
  // Sutherland-Hodgman algo.
  this->support_set_shape1.points().reserve(num_preallocated_supports);
  this->support_set_shape1.direction = SupportSetDirection::DEFAULT;

  // Used for computing the next iterate of the Sutherland-Hodgman algo.
  this->support_set_buffer.points().reserve(num_preallocated_supports);

  // Used for support set computation of shape2 and acts as the "clipper" set in
  // the Sutherland-Hodgman algo.
  this->support_set_shape2.points().reserve(num_preallocated_supports);
  this->support_set_shape2.direction = SupportSetDirection::INVERTED;

  this->max_patch_size = request.getMaxPatchSize();
  this->num_samples_curved_shapes = request.getNumSamplesCurvedShapes();
  this->patch_tolerance = request.getPatchTolerance();
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
void ContactPatchSolver::computePatch(const ShapeType1& s1,
                                      const Transform3f& tf1,
                                      const ShapeType2& s2,
                                      const Transform3f& tf2,
                                      const Contact& contact,
                                      ContactPatch& contact_patch) const {
  // Note: `ContactPatch` is an alias for `SupportSet`.
  // Step 1
  constructContactPatchFrameFromContact(contact, contact_patch);
  contact_patch.points().clear();
  if ((bool)(shape_traits<ShapeType1>::IsStrictlyConvex) ||
      (bool)(shape_traits<ShapeType2>::IsStrictlyConvex)) {
    // If a shape is strictly convex, the support set in any direction is
    // reduced to a single point. Thus, the contact point `contact.pos` is the
    // only point belonging to the contact patch, and it has already been
    // computed.
    // TODO(louis): even for strictly convex shapes, we can sample the support
    // function around the normal and return a pseudo support set. This would
    // allow spheres and ellipsoids to have a contact surface, which does make
    // sense in certain physics simulation cases.
    // Do the same for strictly convex regions of non-strictly convex shapes
    // like the ends of capsules.
    contact_patch.addPoint(contact.pos);
    return;
  }

  // Step 2 - Compute support set of each shape, in the direction of
  // the contact's normal.
  // The first shape's support set is called "current"; it will be the first
  // iterate of the Sutherland-Hodgman algorithm. The second shape's support set
  // is called "clipper"; it will be used to clip "current". The support set
  // computation step computes a convex polygon; its vertices are ordered
  // counter-clockwise. This is important as the Sutherland-Hodgman algorithm
  // expects points to be ranked counter-clockwise.
  this->reset(s1, tf1, s2, tf2, contact_patch);
  assert(this->num_samples_curved_shapes > 3);

  this->supportFuncShape1(&s1, this->support_set_shape1, this->support_guess[0],
                          this->supports_data[0],
                          this->num_samples_curved_shapes,
                          this->patch_tolerance);

  this->supportFuncShape2(&s2, this->support_set_shape2, this->support_guess[1],
                          this->supports_data[1],
                          this->num_samples_curved_shapes,
                          this->patch_tolerance);

  // We can immediatly return if one of the support set has only
  // one point.
  if (this->support_set_shape1.size() <= 1 ||
      this->support_set_shape2.size() <= 1) {
    contact_patch.addPoint(contact.pos);
    return;
  }

  if ((this->support_set_shape1.size() == 2) &&
      (this->support_set_shape2.size() == 2)) {
    // Segment-Segment case
    // We compute the determinant; if it is non-zero, the intersection
    // has already been computed: it's `Contact::pos`.
    const SupportSet::Polygon& pts1 = this->support_set_shape1.points();
    const Vec2f& a = pts1[0];
    const Vec2f& b = pts1[1];

    const SupportSet::Polygon& pts2 = this->support_set_shape2.points();
    const Vec2f& c = pts1[0];
    const Vec2f& d = pts2[1];

    const FCL_REAL det =
        (b(0) - a(0)) * (d(1) - c(1)) >= (b(1) - a(1)) * (c(0) - d(0));
    static constexpr FCL_REAL eps =
        Eigen::NumTraits<FCL_REAL>::dummy_precision();
    if ((std::abs(det) > eps) || ((c - d).squaredNorm() < eps) ||
        ((b - a).squaredNorm() < eps)) {
      contact_patch.addPoint(contact.pos);
      return;
    }

    const Vec2f cd = (d - c);
    const FCL_REAL l = cd.squaredNorm();
    ContactPatch::Polygon& patch = contact_patch.points();

    // Project a onto [c, d]
    FCL_REAL t1 = (a - c).dot(cd);
    t1 = (t1 >= l) ? 1.0 : ((t1 <= 0) ? 0.0 : (t1 / l));
    const Vec2f p1 = c + t1 * cd;
    patch.emplace_back(p1);

    // Project b onto [c, d]
    FCL_REAL t2 = (b - c).dot(cd);
    t2 = (t2 >= l) ? 1.0 : ((t2 <= 0) ? 0.0 : (t2 / l));
    const Vec2f p2 = c + t2 * cd;
    if ((p1 - p2).squaredNorm() >= eps) {
      patch.emplace_back(p2);
    }
    return;
  }

  //
  // Step 3 - Main loop of the algorithm: use the "clipper"
  // to clip the current contact patch. The resulting intersection is the
  // contact patch of the contact between s1 and s2.
  // Currently, to clip one patch with the other, we use the
  // Sutherland-Hodgman algorithm:
  // https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
  //
  // The Sutherland-Hodgman algorithm needs the clipper to be at least a
  // triangle.
  SupportSet::Polygon* clipper_ptr = nullptr;
  SupportSet::Polygon* current_ptr = nullptr;
  SupportSet::Polygon* previous_ptr = &(this->support_set_buffer.points());
  if (support_set_shape2.size() == 2) {
    // Use the first shape as clipper, second as clipped.
    clipper_ptr = &(this->support_set_shape1.points());
    current_ptr = &(this->support_set_shape2.points());
  } else {
    // Use the second shape as clipper, first as clipped.
    clipper_ptr = &(this->support_set_shape2.points());
    current_ptr = &(this->support_set_shape1.points());
  }

  const SupportSet::Polygon& clipper = *(clipper_ptr);
  const size_t clipper_size = clipper.size();

  for (size_t i = 0; i < clipper_size; ++i) {
    const Vec2f a = clipper[i];
    const Vec2f b = clipper[(i + 1) % clipper_size];

    // Swap current/previous
    SupportSet::Polygon* tmp_ptr = previous_ptr;
    previous_ptr = current_ptr;
    current_ptr = tmp_ptr;

    const SupportSet::Polygon& previous = *(previous_ptr);
    SupportSet::Polygon& current = *(current_ptr);
    current.clear();
    const size_t previous_size = previous.size();
    for (size_t j = 0; j < previous_size; ++j) {
      const Vec2f vcurrent = previous[j];
      const Vec2f vnext = previous[(j + 1) % previous_size];
      if (pointIsInsideClippingRegion(vcurrent, a, b)) {
        current.emplace_back(vcurrent);
        if (!pointIsInsideClippingRegion(vnext, a, b)) {
          const Vec2f p = computeLineSegmentIntersection(a, b, vcurrent, vnext);
          current.emplace_back(p);
        }
      } else if (pointIsInsideClippingRegion(vnext, a, b)) {
        const Vec2f p = computeLineSegmentIntersection(a, b, vcurrent, vnext);
        current.emplace_back(p);
      }
    }
    if (current.size() == 0) {
      // No intersection found, the algo can early stop.
      break;
    }
  }

  if (current_ptr->size() <= 1) {
    contact_patch.addPoint(contact.pos);
    return;
  }

  this->getResult(current_ptr, contact_patch);
}

// ============================================================================
inline void ContactPatchSolver::getResult(
    const ContactPatch::Polygon* result_ptr,
    ContactPatch& contact_patch) const {
  assert(this->max_patch_size > 3);
  const ContactPatch::Polygon& result = *(result_ptr);
  ContactPatch::Polygon& patch = contact_patch.points();

  if (result.size() <= this->max_patch_size) {
    patch.emplace_back(result[0]);
    for (size_t i = 1; i < result.size(); ++i) {
      if ((patch.back() - result[i]).squaredNorm() >
          Eigen::NumTraits<FCL_REAL>::dummy_precision()) {
        patch.emplace_back(result[i]);
      }
    }
    return;
  }

  // Post-processing step to select `max_patch_size` points of the computed
  // contact patch.
  // We simply select `max_patch_size` points of the patch by sampling the
  // 2d support function of the patch along the unit circle.
  this->added_to_patch.assign(result.size(), false);
  const FCL_REAL angle_increment =
      2.0 * (FCL_REAL)(EIGEN_PI) / ((FCL_REAL)(this->max_patch_size));
  for (size_t i = 0; i < this->max_patch_size; ++i) {
    const FCL_REAL theta = (FCL_REAL)(i)*angle_increment;
    const Vec2f dir(std::cos(theta), std::sin(theta));
    FCL_REAL support_val = result[0].dot(dir);
    size_t support_idx = 0;
    for (size_t j = 1; j < result.size(); ++j) {
      const FCL_REAL val = result[j].dot(dir);
      if (val > support_val) {
        support_val = val;
        support_idx = j;
      }
    }
    if (!this->added_to_patch[support_idx]) {
      if (patch.empty()) {
        patch.emplace_back(result[support_idx]);
      } else {
        if ((patch.back() - result[support_idx]).squaredNorm() >
            Eigen::NumTraits<FCL_REAL>::dummy_precision()) {
          patch.emplace_back(result[support_idx]);
        }
      }
      this->added_to_patch[support_idx] = true;
    }
  }
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
inline void ContactPatchSolver::reset(const ShapeType1& shape1,
                                      const Transform3f& tf1,
                                      const ShapeType2& shape2,
                                      const Transform3f& tf2,
                                      const ContactPatch& contact_patch) const {
  // Reset internal quantities
  this->support_set_shape1.clear();
  this->support_set_shape2.clear();
  this->support_set_buffer.clear();

  // Get the support function of each shape
  const Transform3f& tfc = contact_patch.tf;

  this->support_set_shape1.direction = SupportSetDirection::DEFAULT;
  // Set the reference frame of the support set of the first shape to be the
  // local frame of shape 1.
  Transform3f& tf1c = this->support_set_shape1.tf;
  tf1c.rotation().noalias() = tf1.rotation().transpose() * tfc.rotation();
  tf1c.translation().noalias() =
      tf1.rotation().transpose() * (tfc.translation() - tf1.translation());
  this->supportFuncShape1 =
      this->makeSupportSetFunction(&shape1, this->supports_data[0]);

  this->support_set_shape2.direction = SupportSetDirection::INVERTED;
  // Set the reference frame of the support set of the second shape to be the
  // local frame of shape 2.
  Transform3f& tf2c = this->support_set_shape2.tf;
  tf2c.rotation().noalias() = tf2.rotation().transpose() * tfc.rotation();
  tf2c.translation().noalias() =
      tf2.rotation().transpose() * (tfc.translation() - tf2.translation());
  this->supportFuncShape2 =
      this->makeSupportSetFunction(&shape2, this->supports_data[1]);
}

// ==========================================================================
inline Vec2f ContactPatchSolver::computeLineSegmentIntersection(
    const Vec2f& a, const Vec2f& b, const Vec2f& c, const Vec2f& d) {
  const Vec2f ab = b - a;
  const Vec2f n(-ab(1), ab(0));
  const FCL_REAL denominator = n.dot(c - d);
  if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
    return d;
  }
  const FCL_REAL nominator = n.dot(a - d);
  FCL_REAL alpha = nominator / denominator;
  alpha = std::min<double>(1.0, std::max<FCL_REAL>(0.0, alpha));
  return alpha * c + (1 - alpha) * d;
}

// ==========================================================================
inline bool ContactPatchSolver::pointIsInsideClippingRegion(const Vec2f& p,
                                                            const Vec2f& a,
                                                            const Vec2f& b) {
  // Note: being inside/outside the clipping zone can easily be determined by
  // looking at the sign of det(b - a, p - a). If det > 0, then (b - a, p - a)
  // forms a right sided base, i.e. p is on the right of the ray.
  // Otherwise (b - a, p - a) forms a left sided base, i.e. p is on the left of
  // the ray.
  return (b(0) - a(0)) * (p(1) - a(1)) >= (b(1) - a(1)) * (p(0) - a(0));
}

}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_CONTACT_PATCH_SOLVER_HXX
