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
  // `m_shapes_support_sets` to be larger than `request.max_size_patch`
  // because we don't know in advance how many supports will be discarded to
  // form the convex-hulls of the shapes supports which will serve as the
  // input of the Sutherland-Hodgman algorithm.
  size_t num_preallocated_supports = default_num_preallocated_supports;
  if (num_preallocated_supports < 2 * request.max_size_patch) {
    num_preallocated_supports = 2 * request.max_size_patch;
  }

  // Used for support set computation of shape1 and for the first iterate of the
  // Sutherland-Hodgman algo.
  this->m_clipping_sets[0].points().reserve(num_preallocated_supports);
  this->m_clipping_sets[0].direction = SupportSetDirection::DEFAULT;

  // Used for computing the next iterate of the Sutherland-Hodgman algo.
  this->m_clipping_sets[1].points().reserve(num_preallocated_supports);

  // Used for support set computation of shape2 and acts as the "clipper" set in
  // the Sutherland-Hodgman algo.
  this->m_clipping_sets[2].points().reserve(num_preallocated_supports);
  this->m_clipping_sets[2].direction = SupportSetDirection::INVERTED;

  this->max_size_patch = request.max_size_patch;
  this->patch_tolerance = request.patch_tolerance;
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
    contact_patch.addPoint<ReferenceFrame::WORLD>(contact.pos);
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
  SupportSet& current = const_cast<SupportSet&>(this->current());
  this->m_supportFuncShape1(&s1, current, this->m_support_guess[0],
                            &(this->m_supports_data[0]), this->max_size_patch,
                            this->patch_tolerance);
  SupportSet& clipper = const_cast<SupportSet&>(this->clipper());
  this->m_supportFuncShape2(&s2, clipper, this->m_support_guess[1],
                            &(this->m_supports_data[1]), this->max_size_patch,
                            this->patch_tolerance);

  //
  // Step 4 - Main loop of the algorithm: use the "clipper"
  // to clip the current contact patch. The resulting intersection is the
  // contact patch of the contact between s1 and s2.
  // Currently, to clip one patch with the other, we use the Sutherland-Hodgman
  // algorithm:
  // https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
  //
  this->m_id_current = 0;
  const size_t clipper_size = this->clipper().points().size();
  for (size_t i = 0; i < clipper_size; ++i) {
    const Vec2f a = this->clipper().point(i);
    const Vec2f b = this->clipper().point((i + 1) % clipper_size);

    this->m_id_current = 1 - this->m_id_current;
    ContactPatch& current = const_cast<ContactPatch&>(this->current());
    current.points().clear();
    // TODO(louis): continue to next iteration as soon as previous has been
    // clipped twice.
    const size_t previous_size = this->previous().points().size();
    for (size_t j = 0; j < previous_size; ++j) {
      const Vec2f vcurrent = this->previous().point(j);
      const Vec2f vnext = this->previous().point((j + 1) % previous_size);
      if (pointIsInsideClippingRegion(vcurrent, a, b)) {
        current.points().emplace_back(vcurrent);
        if (!pointIsInsideClippingRegion(vnext, a, b)) {
          const Vec2f p = computeLineSegmentIntersection(a, b, vcurrent, vnext);
          current.points().emplace_back(p);
        }
      } else if (pointIsInsideClippingRegion(vnext, a, b)) {
        const Vec2f p = computeLineSegmentIntersection(a, b, vcurrent, vnext);
        current.points().emplace_back(p);
      }
    }
  }

  // TODO(louis): retrieve the result from current into contact_patch.
  // If current has more points than the request, take request.max_size number
  // of supports in the directions i/2*pi direction of a 2D unit circle.
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
inline void ContactPatchSolver::reset(const ShapeType1& shape1,
                                      const Transform3f& tf1,
                                      const ShapeType2& shape2,
                                      const Transform3f& tf2,
                                      const ContactPatch& contact_patch) const {
  // Reset internal quantities
  this->m_clipping_sets[0].clear();
  this->m_clipping_sets[1].clear();
  this->m_clipping_sets[2].clear();

  this->m_id_current = 0;

  // Get the support function of each shape
  const Transform3f& tfc = contact_patch.tf;

  SupportSet& current = const_cast<SupportSet&>(this->current());
  current.direction = SupportSetDirection::DEFAULT;
  // Set the reference frame of the support set of the first shape to be the
  // local frame of shape 1.
  Transform3f& tf1c = current.tf;
  tf1c.rotation().noalias() = tf1.rotation().transpose() * tfc.rotation();
  tf1c.translation().noalias() =
      tf1.rotation().transpose() * (tfc.translation() - tf1.translation());
  const size_t prealoccated_size_for_cvx_hull_computation1 =
      current.points().capacity();  // Used only for ConvexBase
  this->m_supportFuncShape1 =
      this->makeSupportSetFunction(&shape1, &(this->m_supports_data[0]),
                                   prealoccated_size_for_cvx_hull_computation1);

  SupportSet& clipper = const_cast<SupportSet&>(this->clipper());
  clipper.direction = SupportSetDirection::INVERTED;
  // Set the reference frame of the support set of the second shape to be the
  // local frame of shape 2.
  Transform3f& tf2c = clipper.tf;
  tf2c.rotation().noalias() = tf2.rotation().transpose() * tfc.rotation();
  tf2c.translation().noalias() =
      tf2.rotation().transpose() * (tfc.translation() - tf2.translation());
  const size_t prealoccated_size_for_cvx_hull_computation2 =
      clipper.points().capacity();  // Used only for ConvexBase
  this->m_supportFuncShape2 =
      this->makeSupportSetFunction(&shape2, &(this->m_supports_data[1]),
                                   prealoccated_size_for_cvx_hull_computation2);
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

// ============================================================================
inline ContactPatchSolver::SupportSetFunction
ContactPatchSolver::makeSupportSetFunction(
    const ShapeBase* shape, ShapeSupportData* support_data,
    size_t support_set_size_used_to_compute_cvx_hull) {
  // Note: because the swept-sphere radius was already taken into account when
  // constructing the contact patch frame, there is actually no need to take the
  // swept-sphere radius of shapes into account. The origin of the contact patch
  // frame already encodes this information.
  using Options = details::SupportOptions;
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      return details::getShapeSupportSetTpl<TriangleP, Options::NoSweptSphere>;
    case GEOM_BOX: {
      const size_t num_corners_box = 8;
      support_data->support_set.points().reserve(num_corners_box);
      return details::getShapeSupportSetTpl<Box, Options::NoSweptSphere>;
    }
    case GEOM_SPHERE:
      return details::getShapeSupportSetTpl<Sphere, Options::NoSweptSphere>;
    case GEOM_ELLIPSOID:
      return details::getShapeSupportSetTpl<Ellipsoid, Options::NoSweptSphere>;
    case GEOM_CAPSULE:
      return details::getShapeSupportSetTpl<Capsule, Options::NoSweptSphere>;
    case GEOM_CONE:
      return details::getShapeSupportSetTpl<Cone, Options::NoSweptSphere>;
    case GEOM_CYLINDER:
      return details::getShapeSupportSetTpl<Cylinder, Options::NoSweptSphere>;
    case GEOM_CONVEX: {
      const ConvexBase* convex = static_cast<const ConvexBase*>(shape);
      if ((size_t)(convex->num_points) >
          ConvexBase::num_vertices_large_convex_threshold) {
        support_data->visited.assign(convex->num_points, false);
        support_data->support_set.points().reserve(
            support_set_size_used_to_compute_cvx_hull);
        return details::getShapeSupportSetTpl<details::LargeConvex,
                                              Options::NoSweptSphere>;
      } else {
        return details::getShapeSupportSetTpl<details::SmallConvex,
                                              Options::NoSweptSphere>;
      }
    }
    default:
      HPP_FCL_THROW_PRETTY("Unsupported geometric shape.", std::logic_error);
  }
}

}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_CONTACT_PATCH_SOLVER_HXX
