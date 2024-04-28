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

namespace hpp {
namespace fcl {

inline void ContactPatchSolver::set(const ContactPatchRequest& request) {
  // Note: it's important for the number of pre-allocated Vec3f in
  // `shapes_supports` to be larger than `request.getMaxSizeContactPatch()`
  // because we don't know in advance how many supports will be discarded to
  // form the convex-hulls of the projected shapes supports which serve as the
  // input of the Sutherland-Hodgman algorithm.
  size_t num_preallocated_supports = default_num_preallocated_supports;
  if (num_preallocated_supports < 2 * request.getMaxSizeContactPatch()) {
    num_preallocated_supports = 2 * request.getMaxSizeContactPatch();
  }

  this->m_projected_shapes_supports[0].reserve(num_preallocated_supports);
  this->m_projected_shapes_supports[1].reserve(num_preallocated_supports);

  this->m_contact_patches[0].reserve(num_preallocated_supports);
  this->m_contact_patches[1].reserve(num_preallocated_supports);
  this->m_contact_patches[2].reserve(num_preallocated_supports);
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
void ContactPatchSolver::computePatch(const ShapeType1& s1,
                                      const Transform3f& tf1,
                                      const ShapeType2& s2,
                                      const Transform3f& tf2,
                                      const Contact& contact,
                                      ContactPatch& contact_patch) const {
  // Step 1
  constructContactPatchFrame(contact, contact_patch);
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
    contact_patch.addContactPoint<ReferenceFrame::WORLD>(contact.pos);
    return;
  }

  // Step 2 - Compute support set of each shape, in the direction of
  // the contact's normal.
  // Definitions:
  //   - "current" -> contact patch that needs to be clipped by the algorithm,
  //   - "clipper" -> contact patch used to clip "clipped".
  // After this step, the current and the clipper contact patches are filled
  // with the projection of the support sets of s1 and s2 in the direction of
  // `contact.normal`.
  this->reset(s1, tf1, s2, tf2, contact_patch);
  // TODO(louis): fill `clipped` and `clipper` with convex-hulls of
  // `m_projected_shapes_supports`.
  const Vec3f support_dir(0, 0, 1);
  int hint = 0;
  ContactPatch& current = const_cast<ContactPatch&>(this->current());
  this->computeSupportSetShape1(hint, current);
  ContactPatch& clipper = const_cast<ContactPatch&>(this->clipper());
  this->computeSupportSetShape2(hint, clipper);

  //
  // Step 3 - Main loop of the algorithm: use the "clipper"
  // to clip the current contact patch. The resulting intersection is the
  // contact patch of the contact between s1 and s2.
  // Currently, to clip one patch with the other, we use the Sutherland-Hodgman
  // algorithm:
  // https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
  //
  this->m_id_current = 0;
  const Index clipper_size = (Index)(this->clipper().size());
  for (Index i = 0; i < clipper_size; ++i) {
    auto a = this->clipper().contactPoint(i);
    auto b = this->clipper().contactPoint((i + 1) % clipper_size);

    this->m_id_current = 1 - this->m_id_current;
    ContactPatch& current = const_cast<ContactPatch&>(this->current());
    current.reset();
    // TODO(louis): continue to next iteration as soon as previous has been
    // clipped twice.
    const Index previous_size = (Index)(this->previous().size());
    for (Index j = 0; j < previous_size; ++j) {
      auto vcurrent = this->previous().contactPoint(j);
      auto vnext = this->previous().contactPoint((j + 1) % previous_size);

      if (pointIsInsideClippingRegion(vcurrent, a, b)) {
        current.addContactPoint(vcurrent);
        if (!pointIsInsideClippingRegion(vnext, a, b)) {
          const ContactPoint p =
              computeLineSegmentIntersection(a, b, vcurrent, vnext);
          current.addContactPoint(p);
        }
      } else if (pointIsInsideClippingRegion(vnext, a, b)) {
        const ContactPoint p =
            computeLineSegmentIntersection(a, b, vcurrent, vnext);
        current.addContactPoint(p);
      }
    }
  }
}

// ============================================================================
inline void constructContactPatchFrame(const Contact& contact,
                                       ContactPatch& contact_patch) {
  contact_patch.penetration_depth = contact.penetration_depth;
  contact_patch.tfc.translation() = contact.pos;
  contact_patch.tfc.rotation() =
      details::constructBasisFromNormal(contact.normal);
}

// ============================================================================
template <typename ShapeType1, typename ShapeType2>
inline void ContactPatchSolver::reset(const ShapeType1& shape1,
                                      const Transform3f& tf1,
                                      const ShapeType2& shape2,
                                      const Transform3f& tf2,
                                      const ContactPatch& contact_patch) const {
  // Get the support function of each shape
  const Transform3f& tfc = contact_patch.tfc;
  this->m_shapes[0] = &shape1;
  this->m_ctf1.rotation().noalias() =
      tfc.rotation().transpose() * tf1.getRotation();
  this->m_ctf1.translation().noalias() =
      tfc.getRotation().transpose() *
      (tf1.getTranslation() - tfc.getTranslation());
  this->m_supportFuncShape1 = details::makeSupportSetFunction(
      &shape1, this->m_ctf1, &(this->m_supports_data[0]));

  this->m_shapes[1] = &shape2;
  this->m_ctf2.rotation().noalias() =
      tfc.rotation().transpose() * tf2.getRotation();
  this->m_ctf2.translation().noalias() =
      tfc.getRotation().transpose() *
      (tf2.getTranslation() - tfc.getTranslation());
  this->m_supportFuncShape2 = details::makeSupportSetFunction(
      &shape2, this->m_ctf2, &(this->m_supports_data[1]));

  // Reset internal quantities
  this->m_projected_shapes_supports[0].clear();
  this->m_projected_shapes_supports[1].clear();

  this->m_contact_patches[0].clear();
  this->m_contact_patches[1].clear();
  this->m_contact_patches[2].clear();

  this->m_id_current = 0;
}

// ==========================================================================
template <typename Vector2dLike>
inline ContactPatchSolver::ContactPoint
ContactPatchSolver::computeLineSegmentIntersection(
    const Eigen::MatrixBase<Vector2dLike>& a,
    const Eigen::MatrixBase<Vector2dLike>& b,
    const Eigen::MatrixBase<Vector2dLike>& c,
    const Eigen::MatrixBase<Vector2dLike>& d) {
  const ContactPoint ab = b - a;
  const ContactPoint n(-ab(1), ab(0));
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
template <typename Vector2dLike>
inline bool ContactPatchSolver::pointIsInsideClippingRegion(
    const Eigen::MatrixBase<Vector2dLike>& p,
    const Eigen::MatrixBase<Vector2dLike>& a,
    const Eigen::MatrixBase<Vector2dLike>& b) {
  // Note: being inside/outside the clipping zone can easily be determined by
  // looking at the sign of det(b - a, p - a). If det > 0, then (b - a, p - a)
  // forms a right sided base, i.e. p is on the right of the ray.
  // Otherwise (b - a, p - a) forms a left sided base, i.e. p is on the left of
  // the ray.
  return (b(0) - a(0)) * (p(1) - a(1)) >= (b(1) - a(1)) * (p(0) - a(0));
}

namespace details {

// ============================================================================
inline Matrix3f constructBasisFromNormal(const Vec3f& vec) {
  Matrix3f basis = Matrix3f::Zero();
  basis.col(2) = vec.normalized();
  basis.col(1) = -vec.unitOrthogonal();
  basis.col(0) = basis.col(1).cross(vec);
  return basis;
}

template <typename ShapeType>
void supportSetFunctionTpl(const ShapeBase* shape_, const Transform3f& ctfi,
                           const Vec3f& dir, const int hint,
                           ShapeSupportData* support_data,
                           ContactPatch& projected_support_set) {
  const ShapeType* shape = static_cast<const ShapeType*>(shape_);
  // getShapeSupportSet(shape, ctfi, dir, hint, support_data,
  // projected_support_set);
}

// ============================================================================
inline ContactPatchSolver::SupportSetFunction makeSupportSetFunction(
    const ShapeBase* shape, const Transform3f& ctfi,
    ShapeSupportData* support_data) {
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      return supportSetFunctionTpl<TriangleP>;
    case GEOM_BOX:
      return supportSetFunctionTpl<Box>;
    case GEOM_SPHERE:
      return supportSetFunctionTpl<Sphere>;
    case GEOM_ELLIPSOID:
      return supportSetFunctionTpl<Ellipsoid>;
    case GEOM_CAPSULE:
      return supportSetFunctionTpl<Capsule>;
    case GEOM_CONE:
      return supportSetFunctionTpl<Cone>;
    case GEOM_CYLINDER:
      return supportSetFunctionTpl<Cylinder>;
    case GEOM_CONVEX: {
      const auto* convex = static_cast<const ConvexBase*>(shape);
      if ((size_t)(convex->num_points) >
          ConvexBase::num_vertices_large_convex_threshold) {
        support_data->visited.assign(convex->num_points, false);
        return supportSetFunctionTpl<LargeConvex>;
      } else {
        return supportSetFunctionTpl<SmallConvex>;
      }
    }
    default:
      HPP_FCL_THROW_PRETTY("Unsupported geometric shape.", std::logic_error);
  }
}

}  // namespace details

}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_CONTACT_PATCH_SOLVER_HXX
