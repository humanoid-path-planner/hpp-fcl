/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2019, CNRS
 *  Author: Florent Lamiraux
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

#include "coal/math/transform.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/shape_shape_func.h"
#include "coal/internal/traversal_node_base.h"

#include "../narrowphase/details.h"

// Note that partial specialization of template functions is not allowed.
// Therefore, two implementations with the default narrow phase solvers are
// provided. If another narrow phase solver were to be used, the default
// template implementation would be called, unless the function is also
// specialized for this new type.
//
// One solution would be to make narrow phase solvers derive from an abstract
// class and specialize the template for this abstract class.
namespace coal {
struct GJKSolver;

namespace internal {
template <>
CoalScalar ShapeShapeDistance<Sphere, Sphere>(
    const CollisionGeometry* o1, const Transform3s& tf1,
    const CollisionGeometry* o2, const Transform3s& tf2, const GJKSolver*,
    const bool, Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  const Sphere& s1 = static_cast<const Sphere&>(*o1);
  const Sphere& s2 = static_cast<const Sphere&>(*o2);
  return details::sphereSphereDistance(s1, tf1, s2, tf2, p1, p2, normal);
}
}  // namespace internal

}  // namespace coal
