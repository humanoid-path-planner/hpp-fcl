/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Center National de la Recherche Scientifique
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

/** \author Florent Lamiraux */

#include "coal/math/transform.h"
#include "coal/shape/geometric_shapes.h"

#include "coal/internal/shape_shape_func.h"
#include "../narrowphase/details.h"

namespace coal {
struct GJKSolver;

namespace internal {
template <>
FCL_REAL ShapeShapeDistance<Cylinder, Halfspace>(
    const CollisionGeometry* o1, const Transform3f& tf1,
    const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver*,
    const bool, Vec3f& p1, Vec3f& p2, Vec3f& normal) {
  const Cylinder& s1 = static_cast<const Cylinder&>(*o1);
  const Halfspace& s2 = static_cast<const Halfspace&>(*o2);
  const FCL_REAL distance =
      details::halfspaceDistance(s2, tf2, s1, tf1, p2, p1, normal);
  normal = -normal;
  return distance;
}

template <>
FCL_REAL ShapeShapeDistance<Halfspace, Cylinder>(
    const CollisionGeometry* o1, const Transform3f& tf1,
    const CollisionGeometry* o2, const Transform3f& tf2, const GJKSolver*,
    const bool, Vec3f& p1, Vec3f& p2, Vec3f& normal) {
  const Halfspace& s1 = static_cast<const Halfspace&>(*o1);
  const Cylinder& s2 = static_cast<const Cylinder&>(*o2);
  return details::halfspaceDistance(s1, tf1, s2, tf2, p1, p2, normal);
}
}  // namespace internal

}  // namespace coal
