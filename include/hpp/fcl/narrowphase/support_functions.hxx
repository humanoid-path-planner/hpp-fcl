/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021-2024, INRIA
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

/** \authors Jia Pan, Florent Lamiraux, Josef Mirabel, Louis Montaut */

#ifndef HPP_FCL_SUPPORT_FUNCTIONS_HXX
#define HPP_FCL_SUPPORT_FUNCTIONS_HXX

namespace hpp {
namespace fcl {

namespace details {

// ============================================================================
#define CALL_GET_SHAPE_SUPPORT(ShapeType)                                     \
  getShapeSupport<_SupportOptions>(static_cast<const ShapeType*>(shape), dir, \
                                   support, hint, nullptr)
template <int _SupportOptions>
Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, int& hint) {
  Vec3f support;
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      CALL_GET_SHAPE_SUPPORT(TriangleP);
      break;
    case GEOM_BOX:
      CALL_GET_SHAPE_SUPPORT(Box);
      break;
    case GEOM_SPHERE:
      CALL_GET_SHAPE_SUPPORT(Sphere);
      break;
    case GEOM_ELLIPSOID:
      CALL_GET_SHAPE_SUPPORT(Ellipsoid);
      break;
    case GEOM_CAPSULE:
      CALL_GET_SHAPE_SUPPORT(Capsule);
      break;
    case GEOM_CONE:
      CALL_GET_SHAPE_SUPPORT(Cone);
      break;
    case GEOM_CYLINDER:
      CALL_GET_SHAPE_SUPPORT(Cylinder);
      break;
    case GEOM_CONVEX:
      CALL_GET_SHAPE_SUPPORT(ConvexBase);
      break;
    case GEOM_PLANE:
    case GEOM_HALFSPACE:
    default:
      support.setZero();
      ;  // nothing
  }

  return support;
}
#undef CALL_GET_SHAPE_SUPPORT

// ============================================================================
template <typename ShapeType, int _SupportOptions>
Vec3f getSupportTpl(const ShapeBase* shape_, const Vec3f& dir, int& hint) {
  const ShapeType* shape = static_cast<const ShapeType*>(shape_);
  Vec3f support;
  getShapeSupport(shape, dir, support, hint, nullptr);
  return support;
}

// ============================================================================
template <typename ShapeType, int _SupportOptions>
void getShapeSupportTpl(const ShapeBase* shape_, const Vec3f& dir,
                        Vec3f& support, int& hint, ShapeSupportData* data) {
  const ShapeType* shape = static_cast<const ShapeType*>(shape_);
  return getShapeSupport(shape, dir, support, hint, data);
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const TriangleP* triangle, const Vec3f& dir,
                     Vec3f& support, int& /*unused*/,
                     ShapeSupportData* /*data*/) {
  FCL_REAL dota = dir.dot(triangle->a);
  FCL_REAL dotb = dir.dot(triangle->b);
  FCL_REAL dotc = dir.dot(triangle->c);
  if (dota > dotb) {
    if (dotc > dota) {
      support = triangle->c;
    } else {
      support = triangle->a;
    }
  } else {
    if (dotc > dotb) {
      support = triangle->c;
    } else {
      support = triangle->b;
    }
  }

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += triangle->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Box* box, const Vec3f& dir, Vec3f& support,
                            int& /*unused*/, ShapeSupportData* /*unused*/) {
  // The inflate value is simply to make the specialized functions with box
  // have a preferred side for edge cases.
  static const FCL_REAL inflate = (dir.array() == 0).any() ? 1 + 1e-10 : 1.;
  static const FCL_REAL dummy_precision =
      Eigen::NumTraits<FCL_REAL>::dummy_precision();
  Vec3f support1 = (dir.array() > dummy_precision).select(box->halfSide, 0);
  Vec3f support2 =
      (dir.array() < -dummy_precision).select(-inflate * box->halfSide, 0);
  support.noalias() = support1 + support2;

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += box->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Sphere* sphere, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData* /*unused*/) {
  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support.noalias() =
        (sphere->radius + sphere->getSweptSphereRadius()) * dir.normalized();
  } else {
    support.setZero();
  }

  HPP_FCL_UNUSED_VARIABLE(sphere);
  HPP_FCL_UNUSED_VARIABLE(dir);
}

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Ellipsoid* ellipsoid, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData* /*unused*/) {
  FCL_REAL a2 = ellipsoid->radii[0] * ellipsoid->radii[0];
  FCL_REAL b2 = ellipsoid->radii[1] * ellipsoid->radii[1];
  FCL_REAL c2 = ellipsoid->radii[2] * ellipsoid->radii[2];

  Vec3f v(a2 * dir[0], b2 * dir[1], c2 * dir[2]);

  FCL_REAL d = std::sqrt(v.dot(dir));

  support = v / d;

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += ellipsoid->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Capsule* capsule, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData* /*unused*/) {
  static const FCL_REAL dummy_precision =
      Eigen::NumTraits<FCL_REAL>::dummy_precision();
  support.setZero();
  if (dir[2] > dummy_precision) {
    support[2] = capsule->halfLength;
  } else if (dir[2] < -dummy_precision) {
    support[2] = -capsule->halfLength;
  }

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support +=
        (capsule->radius + capsule->getSweptSphereRadius()) * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Cone* cone, const Vec3f& dir, Vec3f& support,
                     int& /*unused*/, ShapeSupportData* /*unused*/) {
  static const FCL_REAL dummy_precision =
      Eigen::NumTraits<FCL_REAL>::dummy_precision();

  // The cone radius is, for -h < z < h, (h - z) * r / (2*h)
  // The inflate value is simply to make the specialized functions with cone
  // have a preferred side for edge cases.
  static const FCL_REAL inflate = 1 + 1e-10;
  FCL_REAL h = cone->halfLength;
  FCL_REAL r = cone->radius;

  if (dir.head<2>().isZero(dummy_precision)) {
    support.head<2>().setZero();
    if (dir[2] > dummy_precision) {
      support[2] = h;
    } else {
      support[2] = -inflate * h;
    }
  } else {
    FCL_REAL zdist = dir[0] * dir[0] + dir[1] * dir[1];
    FCL_REAL len = zdist + dir[2] * dir[2];
    zdist = std::sqrt(zdist);

    if (dir[2] <= 0) {
      FCL_REAL rad = r / zdist;
      support.head<2>() = rad * dir.head<2>();
      support[2] = -h;
    } else {
      len = std::sqrt(len);
      FCL_REAL sin_a = r / std::sqrt(r * r + 4 * h * h);

      if (dir[2] > len * sin_a)
        support << 0, 0, h;
      else {
        FCL_REAL rad = r / zdist;
        support.head<2>() = rad * dir.head<2>();
        support[2] = -h;
      }
    }
  }

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += cone->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir, Vec3f& support,
                     int& /*unused*/, ShapeSupportData* /*unused*/) {
  static const FCL_REAL dummy_precision =
      Eigen::NumTraits<FCL_REAL>::dummy_precision();

  // The inflate value is simply to make the specialized functions with cylinder
  // have a preferred side for edge cases.
  static const FCL_REAL inflate = 1 + 1e-10;
  FCL_REAL half_h = cylinder->halfLength;
  FCL_REAL r = cylinder->radius;

  const bool dir_is_aligned_with_z = dir.head<2>().isZero(dummy_precision);
  if (dir_is_aligned_with_z) half_h *= inflate;

  if (dir[2] > dummy_precision) {
    support[2] = half_h;
  } else if (dir[2] < -dummy_precision) {
    support[2] = -half_h;
  } else {
    support[2] = 0;
    r *= inflate;
  }

  if (dir_is_aligned_with_z) {
    support.head<2>().setZero();
  } else {
    support.head<2>() = dir.head<2>().normalized() * r;
  }

  assert(fabs(support[0] * dir[1] - support[1] * dir[0]) <
         sqrt(std::numeric_limits<FCL_REAL>::epsilon()));

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += cylinder->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportLog(const ConvexBase* convex, const Vec3f& dir,
                        Vec3f& support, int& hint, ShapeSupportData* data) {
  assert(data != nullptr && "data is null.");
  assert(convex->neighbors != nullptr && "Convex has no neighbors.");

  // Use warm start if current support direction is distant from last support
  // direction.
  const double use_warm_start_threshold = 0.9;
  Vec3f dir_normalized = dir.normalized();
  if (!data->last_dir.isZero() && !convex->support_warm_starts.points.empty() &&
      data->last_dir.dot(dir_normalized) < use_warm_start_threshold) {
    // Change hint if last dir is too far from current dir.
    FCL_REAL maxdot = convex->support_warm_starts.points[0].dot(dir);
    hint = convex->support_warm_starts.indices[0];
    for (size_t i = 1; i < convex->support_warm_starts.points.size(); ++i) {
      FCL_REAL dot = convex->support_warm_starts.points[i].dot(dir);
      if (dot > maxdot) {
        maxdot = dot;
        hint = convex->support_warm_starts.indices[i];
      }
    }
  }
  data->last_dir = dir_normalized;

  const std::vector<Vec3f>& pts = *(convex->points);
  const std::vector<ConvexBase::Neighbors>& nn = *(convex->neighbors);

  if (hint < 0 || hint >= (int)convex->num_points) {
    hint = 0;
  }
  FCL_REAL maxdot = pts[static_cast<size_t>(hint)].dot(dir);
  std::vector<int8_t>& visited = data->visited;
  assert(data->visited.size() == convex->num_points);
  std::fill(visited.begin(), visited.end(), false);
  visited[static_cast<std::size_t>(hint)] = true;
  // When the first face is orthogonal to dir, all the dot products will be
  // equal. Yet, the neighbors must be visited.
  bool found = true;
  bool loose_check = true;
  while (found) {
    const ConvexBase::Neighbors& n = nn[static_cast<size_t>(hint)];
    found = false;
    for (int in = 0; in < n.count(); ++in) {
      const unsigned int ip = n[in];
      if (visited[ip]) continue;
      visited[ip] = true;
      const FCL_REAL dot = pts[ip].dot(dir);
      bool better = false;
      if (dot > maxdot) {
        better = true;
        loose_check = false;
      } else if (loose_check && dot == maxdot)
        better = true;
      if (better) {
        maxdot = dot;
        hint = static_cast<int>(ip);
        found = true;
      }
    }
  }

  support = pts[static_cast<size_t>(hint)];

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += convex->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportLinear(const ConvexBase* convex, const Vec3f& dir,
                           Vec3f& support, int& hint, ShapeSupportData*) {
  const std::vector<Vec3f>& pts = *(convex->points);

  hint = 0;
  FCL_REAL maxdot = pts[0].dot(dir);
  for (int i = 1; i < (int)convex->num_points; ++i) {
    FCL_REAL dot = pts[static_cast<size_t>(i)].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      hint = i;
    }
  }

  support = pts[static_cast<size_t>(hint)];

  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support += convex->getSweptSphereRadius() * dir.normalized();
  }
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const ConvexBase* convex, const Vec3f& dir, Vec3f& support,
                     int& hint, ShapeSupportData* /*unused*/) {
  // TODO add benchmark to set a proper value for switching between linear and
  // logarithmic.
  if (convex->num_points > ConvexBase::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    ShapeSupportData data;
    data.visited.assign(convex->num_points, false);
    getShapeSupportLog<_SupportOptions>(convex, dir, support, hint, &data);
  } else
    getShapeSupportLinear<_SupportOptions>(convex, dir, support, hint, nullptr);
}

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const SmallConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint, ShapeSupportData* data) {
  getShapeSupportLinear<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), dir, support, hint, data);
}

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const LargeConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint, ShapeSupportData* data) {
  getShapeSupportLog<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), dir, support, hint, data);
}

// ============================================================================
#define CALL_GET_SHAPE_SUPPORT_SET(ShapeType)                               \
  getShapeSupportSet<_SupportOptions>(static_cast<const ShapeType*>(shape), \
                                      dir, ctfi, support_set, hint, nullptr)
template <int _SupportOptions>
void getSupportSet(const ShapeBase* shape, const Vec3f& dir,
                   const Transform3f& ctfi, SupportSet& support_set,
                   const int hint) {
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      CALL_GET_SHAPE_SUPPORT_SET(TriangleP);
      break;
    case GEOM_BOX:
      CALL_GET_SHAPE_SUPPORT_SET(Box);
      break;
    case GEOM_SPHERE:
      CALL_GET_SHAPE_SUPPORT_SET(Sphere);
      break;
    case GEOM_ELLIPSOID:
      CALL_GET_SHAPE_SUPPORT_SET(Ellipsoid);
      break;
    case GEOM_CAPSULE:
      CALL_GET_SHAPE_SUPPORT_SET(Capsule);
      break;
    case GEOM_CONE:
      CALL_GET_SHAPE_SUPPORT_SET(Cone);
      break;
    case GEOM_CYLINDER:
      CALL_GET_SHAPE_SUPPORT_SET(Cylinder);
      break;
    case GEOM_CONVEX:
      CALL_GET_SHAPE_SUPPORT_SET(ConvexBase);
      break;
    case GEOM_PLANE:
    case GEOM_HALFSPACE:
    default:;  // nothing
  }
}
#undef CALL_GET_SHAPE_SUPPORT

// ============================================================================
template <typename ShapeType, int _SupportOptions>
void getSupportSetTpl(const ShapeBase* shape_, const Vec3f& dir,
                      const Transform3f& ctfi, SupportSet& support_set,
                      const int hint) {
  const ShapeType* shape = static_cast<const ShapeType*>(shape_);
  getShapeSupportSet<_SupportOptions>(shape, dir, ctfi, support_set, hint,
                                      nullptr);
}

// ============================================================================
template <typename ShapeType, int _SupportOptions>
void getShapeSupportSetTpl(const ShapeBase* shape_, const Vec3f& dir,
                           const Transform3f& ctfi, SupportSet& support_set,
                           const int hint, ShapeSupportData* data) {
  const ShapeType* shape = static_cast<const ShapeType*>(shape_);
  getShapeSupportSet<_SupportOptions>(shape, dir, ctfi, support_set, hint,
                                      data);
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const TriangleP* triangle, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Box* box, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Sphere* sphere, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Ellipsoid* ellipsoid, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Capsule* capsule, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Cone* cone, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Cylinder* cylinder, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int /*unused*/, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSetLog(const ConvexBase* convex, const Vec3f& dir,
                           const Transform3f& ctfi, SupportSet& support_set,
                           const int hint, ShapeSupportData* data) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSetLinear(const ConvexBase* convex, const Vec3f& dir,
                              const Transform3f& ctfi, SupportSet& support_set,
                              const int hint, ShapeSupportData* /*unused*/) {}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const ConvexBase* convex, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int hint, ShapeSupportData* /*unused*/) {
  if (convex->num_points > ConvexBase::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    ShapeSupportData data;
    data.visited.assign(convex->num_points, false);
    getShapeSupportSetLog<_SupportOptions>(convex, dir, ctfi, support_set, hint,
                                           &data);
  } else {
    getShapeSupportSetLinear<_SupportOptions>(convex, dir, ctfi, support_set,
                                              hint, nullptr);
  }
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const SmallConvex* convex, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int hint, ShapeSupportData* data) {
  getShapeSupportSetLinear<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), dir, ctfi, support_set, hint,
      data);
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const LargeConvex* convex, const Vec3f& dir,
                        const Transform3f& ctfi, SupportSet& support_set,
                        const int hint, ShapeSupportData* data) {
  getShapeSupportSetLog<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), dir, ctfi, support_set, hint,
      data);
}

}  // namespace details

}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_SUPPORT_FUNCTIONS_HXX
