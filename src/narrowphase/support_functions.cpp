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

#include "hpp/fcl/narrowphase/support_functions.h"

namespace hpp {
namespace fcl {
namespace details {

// ============================================================================
#define CALL_GET_SHAPE_SUPPORT(ShapeType)                                     \
  getShapeSupport<_SupportOptions>(static_cast<const ShapeType*>(shape), dir, \
                                   support, hint, support_data)
template <int _SupportOptions>
Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, int& hint) {
  Vec3f support;
  ShapeSupportData support_data;
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

// Explicit instantiation
// clang-format off
template Vec3f getSupport<SupportOptions::NoSweptSphere>(const ShapeBase*, const Vec3f&, int&);

template Vec3f getSupport<SupportOptions::WithSweptSphere>(const ShapeBase*, const Vec3f&, int&);
// clang-format on

// ============================================================================
#define getShapeSupportTplInstantiation(ShapeType)                          \
  template void getShapeSupport<SupportOptions::NoSweptSphere>(             \
      const ShapeType* shape_, const Vec3f& dir, Vec3f& support, int& hint, \
      ShapeSupportData& support_data);                                      \
                                                                            \
  template void getShapeSupport<SupportOptions::WithSweptSphere>(           \
      const ShapeType* shape_, const Vec3f& dir, Vec3f& support, int& hint, \
      ShapeSupportData& support_data);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const TriangleP* triangle, const Vec3f& dir,
                     Vec3f& support, int& /*unused*/,
                     ShapeSupportData& /*unused*/) {
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
getShapeSupportTplInstantiation(TriangleP);

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Box* box, const Vec3f& dir, Vec3f& support,
                            int& /*unused*/, ShapeSupportData& /*unused*/) {
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
getShapeSupportTplInstantiation(Box);

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Sphere* sphere, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData& /*unused*/) {
  if (_SupportOptions == SupportOptions::WithSweptSphere) {
    support.noalias() =
        (sphere->radius + sphere->getSweptSphereRadius()) * dir.normalized();
  } else {
    support.setZero();
  }

  HPP_FCL_UNUSED_VARIABLE(sphere);
  HPP_FCL_UNUSED_VARIABLE(dir);
}
getShapeSupportTplInstantiation(Sphere);

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Ellipsoid* ellipsoid, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData& /*unused*/) {
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
getShapeSupportTplInstantiation(Ellipsoid);

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const Capsule* capsule, const Vec3f& dir,
                            Vec3f& support, int& /*unused*/,
                            ShapeSupportData& /*unused*/) {
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
getShapeSupportTplInstantiation(Capsule);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Cone* cone, const Vec3f& dir, Vec3f& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
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
getShapeSupportTplInstantiation(Cone);

// ============================================================================
template <int _SupportOptions>
void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir, Vec3f& support,
                     int& /*unused*/, ShapeSupportData& /*unused*/) {
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
getShapeSupportTplInstantiation(Cylinder);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportLog(const ConvexBase* convex, const Vec3f& dir,
                        Vec3f& support, int& hint,
                        ShapeSupportData& support_data) {
  assert(convex->neighbors != nullptr && "Convex has no neighbors.");

  // Use warm start if current support direction is distant from last support
  // direction.
  const double use_warm_start_threshold = 0.9;
  Vec3f dir_normalized = dir.normalized();
  if (!support_data.last_dir.isZero() &&
      !convex->support_warm_starts.points.empty() &&
      support_data.last_dir.dot(dir_normalized) < use_warm_start_threshold) {
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
  support_data.last_dir = dir_normalized;

  const std::vector<Vec3f>& pts = *(convex->points);
  const std::vector<ConvexBase::Neighbors>& nn = *(convex->neighbors);

  if (hint < 0 || hint >= (int)convex->num_points) {
    hint = 0;
  }
  FCL_REAL maxdot = pts[static_cast<size_t>(hint)].dot(dir);
  std::vector<int8_t>& visited = support_data.visited;
  if (support_data.visited.size() == convex->num_points) {
    std::fill(visited.begin(), visited.end(), false);
  } else {
    // std::vector::assign not only assigns the values of the vector but also
    // resizes the vector. So if `visited` has not been set up yet, this makes
    // sure the size convex's points and visited are identical.
    support_data.visited.assign(convex->num_points, false);
  }
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
                           Vec3f& support, int& hint,
                           ShapeSupportData& /*unused*/) {
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
                     int& hint, ShapeSupportData& support_data) {
  // TODO add benchmark to set a proper value for switching between linear and
  // logarithmic.
  if (convex->num_points > ConvexBase::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    getShapeSupportLog<_SupportOptions>(convex, dir, support, hint,
                                        support_data);
  } else {
    getShapeSupportLinear<_SupportOptions>(convex, dir, support, hint,
                                           support_data);
  }
}
getShapeSupportTplInstantiation(ConvexBase);

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const SmallConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint,
                            ShapeSupportData& support_data) {
  getShapeSupportLinear<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), dir, support, hint,
      support_data);
}
getShapeSupportTplInstantiation(SmallConvex);

// ============================================================================
template <int _SupportOptions>
inline void getShapeSupport(const LargeConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint,
                            ShapeSupportData& support_data) {
  getShapeSupportLog<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), dir, support, hint,
      support_data);
}
getShapeSupportTplInstantiation(LargeConvex);

// ============================================================================
#define CALL_GET_SHAPE_SUPPORT_SET(ShapeType)                               \
  getShapeSupportSet<_SupportOptions>(static_cast<const ShapeType*>(shape), \
                                      support_set, hint, support_data,      \
                                      max_num_supports, tol)
template <int _SupportOptions>
void getSupportSet(const ShapeBase* shape, SupportSet& support_set, int& hint,
                   size_t max_num_supports, FCL_REAL tol) {
  ShapeSupportData support_data;
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

// Explicit instantiation
// clang-format off
template void getSupportSet<SupportOptions::NoSweptSphere>(const ShapeBase*, SupportSet&, int&, size_t, FCL_REAL);

template void getSupportSet<SupportOptions::WithSweptSphere>(const ShapeBase*, SupportSet&, int&, size_t, FCL_REAL);
// clang-format on

// ============================================================================
#define getShapeSupportSetTplInstantiation(ShapeType)                 \
  template void getShapeSupportSet<SupportOptions::NoSweptSphere>(    \
      const ShapeType* shape_, SupportSet& support_set, int& hint,    \
      ShapeSupportData& data, size_t max_num_supports, FCL_REAL tol); \
                                                                      \
  template void getShapeSupportSet<SupportOptions::WithSweptSphere>(  \
      const ShapeType* shape_, SupportSet& support_set, int& hint,    \
      ShapeSupportData& data, size_t max_num_supports, FCL_REAL tol);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const TriangleP* triangle, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, FCL_REAL tol) {
  assert(tol > 0);
  support_set.clear();

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  // We simply want to compute the support value, no need to take the
  // swept-sphere radius into account.
  getShapeSupport<SupportOptions::NoSweptSphere>(triangle, support_dir, support,
                                                 hint, support_data);
  const FCL_REAL support_value = support.dot(support_dir);

  if (support_value - support_dir.dot(triangle->a) < tol) {
    // Note: at the moment, it's useless to take into account the
    // swept-sphere radius, but in the future we might want to store the
    // offsets to the plane in `SupportSet`.
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      support_set.addPoint(triangle->a +
                           triangle->getSweptSphereRadius() * support_dir);
    } else {
      support_set.addPoint(triangle->a);
    }
  }
  if (support_value - support_dir.dot(triangle->b) < tol) {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      support_set.addPoint(triangle->b +
                           triangle->getSweptSphereRadius() * support_dir);
    } else {
      support_set.addPoint(triangle->b);
    }
  }
  if (support_value - support_dir.dot(triangle->c) < tol) {
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      support_set.addPoint(triangle->c +
                           triangle->getSweptSphereRadius() * support_dir);
    } else {
      support_set.addPoint(triangle->c);
    }
  }
}
getShapeSupportSetTplInstantiation(TriangleP);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Box* box, SupportSet& support_set,
                        int& hint /*unused*/, ShapeSupportData& support_data,
                        size_t /*unused*/, FCL_REAL tol) {
  assert(tol > 0);
  support_set.points().clear();
  support_data.support_set.points().clear();
  support_data.support_set.direction = support_set.direction;
  support_data.support_set.tf = support_set.tf;

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(box, support_dir, support,
                                                 hint, support_data);
  const FCL_REAL support_value = support.dot(support_dir);

  const FCL_REAL x = box->halfSide[0];
  const FCL_REAL y = box->halfSide[1];
  const FCL_REAL z = box->halfSide[2];
  const std::array<Vec3f, 8> corners = {
      Vec3f(x, y, z),  Vec3f(-x, y, z),  Vec3f(-x, -y, z),  Vec3f(x, -y, z),
      Vec3f(x, y, -z), Vec3f(-x, y, -z), Vec3f(-x, -y, -z), Vec3f(x, -y, -z),
  };

  for (const Vec3f& corner : corners) {
    const FCL_REAL val = corner.dot(support_dir);
    if (support_value - val < tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        support_data.support_set.addPoint(corner + box->getSweptSphereRadius() *
                                                       support_dir);
      } else {
        support_data.support_set.addPoint(corner);
      }
    }
  }
  computeSupportSetConvexHull(support_data, support_set);
}
getShapeSupportSetTplInstantiation(Box);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Sphere* sphere, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, FCL_REAL /*unused*/) {
  support_set.points().clear();

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<_SupportOptions>(sphere, support_dir, support, hint,
                                   support_data);
  support_set.addPoint(support);
}
getShapeSupportSetTplInstantiation(Sphere);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Ellipsoid* ellipsoid, SupportSet& support_set,
                        int& hint, ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, FCL_REAL /*unused*/) {
  support_set.points().clear();

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<_SupportOptions>(ellipsoid, support_dir, support, hint,
                                   support_data);
  support_set.addPoint(support);
}
getShapeSupportSetTplInstantiation(Ellipsoid);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Capsule* capsule, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t /*unused*/, FCL_REAL tol) {
  assert(tol > 0);
  support_set.points().clear();

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(capsule, support_dir, support,
                                                 hint, support_data);
  const FCL_REAL support_value = support.dot(support_dir);
  // A capsule can be seen as swept-sphere segment. Segment has only two points:
  // (0, 0, h) and (0, 0, -h) where h = capsule->halfLength.
  // support_dir.dot((0, 0, +-h)) = +- support_dir[2] * h
  //
  // If we denote by `n` the support direction and `OH` the vector (0, 0, h),
  // the 2 ends of the segment belong to the support set if:
  //   | <n, (OH / ||OH||)> | <= tol / ||OH||,
  // where |.| is the absolute value, <.,.> the euclidian dot product and ||.||
  // the euclidian norm.
  // We can easily see that the bigger the length of the capsule (the bigger
  // ||OH||), the bigger the angle between `n` and `OH` has to be for the 2 ends
  // of the segment to be supports (i.e. the more perpendicular `n` has to be to
  // the capsule's axis). The fact that this condition depends on the size of
  // the capsule is perfectly expected and reassuring. This means that `tol`
  // smartly selects which points belonging to the support sets, adapting to the
  // size of the geometry.
  // We will follow the exact same reasoning for cones and cylinders.
  const FCL_REAL h = capsule->halfLength;
  if (support_value - support_dir[2] * capsule->halfLength < tol) {
    Vec3f point = Vec3f(0, 0, h);
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      point += (capsule->radius + capsule->getSweptSphereRadius()) *
               support_dir.normalized();
    }
    support_set.addPoint(point);
  }
  if (support_value + support_dir[2] * capsule->halfLength < tol) {
    Vec3f point = Vec3f(0, 0, -h);
    if (_SupportOptions == SupportOptions::WithSweptSphere) {
      point += (capsule->radius + capsule->getSweptSphereRadius()) *
               support_dir.normalized();
    }
    support_set.addPoint(point);
  }
}
getShapeSupportSetTplInstantiation(Capsule);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Cone* cone, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t max_num_supports, FCL_REAL tol) {
  assert(tol > 0);
  support_set.points().clear();

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(cone, support_dir, support,
                                                 hint, support_data);
  const FCL_REAL support_value = support.dot(support_dir);

  // If the support direction is perpendicular to the cone's basis, there is an
  // infinite amount of support points; otherwise there are up to two support
  // points (two if direction is perpendicular to the side of the cone and one
  // otherwise).
  //
  // The following check follows the same idea as what is done in the Capsule
  // support set computation.
  // For all theta, we want support_value - support_dir.dot(v(theta)) <= eps,
  // with v(theta) = r * cos(theta) + r * sin(theta).
  // Knowing that support_value >= 0, this leads us to:
  // support_value - r * sqrt((nx - ny)**2 + nx * ny) <= eps,
  // with nx = support_dir[0], ny = support_dir[1].
  const FCL_REAL nx = support_dir[0];
  const FCL_REAL ny = support_dir[1];
  const FCL_REAL r = cone->radius;
  if (support_dir[2] <= 0 &&
      (support_value - r * std::sqrt((nx - ny) * (nx - ny) + nx * ny) <= tol)) {
    // If this check passed, support direction is considered perpendicular to
    // the basis of the cone. We sample `max_num_supports` points on the base of
    // the cone. We are guaranteed that these points like at a distance tol of
    // the support plane.
    const FCL_REAL angle_increment =
        2.0 * (FCL_REAL)(EIGEN_PI) / ((FCL_REAL)(max_num_supports));
    for (size_t i = 0; i < max_num_supports; ++i) {
      const FCL_REAL theta = (FCL_REAL)(i)*angle_increment;
      Vec3f point_on_cone_base(std::cos(theta) * cone->radius,
                               std::sin(theta) * cone->radius,
                               -cone->halfLength);
      assert(std::abs(support_value - point_on_cone_base.dot(support_dir)) <=
             tol);
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_cone_base += cone->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_cone_base);
    }
  } else {
    // There are two potential supports to add: the tip of the cone and a point
    // on the basis of the cone. We compare each of these points to the support
    // value.
    Vec3f cone_tip(0, 0, cone->halfLength);
    if (support_value - cone_tip.dot(support_dir) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        cone_tip += cone->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(cone_tip);
    }

    Vec3f point_on_cone_base = Vec3f(cone->radius * support_dir[0],  //
                                     cone->radius * support_dir[1],  //
                                     -cone->halfLength);
    if (support_value - point_on_cone_base.dot(support_dir) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_cone_base += cone->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_cone_base);
    }
  }
}
getShapeSupportSetTplInstantiation(Cone);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const Cylinder* cylinder, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t max_num_supports, FCL_REAL tol) {
  assert(tol > 0);
  support_set.points().clear();

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(cylinder, support_dir, support,
                                                 hint, support_data);
  const FCL_REAL support_value = support.dot(support_dir);

  // The following is very similar to what is done for Cone's support set
  // computation.
  const FCL_REAL nx = support_dir[0];
  const FCL_REAL ny = support_dir[1];
  const FCL_REAL r = cylinder->radius;
  if (support_value - r * std::sqrt((nx - ny) * (nx - ny) + nx * ny) <= tol) {
    const FCL_REAL angle_increment =
        2.0 * (FCL_REAL)(EIGEN_PI) / ((FCL_REAL)(max_num_supports));
    for (size_t i = 0; i < max_num_supports; ++i) {
      const FCL_REAL theta = (FCL_REAL)(i)*angle_increment;
      Vec3f point_on_cone_base(
          std::cos(theta) * cylinder->radius,
          std::sin(theta) * cylinder->radius,
          support_dir[2] >= 0 ? cylinder->halfLength : -cylinder->halfLength);
      assert(std::abs(support_value - point_on_cone_base.dot(support_dir)) <=
             tol);
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_cone_base += cylinder->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_cone_base);
    }
  } else {
    // There are two potential supports to add: one on each circle bases of the
    // cylinder.
    Vec3f point_on_lower_circle = Vec3f(cylinder->radius * support_dir[0],  //
                                        cylinder->radius * support_dir[1],  //
                                        -cylinder->halfLength);
    if (support_value - point_on_lower_circle.dot(support_dir) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_lower_circle += cylinder->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_lower_circle);
    }

    Vec3f point_on_upper_circle = Vec3f(cylinder->radius * support_dir[0],  //
                                        cylinder->radius * support_dir[1],  //
                                        -cylinder->halfLength);
    if (support_value - point_on_upper_circle.dot(support_dir) <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        point_on_upper_circle += cylinder->getSweptSphereRadius() * support_dir;
      }
      support_set.addPoint(point_on_upper_circle);
    }
  }
}
getShapeSupportSetTplInstantiation(Cylinder);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSetLinear(const ConvexBase* convex, SupportSet& support_set,
                              int& hint /*unused*/,
                              ShapeSupportData& support_data, size_t /*unused*/,
                              FCL_REAL tol) {
  assert(tol > 0);
  support_set.points().clear();
  support_data.support_set.points().clear();
  support_data.support_set.direction = support_set.direction;
  support_data.support_set.tf = support_set.tf;

  Vec3f support;
  const Vec3f& support_dir = support_set.getNormal();
  getShapeSupport<SupportOptions::NoSweptSphere>(convex, support_dir, support,
                                                 hint, support_data);
  const FCL_REAL support_value = support.dot(support_dir);

  const std::vector<Vec3f>& pts = *(convex->points);
  assert(pts.size() == (size_t)(convex->num_points));
  hint = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    FCL_REAL dot = pts[i].dot(support_dir);
    if (support_value - dot <= tol) {
      if (_SupportOptions == SupportOptions::WithSweptSphere) {
        support_data.support_set.addPoint(
            pts[i] + convex->getSweptSphereRadius() * support_dir);
      } else {
        support_data.support_set.addPoint(pts[i]);
      }
    }
  }

  computeSupportSetConvexHull(support_data, support_set);
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSetLog(const ConvexBase* convex, SupportSet& support_set,
                           int& hint, ShapeSupportData& support_data,
                           size_t /*unused*/, FCL_REAL tol) {
  // TODO(louis)
  getShapeSupportSetLinear<_SupportOptions>(convex, support_set, hint,
                                            support_data, 0, tol);
}

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const ConvexBase* convex, SupportSet& support_set,
                        int& hint, ShapeSupportData& support_data,
                        size_t max_num_supports /*unused*/, FCL_REAL tol) {
  if (convex->num_points > ConvexBase::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    getShapeSupportSetLog<_SupportOptions>(convex, support_set, hint,
                                           support_data, max_num_supports, tol);
  } else {
    getShapeSupportSetLinear<_SupportOptions>(
        convex, support_set, hint, support_data, max_num_supports, tol);
  }
}
getShapeSupportSetTplInstantiation(ConvexBase);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const SmallConvex* convex, SupportSet& support_set,
                        int& hint /*unused*/,
                        ShapeSupportData& support_data /*unused*/,
                        size_t max_num_supports /*unused*/, FCL_REAL tol) {
  getShapeSupportSetLinear<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), support_set, hint,
      support_data, max_num_supports, tol);
}
getShapeSupportSetTplInstantiation(SmallConvex);

// ============================================================================
template <int _SupportOptions>
void getShapeSupportSet(const LargeConvex* convex, SupportSet& support_set,
                        int& hint, ShapeSupportData& support_data,
                        size_t max_num_supports /*unused*/, FCL_REAL tol) {
  getShapeSupportSetLog<_SupportOptions>(
      reinterpret_cast<const ConvexBase*>(convex), support_set, hint,
      support_data, max_num_supports, tol);
}
getShapeSupportSetTplInstantiation(LargeConvex);

// ============================================================================
HPP_FCL_DLLAPI void computeSupportSetConvexHull(
    ShapeSupportData& support_data, SupportSet& support_set_cvx_hull) {}

}  // namespace details
}  // namespace fcl
}  // namespace hpp
