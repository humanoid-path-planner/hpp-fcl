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

/** \author Jia Pan */

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/narrowphase/gjk.h>
#include <hpp/fcl/internal/intersect.h>
#include <hpp/fcl/internal/tools.h>
#include <hpp/fcl/shape/geometric_shapes_traits.h>
#include <hpp/fcl/narrowphase/narrowphase_defaults.h>

namespace hpp {
namespace fcl {

namespace details {

void getShapeSupport(const TriangleP* triangle, const Vec3f& dir,
                     Vec3f& support, int&, MinkowskiDiff::ShapeData*) {
  FCL_REAL dota = dir.dot(triangle->a);
  FCL_REAL dotb = dir.dot(triangle->b);
  FCL_REAL dotc = dir.dot(triangle->c);
  if (dota > dotb) {
    if (dotc > dota)
      support = triangle->c;
    else
      support = triangle->a;
  } else {
    if (dotc > dotb)
      support = triangle->c;
    else
      support = triangle->b;
  }
}

inline void getShapeSupport(const Box* box, const Vec3f& dir, Vec3f& support,
                            int&, MinkowskiDiff::ShapeData*) {
  const FCL_REAL inflate = (dir.array() == 0).any() ? 1.00000001 : 1.;
  support.noalias() =
      (dir.array() > 0)
          .select(inflate * box->halfSide, -inflate * box->halfSide);
}

inline void getShapeSupport(const Sphere*, const Vec3f& /*dir*/, Vec3f& support,
                            int&, MinkowskiDiff::ShapeData*) {
  support.setZero();
}

inline void getShapeSupport(const Ellipsoid* ellipsoid, const Vec3f& dir,
                            Vec3f& support, int&, MinkowskiDiff::ShapeData*) {
  FCL_REAL a2 = ellipsoid->radii[0] * ellipsoid->radii[0];
  FCL_REAL b2 = ellipsoid->radii[1] * ellipsoid->radii[1];
  FCL_REAL c2 = ellipsoid->radii[2] * ellipsoid->radii[2];

  Vec3f v(a2 * dir[0], b2 * dir[1], c2 * dir[2]);

  FCL_REAL d = std::sqrt(v.dot(dir));

  support = v / d;
}

inline void getShapeSupport(const Capsule* capsule, const Vec3f& dir,
                            Vec3f& support, int&, MinkowskiDiff::ShapeData*) {
  support.head<2>().setZero();
  if (dir[2] > 0)
    support[2] = capsule->halfLength;
  else
    support[2] = -capsule->halfLength;
}

void getShapeSupport(const Cone* cone, const Vec3f& dir, Vec3f& support, int&,
                     MinkowskiDiff::ShapeData*) {
  // The cone radius is, for -h < z < h, (h - z) * r / (2*h)
  static const FCL_REAL inflate = 1.00001;
  FCL_REAL h = cone->halfLength;
  FCL_REAL r = cone->radius;

  if (dir.head<2>().isZero()) {
    support.head<2>().setZero();
    if (dir[2] > 0)
      support[2] = h;
    else
      support[2] = -inflate * h;
    return;
  }
  FCL_REAL zdist = dir[0] * dir[0] + dir[1] * dir[1];
  FCL_REAL len = zdist + dir[2] * dir[2];
  zdist = std::sqrt(zdist);

  if (dir[2] <= 0) {
    FCL_REAL rad = r / zdist;
    support.head<2>() = rad * dir.head<2>();
    support[2] = -h;
    return;
  }

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

void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir, Vec3f& support,
                     int&, MinkowskiDiff::ShapeData*) {
  // The inflation makes the object look strictly convex to GJK and EPA. This
  // helps solving particular cases (e.g. a cylinder with itself at the same
  // position...)
  static const FCL_REAL inflate = 1.00001;
  FCL_REAL half_h = cylinder->halfLength;
  FCL_REAL r = cylinder->radius;

  if (dir.head<2>() == Eigen::Matrix<FCL_REAL, 2, 1>::Zero()) half_h *= inflate;

  if (dir[2] > 0)
    support[2] = half_h;
  else if (dir[2] < 0)
    support[2] = -half_h;
  else {
    support[2] = 0;
    r *= inflate;
  }
  if (dir.head<2>() == Eigen::Matrix<FCL_REAL, 2, 1>::Zero())
    support.head<2>().setZero();
  else
    support.head<2>() = dir.head<2>().normalized() * r;
  assert(fabs(support[0] * dir[1] - support[1] * dir[0]) <
         sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
}

struct SmallConvex : ShapeBase {};
struct LargeConvex : ShapeBase {};

void getShapeSupportLog(const ConvexBase* convex, const Vec3f& dir,
                        Vec3f& support, int& hint,
                        MinkowskiDiff::ShapeData* data) {
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

  if (hint < 0 || hint >= (int)convex->num_points) hint = 0;
  FCL_REAL maxdot = pts[static_cast<size_t>(hint)].dot(dir);
  std::vector<int8_t>& visited = data->visited;
  std::fill(visited.begin(), visited.end(), false);
  visited[static_cast<std::size_t>(hint)] = true;
  // when the first face is orthogonal to dir, all the dot products will be
  // equal. Yet, the neighbors must be visited.
  bool found = true, loose_check = true;
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
}

void getShapeSupportLinear(const ConvexBase* convex, const Vec3f& dir,
                           Vec3f& support, int& hint,
                           MinkowskiDiff::ShapeData*) {
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
}

void getShapeSupport(const ConvexBase* convex, const Vec3f& dir, Vec3f& support,
                     int& hint, MinkowskiDiff::ShapeData*) {
  // TODO add benchmark to set a proper value for switching between linear and
  // logarithmic.
  if (convex->num_points > ConvexBase::num_vertices_large_convex_threshold &&
      convex->neighbors != nullptr) {
    MinkowskiDiff::ShapeData data;
    data.visited.assign(convex->num_points, false);
    getShapeSupportLog(convex, dir, support, hint, &data);
  } else
    getShapeSupportLinear(convex, dir, support, hint, nullptr);
}

inline void getShapeSupport(const SmallConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint,
                            MinkowskiDiff::ShapeData* data) {
  getShapeSupportLinear(reinterpret_cast<const ConvexBase*>(convex), dir,
                        support, hint, data);
}

inline void getShapeSupport(const LargeConvex* convex, const Vec3f& dir,
                            Vec3f& support, int& hint,
                            MinkowskiDiff::ShapeData* data) {
  getShapeSupportLog(reinterpret_cast<const ConvexBase*>(convex), dir, support,
                     hint, data);
}

#define CALL_GET_SHAPE_SUPPORT(ShapeType)                              \
  getShapeSupport(                                                     \
      static_cast<const ShapeType*>(shape),                            \
      (shape_traits<ShapeType>::NeedNormalizedDir && !dirIsNormalized) \
          ? dir.normalized()                                           \
          : dir,                                                       \
      support, hint, NULL)

inline void getSphereSupport(const Sphere* sphere, const Vec3f& dir,
                             Vec3f& support) {
  support = sphere->radius * (dir.normalized());
}

Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, bool dirIsNormalized,
                 int& hint) {
  Vec3f support;
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      CALL_GET_SHAPE_SUPPORT(TriangleP);
      break;
    case GEOM_BOX:
      CALL_GET_SHAPE_SUPPORT(Box);
      break;
    case GEOM_SPHERE:
      getSphereSupport(static_cast<const Sphere*>(shape), dir, support);
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

template <typename Shape0, typename Shape1, bool TransformIsIdentity>
void getSupportTpl(const Shape0* s0, const Shape1* s1, const Matrix3f& oR1,
                   const Vec3f& ot1, const Vec3f& dir, Vec3f& support0,
                   Vec3f& support1, support_func_guess_t& hint,
                   MinkowskiDiff::ShapeData data[2]) {
  getShapeSupport(s0, dir, support0, hint[0], &(data[0]));
  if (TransformIsIdentity)
    getShapeSupport(s1, -dir, support1, hint[1], &(data[1]));
  else {
    getShapeSupport(s1, -oR1.transpose() * dir, support1, hint[1], &(data[1]));
    support1 = oR1 * support1 + ot1;
  }
}

template <typename Shape0, typename Shape1, bool TransformIsIdentity>
void getSupportFuncTpl(const MinkowskiDiff& md, const Vec3f& dir,
                       bool dirIsNormalized, Vec3f& support0, Vec3f& support1,
                       support_func_guess_t& hint,
                       MinkowskiDiff::ShapeData data[2]) {
  enum {
    NeedNormalizedDir = bool((bool)shape_traits<Shape0>::NeedNormalizedDir ||
                             (bool)shape_traits<Shape1>::NeedNormalizedDir)
  };
#ifndef NDEBUG
  // Need normalized direction and direction is normalized
  assert(!NeedNormalizedDir || !dirIsNormalized ||
         fabs(dir.squaredNorm() - 1) < GJK_MINIMUM_TOLERANCE);
  // Need normalized direction but direction is not normalized.
  assert(!NeedNormalizedDir || dirIsNormalized ||
         fabs(dir.normalized().squaredNorm() - 1) < GJK_MINIMUM_TOLERANCE);
  // Don't need normalized direction. Check that dir is not zero.
  assert(NeedNormalizedDir || dir.norm() >= GJK_MINIMUM_TOLERANCE);
#endif
  getSupportTpl<Shape0, Shape1, TransformIsIdentity>(
      static_cast<const Shape0*>(md.shapes[0]),
      static_cast<const Shape1*>(md.shapes[1]), md.oR1, md.ot1,
      (NeedNormalizedDir && !dirIsNormalized) ? dir.normalized() : dir,
      support0, support1, hint, data);
}

template <typename Shape0>
MinkowskiDiff::GetSupportFunction makeGetSupportFunction1(
    const ShapeBase* s1, bool identity, Eigen::Array<FCL_REAL, 1, 2>& inflation,
    MinkowskiDiff::ShapeData data[2]) {
  inflation[1] = 0;
  switch (s1->getNodeType()) {
    case GEOM_TRIANGLE:
      if (identity)
        return getSupportFuncTpl<Shape0, TriangleP, true>;
      else
        return getSupportFuncTpl<Shape0, TriangleP, false>;
    case GEOM_BOX:
      if (identity)
        return getSupportFuncTpl<Shape0, Box, true>;
      else
        return getSupportFuncTpl<Shape0, Box, false>;
    case GEOM_SPHERE:
      inflation[1] = static_cast<const Sphere*>(s1)->radius;
      if (identity)
        return getSupportFuncTpl<Shape0, Sphere, true>;
      else
        return getSupportFuncTpl<Shape0, Sphere, false>;
    case GEOM_ELLIPSOID:
      if (identity)
        return getSupportFuncTpl<Shape0, Ellipsoid, true>;
      else
        return getSupportFuncTpl<Shape0, Ellipsoid, false>;
    case GEOM_CAPSULE:
      inflation[1] = static_cast<const Capsule*>(s1)->radius;
      if (identity)
        return getSupportFuncTpl<Shape0, Capsule, true>;
      else
        return getSupportFuncTpl<Shape0, Capsule, false>;
    case GEOM_CONE:
      if (identity)
        return getSupportFuncTpl<Shape0, Cone, true>;
      else
        return getSupportFuncTpl<Shape0, Cone, false>;
    case GEOM_CYLINDER:
      if (identity)
        return getSupportFuncTpl<Shape0, Cylinder, true>;
      else
        return getSupportFuncTpl<Shape0, Cylinder, false>;
    case GEOM_CONVEX: {
      const ConvexBase* convex1 = static_cast<const ConvexBase*>(s1);
      if ((int)convex1->num_points >
          ConvexBase::num_vertices_large_convex_threshold) {
        data[1].visited.assign(convex1->num_points, false);
        if (identity)
          return getSupportFuncTpl<Shape0, LargeConvex, true>;
        else
          return getSupportFuncTpl<Shape0, LargeConvex, false>;
      } else {
        if (identity)
          return getSupportFuncTpl<Shape0, SmallConvex, true>;
        else
          return getSupportFuncTpl<Shape0, SmallConvex, false>;
      }
    }
    default:
      HPP_FCL_THROW_PRETTY("Unsupported geometric shape.", std::logic_error);
  }
}

MinkowskiDiff::GetSupportFunction makeGetSupportFunction0(
    const ShapeBase* s0, const ShapeBase* s1, bool identity,
    Eigen::Array<FCL_REAL, 1, 2>& inflation, MinkowskiDiff::ShapeData data[2]) {
  inflation[0] = 0;
  switch (s0->getNodeType()) {
    case GEOM_TRIANGLE:
      return makeGetSupportFunction1<TriangleP>(s1, identity, inflation, data);
      break;
    case GEOM_BOX:
      return makeGetSupportFunction1<Box>(s1, identity, inflation, data);
      break;
    case GEOM_SPHERE:
      inflation[0] = static_cast<const Sphere*>(s0)->radius;
      return makeGetSupportFunction1<Sphere>(s1, identity, inflation, data);
      break;
    case GEOM_ELLIPSOID:
      return makeGetSupportFunction1<Ellipsoid>(s1, identity, inflation, data);
      break;
    case GEOM_CAPSULE:
      inflation[0] = static_cast<const Capsule*>(s0)->radius;
      return makeGetSupportFunction1<Capsule>(s1, identity, inflation, data);
      break;
    case GEOM_CONE:
      return makeGetSupportFunction1<Cone>(s1, identity, inflation, data);
      break;
    case GEOM_CYLINDER:
      return makeGetSupportFunction1<Cylinder>(s1, identity, inflation, data);
      break;
    case GEOM_CONVEX: {
      const ConvexBase* convex0 = static_cast<const ConvexBase*>(s0);
      if ((int)convex0->num_points >
          ConvexBase::num_vertices_large_convex_threshold) {
        data[0].visited.assign(convex0->num_points, false);
        return makeGetSupportFunction1<LargeConvex>(s1, identity, inflation,
                                                    data);
      } else
        return makeGetSupportFunction1<SmallConvex>(s1, identity, inflation,
                                                    data);
      break;
    }
    default:
      HPP_FCL_THROW_PRETTY("Unsupported geometric shape", std::logic_error);
  }
}

bool getNormalizeSupportDirection(const ShapeBase* shape) {
  switch (shape->getNodeType()) {
    case GEOM_TRIANGLE:
      return (bool)shape_traits<TriangleP>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_BOX:
      return (bool)shape_traits<Box>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_SPHERE:
      return (bool)shape_traits<Sphere>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_ELLIPSOID:
      return (bool)shape_traits<Ellipsoid>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CAPSULE:
      return (bool)shape_traits<Capsule>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CONE:
      return (bool)shape_traits<Cone>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CYLINDER:
      return (bool)shape_traits<Cylinder>::NeedNesterovNormalizeHeuristic;
      break;
    case GEOM_CONVEX:
      return (bool)shape_traits<ConvexBase>::NeedNesterovNormalizeHeuristic;
      break;
    default:
      HPP_FCL_THROW_PRETTY("Unsupported geometric shape", std::logic_error);
  }
}

void getNormalizeSupportDirectionFromShapes(const ShapeBase* shape0,
                                            const ShapeBase* shape1,
                                            bool& normalize_support_direction) {
  normalize_support_direction = getNormalizeSupportDirection(shape0) &&
                                getNormalizeSupportDirection(shape1);
}

void MinkowskiDiff::set(const ShapeBase* shape0, const ShapeBase* shape1,
                        const Transform3f& tf0, const Transform3f& tf1) {
  shapes[0] = shape0;
  shapes[1] = shape1;
  getNormalizeSupportDirectionFromShapes(shape0, shape1,
                                         normalize_support_direction);

  oR1.noalias() = tf0.getRotation().transpose() * tf1.getRotation();
  ot1.noalias() = tf0.getRotation().transpose() *
                  (tf1.getTranslation() - tf0.getTranslation());

  bool identity = (oR1.isIdentity() && ot1.isZero());

  getSupportFunc =
      makeGetSupportFunction0(shape0, shape1, identity, inflation, data);
}

void MinkowskiDiff::set(const ShapeBase* shape0, const ShapeBase* shape1) {
  shapes[0] = shape0;
  shapes[1] = shape1;
  getNormalizeSupportDirectionFromShapes(shape0, shape1,
                                         normalize_support_direction);

  oR1.setIdentity();
  ot1.setZero();

  getSupportFunc =
      makeGetSupportFunction0(shape0, shape1, true, inflation, data);
}

void GJK::initialize() {
  distance_upper_bound = (std::numeric_limits<FCL_REAL>::max)();
  gjk_variant = GJKVariant::DefaultGJK;
  convergence_criterion = GJKConvergenceCriterion::Default;
  convergence_criterion_type = GJKConvergenceCriterionType::Relative;
  reset(max_iterations, tolerance);
}

void GJK::reset(size_t max_iterations_, FCL_REAL tolerance_) {
  max_iterations = max_iterations_;
  tolerance = tolerance_;
  status = DidNotRun;
  nfree = 0;
  simplex = nullptr;
  iterations = 0;
  iterations_momentum_stop = 0;
}

Vec3f GJK::getGuessFromSimplex() const { return ray; }

namespace details {

bool getClosestPoints(const GJK::Simplex& simplex, Vec3f& w0, Vec3f& w1) {
  GJK::SimplexV* const* vs = simplex.vertex;

  for (GJK::vertex_id_t i = 0; i < simplex.rank; ++i) {
    assert(vs[i]->w.isApprox(vs[i]->w0 - vs[i]->w1));
  }

  Project::ProjectResult projection;
  switch (simplex.rank) {
    case 1:
      w0 = vs[0]->w0;
      w1 = vs[0]->w1;
      return true;
    case 2: {
      const Vec3f &a = vs[0]->w, a0 = vs[0]->w0, a1 = vs[0]->w1, b = vs[1]->w,
                  b0 = vs[1]->w0, b1 = vs[1]->w1;
      FCL_REAL la, lb;
      Vec3f N(b - a);
      la = N.dot(-a);
      if (la <= 0) {
        assert(false);
        w0 = a0;
        w1 = a1;
      } else {
        lb = N.squaredNorm();
        if (la > lb) {
          assert(false);
          w0 = b0;
          w1 = b1;
        } else {
          lb = la / lb;
          la = 1 - lb;
          w0 = la * a0 + lb * b0;
          w1 = la * a1 + lb * b1;
        }
      }
    }
      return true;
    case 3:
      // TODO avoid the reprojection
      projection = Project::projectTriangleOrigin(vs[0]->w, vs[1]->w, vs[2]->w);
      break;
    case 4:  // We are in collision.
      projection = Project::projectTetrahedraOrigin(vs[0]->w, vs[1]->w,
                                                    vs[2]->w, vs[3]->w);
      break;
    default:
      HPP_FCL_THROW_PRETTY("The simplex rank must be in [ 1, 4 ]",
                           std::logic_error);
  }
  w0.setZero();
  w1.setZero();
  for (GJK::vertex_id_t i = 0; i < simplex.rank; ++i) {
    w0 += projection.parameterization[i] * vs[i]->w0;
    w1 += projection.parameterization[i] * vs[i]->w1;
  }
  return true;
}

/// Inflate the points
template <bool Separated>
void inflate(const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1) {
  const Eigen::Array<FCL_REAL, 1, 2>& I(shape.inflation);
  Eigen::Array<bool, 1, 2> inflate(I > 0);
  if (!inflate.any()) return;
  Vec3f w(w0 - w1);
  FCL_REAL n2 = w.squaredNorm();
  // TODO should be use a threshold (Eigen::NumTraits<FCL_REAL>::epsilon()) ?
  if (n2 == 0.) {
    if (inflate[0]) w0[0] += I[0] * (Separated ? -1 : 1);
    if (inflate[1]) w1[0] += I[1] * (Separated ? 1 : -1);
    return;
  }

  w /= std::sqrt(n2);
  if (Separated) {
    if (inflate[0]) w0 -= I[0] * w;
    if (inflate[1]) w1 += I[1] * w;
  } else {
    if (inflate[0]) w0 += I[0] * w;
    if (inflate[1]) w1 -= I[1] * w;
  }
}

}  // namespace details

bool GJK::getClosestPoints(const MinkowskiDiff& shape, Vec3f& w0,
                           Vec3f& w1) const {
  bool res = details::getClosestPoints(*simplex, w0, w1);
  if (!res) return false;
  details::inflate<true>(shape, w0, w1);
  return true;
}

GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess,
                          const support_func_guess_t& supportHint) {
  FCL_REAL alpha = 0;
  iterations = 0;
  const FCL_REAL inflation = shape_.inflation.sum();
  const FCL_REAL upper_bound = distance_upper_bound + inflation;

  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];

  nfree = 4;
  status = NoCollision;
  shape = &shape_;
  distance = 0.0;
  current = 0;
  simplices[current].rank = 0;
  support_hint = supportHint;

  FCL_REAL rl = guess.norm();
  if (rl < tolerance) {
    ray = Vec3f(-1, 0, 0);
    rl = 1;
  } else
    ray = guess;

  // Momentum
  GJKVariant current_gjk_variant = gjk_variant;
  Vec3f w = ray;
  Vec3f dir = ray;
  Vec3f y;
  FCL_REAL momentum;
  bool normalize_support_direction = shape->normalize_support_direction;
  do {
    vertex_id_t next = (vertex_id_t)(1 - current);
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    if (rl < tolerance)  // mean origin is near the face of original simplex,
                         // return touch
    {
      // At this point, GJK has converged but we don't know if GJK is enough to
      // recover penetration information.
      // EPA needs to be run.
      // Unless the Minkowski difference is degenerated, EPA will run fine even
      // if the final simplex of GJK is not a tetrahedron.
      assert(rl > 0);
      status = Collision;
      distance = -inflation;  // should we take rl into account ?
      break;
    }

    // Compute direction for support call
    switch (current_gjk_variant) {
      case DefaultGJK:
        dir = ray;
        break;

      case NesterovAcceleration:
        // Normalize heuristic for collision pairs involving convex but not
        // strictly-convex shapes This corresponds to most use cases.
        if (normalize_support_direction) {
          momentum = (FCL_REAL(iterations) + 2) / (FCL_REAL(iterations) + 3);
          y = momentum * ray + (1 - momentum) * w;
          FCL_REAL y_norm = y.norm();
          // ray is the point of the Minkowski difference which currently the
          // closest to the origin. Therefore, y.norm() > ray.norm() Hence, if
          // check A above has not stopped the algorithm, we necessarily have
          // y.norm() > tolerance. The following assert is just a safety check.
          assert(y_norm > tolerance);
          dir = momentum * dir / dir.norm() + (1 - momentum) * y / y_norm;
        } else {
          momentum = (FCL_REAL(iterations) + 1) / (FCL_REAL(iterations) + 3);
          y = momentum * ray + (1 - momentum) * w;
          dir = momentum * dir + (1 - momentum) * y;
        }
        break;

      case PolyakAcceleration:
        momentum = 1 / (FCL_REAL(iterations) + 1);
        dir = momentum * dir + (1 - momentum) * ray;
        break;

      default:
        HPP_FCL_THROW_PRETTY("Invalid momentum variant.", std::logic_error);
    }

    appendVertex(curr_simplex, -dir, false,
                 support_hint);  // see below, ray points away from origin

    // check removed (by ?): when the new support point is close to previous
    // support points, stop (as the new simplex is degenerated)
    w = curr_simplex.vertex[curr_simplex.rank - 1]->w;

    // check B: no collision if omega > 0
    FCL_REAL omega = dir.dot(w) / dir.norm();
    if (omega > upper_bound) {
      distance = omega - inflation;
      status = NoCollisionEarlyStopped;
      break;
    }

    // Check to remove acceleration
    if (current_gjk_variant != DefaultGJK) {
      FCL_REAL frank_wolfe_duality_gap = 2 * ray.dot(ray - w);
      if (frank_wolfe_duality_gap - tolerance <= 0) {
        removeVertex(simplices[current]);
        current_gjk_variant = DefaultGJK;  // move back to classic GJK
        iterations_momentum_stop = iterations;
        continue;  // continue to next iteration
      }
    }

    // check C: when the new support point is close to the sub-simplex where the
    // ray point lies, stop (as the new simplex again is degenerated)
    bool cv_check_passed = checkConvergence(w, rl, alpha, omega);
    if (iterations > 0 && cv_check_passed) {
      if (iterations > 0) removeVertex(simplices[current]);
      if (current_gjk_variant != DefaultGJK) {
        current_gjk_variant = DefaultGJK;  // move back to classic GJK
        iterations_momentum_stop = iterations;
        continue;
      }
      // TODO When inflation is strictly positive, the distance may be exactly
      // zero (so the ray is not zero) and we are not in the case rl <
      // tolerance.

      // At this point, GJK has converged and penetration information can always
      // be recovered without running EPA.
      distance = rl - inflation;
      if (distance < tolerance) {
        status = CollisionWithPenetrationInformation;
      }
      break;
    }

    // This has been rewritten thanks to the excellent video:
    // https://youtu.be/Qupqu1xe7Io
    bool inside;
    switch (curr_simplex.rank) {
      case 1:  // Only at the first iteration
        assert(iterations == 0);
        ray = w;
        inside = false;
        next_simplex.rank = 1;
        next_simplex.vertex[0] = curr_simplex.vertex[0];
        break;
      case 2:
        inside = projectLineOrigin(curr_simplex, next_simplex);
        break;
      case 3:
        inside = projectTriangleOrigin(curr_simplex, next_simplex);
        break;
      case 4:
        inside = projectTetrahedraOrigin(curr_simplex, next_simplex);
        break;
      default:
        HPP_FCL_THROW_PRETTY("Invalid simplex rank", std::logic_error);
    }
    assert(nfree + next_simplex.rank == 4);
    current = next;
    if (!inside) rl = ray.norm();
    if (inside || rl == 0) {
      status = Collision;
      distance = -inflation;
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;

  } while (status == NoCollision);

  simplex = &simplices[current];
  assert(simplex->rank > 0 && simplex->rank < 5);
  return status;
}

bool GJK::checkConvergence(const Vec3f& w, const FCL_REAL& rl, FCL_REAL& alpha,
                           const FCL_REAL& omega) const {
  // x^* is the optimal solution (projection of origin onto the Minkowski
  // difference).
  //  x^k is the current iterate (x^k = `ray` in the code).
  // Each criterion provides a different guarantee on the distance to the
  // optimal solution.
  switch (convergence_criterion) {
    case Default: {
      // alpha is the distance to the best separating hyperplane found so far
      alpha = std::max(alpha, omega);
      // ||x^*|| - ||x^k|| <= diff
      const FCL_REAL diff = rl - alpha;
      return ((diff - (tolerance + tolerance * rl)) <= 0);
    } break;

    case DualityGap: {
      // ||x^* - x^k||^2 <= diff
      const FCL_REAL diff = 2 * ray.dot(ray - w);
      switch (convergence_criterion_type) {
        case Absolute:
          return ((diff - tolerance) <= 0);
          break;
        case Relative:
          return (((diff / tolerance * rl) - tolerance * rl) <= 0);
          break;
        default:
          HPP_FCL_THROW_PRETTY("Invalid convergence criterion type.",
                               std::logic_error);
      }
    } break;

    case Hybrid: {
      // alpha is the distance to the best separating hyperplane found so far
      alpha = std::max(alpha, omega);
      // ||x^* - x^k||^2 <= diff
      const FCL_REAL diff = rl * rl - alpha * alpha;
      switch (convergence_criterion_type) {
        case Absolute:
          return ((diff - tolerance) <= 0);
          break;
        case Relative:
          return (((diff / tolerance * rl) - tolerance * rl) <= 0);
          break;
        default:
          HPP_FCL_THROW_PRETTY("Invalid convergence criterion type.",
                               std::logic_error);
      }
    } break;

    default:
      HPP_FCL_THROW_PRETTY("Invalid convergence criterion.", std::logic_error);
  }
}

inline void GJK::removeVertex(Simplex& simplex) {
  free_v[nfree++] = simplex.vertex[--simplex.rank];
}

inline void GJK::appendVertex(Simplex& simplex, const Vec3f& v,
                              bool isNormalized, support_func_guess_t& hint) {
  simplex.vertex[simplex.rank] = free_v[--nfree];  // set the memory
  getSupport(v, isNormalized, *simplex.vertex[simplex.rank++], hint);
}

bool GJK::encloseOrigin() {
  Vec3f axis(Vec3f::Zero());
  support_func_guess_t hint = support_func_guess_t::Zero();
  switch (simplex->rank) {
    case 1:
      for (int i = 0; i < 3; ++i) {
        axis[i] = 1;
        appendVertex(*simplex, axis, true, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
        axis[i] = -1;
        appendVertex(*simplex, -axis, true, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
        axis[i] = 0;
      }
      break;
    case 2: {
      Vec3f d = simplex->vertex[1]->w - simplex->vertex[0]->w;
      for (int i = 0; i < 3; ++i) {
        axis[i] = 1;
        Vec3f p = d.cross(axis);
        if (!p.isZero()) {
          appendVertex(*simplex, p, false, hint);
          if (encloseOrigin()) return true;
          removeVertex(*simplex);
          appendVertex(*simplex, -p, false, hint);
          if (encloseOrigin()) return true;
          removeVertex(*simplex);
        }
        axis[i] = 0;
      }
    } break;
    case 3:
      axis.noalias() =
          (simplex->vertex[1]->w - simplex->vertex[0]->w)
              .cross(simplex->vertex[2]->w - simplex->vertex[0]->w);
      if (!axis.isZero()) {
        appendVertex(*simplex, axis, false, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -axis, false, hint);
        if (encloseOrigin()) return true;
        removeVertex(*simplex);
      }
      break;
    case 4:
      if (std::abs(triple(simplex->vertex[0]->w - simplex->vertex[3]->w,
                          simplex->vertex[1]->w - simplex->vertex[3]->w,
                          simplex->vertex[2]->w - simplex->vertex[3]->w)) > 0)
        return true;
      break;
  }

  return false;
}

inline void originToPoint(const GJK::Simplex& current, GJK::vertex_id_t a,
                          const Vec3f& A, GJK::Simplex& next, Vec3f& ray) {
  // A is the closest to the origin
  ray = A;
  next.vertex[0] = current.vertex[a];
  next.rank = 1;
}

inline void originToSegment(const GJK::Simplex& current, GJK::vertex_id_t a,
                            GJK::vertex_id_t b, const Vec3f& A, const Vec3f& B,
                            const Vec3f& AB, const FCL_REAL& ABdotAO,
                            GJK::Simplex& next, Vec3f& ray) {
  // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
  ray = AB.dot(B) * A + ABdotAO * B;

  next.vertex[0] = current.vertex[b];
  next.vertex[1] = current.vertex[a];
  next.rank = 2;

  // To ensure backward compatibility
  ray /= AB.squaredNorm();
}

inline bool originToTriangle(const GJK::Simplex& current, GJK::vertex_id_t a,
                             GJK::vertex_id_t b, GJK::vertex_id_t c,
                             const Vec3f& ABC, const FCL_REAL& ABCdotAO,
                             GJK::Simplex& next, Vec3f& ray) {
  next.rank = 3;
  next.vertex[2] = current.vertex[a];

  if (ABCdotAO == 0) {
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
    ray.setZero();
    return true;
  }
  if (ABCdotAO > 0) {  // Above triangle
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
  } else {
    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[c];
  }

  // To ensure backward compatibility
  ray = -ABCdotAO / ABC.squaredNorm() * ABC;
  return false;
}

bool GJK::projectLineOrigin(const Simplex& current, Simplex& next) {
  const vertex_id_t a = 1, b = 0;
  // A is the last point we added.
  const Vec3f& A = current.vertex[a]->w;
  const Vec3f& B = current.vertex[b]->w;

  const Vec3f AB = B - A;
  const FCL_REAL d = AB.dot(-A);
  assert(d <= AB.squaredNorm());

  if (d == 0) {
    // Two extremely unlikely cases:
    // - AB is orthogonal to A: should never happen because it means the support
    //   function did not do any progress and GJK should have stopped.
    // - A == origin
    // In any case, A is the closest to the origin
    originToPoint(current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
    return A.isZero();
  } else if (d < 0) {
    // A is the closest to the origin
    originToPoint(current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
  } else
    originToSegment(current, a, b, A, B, AB, d, next, ray);
  return false;
}

bool GJK::projectTriangleOrigin(const Simplex& current, Simplex& next) {
  const vertex_id_t a = 2, b = 1, c = 0;
  // A is the last point we added.
  const Vec3f &A = current.vertex[a]->w, B = current.vertex[b]->w,
              C = current.vertex[c]->w;

  const Vec3f AB = B - A, AC = C - A, ABC = AB.cross(AC);

  FCL_REAL edgeAC2o = ABC.cross(AC).dot(-A);
  if (edgeAC2o >= 0) {
    FCL_REAL towardsC = AC.dot(-A);
    if (towardsC >= 0) {  // Region 1
      originToSegment(current, a, c, A, C, AC, towardsC, next, ray);
      free_v[nfree++] = current.vertex[b];
    } else {  // Region 4 or 5
      FCL_REAL towardsB = AB.dot(-A);
      if (towardsB < 0) {  // Region 5
        // A is the closest to the origin
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else  // Region 4
        originToSegment(current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    }
  } else {
    FCL_REAL edgeAB2o = AB.cross(ABC).dot(-A);
    if (edgeAB2o >= 0) {  // Region 4 or 5
      FCL_REAL towardsB = AB.dot(-A);
      if (towardsB < 0) {  // Region 5
        // A is the closest to the origin
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else  // Region 4
        originToSegment(current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    } else {
      return originToTriangle(current, a, b, c, ABC, ABC.dot(-A), next, ray);
    }
  }
  return false;
}

bool GJK::projectTetrahedraOrigin(const Simplex& current, Simplex& next) {
  // The code of this function was generated using doc/gjk.py
  const vertex_id_t a = 3, b = 2, c = 1, d = 0;
  const Vec3f& A(current.vertex[a]->w);
  const Vec3f& B(current.vertex[b]->w);
  const Vec3f& C(current.vertex[c]->w);
  const Vec3f& D(current.vertex[d]->w);
  const FCL_REAL aa = A.squaredNorm();
  const FCL_REAL da = D.dot(A);
  const FCL_REAL db = D.dot(B);
  const FCL_REAL dc = D.dot(C);
  const FCL_REAL dd = D.dot(D);
  const FCL_REAL da_aa = da - aa;
  const FCL_REAL ca = C.dot(A);
  const FCL_REAL cb = C.dot(B);
  const FCL_REAL cc = C.dot(C);
  const FCL_REAL& cd = dc;
  const FCL_REAL ca_aa = ca - aa;
  const FCL_REAL ba = B.dot(A);
  const FCL_REAL bb = B.dot(B);
  const FCL_REAL& bc = cb;
  const FCL_REAL& bd = db;
  const FCL_REAL ba_aa = ba - aa;
  const FCL_REAL ba_ca = ba - ca;
  const FCL_REAL ca_da = ca - da;
  const FCL_REAL da_ba = da - ba;
  const Vec3f a_cross_b = A.cross(B);
  const Vec3f a_cross_c = A.cross(C);

  const FCL_REAL dummy_precision(
      3 * std::sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
  HPP_FCL_UNUSED_VARIABLE(dummy_precision);

#define REGION_INSIDE()               \
  ray.setZero();                      \
  next.vertex[0] = current.vertex[d]; \
  next.vertex[1] = current.vertex[c]; \
  next.vertex[2] = current.vertex[b]; \
  next.vertex[3] = current.vertex[a]; \
  next.rank = 4;                      \
  return true;

  // clang-format off
  if (ba_aa <= 0) {                // if AB.AO >= 0 / a10
    if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / a10.a3
      if (ba * da_ba + bd * ba_aa - bb * da_aa <=
          0) {             // if (ADB ^ AB).AO >= 0 / a10.a3.a9
        if (da_aa <= 0) {  // if AD.AO >= 0 / a10.a3.a9.a12
          assert(da * da_ba + dd * ba_aa - db * da_aa <=
                 dummy_precision);  // (ADB ^ AD).AO >= 0 / a10.a3.a9.a12.a8
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
              0) {  // if (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.a4
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          } else {  // not (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.!a4
            // Region AB
            originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AB).AO >= 0
        } else {  // not AD.AO >= 0 / a10.a3.a9.!a12
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
              0) {  // if (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.a4
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
                0) {  // if (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5
              if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                  0) {  // if (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.a6
                // Region ACD
                originToTriangle(current, a, c, d, (C - A).cross(D - A),
                                 -D.dot(a_cross_c), next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {  // not (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.!a6
                // Region AC
                originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              }  // end of (ACD ^ AC).AO >= 0
            } else {  // not (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.!a5
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            }  // end of (ABC ^ AC).AO >= 0
          } else {  // not (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.!a4
            // Region AB
            originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AB).AO >= 0
        }  // end of AD.AO >= 0
      } else {  // not (ADB ^ AB).AO >= 0 / a10.a3.!a9
        if (da * da_ba + dd * ba_aa - db * da_aa <=
            0) {  // if (ADB ^ AD).AO >= 0 / a10.a3.!a9.a8
          // Region ADB
          originToTriangle(current, a, d, b, (D - A).cross(B - A),
                           D.dot(a_cross_b), next, ray);
          free_v[nfree++] = current.vertex[c];
        } else {  // not (ADB ^ AD).AO >= 0 / a10.a3.!a9.!a8
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.!a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.!a7
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AD).AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        }  // end of (ADB ^ AD).AO >= 0
      }  // end of (ADB ^ AB).AO >= 0
    } else {                        // not ADB.AO >= 0 / a10.!a3
      if (C.dot(a_cross_b) <= 0) {  // if ABC.AO >= 0 / a10.!a3.a1
        if (ba * ba_ca + bb * ca_aa - bc * ba_aa <=
            0) {  // if (ABC ^ AB).AO >= 0 / a10.!a3.a1.a4
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                0) {  // if (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.!a6
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AC).AO >= 0
          } else {  // not (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.!a5
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AC).AO >= 0
        } else {  // not (ABC ^ AB).AO >= 0 / a10.!a3.a1.!a4
          // Region AB
          originToSegment(current, a, b, A, B, B - A, -ba_aa, next, ray);
          free_v[nfree++] = current.vertex[c];
          free_v[nfree++] = current.vertex[d];
        }  // end of (ABC ^ AB).AO >= 0
      } else {                        // not ABC.AO >= 0 / a10.!a3.!a1
        if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / a10.!a3.!a1.a2
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {             // not (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.!a6
            if (ca_aa <= 0) {  // if AC.AO >= 0 / a10.!a3.!a1.a2.!a6.a11
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {  // not AC.AO >= 0 / a10.!a3.!a1.a2.!a6.!a11
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of AC.AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        } else {  // not ACD.AO >= 0 / a10.!a3.!a1.!a2
          // Region Inside
          REGION_INSIDE()
        }  // end of ACD.AO >= 0
      }  // end of ABC.AO >= 0
    }  // end of ADB.AO >= 0
  } else {                          // not AB.AO >= 0 / !a10
    if (ca_aa <= 0) {               // if AC.AO >= 0 / !a10.a11
      if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / !a10.a11.a2
        if (da_aa <= 0) {           // if AD.AO >= 0 / !a10.a11.a2.a12
          if (ca * ca_da + cc * da_aa - cd * ca_aa <=
              0) {  // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7
              if (da * da_ba + dd * ba_aa - db * da_aa <=
                  0) {  // if (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.a8
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {  // not (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.!a8
                // Region AD
                originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }  // end of (ADB ^ AD).AO >= 0
            } else {  // not (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6
            assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                     -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                          // !a10.a11.a2.a12.!a6.!a7
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
                0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.a5
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.!a5
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            }  // end of (ABC ^ AC).AO >= 0
          }  // end of (ACD ^ AC).AO >= 0
        } else {  // not AD.AO >= 0 / !a10.a11.a2.!a12
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <=
                0) {  // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.a6
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                       -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                            // !a10.a11.a2.!a12.a5.a6.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.!a6
              // Region AC
              originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }  // end of (ACD ^ AC).AO >= 0
          } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.!a5
            if (C.dot(a_cross_b) <=
                0) {  // if ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.a1
              assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                     dummy_precision);  // (ABC ^ AB).AO >= 0 /
                                        // !a10.a11.a2.!a12.!a5.a1.a4
              // Region ABC
              originToTriangle(current, a, b, c, (B - A).cross(C - A),
                               -C.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            } else {  // not ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.!a1
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <=
                       -dummy_precision));  // Not (ACD ^ AD).AO >= 0 /
                                            // !a10.a11.a2.!a12.!a5.!a1.!a7
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of ABC.AO >= 0
          }  // end of (ABC ^ AC).AO >= 0
        }  // end of AD.AO >= 0
      } else {                        // not ACD.AO >= 0 / !a10.a11.!a2
        if (C.dot(a_cross_b) <= 0) {  // if ABC.AO >= 0 / !a10.a11.!a2.a1
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <=
              0) {  // if (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.a5
            // Region AC
            originToSegment(current, a, c, A, C, C - A, -ca_aa, next, ray);
            free_v[nfree++] = current.vertex[b];
            free_v[nfree++] = current.vertex[d];
          } else {  // not (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.!a5
            assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                   dummy_precision);  // (ABC ^ AB).AO >= 0 /
                                      // !a10.a11.!a2.a1.!a5.a4
            // Region ABC
            originToTriangle(current, a, b, c, (B - A).cross(C - A),
                             -C.dot(a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          }  // end of (ABC ^ AC).AO >= 0
        } else {                         // not ABC.AO >= 0 / !a10.a11.!a2.!a1
          if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / !a10.a11.!a2.!a1.a3
            if (da * da_ba + dd * ba_aa - db * da_aa <=
                0) {  // if (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.a8
              // Region ADB
              originToTriangle(current, a, d, b, (D - A).cross(B - A),
                               D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.!a8
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of (ADB ^ AD).AO >= 0
          } else {  // not ADB.AO >= 0 / !a10.a11.!a2.!a1.!a3
            // Region Inside
            REGION_INSIDE()
          }  // end of ADB.AO >= 0
        }  // end of ABC.AO >= 0
      }  // end of ACD.AO >= 0
    } else {                           // not AC.AO >= 0 / !a10.!a11
      if (da_aa <= 0) {                // if AD.AO >= 0 / !a10.!a11.a12
        if (-D.dot(a_cross_b) <= 0) {  // if ADB.AO >= 0 / !a10.!a11.a12.a3
          if (da * ca_da + dc * da_aa - dd * ca_aa <=
              0) {  // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7
            if (da * da_ba + dd * ba_aa - db * da_aa <=
                0) {  // if (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.a8
              assert(!(ba * da_ba + bd * ba_aa - bb * da_aa <=
                       -dummy_precision));  // Not (ADB ^ AB).AO >= 0 /
                                            // !a10.!a11.a12.a3.a7.a8.!a9
              // Region ADB
              originToTriangle(current, a, d, b, (D - A).cross(B - A),
                               D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.!a8
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }  // end of (ADB ^ AD).AO >= 0
          } else {  // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.!a7
            if (D.dot(a_cross_c) <=
                0) {  // if ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.a2
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <=
                     dummy_precision);  // (ACD ^ AC).AO >= 0 /
                                        // !a10.!a11.a12.a3.!a7.a2.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {  // not ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2
              if (C.dot(a_cross_b) <=
                  0) {  // if ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.a1
                assert(!(ba * ba_ca + bb * ca_aa - bc * ba_aa <=
                         -dummy_precision));  // Not (ABC ^ AB).AO >= 0 /
                                              // !a10.!a11.a12.a3.!a7.!a2.a1.!a4
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {  // not ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.!a1
                // Region ADB
                originToTriangle(current, a, d, b, (D - A).cross(B - A),
                                 D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              }  // end of ABC.AO >= 0
            }  // end of ACD.AO >= 0
          }  // end of (ACD ^ AD).AO >= 0
        } else {                        // not ADB.AO >= 0 / !a10.!a11.a12.!a3
          if (D.dot(a_cross_c) <= 0) {  // if ACD.AO >= 0 / !a10.!a11.a12.!a3.a2
            if (da * ca_da + dc * da_aa - dd * ca_aa <=
                0) {  // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.a7
              // Region AD
              originToSegment(current, a, d, A, D, D - A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else {  // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.!a7
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <=
                     dummy_precision);  // (ACD ^ AC).AO >= 0 /
                                        // !a10.!a11.a12.!a3.a2.!a7.a6
              // Region ACD
              originToTriangle(current, a, c, d, (C - A).cross(D - A),
                               -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            }  // end of (ACD ^ AD).AO >= 0
          } else {  // not ACD.AO >= 0 / !a10.!a11.a12.!a3.!a2
            // Region Inside
            REGION_INSIDE()
          }  // end of ACD.AO >= 0
        }  // end of ADB.AO >= 0
      } else {  // not AD.AO >= 0 / !a10.!a11.!a12
        // Region A
        originToPoint(current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
        free_v[nfree++] = current.vertex[c];
        free_v[nfree++] = current.vertex[d];
      }  // end of AD.AO >= 0
    }  // end of AC.AO >= 0
  }  // end of AB.AO >= 0
  // clang-format on

#undef REGION_INSIDE
  return false;
}

void EPA::initialize() { reset(max_iterations, tolerance); }

void EPA::reset(size_t max_iterations_, FCL_REAL tolerance_) {
  max_iterations = max_iterations_;
  tolerance = tolerance_;
  // EPA creates only 2 faces and 1 vertex per iteration.
  // (+ the 4 (or 8 in the future) faces at the beginning
  //  + the 4 vertices (or 6 in the future) at the beginning)
  sv_store.resize(max_iterations + 4);
  fc_store.resize(2 * max_iterations + 4);
  status = DidNotRun;
  normal.setZero();
  depth = 0;
  closest_face = nullptr;
  result.reset();
  hull.reset();
  num_vertices = 0;
  stock.reset();
  // The stock is initialized with the faces in reverse order so that the
  // hull and the stock do not overlap (the stock will shring as the hull will
  // grow).
  for (size_t i = 0; i < fc_store.size(); ++i)
    stock.append(&fc_store[fc_store.size() - i - 1]);
  iterations = 0;
}

bool EPA::getEdgeDist(SimplexFace* face, const SimplexVertex& a,
                      const SimplexVertex& b, FCL_REAL& dist) {
  Vec3f ab = b.w - a.w;
  Vec3f n_ab = ab.cross(face->n);
  FCL_REAL a_dot_nab = a.w.dot(n_ab);

  if (a_dot_nab < 0)  // the origin is on the outside part of ab
  {
    // following is similar to projectOrigin for two points
    // however, as we dont need to compute the parameterization, dont need to
    // compute 0 or 1
    FCL_REAL a_dot_ab = a.w.dot(ab);
    FCL_REAL b_dot_ab = b.w.dot(ab);

    if (a_dot_ab > 0)
      dist = a.w.norm();
    else if (b_dot_ab < 0)
      dist = b.w.norm();
    else {
      dist = std::sqrt(std::max(
          a.w.squaredNorm() - a_dot_ab * a_dot_ab / ab.squaredNorm(), 0.));
    }

    return true;
  }

  return false;
}

EPA::SimplexFace* EPA::newFace(size_t id_a, size_t id_b, size_t id_c) {
  if (stock.root != nullptr) {
    SimplexFace* face = stock.root;
    stock.remove(face);
    hull.append(face);
    face->pass = 0;
    face->vertex_id[0] = id_a;
    face->vertex_id[1] = id_b;
    face->vertex_id[2] = id_c;
    const SimplexVertex& a = sv_store[id_a];
    const SimplexVertex& b = sv_store[id_b];
    const SimplexVertex& c = sv_store[id_c];
    face->n = (b.w - a.w).cross(c.w - a.w);

    if (face->n.norm() > Eigen::NumTraits<FCL_REAL>::epsilon()) {
      face->n.normalize();

      // If the origin projects outside the face, skip it in the
      // `findClosestFace` method.
      // The origin always projects inside the closest face.
      FCL_REAL a_dot_nab = a.w.dot((b.w - a.w).cross(face->n));
      FCL_REAL b_dot_nbc = b.w.dot((c.w - b.w).cross(face->n));
      FCL_REAL c_dot_nca = c.w.dot((a.w - c.w).cross(face->n));
      if (a_dot_nab >= -tolerance &&  //
          b_dot_nbc >= -tolerance &&  //
          c_dot_nca >= -tolerance) {
        face->d = a.w.dot(face->n);
        face->ignore = false;
      } else {
        // We will never check this face, so we don't care about
        // its true distance to the origin.
        face->d = std::numeric_limits<FCL_REAL>::max();
        face->ignore = true;
      }

      if (face->d >= -tolerance)
        return face;
      else
        status = NonConvex;
    } else
      status = Degenerated;

    hull.remove(face);
    stock.append(face);
    return nullptr;
  }

  assert(hull.count >= fc_store.size() && "EPA: should not be out of faces.");
  status = OutOfFaces;
  return nullptr;
}

/** @brief Find the best polytope face to split */
EPA::SimplexFace* EPA::findClosestFace() {
  SimplexFace* minf = hull.root;
  FCL_REAL mind = std::numeric_limits<FCL_REAL>::max();
  for (SimplexFace* f = minf; f; f = f->next_face) {
    if (f->ignore) continue;
    FCL_REAL sqd = f->d * f->d;
    if (sqd < mind) {
      minf = f;
      mind = sqd;
    }
  }
  assert(minf && !(minf->ignore) && "EPA: minf should not be flagged ignored.");
  return minf;
}

EPA::Status EPA::evaluate(GJK& gjk, const Vec3f& guess) {
  GJK::Simplex& simplex = *gjk.getSimplex();
  support_func_guess_t hint(gjk.support_hint);

  // TODO(louis): we might want to start with a hexahedron if the
  // simplex given by GJK is of rank <= 3.
  bool enclosed_origin = gjk.encloseOrigin();
  if ((simplex.rank > 1) && enclosed_origin) {
    assert(simplex.rank == 4 &&
           "When starting EPA, simplex should be of rank 4.");
    while (hull.root) {
      SimplexFace* f = hull.root;
      hull.remove(f);
      stock.append(f);
    }
    assert(hull.count == 0);
    assert(stock.count == fc_store.size());

    status = Valid;
    num_vertices = 0;

    // Make sure the tetrahedron has its normals pointing outside.
    if ((simplex.vertex[0]->w - simplex.vertex[3]->w)
            .dot((simplex.vertex[1]->w - simplex.vertex[3]->w)
                     .cross(simplex.vertex[2]->w - simplex.vertex[3]->w)) < 0) {
      SimplexVertex* tmp = simplex.vertex[0];
      simplex.vertex[0] = simplex.vertex[1];
      simplex.vertex[1] = tmp;
    }

    // Add the 4 vertices to sv_store
    for (size_t i = 0; i < 4; ++i) {
      sv_store[num_vertices++] = *simplex.vertex[i];
    }

    SimplexFace* tetrahedron[] = {newFace(0, 1, 2),  //
                                  newFace(1, 0, 3),  //
                                  newFace(2, 1, 3),  //
                                  newFace(0, 2, 3)};

    if (hull.count == 4) {
      // set the face connectivity
      bind(tetrahedron[0], 0, tetrahedron[1], 0);
      bind(tetrahedron[0], 1, tetrahedron[2], 0);
      bind(tetrahedron[0], 2, tetrahedron[3], 0);
      bind(tetrahedron[1], 1, tetrahedron[3], 2);
      bind(tetrahedron[1], 2, tetrahedron[2], 1);
      bind(tetrahedron[2], 2, tetrahedron[3], 1);

      closest_face =
          findClosestFace();  // find the best face (the face with the
                              // minimum distance to origin) to split
      SimplexFace outer = *closest_face;

      status = Valid;
      iterations = 0;
      size_t pass = 0;
      for (; iterations < max_iterations; ++iterations) {
        if (num_vertices >= sv_store.size()) {
          status = OutOfVertices;
          break;
        }

        // Step 1: find the support point in the direction of the closest_face
        // normal.
        // --------------------------------------------------------------------------
        SimplexHorizon horizon;
        SimplexVertex& w = sv_store[num_vertices++];
        bool valid = true;
        closest_face->pass = ++pass;
        // At the moment, SimplexF.n is always normalized. This could be revised
        // in the future...
        gjk.getSupport(closest_face->n, true, w, hint);

        // Step 2: check for convergence.
        // ------------------------------
        // Preambule to understand the convergence criterion of EPA:
        // the support we just added is in the direction of the normal of
        // the closest_face. Therefore, the support point will **always**
        // lie "after" the closest_face, i.e closest_face.n.dot(w.w) > 0.
        if (iterations > 0) {
          assert(closest_face->n.dot(w.w) > -tolerance &&
                 "The support is not in the right direction.");
        }
        //
        // 1) First check: `fdist` (see below) is an upper bound of how much
        // more penetration depth we can expect to "gain" by adding `w` to EPA's
        // polytope. This first check, as any convergence check, should be both
        // absolute and relative. This allows to adapt the tolerance to the
        // scale of the objects.
        const SimplexVertex& vf1 = sv_store[closest_face->vertex_id[0]];
        const SimplexVertex& vf2 = sv_store[closest_face->vertex_id[1]];
        const SimplexVertex& vf3 = sv_store[closest_face->vertex_id[2]];
        FCL_REAL fdist = closest_face->n.dot(w.w - vf1.w);
        FCL_REAL wnorm = w.w.norm();
        // TODO(louis): we might want to use tol_abs and tol_rel; this might
        // obfuscate the code for the user though.
        if (fdist <= tolerance + tolerance * wnorm) {
          status = AccuracyReached;
          break;
        }
        // 2) Second check: the expand function **assumes** that the support we
        // just computed is not a vertex of the face. We make sure that this
        // is the case:
        // TODO(louis): should we use squaredNorm everywhere instead of norm?
        if ((w.w - vf1.w).norm() <= tolerance + tolerance * wnorm ||
            (w.w - vf2.w).norm() <= tolerance + tolerance * wnorm ||
            (w.w - vf3.w).norm() <= tolerance + tolerance * wnorm) {
          status = AccuracyReached;
          break;
        }

        // Step 3: expand the polytope
        // ---------------------------
        for (size_t j = 0; (j < 3) && valid; ++j)
          valid &= expand(pass, w, closest_face->adjacent_faces[j],
                          closest_face->adjacent_edge[j], horizon);

        if (!valid || horizon.num_faces < 3) {
          // The status has already been set by the expand function.
          assert(!(status & Valid));
          break;
        }
        // need to add the edge connectivity between first and last faces
        bind(horizon.first_face, 2, horizon.current_face, 1);
        hull.remove(closest_face);
        stock.append(closest_face);
        closest_face = findClosestFace();
        outer = *closest_face;
      }

      status = ((iterations) < max_iterations) ? status : Failed;
      normal = outer.n;
      depth = outer.d;
      result.rank = 3;
      result.vertex[0] = &sv_store[outer.vertex_id[0]];
      result.vertex[1] = &sv_store[outer.vertex_id[1]];
      result.vertex[2] = &sv_store[outer.vertex_id[2]];
      return status;
    }
    assert(false && "The tetrahedron with which EPA started is degenerated.");
  }

  // FallBack when the simplex given by GJK is of rank 1.
  // Since the simplex only contains support points which convex
  // combination describe the origin, the point in the simplex is actually
  // the origin.
  status = FallBack;
  // TODO: define a better normal
  assert(simplex.rank == 1 && simplex.vertex[0]->w.isZero(gjk.getTolerance()));
  normal = -guess;
  FCL_REAL nl = normal.norm();
  if (nl > 0)
    normal /= nl;
  else
    normal = Vec3f(1, 0, 0);
  depth = 0;
  result.rank = 1;
  result.vertex[0] = simplex.vertex[0];
  return status;
}

// Use this function to debug `EPA::expand` if needed.
// void EPA::PrintExpandLooping(const SimplexFace* f, const SimplexVertex& w) {
//     std::cout << "Vertices:\n";
//     for (size_t i = 0; i < num_vertices; ++i) {
//       std::cout << "[";
//       std::cout << sv_store[i].w(0) << ", ";
//       std::cout << sv_store[i].w(1) << ", ";
//       std::cout << sv_store[i].w(2) << "]\n";
//     }
//     //
//     std::cout << "\nTriangles:\n";
//     SimplexFace* face = hull.root;
//     for (size_t i = 0; i < hull.count; ++i) {
//       std::cout << "[";
//       std::cout << face->vertex_id[0] << ", ";
//       std::cout << face->vertex_id[1] << ", ";
//       std::cout << face->vertex_id[2] << "]\n";
//       face = face->next_face;
//     }
//     //
//     std::cout << "\nNormals:\n";
//     face = hull.root;
//     for (size_t i = 0; i < hull.count; ++i) {
//       std::cout << "[";
//       std::cout << face->n(0) << ", ";
//       std::cout << face->n(1) << ", ";
//       std::cout << face->n(2) << "]\n";
//       face = face->next_face;
//     }
//     //
//     std::cout << "\nClosest face:\n";
//     face = hull.root;
//     for (size_t i = 0; i < hull.count; ++i) {
//       if (face == closest_face) {
//         std::cout << i << "\n";
//       }
//       face = face->next_face;
//     }
//     std::cout << "\nSupport point:\n";
//     std::cout << "[" << w.w(0) << ", " << w.w(1) << ", " << w.w(2) << "]\n";
// }

/** @brief the goal is to add a face connecting vertex w and face edge f[e] */
bool EPA::expand(size_t pass, const SimplexVertex& w, SimplexFace* f, size_t e,
                 SimplexHorizon& horizon) {
  static const size_t nexti[] = {1, 2, 0};
  static const size_t previ[] = {2, 0, 1};
  const size_t id_w =
      num_vertices - 1;  // w is always the last vertex added to sv_store

  // Check if we loop through expand indefinitely.
  if (f->pass == pass) {
    // Uncomment the following line and the associated EPA method
    // to debug the infinite loop if needed.
    // EPAPrintExpandLooping(this, f);
    assert(f != closest_face && "EPA is looping indefinitely.");
    status = InvalidHull;
    return false;
  }

  const size_t e1 = nexti[e];

  // Preambule: when expanding the polytope, the `closest_face` is always
  // deleted. This is handled in EPA::evaluate after calling the expand
  // function. This function handles how the neighboring face `f` of the
  // `closest_face` is connected to the new support point. (Note: because
  // `expand` is recursive, `f` can also denote a face of a face of the
  // `closest_face`, and so on. But the reasoning is the same.)
  //
  // EPA can handle `f` in two ways, depending on where the new support point
  // is located:
  // 1) If it is "below" `f`, then `f` is preserved. A new face is created
  //    and connects to the edge `e` of `f`. This new face is made of the
  //    two points of the edge `e` of `f` and the new support point `w`.
  //    Geometrically, this corresponds to the case where the projection of
  //    the origin on the `closest_face` is **inside** the triangle defined by
  //    the `closest_face`.
  // 2) If it is "above" `f`, then `f` has to be deleted, simply because the
  //    edge `e` of `f` is not part of the convex hull anymore.
  //    The two faces adjacent to `f` are thus expanded following
  //    either 1) or 2).
  //    Geometrically, this corresponds to the case where the projection of
  //    the origin on the `closest_face` is on an edge of the triangle defined
  //    by the `closest_face`. The projection of the origin cannot lie on a
  //    vertex of the `closest_face` because EPA should have exited before
  //    reaching this point.
  //
  // The following checks for these two cases.
  // This check is however subject to numerical precision and due to the
  // recursive nature of `expand`, it is safer to go through the first case.
  // This is because `expand` can potentially loop indefinitly if the
  // Minkowski difference is very flat (hence the check above).
  const FCL_REAL dummy_precision(
      3 * std::sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
  const SimplexVertex& vf = sv_store[f->vertex_id[e]];
  if (f->n.dot(w.w - vf.w) < dummy_precision) {
    // case 1: the support point is "below" `f`.
    SimplexFace* new_face = newFace(f->vertex_id[e1], f->vertex_id[e], id_w);
    if (new_face != nullptr) {
      // add face-face connectivity
      bind(new_face, 0, f, e);

      // if there is last face in the horizon, then need to add another
      // connectivity, i.e. the edge connecting the current new add edge and the
      // last new add edge. This does not finish all the connectivities because
      // the final face need to connect with the first face, this will be
      // handled in the evaluate function. Notice the face is anti-clockwise, so
      // the edges are 0 (bottom), 1 (right), 2 (left)
      if (horizon.current_face != nullptr) {
        bind(new_face, 2, horizon.current_face, 1);
      } else {
        horizon.first_face = new_face;
      }

      horizon.current_face = new_face;
      ++horizon.num_faces;
      return true;
    }
    return false;
  }

  // case 2: the support point is "above" `f`.
  const size_t e2 = previ[e];
  f->pass = pass;
  if (expand(pass, w, f->adjacent_faces[e1], f->adjacent_edge[e1], horizon) &&
      expand(pass, w, f->adjacent_faces[e2], f->adjacent_edge[e2], horizon)) {
    hull.remove(f);
    stock.append(f);
    return true;
  }
  return false;
}

bool EPA::getClosestPoints(const MinkowskiDiff& shape, Vec3f& w0,
                           Vec3f& w1) const {
  bool res = details::getClosestPoints(result, w0, w1);
  if (!res) return false;
  details::inflate<false>(shape, w0, w1);
  return true;
}

}  // namespace details

void ConvexBase::buildSupportWarmStart() {
  if (this->points->size() < ConvexBase::num_vertices_large_convex_threshold) {
    return;
  }

  this->support_warm_starts.points.reserve(ConvexBase::num_support_warm_starts);
  this->support_warm_starts.indices.reserve(
      ConvexBase::num_support_warm_starts);

  Vec3f axiis(0, 0, 0);
  for (int i = 0; i < 3; ++i) {
    axiis(i) = 1;
    {
      Vec3f support;
      int support_index{0};
      hpp::fcl::details::getShapeSupport(this, axiis, support, support_index,
                                         nullptr);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_index);
    }

    axiis(i) = -1;
    {
      Vec3f support;
      int support_index{0};
      hpp::fcl::details::getShapeSupport(this, axiis, support, support_index,
                                         nullptr);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_index);
    }

    axiis(i) = 0;
  }

  std::array<Vec3f, 4> eis = {Vec3f(1, 1, 1),    //
                              Vec3f(-1, 1, 1),   //
                              Vec3f(-1, -1, 1),  //
                              Vec3f(1, -1, 1)};

  for (size_t ei_index = 0; ei_index < 4; ++ei_index) {
    {
      Vec3f support;
      int support_index{0};
      hpp::fcl::details::getShapeSupport(this, eis[ei_index], support,
                                         support_index, nullptr);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_index);
    }

    {
      Vec3f support;
      int support_index{0};
      hpp::fcl::details::getShapeSupport(this, -eis[ei_index], support,
                                         support_index, nullptr);
      this->support_warm_starts.points.emplace_back(support);
      this->support_warm_starts.indices.emplace_back(support_index);
    }
  }

  if (this->support_warm_starts.points.size() !=
          ConvexBase::num_support_warm_starts ||
      this->support_warm_starts.indices.size() !=
          ConvexBase::num_support_warm_starts) {
    HPP_FCL_THROW_PRETTY("Wrong number of support warm starts.",
                         std::runtime_error);
  }
}

}  // namespace fcl

}  // namespace hpp
