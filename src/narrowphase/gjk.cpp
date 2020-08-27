/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
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

#include <hpp/fcl/narrowphase/gjk.h>
#include <hpp/fcl/internal/intersect.h>
#include <hpp/fcl/internal/tools.h>

namespace hpp
{
namespace fcl
{

namespace details
{

struct HPP_FCL_LOCAL shape_traits_base
{
  enum { NeedNormalizedDir = true
  };
};

template <typename Shape> struct HPP_FCL_LOCAL shape_traits : shape_traits_base {};

template <> struct HPP_FCL_LOCAL shape_traits<TriangleP> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct HPP_FCL_LOCAL shape_traits<Box> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct HPP_FCL_LOCAL shape_traits<Sphere> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct HPP_FCL_LOCAL shape_traits<Capsule> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct HPP_FCL_LOCAL shape_traits<Cone> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct HPP_FCL_LOCAL shape_traits<Cylinder> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct HPP_FCL_LOCAL shape_traits<ConvexBase> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

void getShapeSupport(const TriangleP* triangle, const Vec3f& dir, Vec3f& support, int&, MinkowskiDiff::ShapeData*)
{
  FCL_REAL dota = dir.dot(triangle->a);
  FCL_REAL dotb = dir.dot(triangle->b);
  FCL_REAL dotc = dir.dot(triangle->c);
  if(dota > dotb)
  {
    if(dotc > dota)
      support = triangle->c;
    else
      support = triangle->a;
  }
  else
  {
    if(dotc > dotb)
      support = triangle->c;
    else
      support = triangle->b;
  }
}

inline void getShapeSupport(const Box* box, const Vec3f& dir, Vec3f& support, int&, MinkowskiDiff::ShapeData*)
{
  const FCL_REAL inflate = (dir.array() == 0).any() ? 1.00000001 : 1.;
  support.noalias() = (dir.array() > 0).select(inflate * box->halfSide, -inflate * box->halfSide);
}

inline void getShapeSupport(const Sphere*, const Vec3f& /*dir*/, Vec3f& support, int&, MinkowskiDiff::ShapeData*)
{
  support.setZero();
}

inline void getShapeSupport(const Capsule* capsule, const Vec3f& dir, Vec3f& support, int&, MinkowskiDiff::ShapeData*)
{
  support.head<2>().setZero();
  if (dir[2] > 0) support[2] =   capsule->halfLength;
  else            support[2] = - capsule->halfLength;
}

void getShapeSupport(const Cone* cone, const Vec3f& dir, Vec3f& support, int&, MinkowskiDiff::ShapeData*)
{
  // The cone radius is, for -h < z < h, (h - z) * r / (2*h)
  static const FCL_REAL inflate = 1.00001;
  FCL_REAL h = cone->halfLength;
  FCL_REAL r = cone->radius;

  if (dir.head<2>().isZero()) {
    support.head<2>().setZero();
    if (dir[2] > 0)
      support[2] = h;
    else
      support[2] = - inflate * h;
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

  if(dir[2] > len * sin_a)
    support << 0, 0, h;
  else {
    FCL_REAL rad = r / zdist;
    support.head<2>() = rad * dir.head<2>();
    support[2] = -h;
  }
}

void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir, Vec3f& support, int&, MinkowskiDiff::ShapeData*)
{
  // The inflation makes the object look strictly convex to GJK and EPA. This
  // helps solving particular cases (e.g. a cylinder with itself at the same
  // position...)
  static const FCL_REAL inflate = 1.00001;
  FCL_REAL half_h = cylinder->halfLength;
  FCL_REAL r = cylinder->radius;

  if (dir.head<2>() == Eigen::Matrix<FCL_REAL,2,1>::Zero()) half_h *= inflate;

  if      (dir[2] > 0) support[2] =  half_h;
  else if (dir[2] < 0) support[2] = -half_h;
  else               { support[2] = 0; r *= inflate; }
  if (dir.head<2>() == Eigen::Matrix<FCL_REAL,2,1>::Zero())
    support.head<2>().setZero();
  else
    support.head<2>() = dir.head<2>().normalized() * r;
  assert (fabs (support [0] * dir [1] - support [1] * dir [0])
      < sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
}

struct SmallConvex : ShapeBase{};
struct LargeConvex : ShapeBase{};

void getShapeSupportLog(const ConvexBase* convex, const Vec3f& dir, Vec3f& support, int& hint, MinkowskiDiff::ShapeData* data)
{
  assert(data != NULL);

  const Vec3f* pts = convex->points;
  const ConvexBase::Neighbors* nn = convex->neighbors;

  if (hint < 0 || hint >= convex->num_points)
    hint = 0;
  FCL_REAL maxdot = pts[hint].dot(dir);
  FCL_REAL dot;
  std::vector<int8_t>& visited = data->visited;
  visited.assign(convex->num_points, false);
  visited[hint] = true;
  // when the first face is orthogonal to dir, all the dot products will be
  // equal. Yet, the neighbors must be visited.
  bool found = true, loose_check = true;
  while (found)
  {
    const ConvexBase::Neighbors& n = nn[hint];
    found = false;
    for (int in = 0; in < n.count(); ++in) {
      const int ip = n[in];
      if (visited[ip]) continue;
      visited[ip] = true;
      dot = pts[ip].dot(dir);
      bool better = false;
      if (dot > maxdot) {
        better = true;
        loose_check = false;
      } else if (loose_check && dot == maxdot)
        better = true;
      if (better) {
        maxdot = dot;
        hint = ip;
        found = true;
      }
    }
  }

  support = pts[hint];
}

void getShapeSupportLinear(const ConvexBase* convex, const Vec3f& dir, Vec3f& support, int& hint, MinkowskiDiff::ShapeData*)
{
  const Vec3f* pts = convex->points;

  hint = 0;
  FCL_REAL maxdot = pts[0].dot(dir);
  for (int i = 1; i < convex->num_points; ++i) {
    FCL_REAL dot = pts[i].dot(dir);
    if (dot > maxdot) {
      maxdot = dot;
      hint = i;
    }
  }
  support = pts[hint];
}

void getShapeSupport(const ConvexBase* convex, const Vec3f& dir, Vec3f& support, int& hint, MinkowskiDiff::ShapeData*)
{
  // TODO add benchmark to set a proper value for switching between linear and
  // logarithmic.
  if (convex->num_points > 32) {
    MinkowskiDiff::ShapeData data;
    getShapeSupportLog(convex, dir, support, hint, &data);
  }
  else
    getShapeSupportLinear(convex, dir, support, hint, NULL);
}

inline void getShapeSupport(const SmallConvex* convex, const Vec3f& dir, Vec3f& support, int& hint, MinkowskiDiff::ShapeData* data)
{
  getShapeSupportLinear(reinterpret_cast<const ConvexBase*>(convex), dir, support, hint, data);
}

inline void getShapeSupport(const LargeConvex* convex, const Vec3f& dir, Vec3f& support, int& hint, MinkowskiDiff::ShapeData* data)
{
  getShapeSupportLog(reinterpret_cast<const ConvexBase*>(convex), dir, support, hint, data);
}

#define CALL_GET_SHAPE_SUPPORT(ShapeType)                                      \
  getShapeSupport (static_cast<const ShapeType*>(shape),                       \
      (shape_traits<ShapeType>::NeedNormalizedDir && !dirIsNormalized)         \
      ? dir.normalized() : dir,                                                \
      support, hint, NULL)

Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, bool dirIsNormalized, int& hint)
{
  Vec3f support;
  switch(shape->getNodeType())
  {
  case GEOM_TRIANGLE:
    CALL_GET_SHAPE_SUPPORT(TriangleP);
    break;
  case GEOM_BOX:
    CALL_GET_SHAPE_SUPPORT(Box);
    break;
  case GEOM_SPHERE:
    CALL_GET_SHAPE_SUPPORT(Sphere);
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
    ; // nothing
  }

  return support;
}

#undef CALL_GET_SHAPE_SUPPORT

template <typename Shape0, typename Shape1, bool TransformIsIdentity>
void getSupportTpl (const Shape0* s0, const Shape1* s1,
    const Matrix3f& oR1, const Vec3f& ot1,
    const Vec3f& dir, Vec3f& support0, Vec3f& support1,
    support_func_guess_t& hint, MinkowskiDiff::ShapeData data[2])
{
  getShapeSupport (s0, dir, support0, hint[0], &(data[0]));
  if (TransformIsIdentity)
    getShapeSupport (s1, - dir, support1, hint[1], &(data[1]));
  else {
    getShapeSupport (s1, - oR1.transpose() * dir, support1, hint[1], &(data[1]));
    support1 = oR1 * support1 + ot1;
  }
}

template <typename Shape0, typename Shape1, bool TransformIsIdentity>
void getSupportFuncTpl (const MinkowskiDiff& md,
    const Vec3f& dir, bool dirIsNormalized, Vec3f& support0, Vec3f& support1,
    support_func_guess_t& hint, MinkowskiDiff::ShapeData data[2])
{
  enum { NeedNormalizedDir =
    bool ( (bool)shape_traits<Shape0>::NeedNormalizedDir
        || (bool)shape_traits<Shape1>::NeedNormalizedDir)
  };
#ifndef NDEBUG
  // Need normalized direction and direction is normalized
  assert(!NeedNormalizedDir || !dirIsNormalized || fabs(dir.squaredNorm() - 1) < 1e-6);
  // Need normalized direction but direction is not normalized.
  assert(!NeedNormalizedDir ||  dirIsNormalized || fabs(dir.normalized().squaredNorm() - 1) < 1e-6);
  // Don't need normalized direction. Check that dir is not zero.
  assert( NeedNormalizedDir || dir.cwiseAbs().maxCoeff() >= 1e-6);
#endif
  getSupportTpl<Shape0, Shape1, TransformIsIdentity> (
      static_cast <const Shape0*>(md.shapes[0]),
      static_cast <const Shape1*>(md.shapes[1]),
      md.oR1, md.ot1,
      (NeedNormalizedDir && !dirIsNormalized) ? dir.normalized() : dir,
      support0, support1, hint, data);
}

template <typename Shape0>
MinkowskiDiff::GetSupportFunction makeGetSupportFunction1 (const ShapeBase* s1, bool identity,
    Eigen::Array<FCL_REAL, 1, 2>& inflation, int linear_log_convex_threshold)
{
  inflation[1] = 0;
  switch(s1->getNodeType())
  {
  case GEOM_TRIANGLE:
    if (identity) return getSupportFuncTpl<Shape0, TriangleP, true >;
    else          return getSupportFuncTpl<Shape0, TriangleP, false>;
  case GEOM_BOX:
    if (identity) return getSupportFuncTpl<Shape0, Box, true >;
    else          return getSupportFuncTpl<Shape0, Box, false>;
  case GEOM_SPHERE:
    inflation[1] = static_cast<const Sphere*>(s1)->radius;
    if (identity) return getSupportFuncTpl<Shape0, Sphere, true >;
    else          return getSupportFuncTpl<Shape0, Sphere, false>;
  case GEOM_CAPSULE:
    inflation[1] = static_cast<const Capsule*>(s1)->radius;
    if (identity) return getSupportFuncTpl<Shape0, Capsule, true >;
    else          return getSupportFuncTpl<Shape0, Capsule, false>;
  case GEOM_CONE:
    if (identity) return getSupportFuncTpl<Shape0, Cone, true >;
    else          return getSupportFuncTpl<Shape0, Cone, false>;
  case GEOM_CYLINDER:
    if (identity) return getSupportFuncTpl<Shape0, Cylinder, true >;
    else          return getSupportFuncTpl<Shape0, Cylinder, false>;
  case GEOM_CONVEX:
    if (static_cast<const ConvexBase*>(s1)->num_points > linear_log_convex_threshold) {
      if (identity) return getSupportFuncTpl<Shape0, LargeConvex, true >;
      else          return getSupportFuncTpl<Shape0, LargeConvex, false>;
    } else {
      if (identity) return getSupportFuncTpl<Shape0, SmallConvex, true >;
      else          return getSupportFuncTpl<Shape0, SmallConvex, false>;
    }
  default:
    throw std::logic_error ("Unsupported geometric shape");
  }
}

MinkowskiDiff::GetSupportFunction makeGetSupportFunction0 (const ShapeBase* s0, const ShapeBase* s1, bool identity,
    Eigen::Array<FCL_REAL, 1, 2>& inflation, int linear_log_convex_threshold)
{
  inflation[0] = 0;
  switch(s0->getNodeType())
  {
  case GEOM_TRIANGLE:
    return makeGetSupportFunction1<TriangleP> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  case GEOM_BOX:
    return makeGetSupportFunction1<Box> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  case GEOM_SPHERE:
    inflation[0] = static_cast<const Sphere*>(s0)->radius;
    return makeGetSupportFunction1<Sphere> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  case GEOM_CAPSULE:
    inflation[0] = static_cast<const Capsule*>(s0)->radius;
    return makeGetSupportFunction1<Capsule> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  case GEOM_CONE:
    return makeGetSupportFunction1<Cone> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  case GEOM_CYLINDER:
    return makeGetSupportFunction1<Cylinder> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  case GEOM_CONVEX:
    if (static_cast<const ConvexBase*>(s0)->num_points > linear_log_convex_threshold)
      return makeGetSupportFunction1<LargeConvex> (s1, identity, inflation, linear_log_convex_threshold);
    else
      return makeGetSupportFunction1<SmallConvex> (s1, identity, inflation, linear_log_convex_threshold);
    break;
  default:
    throw std::logic_error ("Unsupported geometric shape");
  }
}

void MinkowskiDiff::set (const ShapeBase* shape0, const ShapeBase* shape1,
    const Transform3f& tf0, const Transform3f& tf1)
{
  shapes[0] = shape0;
  shapes[1] = shape1;

  oR1 = tf0.getRotation().transpose() * tf1.getRotation();
  ot1 = tf0.getRotation().transpose() * (tf1.getTranslation() - tf0.getTranslation());

  bool identity = (oR1.isIdentity() && ot1.isZero());

  getSupportFunc = makeGetSupportFunction0 (shape0, shape1, identity, inflation, linear_log_convex_threshold);
}

void MinkowskiDiff::set (const ShapeBase* shape0, const ShapeBase* shape1)
{
  shapes[0] = shape0;
  shapes[1] = shape1;

  oR1.setIdentity();
  ot1.setZero();

  getSupportFunc = makeGetSupportFunction0 (shape0, shape1, true, inflation, linear_log_convex_threshold);
}

void GJK::initialize()
{
  nfree = 0;
  status = Failed;
  distance_upper_bound = std::numeric_limits<FCL_REAL>::max();
  simplex = NULL;
}

Vec3f GJK::getGuessFromSimplex() const
{
  return ray;
}

namespace details {

bool getClosestPoints (const GJK::Simplex& simplex, Vec3f& w0, Vec3f& w1)
{
  GJK::SimplexV* const* vs = simplex.vertex;

  for (GJK::vertex_id_t i = 0; i < simplex.rank; ++i) {
    assert (vs[i]->w.isApprox (vs[i]->w0 - vs[i]->w1));
  }

  Project::ProjectResult projection;
  switch (simplex.rank) {
    case 1:
      w0 = vs[0]->w0;
      w1 = vs[0]->w1;
      return true;
    case 2:
      {
        const Vec3f& a  = vs[0]->w, a0 = vs[0]->w0, a1 = vs[0]->w1,
                     b  = vs[1]->w, b0 = vs[1]->w0, b1 = vs[1]->w1;
        FCL_REAL la, lb;
        Vec3f N (b - a);
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
      projection = Project::projectTriangleOrigin   (vs[0]->w, vs[1]->w, vs[2]->w);
      break;
    case 4: // We are in collision.
      projection = Project::projectTetrahedraOrigin (vs[0]->w, vs[1]->w, vs[2]->w, vs[3]->w);
      break;
    default:
      throw std::logic_error ("The simplex rank must be in [ 1, 4 ]");
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
void inflate (const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1)
{
  const Eigen::Array<FCL_REAL, 1, 2>& I (shape.inflation);
  Eigen::Array<bool, 1, 2> inflate (I > 0);
  if (!inflate.any()) return;
  Vec3f w (w0 - w1);
  FCL_REAL n2 = w.squaredNorm();
  // TODO should be use a threshold (Eigen::NumTraits<FCL_REAL>::epsilon()) ?
  if (n2 == 0.) {
    if (inflate[0]) w0[0] += I[0] * (Separated ? -1 :  1);
    if (inflate[1]) w1[0] += I[1] * (Separated ?  1 : -1);
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

} // namespace details

bool GJK::getClosestPoints (const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1)
{
  bool res = details::getClosestPoints(*simplex, w0, w1);
  if (!res) return false;
  details::inflate<true> (shape, w0, w1);
  return true;
}

GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess,
    const support_func_guess_t& supportHint)
{
  size_t iterations = 0;
  FCL_REAL alpha = 0;
  const FCL_REAL inflation = shape_.inflation.sum();
  const FCL_REAL upper_bound = distance_upper_bound + inflation;

  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];
    
  nfree = 4;
  current = 0;
  status = Valid;
  shape = &shape_;
  distance = 0.0;
  simplices[0].rank = 0;
  support_hint = supportHint;

  FCL_REAL rl = guess.norm();
  if (rl < tolerance) {
    ray = Vec3f(-1,0,0);
    rl = 1;
  } else
    ray = guess;

  do
  {
    vertex_id_t next = (vertex_id_t)(1 - current);
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    // TODO this is an early stop which may cause the following issue.
    // - EPA will not run correctly because it starts with a tetrahedron which
    //   does not include the origin. Note that, at this stage, we do not know
    //   whether a tetrahedron including the origin exists.
    if(rl < tolerance) // mean origin is near the face of original simplex, return touch
    {
      assert(rl > 0);
      status = Inside;
      distance = - inflation; // should we take rl into account ?
      break;
    }

    appendVertex(curr_simplex, -ray, false, support_hint); // see below, ray points away from origin

    // check removed (by ?): when the new support point is close to previous support points, stop (as the new simplex is degenerated)
    const Vec3f& w = curr_simplex.vertex[curr_simplex.rank - 1]->w;

    // check B: no collision if omega > 0
    FCL_REAL omega = ray.dot(w) / rl;
    if (omega > upper_bound)
    {
      distance = omega - inflation;
      break;
    }

    // check C: when the new support point is close to the sub-simplex where the ray point lies, stop (as the new simplex again is degenerated)
    alpha = std::max(alpha, omega);
    FCL_REAL diff (rl - alpha);
    if (iterations == 0) diff = std::abs(diff);
    // TODO here, we can stop at iteration 0 if this condition is met.
    // We stopping at iteration 0, the closest point will not be valid.
    // if(diff - tolerance * rl <= 0)
    if(iterations > 0 && diff - tolerance * rl <= 0)
    {
      if (iterations > 0)
        removeVertex(simplices[current]);
      distance = rl - inflation;
      // TODO When inflation is strictly positive, the distance may be exactly
      // zero (so the ray is not zero) and we are not in the case rl < tolerance.
      if (distance < tolerance)
        status = Inside;
      break;
    }

    // This has been rewritten thanks to the excellent video:
    // https://youtu.be/Qupqu1xe7Io
    bool inside;
    switch(curr_simplex.rank)
    {
    case 1: // Only at the first iteration
      assert(iterations == 0);
      ray = w;
      inside = false;
      next_simplex.rank = 1;
      next_simplex.vertex[0] = curr_simplex.vertex[0];
      break;
    case 2:
      inside = projectLineOrigin (curr_simplex, next_simplex);
      break;
    case 3:
      inside = projectTriangleOrigin (curr_simplex, next_simplex);
      break;
    case 4:
      inside = projectTetrahedraOrigin (curr_simplex, next_simplex);
      break;
    default:
      throw std::logic_error("Invalid simplex rank");
    }
    assert (nfree+next_simplex.rank == 4);
    current = next;
    if (!inside)
      rl = ray.norm();
    if(inside || rl == 0) {
      status = Inside;
      distance = - inflation - 1.;
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;
      
  } while(status == Valid);

  simplex = &simplices[current];
  assert(simplex->rank > 0 && simplex->rank < 5);
  return status;
}

inline void GJK::removeVertex(Simplex& simplex)
{
  free_v[nfree++] = simplex.vertex[--simplex.rank];
}

inline void GJK::appendVertex(Simplex& simplex, const Vec3f& v, bool isNormalized, support_func_guess_t& hint)
{
  simplex.vertex[simplex.rank] = free_v[--nfree]; // set the memory
  getSupport (v, isNormalized, *simplex.vertex[simplex.rank++], hint);
}

bool GJK::encloseOrigin()
{
  Vec3f axis(Vec3f::Zero());
  support_func_guess_t hint = support_func_guess_t::Zero();
  switch(simplex->rank)
  {
  case 1:
    for(size_t i = 0; i < 3; ++i)
    {
      axis[i] = 1;
      appendVertex(*simplex, axis, true, hint);
      if(encloseOrigin()) return true;
      removeVertex(*simplex);
      axis[i] = -1;
      appendVertex(*simplex, -axis, true, hint);
      if(encloseOrigin()) return true;
      removeVertex(*simplex);
      axis[i] = 0;
    }
    break;
  case 2:
    {
      Vec3f d = simplex->vertex[1]->w - simplex->vertex[0]->w;
      for(size_t i = 0; i < 3; ++i)
      {
        axis[i] = 1;
        Vec3f p = d.cross(axis);
        if(!p.isZero())
        {
          appendVertex(*simplex, p, false, hint);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
          appendVertex(*simplex, -p, false, hint);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
        }
        axis[i] = 0;
      }
    }
    break;
  case 3:
    axis.noalias() =
      (simplex->vertex[1]->w - simplex->vertex[0]->w).cross
      (simplex->vertex[2]->w - simplex->vertex[0]->w);
    if(!axis.isZero())
    {
      appendVertex(*simplex, axis, false, hint);
      if(encloseOrigin()) return true;
      removeVertex(*simplex);
      appendVertex(*simplex, -axis, false, hint);
      if(encloseOrigin()) return true;
      removeVertex(*simplex);
    }
    break;
  case 4:
    if(std::abs(triple(simplex->vertex[0]->w - simplex->vertex[3]->w,
                       simplex->vertex[1]->w - simplex->vertex[3]->w,
                       simplex->vertex[2]->w - simplex->vertex[3]->w)) > 0)
      return true;
    break;
  }

  return false;
}

inline void originToPoint (
    const GJK::Simplex& current, GJK::vertex_id_t a,
    const Vec3f& A,
    GJK::Simplex& next,
    Vec3f& ray)
{
    // A is the closest to the origin
    ray = A;
    next.vertex[0] = current.vertex[a];
    next.rank = 1;
}

inline void originToSegment (
    const GJK::Simplex& current, GJK::vertex_id_t a, GJK::vertex_id_t b,
    const Vec3f& A, const Vec3f& B,
    const Vec3f& AB,
    const FCL_REAL& ABdotAO,
    GJK::Simplex& next,
    Vec3f& ray)
{
    // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
    ray = AB.dot(B) * A + ABdotAO * B;

    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[a];
    next.rank = 2;

    // To ensure backward compatibility
    ray /= AB.squaredNorm();
}

inline bool originToTriangle (
    const GJK::Simplex& current,
    GJK::vertex_id_t a, GJK::vertex_id_t b, GJK::vertex_id_t c,
    const Vec3f& ABC,
    const FCL_REAL& ABCdotAO,
    GJK::Simplex& next,
    Vec3f& ray)
{
  next.rank = 3;
  next.vertex[2] = current.vertex[a];

  if (ABCdotAO == 0) {
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
    ray.setZero();
    return true;
  }
  if (ABCdotAO > 0) { // Above triangle
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
  } else {
    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[c];
  }

  // To ensure backward compatibility
  ray = - ABCdotAO / ABC.squaredNorm() * ABC;
  return false;
}

bool GJK::projectLineOrigin(const Simplex& current, Simplex& next)
{
  const vertex_id_t a = 1, b = 0;
  // A is the last point we added.
  const Vec3f& A = current.vertex[a]->w;
  const Vec3f& B = current.vertex[b]->w;

  const Vec3f AB = B - A;
  const FCL_REAL d = AB.dot(-A);
  assert (d <= AB.squaredNorm());

  if (d == 0) {
    // Two extremely unlikely cases:
    // - AB is orthogonal to A: should never happen because it means the support
    //   function did not do any progress and GJK should have stopped.
    // - A == origin
    // In any case, A is the closest to the origin
    originToPoint (current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
    return A.isZero();
  } else if (d < 0) {
    // A is the closest to the origin
    originToPoint (current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
  } else
    originToSegment (current, a, b, A, B, AB, d, next, ray);
  return false;
}

bool GJK::projectTriangleOrigin(const Simplex& current, Simplex& next)
{
  const vertex_id_t a = 2, b = 1, c = 0;
  // A is the last point we added.
  const Vec3f& A = current.vertex[a]->w,
               B = current.vertex[b]->w,
               C = current.vertex[c]->w;

  const Vec3f AB = B - A,
              AC = C - A,
              ABC = AB.cross(AC);

  FCL_REAL edgeAC2o = ABC.cross(AC).dot (-A);
  if (edgeAC2o >= 0) {
    FCL_REAL towardsC = AC.dot (-A);
    if (towardsC >= 0) { // Region 1
      originToSegment (current, a, c, A, C, AC, towardsC, next, ray);
      free_v[nfree++] = current.vertex[b];
    } else { // Region 4 or 5
      FCL_REAL towardsB = AB.dot(-A);
      if (towardsB < 0) { // Region 5
        // A is the closest to the origin
        originToPoint (current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else // Region 4
        originToSegment (current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    }
  } else {
    FCL_REAL edgeAB2o = AB.cross(ABC).dot (-A);
    if (edgeAB2o >= 0) { // Region 4 or 5
      FCL_REAL towardsB = AB.dot(-A);
      if (towardsB < 0) { // Region 5
        // A is the closest to the origin
        originToPoint (current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
      } else // Region 4
        originToSegment (current, a, b, A, B, AB, towardsB, next, ray);
      free_v[nfree++] = current.vertex[c];
    } else {
      return originToTriangle (current, a, b, c, ABC, ABC.dot(-A), next, ray);
    }
  }
  return false;
}

bool GJK::projectTetrahedraOrigin(const Simplex& current, Simplex& next)
{
  // The code of this function was generated using doc/gjk.py
  const vertex_id_t a = 3, b = 2, c = 1, d = 0;
  const Vec3f& A (current.vertex[a]->w);
  const Vec3f& B (current.vertex[b]->w);
  const Vec3f& C (current.vertex[c]->w);
  const Vec3f& D (current.vertex[d]->w);
  const FCL_REAL aa = A.squaredNorm();
  const FCL_REAL da    = D.dot(A);
  const FCL_REAL db    = D.dot(B);
  const FCL_REAL dc    = D.dot(C);
  const FCL_REAL dd    = D.dot(D);
  const FCL_REAL da_aa = da - aa;
  const FCL_REAL ca    = C.dot(A);
  const FCL_REAL cb    = C.dot(B);
  const FCL_REAL cc    = C.dot(C);
  const FCL_REAL& cd    = dc;
  const FCL_REAL ca_aa = ca - aa;
  const FCL_REAL ba    = B.dot(A);
  const FCL_REAL bb    = B.dot(B);
  const FCL_REAL& bc    = cb;
  const FCL_REAL& bd    = db;
  const FCL_REAL ba_aa = ba - aa;
  const FCL_REAL ba_ca = ba - ca;
  const FCL_REAL ca_da = ca - da;
  const FCL_REAL da_ba = da - ba;
  const Vec3f a_cross_b = A.cross(B);
  const Vec3f a_cross_c = A.cross(C);

#define REGION_INSIDE()                 \
    ray.setZero();                      \
    next.vertex[0] = current.vertex[d]; \
    next.vertex[1] = current.vertex[c]; \
    next.vertex[2] = current.vertex[b]; \
    next.vertex[3] = current.vertex[a]; \
    next.rank=4;                        \
    return true;

  if (ba_aa <= 0) { // if AB.AO >= 0 / a10
    if (-D.dot(a_cross_b) <= 0) { // if ADB.AO >= 0 / a10.a3
      if (ba * da_ba + bd * ba_aa - bb * da_aa <= 0) { // if (ADB ^ AB).AO >= 0 / a10.a3.a9
        if (da_aa <= 0) { // if AD.AO >= 0 / a10.a3.a9.a12
          assert(da * da_ba + dd * ba_aa - db * da_aa <= 0); // (ADB ^ AD).AO >= 0 / a10.a3.a9.a12.a8
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <= 0) { // if (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.a4
            // Region ABC
            originToTriangle (current, a, b, c, (B-A).cross(C-A), -C.dot (a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          } else { // not (ABC ^ AB).AO >= 0 / a10.a3.a9.a12.a8.!a4
            // Region AB
            originToSegment (current, a, b, A, B, B-A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          } // end of (ABC ^ AB).AO >= 0
        } else { // not AD.AO >= 0 / a10.a3.a9.!a12
          if (ba * ba_ca + bb * ca_aa - bc * ba_aa <= 0) { // if (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.a4
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <= 0) { // if (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5
              if (ca * ca_da + cc * da_aa - cd * ca_aa <= 0) { // if (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.a6
                // Region ACD
                originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
                free_v[nfree++] = current.vertex[b];
              } else { // not (ACD ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.a5.!a6
                // Region AC
                originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              } // end of (ACD ^ AC).AO >= 0
            } else { // not (ABC ^ AC).AO >= 0 / a10.a3.a9.!a12.a4.!a5
              // Region ABC
              originToTriangle (current, a, b, c, (B-A).cross(C-A), -C.dot (a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            } // end of (ABC ^ AC).AO >= 0
          } else { // not (ABC ^ AB).AO >= 0 / a10.a3.a9.!a12.!a4
            // Region AB
            originToSegment (current, a, b, A, B, B-A, -ba_aa, next, ray);
            free_v[nfree++] = current.vertex[c];
            free_v[nfree++] = current.vertex[d];
          } // end of (ABC ^ AB).AO >= 0
        } // end of AD.AO >= 0
      } else { // not (ADB ^ AB).AO >= 0 / a10.a3.!a9
        if (da * da_ba + dd * ba_aa - db * da_aa <= 0) { // if (ADB ^ AD).AO >= 0 / a10.a3.!a9.a8
          // Region ADB
          originToTriangle (current, a, d, b, (D-A).cross(B-A), D.dot(a_cross_b), next, ray);
          free_v[nfree++] = current.vertex[c];
        } else { // not (ADB ^ AD).AO >= 0 / a10.a3.!a9.!a8
          if (ca * ca_da + cc * da_aa - cd * ca_aa <= 0) { // if (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <= 0) { // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.a7
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else { // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.a6.!a7
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } // end of (ACD ^ AD).AO >= 0
          } else { // not (ACD ^ AC).AO >= 0 / a10.a3.!a9.!a8.!a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <= 0) { // if (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.a7
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else { // not (ACD ^ AD).AO >= 0 / a10.a3.!a9.!a8.!a6.!a7
              // Region AC
              originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } // end of (ACD ^ AD).AO >= 0
          } // end of (ACD ^ AC).AO >= 0
        } // end of (ADB ^ AD).AO >= 0
      } // end of (ADB ^ AB).AO >= 0
    } else { // not ADB.AO >= 0 / a10.!a3
      if (C.dot (a_cross_b) <= 0) { // if ABC.AO >= 0 / a10.!a3.a1
        if (ba * ba_ca + bb * ca_aa - bc * ba_aa <= 0) { // if (ABC ^ AB).AO >= 0 / a10.!a3.a1.a4
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <= 0) { // if (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <= 0) { // if (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.a6
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else { // not (ACD ^ AC).AO >= 0 / a10.!a3.a1.a4.a5.!a6
              // Region AC
              originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } // end of (ACD ^ AC).AO >= 0
          } else { // not (ABC ^ AC).AO >= 0 / a10.!a3.a1.a4.!a5
            // Region ABC
            originToTriangle (current, a, b, c, (B-A).cross(C-A), -C.dot (a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          } // end of (ABC ^ AC).AO >= 0
        } else { // not (ABC ^ AB).AO >= 0 / a10.!a3.a1.!a4
          // Region AB
          originToSegment (current, a, b, A, B, B-A, -ba_aa, next, ray);
          free_v[nfree++] = current.vertex[c];
          free_v[nfree++] = current.vertex[d];
        } // end of (ABC ^ AB).AO >= 0
      } else { // not ABC.AO >= 0 / a10.!a3.!a1
        if (D.dot(a_cross_c) <= 0) { // if ACD.AO >= 0 / a10.!a3.!a1.a2
          if (ca * ca_da + cc * da_aa - cd * ca_aa <= 0) { // if (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <= 0) { // if (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.a7
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else { // not (ACD ^ AD).AO >= 0 / a10.!a3.!a1.a2.a6.!a7
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } // end of (ACD ^ AD).AO >= 0
          } else { // not (ACD ^ AC).AO >= 0 / a10.!a3.!a1.a2.!a6
            if (ca_aa <= 0) { // if AC.AO >= 0 / a10.!a3.!a1.a2.!a6.a11
              // Region AC
              originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else { // not AC.AO >= 0 / a10.!a3.!a1.a2.!a6.!a11
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } // end of AC.AO >= 0
          } // end of (ACD ^ AC).AO >= 0
        } else { // not ACD.AO >= 0 / a10.!a3.!a1.!a2
          // Region Inside
          REGION_INSIDE()
        } // end of ACD.AO >= 0
      } // end of ABC.AO >= 0
    } // end of ADB.AO >= 0
  } else { // not AB.AO >= 0 / !a10
    if (ca_aa <= 0) { // if AC.AO >= 0 / !a10.a11
      if (D.dot(a_cross_c) <= 0) { // if ACD.AO >= 0 / !a10.a11.a2
        if (da_aa <= 0) { // if AD.AO >= 0 / !a10.a11.a2.a12
          if (ca * ca_da + cc * da_aa - cd * ca_aa <= 0) { // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.a6
            if (da * ca_da + dc * da_aa - dd * ca_aa <= 0) { // if (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7
              if (da * da_ba + dd * ba_aa - db * da_aa <= 0) { // if (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.a8
                // Region ADB
                originToTriangle (current, a, d, b, (D-A).cross(B-A), D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else { // not (ADB ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.a7.!a8
                // Region AD
                originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              } // end of (ADB ^ AD).AO >= 0
            } else { // not (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.a6.!a7
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } // end of (ACD ^ AD).AO >= 0
          } else { // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6
            assert(!(da * ca_da + dc * da_aa - dd * ca_aa <= 0)); // Not (ACD ^ AD).AO >= 0 / !a10.a11.a2.a12.!a6.!a7
            if (ca * ba_ca + cb * ca_aa - cc * ba_aa <= 0) { // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.a5
              // Region AC
              originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else { // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.a12.!a6.!a7.!a5
              // Region ABC
              originToTriangle (current, a, b, c, (B-A).cross(C-A), -C.dot (a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            } // end of (ABC ^ AC).AO >= 0
          } // end of (ACD ^ AC).AO >= 0
        } else { // not AD.AO >= 0 / !a10.a11.a2.!a12
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <= 0) { // if (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5
            if (ca * ca_da + cc * da_aa - cd * ca_aa <= 0) { // if (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.a6
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <= 0)); // Not (ACD ^ AD).AO >= 0 / !a10.a11.a2.!a12.a5.a6.!a7
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else { // not (ACD ^ AC).AO >= 0 / !a10.a11.a2.!a12.a5.!a6
              // Region AC
              originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } // end of (ACD ^ AC).AO >= 0
          } else { // not (ABC ^ AC).AO >= 0 / !a10.a11.a2.!a12.!a5
            if (C.dot (a_cross_b) <= 0) { // if ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.a1
              assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <= 0); // (ABC ^ AB).AO >= 0 / !a10.a11.a2.!a12.!a5.a1.a4
              // Region ABC
              originToTriangle (current, a, b, c, (B-A).cross(C-A), -C.dot (a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[d];
            } else { // not ABC.AO >= 0 / !a10.a11.a2.!a12.!a5.!a1
              assert(!(da * ca_da + dc * da_aa - dd * ca_aa <= 0)); // Not (ACD ^ AD).AO >= 0 / !a10.a11.a2.!a12.!a5.!a1.!a7
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } // end of ABC.AO >= 0
          } // end of (ABC ^ AC).AO >= 0
        } // end of AD.AO >= 0
      } else { // not ACD.AO >= 0 / !a10.a11.!a2
        if (C.dot (a_cross_b) <= 0) { // if ABC.AO >= 0 / !a10.a11.!a2.a1
          if (ca * ba_ca + cb * ca_aa - cc * ba_aa <= 0) { // if (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.a5
            // Region AC
            originToSegment (current, a, c, A, C, C-A, -ca_aa, next, ray);
            free_v[nfree++] = current.vertex[b];
            free_v[nfree++] = current.vertex[d];
          } else { // not (ABC ^ AC).AO >= 0 / !a10.a11.!a2.a1.!a5
            assert(ba * ba_ca + bb * ca_aa - bc * ba_aa <= 0); // (ABC ^ AB).AO >= 0 / !a10.a11.!a2.a1.!a5.a4
            // Region ABC
            originToTriangle (current, a, b, c, (B-A).cross(C-A), -C.dot (a_cross_b), next, ray);
            free_v[nfree++] = current.vertex[d];
          } // end of (ABC ^ AC).AO >= 0
        } else { // not ABC.AO >= 0 / !a10.a11.!a2.!a1
          if (-D.dot(a_cross_b) <= 0) { // if ADB.AO >= 0 / !a10.a11.!a2.!a1.a3
            if (da * da_ba + dd * ba_aa - db * da_aa <= 0) { // if (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.a8
              // Region ADB
              originToTriangle (current, a, d, b, (D-A).cross(B-A), D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else { // not (ADB ^ AD).AO >= 0 / !a10.a11.!a2.!a1.a3.!a8
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } // end of (ADB ^ AD).AO >= 0
          } else { // not ADB.AO >= 0 / !a10.a11.!a2.!a1.!a3
            // Region Inside
            REGION_INSIDE()
          } // end of ADB.AO >= 0
        } // end of ABC.AO >= 0
      } // end of ACD.AO >= 0
    } else { // not AC.AO >= 0 / !a10.!a11
      if (da_aa <= 0) { // if AD.AO >= 0 / !a10.!a11.a12
        if (-D.dot(a_cross_b) <= 0) { // if ADB.AO >= 0 / !a10.!a11.a12.a3
          if (da * ca_da + dc * da_aa - dd * ca_aa <= 0) { // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7
            if (da * da_ba + dd * ba_aa - db * da_aa <= 0) { // if (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.a8
              assert(!(ba * da_ba + bd * ba_aa - bb * da_aa <= 0)); // Not (ADB ^ AB).AO >= 0 / !a10.!a11.a12.a3.a7.a8.!a9
              // Region ADB
              originToTriangle (current, a, d, b, (D-A).cross(B-A), D.dot(a_cross_b), next, ray);
              free_v[nfree++] = current.vertex[c];
            } else { // not (ADB ^ AD).AO >= 0 / !a10.!a11.a12.a3.a7.!a8
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } // end of (ADB ^ AD).AO >= 0
          } else { // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.a3.!a7
            if (D.dot(a_cross_c) <= 0) { // if ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.a2
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <= 0); // (ACD ^ AC).AO >= 0 / !a10.!a11.a12.a3.!a7.a2.a6
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } else { // not ACD.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2
              if (C.dot (a_cross_b) <= 0) { // if ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.a1
                assert(!(ba * ba_ca + bb * ca_aa - bc * ba_aa <= 0)); // Not (ABC ^ AB).AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.a1.!a4
                // Region ADB
                originToTriangle (current, a, d, b, (D-A).cross(B-A), D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } else { // not ABC.AO >= 0 / !a10.!a11.a12.a3.!a7.!a2.!a1
                // Region ADB
                originToTriangle (current, a, d, b, (D-A).cross(B-A), D.dot(a_cross_b), next, ray);
                free_v[nfree++] = current.vertex[c];
              } // end of ABC.AO >= 0
            } // end of ACD.AO >= 0
          } // end of (ACD ^ AD).AO >= 0
        } else { // not ADB.AO >= 0 / !a10.!a11.a12.!a3
          if (D.dot(a_cross_c) <= 0) { // if ACD.AO >= 0 / !a10.!a11.a12.!a3.a2
            if (da * ca_da + dc * da_aa - dd * ca_aa <= 0) { // if (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.a7
              // Region AD
              originToSegment (current, a, d, A, D, D-A, -da_aa, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            } else { // not (ACD ^ AD).AO >= 0 / !a10.!a11.a12.!a3.a2.!a7
              assert(ca * ca_da + cc * da_aa - cd * ca_aa <= 0); // (ACD ^ AC).AO >= 0 / !a10.!a11.a12.!a3.a2.!a7.a6
              // Region ACD
              originToTriangle (current, a, c, d, (C-A).cross(D-A), -D.dot(a_cross_c), next, ray);
              free_v[nfree++] = current.vertex[b];
            } // end of (ACD ^ AD).AO >= 0
          } else { // not ACD.AO >= 0 / !a10.!a11.a12.!a3.!a2
            // Region Inside
            REGION_INSIDE()
          } // end of ACD.AO >= 0
        } // end of ADB.AO >= 0
      } else { // not AD.AO >= 0 / !a10.!a11.!a12
        // Region A
        originToPoint (current, a, A, next, ray);
        free_v[nfree++] = current.vertex[b];
        free_v[nfree++] = current.vertex[c];
        free_v[nfree++] = current.vertex[d];
      } // end of AD.AO >= 0
    } // end of AC.AO >= 0
  } // end of AB.AO >= 0

#undef REGION_INSIDE
  return false;
}

void EPA::initialize()
{
  sv_store = new SimplexV[max_vertex_num];
  fc_store = new SimplexF[max_face_num];
  status = Failed;
  normal = Vec3f(0, 0, 0);
  depth = 0;
  nextsv = 0;
  for(size_t i = 0; i < max_face_num; ++i)
    stock.append(&fc_store[max_face_num-i-1]);
}

bool EPA::getEdgeDist(SimplexF* face, SimplexV* a, SimplexV* b, FCL_REAL& dist)
{
  Vec3f ab = b->w - a->w;
  Vec3f n_ab = ab.cross(face->n);
  FCL_REAL a_dot_nab = a->w.dot(n_ab);

  if(a_dot_nab < 0) // the origin is on the outside part of ab
  {
    // following is similar to projectOrigin for two points
    // however, as we dont need to compute the parameterization, dont need to compute 0 or 1
    FCL_REAL a_dot_ab = a->w.dot(ab); 
    FCL_REAL b_dot_ab = b->w.dot(ab);

    if(a_dot_ab > 0) 
      dist = a->w.norm();
    else if(b_dot_ab < 0)
      dist = b->w.norm();
    else
    {
      dist = std::sqrt(std::max(
        a->w.squaredNorm() - a_dot_ab * a_dot_ab / ab.squaredNorm(),
        0.));
    }

    return true;
  }

  return false;
}

EPA::SimplexF* EPA::newFace(SimplexV* a, SimplexV* b, SimplexV* c, bool forced)
{
  if(stock.root)
  {
    SimplexF* face = stock.root;
    stock.remove(face);
    hull.append(face);
    face->pass = 0;
    face->vertex[0] = a;
    face->vertex[1] = b;
    face->vertex[2] = c;
    face->n = (b->w - a->w).cross(c->w - a->w);
    FCL_REAL l = face->n.norm();
      
    if(l > Eigen::NumTraits<FCL_REAL>::epsilon())
    {
      face->n /= l;

      if(!(getEdgeDist(face, a, b, face->d) ||
           getEdgeDist(face, b, c, face->d) ||
           getEdgeDist(face, c, a, face->d)))
      {
        face->d = a->w.dot(face->n);
      }

      if(forced || face->d >= -tolerance)
        return face;
      else
        status = NonConvex;
    }
    else
      status = Degenerated;

    hull.remove(face);
    stock.append(face);
    return NULL;
  }
    
  status = stock.root ? OutOfVertices : OutOfFaces;
  return NULL;
}

/** @brief Find the best polytope face to split */
EPA::SimplexF* EPA::findBest()
{
  SimplexF* minf = hull.root;
  FCL_REAL mind = minf->d * minf->d;
  for(SimplexF* f = minf->l[1]; f; f = f->l[1])
  {
    FCL_REAL sqd = f->d * f->d;
    if(sqd < mind)
    {
      minf = f;
      mind = sqd;
    }
  }
  return minf;
}

EPA::Status EPA::evaluate(GJK& gjk, const Vec3f& guess)
{
  GJK::Simplex& simplex = *gjk.getSimplex();
  support_func_guess_t hint (gjk.support_hint);
  if((simplex.rank > 1) && gjk.encloseOrigin())
  {
    while(hull.root)
    {
      SimplexF* f = hull.root;
      hull.remove(f);
      stock.append(f);
    }

    status = Valid;
    nextsv = 0;

    if((simplex.vertex[0]->w - simplex.vertex[3]->w).dot
       ((simplex.vertex[1]->w - simplex.vertex[3]->w).cross
        (simplex.vertex[2]->w - simplex.vertex[3]->w)) < 0)
    {
      SimplexV* tmp = simplex.vertex[0];
      simplex.vertex[0] = simplex.vertex[1];
      simplex.vertex[1] = tmp;
    }

    SimplexF* tetrahedron[] = {newFace(simplex.vertex[0], simplex.vertex[1],
                                       simplex.vertex[2], true),
                               newFace(simplex.vertex[1], simplex.vertex[0],
                                       simplex.vertex[3], true),
                               newFace(simplex.vertex[2], simplex.vertex[1],
                                       simplex.vertex[3], true),
                               newFace(simplex.vertex[0], simplex.vertex[2],
                                       simplex.vertex[3], true) };

    if(hull.count == 4)
    {
      SimplexF* best = findBest(); // find the best face (the face with the minimum distance to origin) to split
      SimplexF outer = *best;
      size_t pass = 0;
      size_t iterations = 0;
        
      // set the face connectivity
      bind(tetrahedron[0], 0, tetrahedron[1], 0);
      bind(tetrahedron[0], 1, tetrahedron[2], 0);
      bind(tetrahedron[0], 2, tetrahedron[3], 0);
      bind(tetrahedron[1], 1, tetrahedron[3], 2);
      bind(tetrahedron[1], 2, tetrahedron[2], 1);
      bind(tetrahedron[2], 2, tetrahedron[3], 1);

      status = Valid;
      for(; iterations < max_iterations; ++iterations)
      {
        if (nextsv >= max_vertex_num) {
          status = OutOfVertices;
          break;
        }

        SimplexHorizon horizon;
        SimplexV* w = &sv_store[nextsv++];
        bool valid = true;
        best->pass = ++pass;
        // At the moment, SimplexF.n is always normalized. This could be revised in the future...
        gjk.getSupport(best->n, true, *w, hint);
        FCL_REAL wdist = best->n.dot(w->w) - best->d;
        if(wdist <= tolerance) {
          status = AccuracyReached;
          break;
        }
        for(size_t j = 0; (j < 3) && valid; ++j)
          valid &= expand(pass, w, best->f[j], best->e[j], horizon);

        if(!valid || horizon.nf < 3) {
          // The status has already been set by the expand function.
          assert(!(status & Valid));
          break;
        }
        // need to add the edge connectivity between first and last faces
        bind(horizon.ff, 2, horizon.cf, 1);
        hull.remove(best);
        stock.append(best);
        best = findBest();
        outer = *best;
      }

      normal = outer.n;
      depth = outer.d;
      result.rank = 3;
      result.vertex[0] = outer.vertex[0];
      result.vertex[1] = outer.vertex[1];
      result.vertex[2] = outer.vertex[2];
      return status;
    }
  }

  status = FallBack;
  normal = -guess;
  FCL_REAL nl = normal.norm();
  if(nl > 0) normal /= nl;
  else normal = Vec3f(1, 0, 0);
  depth = 0;
  result.rank = 1;
  result.vertex[0] = simplex.vertex[0];
  return status;
}


/** @brief the goal is to add a face connecting vertex w and face edge f[e] */
bool EPA::expand(size_t pass, SimplexV* w, SimplexF* f, size_t e, SimplexHorizon& horizon)
{
  static const size_t nexti[] = {1, 2, 0};
  static const size_t previ[] = {2, 0, 1};

  if(f->pass == pass)
  {
    status = InvalidHull;
    return false;
  }

  const size_t e1 = nexti[e];
    
  // case 1: the new face is not degenerated, i.e., the new face is not coplanar with the old face f.
  if(f->n.dot(w->w - f->vertex[e]->w) < -Eigen::NumTraits<FCL_REAL>::epsilon())
  {
    SimplexF* nf = newFace(f->vertex[e1], f->vertex[e], w, false);
    if(nf)
    {
      // add face-face connectivity
      bind(nf, 0, f, e);
        
      // if there is last face in the horizon, then need to add another connectivity, i.e. the edge connecting the current new add edge and the last new add edge. 
      // This does not finish all the connectivities because the final face need to connect with the first face, this will be handled in the evaluate function.
      // Notice the face is anti-clockwise, so the edges are 0 (bottom), 1 (right), 2 (left)
      if(horizon.cf)  
        bind(nf, 2, horizon.cf, 1);
      else
        horizon.ff = nf; 
        
      horizon.cf = nf;
      ++horizon.nf;
      return true;
    }
    return false;
  }

  // case 2: the new face is coplanar with the old face f. We need to add two faces and delete the old face
  const size_t e2 = previ[e];
  f->pass = pass;
  if(expand(pass, w, f->f[e1], f->e[e1], horizon) && expand(pass, w, f->f[e2], f->e[e2], horizon))
  {
    hull.remove(f);
    stock.append(f);
    return true;
  }
  return false;
}

bool EPA::getClosestPoints (const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1)
{
  bool res = details::getClosestPoints(result, w0, w1);
  if (!res) return false;
  details::inflate<false> (shape, w0, w1);
  return true;
}

} // details

} // fcl

} // namespace hpp
