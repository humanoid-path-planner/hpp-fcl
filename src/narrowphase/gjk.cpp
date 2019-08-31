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
#include <hpp/fcl/intersect.h>

namespace hpp
{
namespace fcl
{

namespace details
{

struct shape_traits_base
{
  enum { NeedNormalizedDir = true
  };
};

template <typename Shape> struct shape_traits : shape_traits_base {};

template <> struct shape_traits<TriangleP> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

template <> struct shape_traits<Box> : shape_traits_base
{
  enum { NeedNormalizedDir = false
  };
};

void getShapeSupport(const TriangleP* triangle, const Vec3f& dir, Vec3f& support)
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

inline void getShapeSupport(const Box* box, const Vec3f& dir, Vec3f& support)
{
  support.noalias() = (dir.array() > 0).select(box->halfSide, -box->halfSide);
}

inline void getShapeSupport(const Sphere* sphere, const Vec3f& dir, Vec3f& support)
{
  support = dir * sphere->radius;
}

inline void getShapeSupport(const Capsule* capsule, const Vec3f& dir, Vec3f& support)
{
  support = capsule->radius * dir;
  if (dir[2] > 0) support[2] += capsule->lz / 2;
  else            support[2] -= capsule->lz / 2;
}

void getShapeSupport(const Cone* cone, const Vec3f& dir, Vec3f& support)
{
  // TODO (Joseph Mirabel)
  // this assumes that the cone radius is, for -0.5 < z < 0.5:
  // (lz/2 - z) * radius / lz
  //
  // I did not change the code myself. However, I think it should be revised.
  // 1. It can be optimized.
  // 2. I am not sure this is what conePlaneIntersect and coneHalfspaceIntersect
  //    assumes...
  FCL_REAL zdist = dir[0] * dir[0] + dir[1] * dir[1];
  FCL_REAL len = zdist + dir[2] * dir[2];
  zdist = std::sqrt(zdist);
  len = std::sqrt(len);
  FCL_REAL half_h = cone->lz * 0.5;
  FCL_REAL radius = cone->radius;

  FCL_REAL sin_a = radius / std::sqrt(radius * radius + 4 * half_h * half_h);

  if(dir[2] > len * sin_a)
    support = Vec3f(0, 0, half_h);
  else if(zdist > 0)
  {
    FCL_REAL rad = radius / zdist;
    support = Vec3f(rad * dir[0], rad * dir[1], -half_h);
  }
  else
    support = Vec3f(0, 0, -half_h);
}

void getShapeSupport(const Cylinder* cylinder, const Vec3f& dir, Vec3f& support)
{
  static const FCL_REAL eps (sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
  FCL_REAL zdist = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
  FCL_REAL half_h = cylinder->lz * 0.5;
  if(zdist == 0.0)
    support = Vec3f(0, 0, (dir[2]>0)? half_h:-half_h);
  else {
    FCL_REAL d = cylinder->radius / zdist;
    FCL_REAL z (0.);
    if (dir [2] > eps) z = half_h;
    else if (dir [2] < -eps) z = -half_h;
    support << d * dir.head<2>(), z;
  }
  assert (fabs (support [0] * dir [1] - support [1] * dir [0]) < eps);
}

void getShapeSupport(const Convex* convex, const Vec3f& dir, Vec3f& support)
{
  FCL_REAL maxdot = - std::numeric_limits<FCL_REAL>::max();
  Vec3f* curp = convex->points;
  for(int i = 0; i < convex->num_points; ++i, curp+=1)
  {
    FCL_REAL dot = dir.dot(*curp);
    if(dot > maxdot)
    {
      support = *curp;
      maxdot = dot;
    }
  }
}

#define CALL_GET_SHAPE_SUPPORT(ShapeType)                                      \
  getShapeSupport (static_cast<const ShapeType*>(shape),                       \
      (shape_traits<ShapeType>::NeedNormalizedDir && !dirIsNormalized)         \
      ? dir.normalized() : dir,                                                \
      support)

Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir, bool dirIsNormalized)
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
    CALL_GET_SHAPE_SUPPORT(Convex);
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

template <typename Shape0, typename Shape1>
void getSupportTpl (const Shape0* s0, const Shape1* s1,
    const Matrix3f& oR1, const Vec3f& ot1,
    const Vec3f& dir, Vec3f& support0, Vec3f& support1)
{
  getShapeSupport (s0, dir, support0);
  getShapeSupport (s1, - oR1.transpose() * dir, support1);
  support1 = oR1 * support1 + ot1;
}

template <typename Shape0, typename Shape1>
void getSupportFuncTpl (const MinkowskiDiff& md,
    const Vec3f& dir, bool dirIsNormalized, Vec3f& support0, Vec3f& support1)
{
  enum { NeedNormalizedDir =
    bool ( (bool)shape_traits<Shape0>::NeedNormalizedDir
        || (bool)shape_traits<Shape1>::NeedNormalizedDir)
  };
  getSupportTpl<Shape0, Shape1> (
      static_cast <const Shape0*>(md.shapes[0]),
      static_cast <const Shape1*>(md.shapes[1]),
      md.oR1, md.ot1,
      (NeedNormalizedDir && !dirIsNormalized) ? dir.normalized() : dir,
      support0, support1);
}

template <typename Shape0>
MinkowskiDiff::GetSupportFunction makeGetSupportFunction1 (const ShapeBase* s1)
{
  switch(s1->getNodeType())
  {
  case GEOM_TRIANGLE:
    return getSupportFuncTpl<Shape0, TriangleP>;
  case GEOM_BOX:
    return getSupportFuncTpl<Shape0, Box>;
  case GEOM_SPHERE:
    return getSupportFuncTpl<Shape0, Sphere>;
  case GEOM_CAPSULE:
    return getSupportFuncTpl<Shape0, Capsule>;
  case GEOM_CONE:
    return getSupportFuncTpl<Shape0, Cone>;
  case GEOM_CYLINDER:
    return getSupportFuncTpl<Shape0, Cylinder>;
  case GEOM_CONVEX:
    return getSupportFuncTpl<Shape0, Convex>;
  default:
    throw std::logic_error ("Unsupported geometric shape");
  }
}

void MinkowskiDiff::set (const ShapeBase* shape0, const ShapeBase* shape1)
{
  shapes[0] = shape0;
  shapes[1] = shape1;

  switch(shape0->getNodeType())
  {
  case GEOM_TRIANGLE:
    getSupportFunc = makeGetSupportFunction1<TriangleP> (shape1);
    break;
  case GEOM_BOX:
    getSupportFunc = makeGetSupportFunction1<Box> (shape1);
    break;
  case GEOM_SPHERE:
    getSupportFunc = makeGetSupportFunction1<Sphere> (shape1);
    break;
  case GEOM_CAPSULE:
    getSupportFunc = makeGetSupportFunction1<Capsule> (shape1);
    break;
  case GEOM_CONE:
    getSupportFunc = makeGetSupportFunction1<Cone> (shape1);
    break;
  case GEOM_CYLINDER:
    getSupportFunc = makeGetSupportFunction1<Cylinder> (shape1);
    break;
  case GEOM_CONVEX:
    getSupportFunc = makeGetSupportFunction1<Convex> (shape1);
    break;
  default:
    throw std::logic_error ("Unsupported geometric shape");
  }
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

bool GJK::getClosestPoints (const Simplex& simplex, Vec3f& w0, Vec3f& w1)
{
  SimplexV* const* vs = simplex.vertex;

  for (short i = 0; i < simplex.rank; ++i) {
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
          w0 = a0;
          w1 = a1;
        } else {
          lb = N.squaredNorm();
          if (la > lb) {
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
  for (short i = 0; i < simplex.rank; ++i) {
    w0 += projection.parameterization[i] * vs[i]->w0;
    w1 += projection.parameterization[i] * vs[i]->w1;
  }
  return true;
}

GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess)
{
  size_t iterations = 0;
  FCL_REAL alpha = 0;

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
  ray = guess;

  if (ray.squaredNorm() > 0) appendVertex(simplices[0], -ray);
  else                       appendVertex(simplices[0], Vec3f(1, 0, 0), true);
  ray = simplices[0].vertex[0]->w;

  do
  {
    short next = (short)(1 - current);
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    FCL_REAL rl = ray.norm();
    if(rl < tolerance) // mean origin is near the face of original simplex, return touch
    {
      status = Inside;
      distance = 0; // rl ?
      break;
    }

    appendVertex(curr_simplex, -ray); // see below, ray points away from origin

    // check removed (by ?): when the new support point is close to previous support points, stop (as the new simplex is degenerated)
    const Vec3f& w = curr_simplex.vertex[curr_simplex.rank - 1]->w;

    // check B: no collision if omega > 0
    FCL_REAL omega = ray.dot(w) / rl;
    if (omega > distance_upper_bound)
    {
      distance = omega;
      break;
    }

    // check C: when the new support point is close to the sub-simplex where the ray point lies, stop (as the new simplex again is degenerated)
    alpha = std::max(alpha, omega);
    if((rl - alpha) - tolerance * rl <= 0)
    {
      removeVertex(simplices[current]);
      distance = rl;
      break;
    }

    // This has been rewritten thanks to the excellent video:
    // https://youtu.be/Qupqu1xe7Io
    bool inside;
    switch(curr_simplex.rank)
    {
    case 2:
      inside = projectLineOrigin (curr_simplex, next_simplex);
      break;
    case 3:
      inside = projectTriangleOrigin (curr_simplex, next_simplex);
      break;
    case 4:
      inside = projectTetrahedraOrigin (curr_simplex, next_simplex);
      break;
    }
    assert (nfree+next_simplex.rank == 4);
    current = next;
    if(inside) {
      status = Inside;
      distance = 0;
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;
      
  } while(status == Valid);

  simplex = &simplices[current];
  return status;
}

void GJK::getSupport(const Vec3f& d, bool dIsNormalized, SimplexV& sv) const
{
  shape->support(d, dIsNormalized, sv.w0, sv.w1);
  sv.w.noalias() = sv.w0 - sv.w1;
}

void GJK::removeVertex(Simplex& simplex)
{
  free_v[nfree++] = simplex.vertex[--simplex.rank];
}

void GJK::appendVertex(Simplex& simplex, const Vec3f& v, bool isNormalized)
{
  simplex.vertex[simplex.rank] = free_v[--nfree]; // set the memory
  getSupport (v, isNormalized, *simplex.vertex[simplex.rank++]);
}

bool GJK::encloseOrigin()
{
  switch(simplex->rank)
  {
  case 1:
    {
      for(size_t i = 0; i < 3; ++i)
      {
        Vec3f axis(Vec3f::Zero());
        axis[i] = 1;
        appendVertex(*simplex, axis, true);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -axis, true);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
      }
    }
    break;
  case 2:
    {
      Vec3f d = simplex->vertex[1]->w - simplex->vertex[0]->w;
      for(size_t i = 0; i < 3; ++i)
      {
        Vec3f axis(0,0,0);
        axis[i] = 1;
        Vec3f p = d.cross(axis);
        if(p.squaredNorm() > 0)
        {
          appendVertex(*simplex, p);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
          appendVertex(*simplex, -p);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
        }
      }
    }
    break;
  case 3:
    {
      Vec3f n = (simplex->vertex[1]->w - simplex->vertex[0]->w).cross
        (simplex->vertex[2]->w - simplex->vertex[0]->w);
      if(n.squaredNorm() > 0)
      {
        appendVertex(*simplex, n);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -n);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
      }
    }
    break;
  case 4:
    {
      if(std::abs(triple(simplex->vertex[0]->w - simplex->vertex[3]->w,
                         simplex->vertex[1]->w - simplex->vertex[3]->w,
                         simplex->vertex[2]->w - simplex->vertex[3]->w)) > 0)
        return true;
    }
    break;
  }

  return false;
}

inline void originToPoint (
    const GJK::Simplex& current, int a,
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
    const GJK::Simplex& current, int a, int b,
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

inline void originToTriangle (
    const GJK::Simplex& current,
    int a, int b, int c,
    const Vec3f& ABC,
    const FCL_REAL& ABCdotAO,
    GJK::Simplex& next,
    Vec3f& ray)
{
  bool aboveTri (ABCdotAO >= 0);
  ray = ABC;

  if (aboveTri) {
    next.vertex[0] = current.vertex[c];
    next.vertex[1] = current.vertex[b];
  } else {
    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[c];
  }
  next.vertex[2] = current.vertex[a];
  next.rank = 3;

  // To ensure backward compatibility
  ray *= -ABCdotAO / ABC.squaredNorm();
}

bool GJK::projectLineOrigin(const Simplex& current, Simplex& next)
{
  const int a = 1, b = 0;
  // A is the last point we added.
  const Vec3f& A = current.vertex[a]->w;
  const Vec3f& B = current.vertex[b]->w;

  const Vec3f AB = B - A;
  const FCL_REAL d = AB.dot(-A);
  assert (d <= AB.squaredNorm());

  if (d <= 0) {
    // A is the closest to the origin
    originToPoint (current, a, A, next, ray);
    free_v[nfree++] = current.vertex[b];
  } else
    originToSegment (current, a, b, A, B, AB, d, next, ray);
  return false;
}

bool GJK::projectTriangleOrigin(const Simplex& current, Simplex& next)
{
  const int a = 2, b = 1, c = 0;
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
      originToTriangle (current, a, b, c, ABC, ABC.dot(-A), next, ray);
    }
  }
  return false;
}

bool GJK::projectTetrahedraOrigin(const Simplex& current, Simplex& next)
{
  const int a = 3, b = 2, c = 1, d = 0;
  const Vec3f& A (current.vertex[a]->w);
  const Vec3f& B (current.vertex[b]->w);
  const Vec3f& C (current.vertex[c]->w);
  const Vec3f& D (current.vertex[d]->w);
  const Vec3f AB (B-A);
  const FCL_REAL AB_dot_AO = AB.dot(-A);
  const Vec3f AC (C-A);
  const FCL_REAL AC_dot_AO = AC.dot(-A);
  const Vec3f AD (D-A);
  const FCL_REAL AD_dot_AO = AD.dot(-A);
  const Vec3f ABC (AB.cross(AC));
  const FCL_REAL ABC_dot_AO = ABC.dot(-A);
  const Vec3f ACD (AC.cross(AD));
  const FCL_REAL ACD_dot_AO = ACD.dot(-A);
  const Vec3f ADB (AD.cross(AB));
  const FCL_REAL ADB_dot_AO = ADB.dot(-A);
  Vec3f cross;


#ifndef NDEBUG
  Vec3f BC (C-B),
        CD (D-C),
        BD (D-B),
        BCD ((C-B).cross(D-B));
  FCL_REAL t = BCD.dot(-AB);
  assert (t >= 0);
  t = BCD.dot(-B);
  assert (t >= 0);
  t = BCD.dot(-ray);
  assert (t >= 0);

  // We necessarily come from the triangle case with the closest point lying on
  // the triangle. Hence the assertions below.
  assert (BCD.cross(-BC).dot (B) >= 0);
  assert (BCD.cross(-CD).dot (C) >= 0);
  assert (BCD.cross( BD).dot (D) >= 0);
#endif

#define REGION_INSIDE()                 \
    ray.setZero();                      \
    next.vertex[0] = current.vertex[d]; \
    next.vertex[1] = current.vertex[c]; \
    next.vertex[2] = current.vertex[b]; \
    next.vertex[3] = current.vertex[a]; \
    next.rank=4;                        \
    return true;

  if (AB_dot_AO >= 0) {
    cross.noalias() = ABC.cross(AB);
    if (cross.dot(-A) >= 0) {
      cross.noalias() = ABC.cross(AC);
      if (cross.dot(-A) >= 0) {
        cross.noalias() = ACD.cross(AD);
        if (cross.dot(-A) >= 0) {
          if (AC_dot_AO >= 0) {
            cross.noalias() = ACD.cross(AC);
            if (cross.dot(-A) >= 0) {
              if (AD_dot_AO >= 0) {
                // Region Inside
                REGION_INSIDE()
              } else {
                if (ADB_dot_AO >= 0) {
                  // Region ADB
                  originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                  free_v[nfree++] = current.vertex[c];
                } else {
                  // Region Inside
                  REGION_INSIDE()
                }
              }
            } else {
              // Region AC
              originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            }
          } else {
            if (ADB_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (ACD_dot_AO >= 0) {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region Inside
                REGION_INSIDE()
              }
            }
          }
        } else {
          if (ADB_dot_AO >= 0) {
            cross.noalias() = ACD.cross(AC);
            if (cross.dot(-A) >= 0) {
              if (ACD_dot_AO >= 0) {
                // Region ACD
                originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AC_dot_AO >= 0) {
                // Region AC
                originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            }
          } else {
            if (ACD_dot_AO >= 0) {
              cross.noalias() = ACD.cross(AC);
              if (cross.dot(-A) >= 0) {
                // Region ACD
                originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {
                // Region AC
                originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              }
            } else {
              if (ABC_dot_AO >= 0) {
                // Region AC
                originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region Inside
                REGION_INSIDE()
              }
            }
          }
        }
      } else {
        if (ABC_dot_AO >= 0) {
          // Region ABC
          originToTriangle (current, a, b, c, ABC, ABC_dot_AO, next, ray);
          free_v[nfree++] = current.vertex[d];
        } else {
          if (ACD_dot_AO >= 0) {
            cross.noalias() = ACD.cross(AD);
            if (cross.dot(-A) >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              cross.noalias() = ACD.cross(AC);
              if (cross.dot(-A) >= 0) {
                // Region ACD
                originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            }
          } else {
            if (ADB_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              // Region Inside
              REGION_INSIDE()
            }
          }
        }
      }
    } else {
      cross.noalias() = ACD.cross(AC);
      if (cross.dot(-A) >= 0) {
        if (ACD_dot_AO >= 0) {
          cross.noalias() = ADB.cross(AD);
          if (cross.dot(-A) >= 0) {
            if (ADB_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AB);
              if (cross.dot(-A) >= 0) {
                // Region AB
                originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              cross.noalias() = ACD.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region AB
                originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region ACD
                originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
              }
            }
          } else {
            cross.noalias() = ACD.cross(AD);
            if (cross.dot(-A) >= 0) {
              if (AD_dot_AO >= 0) {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AB
                originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            } else {
              // Region ACD
              originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
            }
          }
        } else {
          if (ADB_dot_AO >= 0) {
            cross.noalias() = ADB.cross(AD);
            if (cross.dot(-A) >= 0) {
              cross.noalias() = ADB.cross(AB);
              if (cross.dot(-A) >= 0) {
                // Region AB
                originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              cross.noalias() = ADB.cross(AB);
              if (cross.dot(-A) >= 0) {
                // Region AB
                originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            }
          } else {
            if (ABC_dot_AO >= 0) {
              // Region AB
              originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[c];
              free_v[nfree++] = current.vertex[d];
            } else {
              // Region Inside
              REGION_INSIDE()
            }
          }
        }
      } else {
        cross.noalias() = ADB.cross(AB);
        if (cross.dot(-A) >= 0) {
          // Region AB
          originToSegment (current, a, b, A, B, AB, AB_dot_AO, next, ray);
          free_v[nfree++] = current.vertex[c];
          free_v[nfree++] = current.vertex[d];
        } else {
          if (AC_dot_AO >= 0) {
            cross.noalias() = ABC.cross(AC);
            if (cross.dot(-A) >= 0) {
              // Region AC
              originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {
              if (AD_dot_AO >= 0) {
                // Region Inside
                REGION_INSIDE()
              } else {
                if (ADB_dot_AO >= 0) {
                  // Region ADB
                  originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                  free_v[nfree++] = current.vertex[c];
                } else {
                  // Region Inside
                  REGION_INSIDE()
                }
              }
            }
          } else {
            if (ADB_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (ACD_dot_AO >= 0) {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region Inside
                REGION_INSIDE()
              }
            }
          }
        }
      }
    }
  } else {
    if (ABC_dot_AO >= 0) {
      cross.noalias() = ABC.cross(AC);
      if (cross.dot(-A) >= 0) {
        cross.noalias() = ACD.cross(AC);
        if (cross.dot(-A) >= 0) {
          cross.noalias() = ACD.cross(AD);
          if (cross.dot(-A) >= 0) {
            if (AD_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AC_dot_AO >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          } else {
            if (ACD_dot_AO >= 0) {
              // Region ACD
              originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
            } else {
              // Region ADB
              originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[c];
            }
          }
        } else {
          cross.noalias() = ADB.cross(AD);
          if (cross.dot(-A) >= 0) {
            if (AC_dot_AO >= 0) {
              // Region AC
              originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {
              if (AD_dot_AO >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          } else {
            if (AC_dot_AO >= 0) {
              // Region AC
              originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[d];
            } else {
              if (AD_dot_AO >= 0) {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          }
        }
      } else {
        cross.noalias() = ABC.cross(AB);
        if (cross.dot(-A) >= 0) {
          // Region ABC
          originToTriangle (current, a, b, c, ABC, ABC_dot_AO, next, ray);
          free_v[nfree++] = current.vertex[d];
        } else {
          cross.noalias() = ACD.cross(AD);
          if (cross.dot(-A) >= 0) {
            if (AD_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AC_dot_AO >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          } else {
            cross.noalias() = ACD.cross(AC);
            if (cross.dot(-A) >= 0) {
              if (ACD_dot_AO >= 0) {
                // Region ACD
                originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AC_dot_AO >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                if (AD_dot_AO >= 0) {
                  // Region ADB
                  originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                  free_v[nfree++] = current.vertex[c];
                } else {
                  // Region A
                  originToPoint (current, a, A, next, ray);
                  free_v[nfree++] = current.vertex[b];
                  free_v[nfree++] = current.vertex[c];
                  free_v[nfree++] = current.vertex[d];
                }
              }
            }
          }
        }
      }
    } else {
      if (ACD_dot_AO >= 0) {
        cross.noalias() = ACD.cross(AC);
        if (cross.dot(-A) >= 0) {
          cross.noalias() = ACD.cross(AD);
          if (cross.dot(-A) >= 0) {
            if (AD_dot_AO >= 0) {
              cross.noalias() = ADB.cross(AD);
              if (cross.dot(-A) >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AC_dot_AO >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          } else {
            // Region ACD
            originToTriangle (current, a, c, d, ACD, ACD_dot_AO, next, ray);
            free_v[nfree++] = current.vertex[b];
          }
        } else {
          cross.noalias() = ADB.cross(AD);
          if (cross.dot(-A) >= 0) {
            if (AC_dot_AO >= 0) {
              cross.noalias() = ABC.cross(AC);
              if (cross.dot(-A) >= 0) {
                // Region AC
                originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AD_dot_AO >= 0) {
                // Region ADB
                originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          } else {
            if (AC_dot_AO >= 0) {
              cross.noalias() = ABC.cross(AC);
              if (cross.dot(-A) >= 0) {
                // Region AC
                originToSegment (current, a, c, A, C, AC, AC_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[d];
              } else {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              }
            } else {
              if (AD_dot_AO >= 0) {
                // Region AD
                originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
              } else {
                // Region A
                originToPoint (current, a, A, next, ray);
                free_v[nfree++] = current.vertex[b];
                free_v[nfree++] = current.vertex[c];
                free_v[nfree++] = current.vertex[d];
              }
            }
          }
        }
      } else {
        if (ADB_dot_AO >= 0) {
          if (AD_dot_AO >= 0) {
            cross.noalias() = ADB.cross(AD);
            if (cross.dot(-A) >= 0) {
              // Region ADB
              originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {
              // Region AD
              originToSegment (current, a, d, A, D, AD, AD_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
            }
          } else {
            if (AC_dot_AO >= 0) {
              // Region ADB
              originToTriangle (current, a, d, b, ADB, ADB_dot_AO, next, ray);
              free_v[nfree++] = current.vertex[c];
            } else {
              // Region A
              originToPoint (current, a, A, next, ray);
              free_v[nfree++] = current.vertex[b];
              free_v[nfree++] = current.vertex[c];
              free_v[nfree++] = current.vertex[d];
            }
          }
        } else {
          // Region Inside
          REGION_INSIDE()
        }
      }
    }
  }

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
  Vec3f ba = b->w - a->w;
  Vec3f n_ab = ba.cross(face->n);
  FCL_REAL a_dot_nab = a->w.dot(n_ab);

  if(a_dot_nab < 0) // the origin is on the outside part of ab
  {
    // following is similar to projectOrigin for two points
    // however, as we dont need to compute the parameterization, dont need to compute 0 or 1
    FCL_REAL a_dot_ba = a->w.dot(ba); 
    FCL_REAL b_dot_ba = b->w.dot(ba);

    if(a_dot_ba > 0) 
      dist = a->w.norm();
    else if(b_dot_ba < 0)
      dist = b->w.norm();
    else
    {
      FCL_REAL a_dot_b = a->w.dot(b->w);
      dist = std::sqrt(std::max(a->w.squaredNorm() * b->w.squaredNorm() - a_dot_b * a_dot_b, (FCL_REAL)0));
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
      
    if(l > tolerance)
    {
      if(!(getEdgeDist(face, a, b, face->d) ||
           getEdgeDist(face, b, c, face->d) ||
           getEdgeDist(face, c, a, face->d)))
      {
        face->d = a->w.dot(face->n) / l;
      }

      face->n /= l;
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

/** \brief Find the best polytope face to split */
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
        if(nextsv < max_vertex_num)
        {
          SimplexHorizon horizon;
          SimplexV* w = &sv_store[nextsv++];
          bool valid = true;
          best->pass = ++pass;
          // At the moment, SimplexF.n is always normalized. This could be revised in the future...
          gjk.getSupport(best->n, true, *w);
          FCL_REAL wdist = best->n.dot(w->w) - best->d;
          if(wdist > tolerance)
          {
            for(size_t j = 0; (j < 3) && valid; ++j)
            {
              valid &= expand(pass, w, best->f[j], best->e[j], horizon);
            }

              
            if(valid && horizon.nf >= 3)
            {
              // need to add the edge connectivity between first and last faces
              bind(horizon.ff, 2, horizon.cf, 1);
              hull.remove(best);
              stock.append(best);
              best = findBest();
              outer = *best;
            }
            else
            {
              status = InvalidHull; break;
            }
          }
          else
          {
            status = AccuracyReached; break;
          }
        }
        else
        {
          status = OutOfVertices; break;
        }
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


/** \brief the goal is to add a face connecting vertex w and face edge f[e] */
bool EPA::expand(size_t pass, SimplexV* w, SimplexF* f, size_t e, SimplexHorizon& horizon)
{
  static const size_t nexti[] = {1, 2, 0};
  static const size_t previ[] = {2, 0, 1};

  if(f->pass != pass)
  {
    const size_t e1 = nexti[e];
      
    // case 1: the new face is not degenerated, i.e., the new face is not coplanar with the old face f.
    if(f->n.dot(w->w) - f->d < -tolerance)
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
    }
    else // case 2: the new face is coplanar with the old face f. We need to add two faces and delete the old face
    {
      const size_t e2 = previ[e];
      f->pass = pass;
      if(expand(pass, w, f->f[e1], f->e[e1], horizon) && expand(pass, w, f->f[e2], f->e[e2], horizon))
      {
        hull.remove(f);
        stock.append(f);
        return true;
      }
    }
  }

  return false;
}

} // details

} // fcl

} // namespace hpp
