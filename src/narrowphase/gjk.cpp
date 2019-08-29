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
    const Vec3f& dir, Vec3f& support)
{
  getShapeSupport (s0, dir, support);
  Vec3f support1;
  getShapeSupport (s1, - oR1.transpose() * dir, support1);
  support.noalias() -= oR1 * support1 + ot1;
}

template <typename Shape0, typename Shape1>
void getSupportFuncTpl (const MinkowskiDiff& md,
    const Vec3f& dir, bool dirIsNormalized, Vec3f& support)
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
      support);
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
  ray = Vec3f::Zero();
  nfree = 0;
  status = Failed;
  current = 0;
  distance = 0.0;
  simplex = NULL;
}

Vec3f GJK::getGuessFromSimplex() const
{
  return ray;
}

bool GJK::getClosestPoints (const MinkowskiDiff& shape, Vec3f& w0, Vec3f& w1) const
{
  w0.setZero();
  w1.setZero();
  for(size_t i = 0; i < getSimplex()->rank; ++i)
  {
    FCL_REAL p = getSimplex()->coefficient[i];
    w0 += shape.support0( getSimplex()->vertex[i]->d, false) * p;
    w1 += shape.support1(-getSimplex()->vertex[i]->d, false) * p;
  }
  return true;
}

GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess)
{
  size_t iterations = 0;
  FCL_REAL alpha = 0;
  Vec3f lastw[4];
  size_t clastw = 0;
  Project::ProjectResult project_res;
    
  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];
    
  nfree = 4;
  current = 0;
  status = Valid;
  shape = shape_;
  distance = 0.0;
  simplices[0].rank = 0;
  ray = guess;

  if (ray.squaredNorm() > 0) appendVertex(simplices[0], -ray);
  else                       appendVertex(simplices[0], Vec3f(1, 0, 0), true);
  simplices[0].coefficient[0] = 1;
  ray = simplices[0].vertex[0]->w;
  lastw[0] = lastw[1] = lastw[2] = lastw[3] = ray; // cache previous support points, the new support point will compare with it to avoid too close support points

  do
  {
    size_t next = 1 - current;
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    FCL_REAL rl = ray.norm();
    if(rl < tolerance) // mean origin is near the face of original simplex, return touch
    {
      status = Inside;
      break;
    }

    appendVertex(curr_simplex, -ray); // see below, ray points away from origin

    // check B: when the new support point is close to previous support points, stop (as the new simplex is degenerated)
    const Vec3f& w = curr_simplex.vertex[curr_simplex.rank - 1]->w;
    lastw[clastw = (clastw+1)&3] = w;

    // check C: when the new support point is close to the sub-simplex where the ray point lies, stop (as the new simplex again is degenerated)
    FCL_REAL omega = ray.dot(w) / rl;
    alpha = std::max(alpha, omega);
    if((rl - alpha) - tolerance * rl <= 0)
    {
      removeVertex(simplices[current]);
      break;
    }

    // This has been rewritten thanks to the excellent video:
    // https://youtu.be/Qupqu1xe7Io
    switch(curr_simplex.rank)
    {
    case 2:
      project_res.sqr_distance = projectLineOrigin (curr_simplex, next_simplex);
      { // This only checks that, so far, the behaviour did not change.
        FCL_REAL d = project_res.sqr_distance;
        project_res = Project::projectLineOrigin(curr_simplex.vertex[0]->w,
            curr_simplex.vertex[1]->w);

        assert (fabs(d-project_res.sqr_distance) < 1e-10);

        Vec3f _ray(0,0,0);
        size_t k = 0;
        for(size_t i = 0; i < curr_simplex.rank; ++i)
        {
          if(project_res.encode & (1 << i))
          {
            assert (next_simplex.vertex[k] == curr_simplex.vertex[i]);
            assert (
                fabs(next_simplex.coefficient[k]-project_res.parameterization[i]) < 1e-10);
            _ray += curr_simplex.vertex[i]->w * project_res.parameterization[i];
            ++k;
          }
        }
        //assert (_ray.isApprox (ray));
        assert ((_ray.isZero() && ray.isZero()) || _ray.isApprox (ray));
        assert (k = next_simplex.rank);
      }
      break;
    case 3:
      project_res.sqr_distance = projectTriangleOrigin (curr_simplex, next_simplex);
      { // This only checks that, so far, the behaviour did not change.
        FCL_REAL d = project_res.sqr_distance;
        project_res = Project::projectTriangleOrigin(curr_simplex.vertex[0]->w,
                                                     curr_simplex.vertex[1]->w,
                                                     curr_simplex.vertex[2]->w);
        assert (fabs(d-project_res.sqr_distance) < 1e-10);

        Vec3f _ray(0,0,0);
        size_t k = 0;
        for(size_t i = 0; i < curr_simplex.rank; ++i)
        {
          if(project_res.encode & (1 << i))
          {
            assert (next_simplex.vertex[k] == curr_simplex.vertex[i]);
            assert (
                fabs(next_simplex.coefficient[k]-project_res.parameterization[i]) < 1e-10);
            _ray += curr_simplex.vertex[i]->w * project_res.parameterization[i];
            ++k;
          }
        }
        assert ((_ray - ray).array().abs().maxCoeff() < 1e-10);
        assert (k = next_simplex.rank);
      }
      break;
    case 4:
      project_res = Project::projectTetrahedraOrigin(curr_simplex.vertex[0]->w,
                                                     curr_simplex.vertex[1]->w,
                                                     curr_simplex.vertex[2]->w,
                                                     curr_simplex.vertex[3]->w);
      break;
    }
    if(project_res.sqr_distance >= 0)
    {
      current = next;
      if (curr_simplex.rank > 3) {
      next_simplex.rank = 0;
      ray = Vec3f(0,0,0);
      for(size_t i = 0; i < curr_simplex.rank; ++i)
      {
        if(project_res.encode & (1 << i))
        {
          next_simplex.vertex[next_simplex.rank] = curr_simplex.vertex[i];
          // weights[i]
          next_simplex.coefficient[next_simplex.rank++] =
            project_res.parameterization[i];
          // weights[i]
          ray += curr_simplex.vertex[i]->w * project_res.parameterization[i];
        }
        else
          free_v[nfree++] = curr_simplex.vertex[i];
      }
      if(project_res.encode == 15) status = Inside; // the origin is within the 4-simplex, collision
      }
    }
    else
    {
      removeVertex(simplices[current]);
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;
      
  } while(status == Valid);

  simplex = &simplices[current];
  switch(status)
  {
  case Valid: distance = ray.norm(); break;
  case Inside: distance = 0; break;
  default:
    distance = sqrt (project_res.sqr_distance);
    break;
  }
  return status;
}

void GJK::getSupport(const Vec3f& d, bool dIsNormalized, SimplexV& sv) const
{
  // Was sv.d.noalias() = d.normalized();
  sv.d.noalias() = d;
  shape.support(sv.d, dIsNormalized, sv.w);
}

void GJK::removeVertex(Simplex& simplex)
{
  free_v[nfree++] = simplex.vertex[--simplex.rank];
}

void GJK::appendVertex(Simplex& simplex, const Vec3f& v, bool isNormalized)
{
  simplex.coefficient[simplex.rank] = 0; // initial weight 0
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

FCL_REAL GJK::projectLineOrigin(const Simplex& current, Simplex& next)
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
    ray = A;
    next.vertex[0] = current.vertex[a];
    next.rank = 1;
    free_v[nfree++] = current.vertex[b];

    // To ensure backward compatibility
    next.coefficient[0] = 1;
    return A.squaredNorm();
  } else {
    // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
    ray = AB.dot(B) * A + d * B;

    next.vertex[0] = current.vertex[b];
    next.vertex[1] = current.vertex[a];
    next.rank = 2;

    // To ensure backward compatibility
    FCL_REAL alpha = d / AB.squaredNorm();
    next.coefficient[1] = 1 - alpha; // A
    next.coefficient[0] =     alpha; // B
    ray /= AB.squaredNorm();

    return ray.squaredNorm();
  }
}

FCL_REAL GJK::projectTriangleOrigin(const Simplex& current, Simplex& next)
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
      // ray = - ( AC ^ AO ) ^ AC = (AC.C) A + (-AC.A) C
      ray = AC.dot(C) * A + towardsC * C;
      next.vertex[0] = current.vertex[c];
      next.vertex[1] = current.vertex[a];
      next.rank = 2;
      free_v[nfree++] = current.vertex[b];
      // To ensure backward compatibility

      FCL_REAL alpha = towardsC / AC.squaredNorm();
      next.coefficient[1] = 1 - alpha; // A
      next.coefficient[0] =     alpha; // C
      ray /= AC.squaredNorm();
      return ray.squaredNorm();
    } else { // Region 4 or 5
      FCL_REAL towardsB = AB.dot(-A);
      if (towardsB < 0) { // Region 5
        // A is the closest to the origin
        ray = A;
        next.vertex[0] = current.vertex[a];
        next.rank = 1;
        free_v[nfree++] = current.vertex[c];
        free_v[nfree++] = current.vertex[b];

        // To ensure backward compatibility
        next.coefficient[0] = 1;
        return A.squaredNorm();
      } else { // Region 4
        // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
        ray = AB.dot(B) * A + towardsB * B;

        next.vertex[0] = current.vertex[b];
        next.vertex[1] = current.vertex[a];
        next.rank = 2;
        free_v[nfree++] = current.vertex[c];

        // To ensure backward compatibility
        FCL_REAL alpha = towardsB / AB.squaredNorm();
        next.coefficient[1] = 1 - alpha; // A
        next.coefficient[0] =     alpha; // B
        ray /= AB.squaredNorm();

        return ray.squaredNorm();
      }
    }
  } else {
    FCL_REAL edgeAB2o = AB.cross(ABC).dot (-A);
    if (edgeAB2o >= 0) { // Region 4 or 5
      FCL_REAL towardsB = AB.dot(-A);
      if (towardsB < 0) { // Region 5
        // A is the closest to the origin
        ray = A;
        next.vertex[0] = current.vertex[a];
        next.rank = 1;
        free_v[nfree++] = current.vertex[c];
        free_v[nfree++] = current.vertex[b];

        // To ensure backward compatibility
        next.coefficient[0] = 1;
        return A.squaredNorm();
      } else { // Region 4
        // ray = - ( AB ^ AO ) ^ AB = (AB.B) A + (-AB.A) B
        ray = AB.dot(B) * A + towardsB * B;

        next.vertex[0] = current.vertex[b];
        next.vertex[1] = current.vertex[a];
        next.rank = 2;
        free_v[nfree++] = current.vertex[c];

        // To ensure backward compatibility
        FCL_REAL alpha = towardsB / AB.squaredNorm();
        next.coefficient[1] = 1 - alpha; // A
        next.coefficient[0] =     alpha; // B
        ray /= AB.squaredNorm();

        return ray.squaredNorm();
      }
    } else {
      FCL_REAL aboveTri = ABC.dot(-A);
      if (aboveTri) { // Region 2
        ray = ABC;

        next.vertex[0] = current.vertex[c];
        next.vertex[1] = current.vertex[b];
        next.vertex[2] = current.vertex[a];
        next.rank = 3;

        // To ensure backward compatibility
        FCL_REAL s = ABC.squaredNorm();
        ray *= A.dot(ABC) / s;
        Vec3f AQ (ray - A);
        Vec3f m (ABC.cross(AB));
        FCL_REAL _u2 = 1/AB.squaredNorm();
        FCL_REAL wu_u2 = AQ.dot (AB) * _u2;
        FCL_REAL vu_u2 = AC.dot (AB) * _u2;
        FCL_REAL wm_vm = AQ.dot(m) / AC.dot (m);

        next.coefficient[0] = wm_vm; // C
        next.coefficient[1] = wu_u2 - wm_vm*vu_u2; // B
        next.coefficient[2] = 1 - next.coefficient[0] - next.coefficient[1]; // A
        assert (0. <= next.coefficient[0] && next.coefficient[0] <= 1.);
        assert (0. <= next.coefficient[1] && next.coefficient[1] <= 1.);
        assert (0. <= next.coefficient[2] && next.coefficient[2] <= 1.);

        return ray.squaredNorm();
      } else { // Region 3
        ray = ABC;

        //next.vertex[0] = current.vertex[b];
        next.vertex[0] = current.vertex[c];
        next.vertex[1] = current.vertex[b];
        next.vertex[2] = current.vertex[a];
        next.rank = 3;

        // To ensure backward compatibility
        FCL_REAL s = ABC.squaredNorm();
        ray *= A.dot(ABC) / s;
        next.coefficient[2] = -1;
        next.coefficient[1] = -1;
        next.coefficient[0] = -1;
        assert (0. <= next.coefficient[0] && next.coefficient[0] <= 1.);
        assert (0. <= next.coefficient[1] && next.coefficient[1] <= 1.);
        assert (0. <= next.coefficient[2] && next.coefficient[2] <= 1.);

        return ray.squaredNorm();
      }
    }
  }
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

      FCL_REAL tmpv = simplex.coefficient[0];
      simplex.coefficient[0] = simplex.coefficient[1];
      simplex.coefficient[1] = tmpv;
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

      Vec3f projection = outer.n * outer.d;
      normal = outer.n;
      depth = outer.d;
      result.rank = 3;
      result.vertex[0] = outer.vertex[0];
      result.vertex[1] = outer.vertex[1];
      result.vertex[2] = outer.vertex[2];
      result.coefficient[0] = ((outer.vertex[1]->w - projection).cross
                               (outer.vertex[2]->w - projection)).norm();
      result.coefficient[1] = ((outer.vertex[2]->w - projection).cross
                               (outer.vertex[0]->w - projection)).norm();
      result.coefficient[2] = ((outer.vertex[0]->w - projection).cross
                               (outer.vertex[1]->w - projection)).norm();

      FCL_REAL sum = result.coefficient[0] + result.coefficient[1] +
        result.coefficient[2];
      result.coefficient[0] /= sum;
      result.coefficient[1] /= sum;
      result.coefficient[2] /= sum;
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
  result.coefficient[0] = 1;
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
