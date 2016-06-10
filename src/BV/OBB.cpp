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

#include <hpp/fcl/BV/OBB.h>
#include <hpp/fcl/BVH/BVH_utility.h>
#include <hpp/fcl/math/transform.h>

#include <iostream>
#include <limits>

namespace fcl
{

/// @brief Compute the 8 vertices of a OBB
inline void computeVertices(const OBB& b, Vec3f vertices[8])
{
  Matrix3f extAxes (b.axes * b.extent.asDiagonal());
  vertices[0].noalias() = b.To + extAxes * Vec3f(-1,-1,-1);
  vertices[1].noalias() = b.To + extAxes * Vec3f( 1,-1,-1);
  vertices[2].noalias() = b.To + extAxes * Vec3f( 1, 1,-1);
  vertices[3].noalias() = b.To + extAxes * Vec3f(-1, 1,-1);
  vertices[4].noalias() = b.To + extAxes * Vec3f(-1,-1, 1);
  vertices[5].noalias() = b.To + extAxes * Vec3f( 1,-1, 1);
  vertices[6].noalias() = b.To + extAxes * Vec3f( 1, 1, 1);
  vertices[7].noalias() = b.To + extAxes * Vec3f(-1, 1, 1);
}

/// @brief OBB merge method when the centers of two smaller OBB are far away
inline OBB merge_largedist(const OBB& b1, const OBB& b2)
{
  OBB b;
  Vec3f vertex[16];
  computeVertices(b1, vertex);
  computeVertices(b2, vertex + 8);
  Matrix3f M;
  Vec3f E[3];
  Matrix3f::Scalar s[3] = {0, 0, 0};

  // obb axes
  b.axes.col(0).noalias() = (b1.To - b2.To).normalized();

  Vec3f vertex_proj[16];
  for(int i = 0; i < 16; ++i)
    vertex_proj[i] = vertex[i] - b.axes.col(0) * vertex[i].dot(b.axes.col(0));

  getCovariance(vertex_proj, NULL, NULL, NULL, 16, M);
  eigen(M, s, E);

  int min, mid, max;
  if (s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if (s[2] < s[min]) { mid = min; min = 2; }
  else if (s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }


  b.axes.col(1) << E[0][max], E[1][max], E[2][max];
  b.axes.col(2) << E[0][mid], E[1][mid], E[2][mid];

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(vertex, NULL, NULL, NULL, 16, b.axes, center, extent);

  b.To = center;
  b.extent = extent;

  return b;
}


/// @brief OBB merge method when the centers of two smaller OBB are close
inline OBB merge_smalldist(const OBB& b1, const OBB& b2)
{
  OBB b;
  b.To = (b1.To + b2.To) * 0.5;
  Quaternion3f q0, q1;
  q0.fromAxes(b1.axes);
  q1.fromAxes(b2.axes);
  if(q0.dot(q1) < 0)
    q1 = -q1;

  Quaternion3f q = q0 + q1;
  FCL_REAL inv_length = 1.0 / std::sqrt(q.dot(q));
  q = q * inv_length;
  q.toAxes(b.axes);


  Vec3f vertex[8], diff;
  FCL_REAL real_max = std::numeric_limits<FCL_REAL>::max();
  Vec3f pmin(real_max, real_max, real_max);
  Vec3f pmax(-real_max, -real_max, -real_max);

  computeVertices(b1, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      FCL_REAL dot = diff.dot(b.axes.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  computeVertices(b2, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      FCL_REAL dot = diff.dot(b.axes.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  for(int j = 0; j < 3; ++j)
  {
    b.To.noalias() += (b.axes.col(j) * (0.5 * (pmax[j] + pmin[j])));
    b.extent[j] = 0.5 * (pmax[j] - pmin[j]);
  }

  return b;
}

bool obbDisjoint(const Matrix3f& B, const Vec3f& T, const Vec3f& a, const Vec3f& b)
{
  register FCL_REAL t, s;
  const FCL_REAL reps = 1e-6;

  Matrix3f Bf (B.array().abs() + reps);
  // Bf += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  // if(t > (a[0] + Bf.dotX(b)))
  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  // s =  B.transposeDotX(T);
  s =  B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[0] + Bf.transposeDotX(a)))
  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  // if(t > (a[1] + Bf.dotY(b)))
  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  // if(t > (a[2] + Bf.dotZ(b)))
  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  // s = B.transposeDotY(T);
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[1] + Bf.transposeDotY(a)))
  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  // s = B.transposeDotZ(T);
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  // if(t > (b[2] + Bf.transposeDotZ(a)))
  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;

}

// B, T orientation and position of 2nd OBB in frame of 1st OBB,
// a extent of 1st OBB,
// b extent of 2nd OBB.
bool obbDisjointAndLowerBoundDistance (const Matrix3f& B, const Vec3f& T,
				       const Vec3f& a, const Vec3f& b,
				       FCL_REAL& squaredLowerBoundDistance)
{
  FCL_REAL t, s;
  const FCL_REAL reps = 1e-6;
  FCL_REAL diff;
  FCL_REAL breakDistance = 2e-3 * (a [0] + a [1] + a [2] +
				   b [0] + b [1] + b [2]);
  FCL_REAL breakDistance2 = breakDistance * breakDistance;

  // Matrix3f Bf = abs(B);
  // Bf += reps;
  Matrix3f Bf (B.array().abs() + reps);
  squaredLowerBoundDistance = 0;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);

  diff = t - (a[0] + Bf.row(0).dot(b));
  if (diff > 0) {
    squaredLowerBoundDistance += diff*diff;
  }

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  diff = t - (a[1] + Bf.row(1).dot(b));
  if (diff > 0) {
    squaredLowerBoundDistance += diff*diff;
  }

  // A0 x A1 = A2
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  diff = t - (a[2] + Bf.row(2).dot(b));
  if (diff > 0) {
    squaredLowerBoundDistance += diff*diff;
  }

  if (squaredLowerBoundDistance > breakDistance2)
    return true;

  // B1 x B2 = B0
  s =  B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  diff = t - (b[0] + Bf.col(0).dot(a));
  if (diff > 0) {
    squaredLowerBoundDistance += diff*diff;
  }

  // B2 x B0 = B1
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  diff = t - (b[1] + Bf.col(1).dot(a));
  if (diff > 0) {
    squaredLowerBoundDistance += diff*diff;
  }

  // B0 x B1 = B2
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  diff = t - (b[2] + Bf.col(2).dot(a));
  if (diff > 0) {
    squaredLowerBoundDistance += diff*diff;
  }

  if (squaredLowerBoundDistance > breakDistance2)
    return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  FCL_REAL sinus2;
  diff = t - (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
	      b[1] * Bf(0, 2) + b[2] * Bf(0, 1));
  // We need to divide by the norm || A0 x B0 ||
  // As ||A0|| = ||B0|| = 1,
  //              2            2
  // || A0 x B0 ||  + (A0 | B0)  = 1
  if (diff > 0) {
    sinus2 = 1 - Bf (0,0) * Bf (0,0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
	      b[0] * Bf(0, 2) + b[2] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf (0,1) * Bf (0,1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
	      b[0] * Bf(0, 1) + b[1] * Bf(0, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf (0,2) * Bf (0,2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A1 x B0
  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
	      b[1] * Bf(1, 2) + b[2] * Bf(1, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf (1,0) * Bf (1,0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
	      b[0] * Bf(1, 2) + b[2] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf (1,1) * Bf (1,1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
	      b[0] * Bf(1, 1) + b[1] * Bf(1, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf (1,2) * Bf (1,2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
	      b[1] * Bf(2, 2) + b[2] * Bf(2, 1));
  if (diff > 0) {
    sinus2 = 1 - Bf (2,0) * Bf (2,0);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
	      b[0] * Bf(2, 2) + b[2] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf (2,1) * Bf (2,1);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  diff = t - (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
	      b[0] * Bf(2, 1) + b[1] * Bf(2, 0));
  if (diff > 0) {
    sinus2 = 1 - Bf (2,2) * Bf (2,2);
    if (sinus2 > 1e-6) {
      squaredLowerBoundDistance = diff * diff / sinus2;
      if (squaredLowerBoundDistance > breakDistance2) {
	return true;
      }
    }
  }

  return false;

}



bool OBB::overlap(const OBB& other) const
{
  /// compute what transform [R,T] that takes us from cs1 to cs2.
  /// [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
  /// First compute the rotation part, then translation part
  Vec3f T (axes.transpose() * (other.To - To));
  Matrix3f R (axes.transpose() * other.axes);

  return !obbDisjoint(R, T, extent, other.extent);
}

  bool OBB::overlap(const OBB& other, FCL_REAL& sqrDistLowerBound) const
  {
    /// compute what transform [R,T] that takes us from cs1 to cs2.
    /// [R,T] = [R1,T1]'[R2,T2] = [R1',-R1'T][R2,T2] = [R1'R2, R1'(T2-T1)]
    /// First compute the rotation part, then translation part
    // Vec3f t = other.To - To; // T2 - T1
    // Vec3f T(t.dot(axis[0]), t.dot(axis[1]), t.dot(axis[2])); // R1'(T2-T1)
    // Matrix3f R(axis[0].dot(other.axis[0]), axis[0].dot(other.axis[1]),
	       // axis[0].dot(other.axis[2]),
	       // axis[1].dot(other.axis[0]), axis[1].dot(other.axis[1]),
	       // axis[1].dot(other.axis[2]),
	       // axis[2].dot(other.axis[0]), axis[2].dot(other.axis[1]),
	       // axis[2].dot(other.axis[2]));
  Vec3f T (axes.transpose() * (other.To - To));
  Matrix3f R (axes.transpose() * other.axes);

  return !obbDisjointAndLowerBoundDistance
    (R, T, extent, other.extent, sqrDistLowerBound);
}


bool OBB::contain(const Vec3f& p) const
{
  Vec3f local_p = p - To;
  FCL_REAL proj = local_p.dot(axes.col(0));
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(axes.col(1));
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(axes.col(2));
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

OBB& OBB::operator += (const Vec3f& p)
{
  OBB bvp;
  bvp.To.noalias() = p;
  bvp.axes.noalias() = axes;
  bvp.extent.setZero();

  *this += bvp;
  return *this;
}

OBB OBB::operator + (const OBB& other) const
{
  Vec3f center_diff = To - other.To;
  FCL_REAL max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  FCL_REAL max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
  if(center_diff.norm() > 2 * (max_extent + max_extent2))
  {
    return merge_largedist(*this, other);
  }
  else
  {
    return merge_smalldist(*this, other);
  }
}


FCL_REAL OBB::distance(const OBB& /*other*/, Vec3f* /*P*/, Vec3f* /*Q*/) const
{
  std::cerr << "OBB distance not implemented!" << std::endl;
  return 0.0;
}


bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBB& b1, const OBB& b2)
{
  Vec3f Ttemp (R0 * b2.To + T0 - b1.To);
  Vec3f T (b1.axes.transpose() * Ttemp);
  Matrix3f R (b1.axes.transpose() * R0 * b2.axes);

  return !obbDisjoint(R, T, b1.extent, b2.extent);
}

bool overlap(const Matrix3f& R0, const Vec3f& T0, const OBB& b1, const OBB& b2,
	     FCL_REAL& sqrDistLowerBound)
{
  Vec3f Ttemp (R0 * b2.To + T0 - b1.To);
  Vec3f T (b1.axes.transpose() * Ttemp);
  Matrix3f R (b1.axes.transpose() * R0 * b2.axes);

  return !obbDisjointAndLowerBoundDistance (R, T, b1.extent, b2.extent,
					    sqrDistLowerBound);
}

OBB translate(const OBB& bv, const Vec3f& t)
{
  OBB res(bv);
  res.To += t;
  return res;
}

}
