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

#ifndef HPP_FCL_INTERSECT_H
#define HPP_FCL_INTERSECT_H

/// @cond INTERNAL

#include <hpp/fcl/math/transform.h>

namespace hpp {
namespace fcl {

/// @brief CCD intersect kernel among primitives
class HPP_FCL_DLLAPI Intersect {
 public:
  static bool buildTrianglePlane(const Vec3f& v1, const Vec3f& v2,
                                 const Vec3f& v3, Vec3f* n, FCL_REAL* t);
};  // class Intersect

/// @brief Project functions
class HPP_FCL_DLLAPI Project {
 public:
  struct HPP_FCL_DLLAPI ProjectResult {
    /// @brief Parameterization of the projected point (based on the simplex to
    /// be projected, use 2 or 3 or 4 of the array)
    FCL_REAL parameterization[4];

    /// @brief square distance from the query point to the projected simplex
    FCL_REAL sqr_distance;

    /// @brief the code of the projection type
    unsigned int encode;

    ProjectResult() : sqr_distance(-1), encode(0) {}
  };

  /// @brief Project point p onto line a-b
  static ProjectResult projectLine(const Vec3f& a, const Vec3f& b,
                                   const Vec3f& p);

  /// @brief Project point p onto triangle a-b-c
  static ProjectResult projectTriangle(const Vec3f& a, const Vec3f& b,
                                       const Vec3f& c, const Vec3f& p);

  /// @brief Project point p onto tetrahedra a-b-c-d
  static ProjectResult projectTetrahedra(const Vec3f& a, const Vec3f& b,
                                         const Vec3f& c, const Vec3f& d,
                                         const Vec3f& p);

  /// @brief Project origin (0) onto line a-b
  static ProjectResult projectLineOrigin(const Vec3f& a, const Vec3f& b);

  /// @brief Project origin (0) onto triangle a-b-c
  static ProjectResult projectTriangleOrigin(const Vec3f& a, const Vec3f& b,
                                             const Vec3f& c);

  /// @brief Project origin (0) onto tetrahedran a-b-c-d
  static ProjectResult projectTetrahedraOrigin(const Vec3f& a, const Vec3f& b,
                                               const Vec3f& c, const Vec3f& d);
};

/// @brief Triangle distance functions
class HPP_FCL_DLLAPI TriangleDistance {
 public:
  /// @brief Returns closest points between an segment pair.
  /// The first segment is P + t * A
  /// The second segment is Q + t * B
  /// X, Y are the closest points on the two segments
  /// VEC is the vector between X and Y
  static void segPoints(const Vec3f& P, const Vec3f& A, const Vec3f& Q,
                        const Vec3f& B, Vec3f& VEC, Vec3f& X, Vec3f& Y);

  /// Compute squared distance between triangles
  /// @param S and T are two triangles
  /// @retval P, Q closest points if triangles do not intersect.
  /// @return squared distance if triangles do not intersect, 0 otherwise.
  /// If the triangles are disjoint, P and Q give the closet points of
  /// S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points
  /// from the triangles, not coincident points on the intersection of the
  /// triangles, as might be expected.
  static FCL_REAL sqrTriDistance(const Vec3f S[3], const Vec3f T[3], Vec3f& P,
                                 Vec3f& Q);

  static FCL_REAL sqrTriDistance(const Vec3f& S1, const Vec3f& S2,
                                 const Vec3f& S3, const Vec3f& T1,
                                 const Vec3f& T2, const Vec3f& T3, Vec3f& P,
                                 Vec3f& Q);

  /// Compute squared distance between triangles
  /// @param S and T are two triangles
  /// @param R, Tl, rotation and translation applied to T,
  /// @retval P, Q closest points if triangles do not intersect.
  /// @return squared distance if triangles do not intersect, 0 otherwise.
  /// If the triangles are disjoint, P and Q give the closet points of
  /// S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points
  /// from the triangles, not coincident points on the intersection of the
  /// triangles, as might be expected.
  static FCL_REAL sqrTriDistance(const Vec3f S[3], const Vec3f T[3],
                                 const Matrix3f& R, const Vec3f& Tl, Vec3f& P,
                                 Vec3f& Q);

  /// Compute squared distance between triangles
  /// @param S and T are two triangles
  /// @param tf, rotation and translation applied to T,
  /// @retval P, Q closest points if triangles do not intersect.
  /// @return squared distance if triangles do not intersect, 0 otherwise.
  /// If the triangles are disjoint, P and Q give the closet points of
  /// S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points
  /// from the triangles, not coincident points on the intersection of the
  /// triangles, as might be expected.
  static FCL_REAL sqrTriDistance(const Vec3f S[3], const Vec3f T[3],
                                 const Transform3f& tf, Vec3f& P, Vec3f& Q);

  /// Compute squared distance between triangles
  /// @param S1, S2, S3 and T1, T2, T3 are triangle vertices
  /// @param R, Tl, rotation and translation applied to T1, T2, T3,
  /// @retval P, Q closest points if triangles do not intersect.
  /// @return squared distance if triangles do not intersect, 0 otherwise.
  /// If the triangles are disjoint, P and Q give the closet points of
  /// S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points
  /// from the triangles, not coincident points on the intersection of the
  /// triangles, as might be expected.
  static FCL_REAL sqrTriDistance(const Vec3f& S1, const Vec3f& S2,
                                 const Vec3f& S3, const Vec3f& T1,
                                 const Vec3f& T2, const Vec3f& T3,
                                 const Matrix3f& R, const Vec3f& Tl, Vec3f& P,
                                 Vec3f& Q);

  /// Compute squared distance between triangles
  /// @param S1, S2, S3 and T1, T2, T3 are triangle vertices
  /// @param tf, rotation and translation applied to T1, T2, T3,
  /// @retval P, Q closest points if triangles do not intersect.
  /// @return squared distance if triangles do not intersect, 0 otherwise.
  /// If the triangles are disjoint, P and Q give the closet points of
  /// S and T respectively. However,
  /// if the triangles overlap, P and Q are basically a random pair of points
  /// from the triangles, not coincident points on the intersection of the
  /// triangles, as might be expected.
  static FCL_REAL sqrTriDistance(const Vec3f& S1, const Vec3f& S2,
                                 const Vec3f& S3, const Vec3f& T1,
                                 const Vec3f& T2, const Vec3f& T3,
                                 const Transform3f& tf, Vec3f& P, Vec3f& Q);
};

}  // namespace fcl

}  // namespace hpp

/// @endcond

#endif
