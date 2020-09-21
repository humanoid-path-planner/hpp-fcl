/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2018-2019, Centre National de la Recherche Scientifique
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
/** \author Jia Pan, Florent Lamiraux */

#ifndef HPP_FCL_SRC_NARROWPHASE_DETAILS_H
# define HPP_FCL_SRC_NARROWPHASE_DETAILS_H

#include <hpp/fcl/internal/traversal_node_setup.h>
#include <hpp/fcl/narrowphase/narrowphase.h>

namespace hpp
{
namespace fcl {
  namespace details
  {
    // Compute the point on a line segment that is the closest point on the
    // segment to to another point. The code is inspired by the explanation
    // given by Dan Sunday's page:
    //   http://geomalgorithms.com/a02-_lines.html
    static inline void lineSegmentPointClosestToPoint (const Vec3f &p, const Vec3f &s1, const Vec3f &s2, Vec3f &sp) {
      Vec3f v = s2 - s1;
      Vec3f w = p - s1;

      FCL_REAL c1 = w.dot(v);
      FCL_REAL c2 = v.dot(v);

      if (c1 <= 0) {
        sp = s1;
      } else if (c2 <= c1) {
        sp = s2;
      } else {
        FCL_REAL b = c1/c2;
        Vec3f Pb = s1 + v * b;
        sp = Pb;
      }
    }

    inline bool sphereCapsuleIntersect
      (const Sphere& s1, const Transform3f& tf1,
       const Capsule& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f* contact_points, Vec3f* normal_)
    {
      Vec3f pos1 (tf2.transform (Vec3f (0., 0.,  s2.halfLength))); // from distance function
      Vec3f pos2 (tf2.transform (Vec3f (0., 0., -s2.halfLength)));
      Vec3f s_c = tf1.getTranslation ();

      Vec3f segment_point;

      lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
      Vec3f diff = s_c - segment_point;

      FCL_REAL diffN = diff.norm();
      distance = diffN - s1.radius - s2.radius;

      if (distance > 0)
        return false;

      // Vec3f normal (-diff.normalized());

      if (normal_)
        *normal_ = -diff / diffN;

      if (contact_points) {
        *contact_points = segment_point + diff * s2.radius;
      }

      return true;
    }

    inline bool sphereCapsuleDistance
      (const Sphere& s1, const Transform3f& tf1,
       const Capsule& s2, const Transform3f& tf2,
       FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal)
    {
      Vec3f pos1 (tf2.transform (Vec3f (0., 0.,  s2.halfLength)));
      Vec3f pos2 (tf2.transform (Vec3f (0., 0., -s2.halfLength)));
      Vec3f s_c = tf1.getTranslation ();

      Vec3f segment_point;

      lineSegmentPointClosestToPoint (s_c, pos1, pos2, segment_point);
      normal = segment_point - s_c;
      FCL_REAL norm (normal.norm());
      dist = norm - s1.radius - s2.radius;

      static const FCL_REAL eps (std::numeric_limits<FCL_REAL>::epsilon());
      if (norm > eps) {
        normal.normalize();
      } else {
        normal << 1,0,0;
      }
      p1 = s_c + normal * s1.radius;
      p2 = segment_point - normal * s2.radius;

      if(dist <= 0) {
        p1 = p2 = .5 * (p1+p2);
        return false;
      }
      return true;
    }

    inline bool sphereCylinderDistance
      (const Sphere& s1, const Transform3f& tf1,
       const Cylinder& s2, const Transform3f& tf2,
       FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal)
    {
      static const FCL_REAL eps (sqrt (std::numeric_limits <FCL_REAL>::epsilon ()));
      FCL_REAL r1 (s1.radius);
      FCL_REAL r2 (s2.radius);
      FCL_REAL lz2 (s2.halfLength);
      // boundaries of the cylinder axis
      Vec3f A (tf2.transform (Vec3f (0, 0, -lz2)));
      Vec3f B (tf2.transform (Vec3f (0, 0,  lz2)));
      // Position of the center of the sphere
      Vec3f S (tf1.getTranslation ());
      // axis of the cylinder
      Vec3f u (tf2.getRotation ().col (2));
      /// @todo a tiny performance improvement could be achieved using the abscissa with S as the origin
      assert ((B - A - (s2.halfLength * 2) * u).norm () < eps);
      Vec3f AS (S - A);
      // abscissa of S on cylinder axis with A as the origin
      FCL_REAL s (u.dot (AS));
      Vec3f P (A + s*u);
      Vec3f PS (S - P);
      FCL_REAL dPS = PS.norm ();
      // Normal to cylinder axis such that plane (A, u, v) contains sphere
      // center
      Vec3f v (0, 0, 0);
      if (dPS > eps) {
        // S is not on cylinder axis
        v = (1/dPS) * PS;
      }
      if (s <= 0) {
        if (dPS <= r2) {
          // closest point on cylinder is on cylinder disc basis
          dist = -s - r1; p1 = S + r1 * u; p2 = A + dPS * v; normal = u;
        } else {
          // closest point on cylinder is on cylinder circle basis
          p2 = A + r2 * v;
          Vec3f Sp2 (p2 - S);
          FCL_REAL dSp2 = Sp2.norm ();
          if (dSp2 > eps) {
            normal = (1/dSp2) * Sp2;
            p1 = S + r1 * normal;
            dist = dSp2 - r1;
            assert (fabs (dist) - (p1-p2).norm () < eps);
          } else {
            // Center of sphere is on cylinder boundary
            normal = .5 * (A + B) - p2; normal.normalize ();
            p1 = p2;
            dist = -r1;
          }
        }
      } else if (s <= (s2.halfLength * 2)) {
        // 0 < s <= s2.lz
        normal = -v;
        dist = dPS - r1 - r2;
        if (dPS <= r2) {
          // Sphere center is inside cylinder
          p1 = p2 = S;
        } else {
          p2 = P + r2*v; p1 = S - r1*v;
        }
      } else {
        // lz < s
        if (dPS <= r2) {
          // closest point on cylinder is on cylinder disc basis
          dist = s - (s2.halfLength * 2) - r1; p1 = S - r1 * u; p2 = B + dPS * v; normal = -u;
        } else {
          // closest point on cylinder is on cylinder circle basis
          p2 = B + r2 * v;
          Vec3f Sp2 (p2 - S);
          FCL_REAL dSp2 = Sp2.norm ();
          if (dSp2 > eps) {
            normal = (1/dSp2) * Sp2;
            p1 = S + r1 * normal;
            dist = dSp2 - r1;
            assert (fabs (dist) - (p1-p2).norm () < eps);
          } else {
            // Center of sphere is on cylinder boundary
            normal = .5 * (A + B) - p2; normal.normalize ();
            p1 = p2;
            dist = -r1;
          }
        }
      }
      if (dist < 0) {
        p1 = p2 = .5 *  (p1 + p2);
      }
      return (dist > 0);
    }

    inline bool sphereSphereIntersect
      (const Sphere& s1, const Transform3f& tf1,
       const Sphere& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f* contact_points, Vec3f* normal)
    {
      const Vec3f diff = tf2.getTranslation() - tf1.getTranslation();
      FCL_REAL len = diff.norm();
      distance = len - s1.radius - s2.radius;
      if(distance > 0)
        return false;

      // If the centers of two sphere are at the same position, the normal is (0, 0, 0).
      // Otherwise, normal is pointing from center of object 1 to center of object 2
      if(normal)
        {
          if(len > 0)
            *normal = diff / len;
          else
            *normal = diff;
        }

      if(contact_points)
        *contact_points = tf1.getTranslation() + diff * s1.radius / (s1.radius + s2.radius);

      return true;
    }


    inline bool sphereSphereDistance(const Sphere& s1, const Transform3f& tf1,
                                     const Sphere& s2, const Transform3f& tf2,
                                     FCL_REAL& dist, Vec3f& p1, Vec3f& p2,
                                     Vec3f& normal)
    {
      const Vec3f & o1 = tf1.getTranslation();
      const Vec3f & o2 = tf2.getTranslation();
      Vec3f diff = o1 - o2;
      FCL_REAL len = diff.norm();
      normal = -diff/len;
      dist = len - s1.radius - s2.radius;

      p1.noalias() = o1 + normal * s1.radius;
      p2.noalias() = o2 - normal * s2.radius;

      return (dist >=0);
    }

    /** @brief the minimum distance from a point to a line */
    inline FCL_REAL segmentSqrDistance
      (const Vec3f& from, const Vec3f& to,const Vec3f& p, Vec3f& nearest)
    {
      Vec3f diff = p - from;
      Vec3f v = to - from;
      FCL_REAL t = v.dot(diff);

      if(t > 0)
        {
          FCL_REAL dotVV = v.squaredNorm();
          if(t < dotVV)
            {
              t /= dotVV;
              diff -= v * t;
            }
          else
            {
              t = 1;
              diff -= v;
            }
        }
      else
        t = 0;

      nearest.noalias() = from + v * t;
      return diff.squaredNorm();
    }

    /// @brief Whether a point's projection is in a triangle
    inline bool projectInTriangle (const Vec3f& p1, const Vec3f& p2,
                                   const Vec3f& p3, const Vec3f& normal,
                                   const Vec3f& p)
    {
      Vec3f edge1(p2 - p1);
      Vec3f edge2(p3 - p2);
      Vec3f edge3(p1 - p3);

      Vec3f p1_to_p(p - p1);
      Vec3f p2_to_p(p - p2);
      Vec3f p3_to_p(p - p3);

      Vec3f edge1_normal(edge1.cross(normal));
      Vec3f edge2_normal(edge2.cross(normal));
      Vec3f edge3_normal(edge3.cross(normal));

      FCL_REAL r1, r2, r3;
      r1 = edge1_normal.dot(p1_to_p);
      r2 = edge2_normal.dot(p2_to_p);
      r3 = edge3_normal.dot(p3_to_p);
      if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
           ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
        return true;
      return false;
    }

    // Intersection between sphere and triangle
    // Sphere is in position tf1, Triangle is expressed in global frame
    inline bool sphereTriangleIntersect
      (const Sphere& s, const Transform3f& tf1,
       const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal_)
    {
      Vec3f normal = (P2 - P1).cross(P3 - P1);
      normal.normalize();
      const Vec3f& center = tf1.getTranslation();
      const FCL_REAL& radius = s.radius;
      assert (radius >= 0);
      Vec3f p1_to_center = center - P1;
      FCL_REAL distance_from_plane = p1_to_center.dot(normal);
      Vec3f closest_point
        (Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN()));
      FCL_REAL min_distance_sqr, distance_sqr;

      if(distance_from_plane < 0) {
        distance_from_plane *= -1;
        normal *= -1;
      }

      if (projectInTriangle (P1, P2, P3, normal, center)) {
        closest_point = center - normal * distance_from_plane;
        min_distance_sqr = distance_from_plane;
      } else {
        // Compute distance to each each and take minimal distance
        Vec3f nearest_on_edge;
        min_distance_sqr = segmentSqrDistance(P1, P2, center, closest_point);

        distance_sqr = segmentSqrDistance(P2, P3, center, nearest_on_edge);
        if (distance_sqr < min_distance_sqr) {
          min_distance_sqr = distance_sqr;
          closest_point = nearest_on_edge;
        }
        distance_sqr = segmentSqrDistance(P3, P1, center, nearest_on_edge);
        if (distance_sqr < min_distance_sqr) {
          min_distance_sqr = distance_sqr;
          closest_point = nearest_on_edge;
        }
      }

      if (min_distance_sqr < radius * radius) {
        // Collision
        normal_ = (closest_point - center).normalized ();
        p1 = p2 = closest_point;
        distance = sqrt (min_distance_sqr) - radius;
        assert (distance < 0);
        return true;
      } else {
        normal_ = (closest_point - center).normalized ();
        p1 = center + normal_ * radius;
        p2 = closest_point;
        distance = sqrt (min_distance_sqr) - radius;
        assert (distance >= 0);
        return false;
      }
    }

    inline bool sphereTriangleDistance
      (const Sphere& sp, const Transform3f& tf,
       const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, FCL_REAL* dist)
    {
      // from geometric tools, very different from the collision code.

      const Vec3f& center = tf.getTranslation();
      FCL_REAL radius = sp.radius;
      Vec3f diff = P1 - center;
      Vec3f edge0 = P2 - P1;
      Vec3f edge1 = P3 - P1;
      FCL_REAL a00 = edge0.squaredNorm();
      FCL_REAL a01 = edge0.dot(edge1);
      FCL_REAL a11 = edge1.squaredNorm();
      FCL_REAL b0 = diff.dot(edge0);
      FCL_REAL b1 = diff.dot(edge1);
      FCL_REAL c = diff.squaredNorm();
      FCL_REAL det = fabs(a00*a11 - a01*a01);
      FCL_REAL s = a01*b1 - a11*b0;
      FCL_REAL t = a01*b0 - a00*b1;

      FCL_REAL sqr_dist;

      if(s + t <= det)
        {
          if(s < 0)
            {
              if(t < 0)  // region 4
                {
                  if(b0 < 0)
                    {
                      t = 0;
                      if(-b0 >= a00)
                        {
                          s = 1;
                          sqr_dist = a00 + 2*b0 + c;
                        }
                      else
                        {
                          s = -b0/a00;
                          sqr_dist = b0*s + c;
                        }
                    }
                  else
                    {
                      s = 0;
                      if(b1 >= 0)
                        {
                          t = 0;
                          sqr_dist = c;
                        }
                      else if(-b1 >= a11)
                        {
                          t = 1;
                          sqr_dist = a11 + 2*b1 + c;
                        }
                      else
                        {
                          t = -b1/a11;
                          sqr_dist = b1*t + c;
                        }
                    }
                }
              else  // region 3
                {
                  s = 0;
                  if(b1 >= 0)
                    {
                      t = 0;
                      sqr_dist = c;
                    }
                  else if(-b1 >= a11)
                    {
                      t = 1;
                      sqr_dist = a11 + 2*b1 + c;
                    }
                  else
                    {
                      t = -b1/a11;
                      sqr_dist = b1*t + c;
                    }
                }
            }
          else if(t < 0)  // region 5
            {
              t = 0;
              if(b0 >= 0)
                {
                  s = 0;
                  sqr_dist = c;
                }
              else if(-b0 >= a00)
                {
                  s = 1;
                  sqr_dist = a00 + 2*b0 + c;
                }
              else
                {
                  s = -b0/a00;
                  sqr_dist = b0*s + c;
                }
            }
          else  // region 0
            {
              // minimum at interior point
              FCL_REAL inv_det = (1)/det;
              s *= inv_det;
              t *= inv_det;
              sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
            }
        }
      else
        {
          FCL_REAL tmp0, tmp1, numer, denom;

          if(s < 0)  // region 2
            {
              tmp0 = a01 + b0;
              tmp1 = a11 + b1;
              if(tmp1 > tmp0)
                {
                  numer = tmp1 - tmp0;
                  denom = a00 - 2*a01 + a11;
                  if(numer >= denom)
                    {
                      s = 1;
                      t = 0;
                      sqr_dist = a00 + 2*b0 + c;
                    }
                  else
                    {
                      s = numer/denom;
                      t = 1 - s;
                      sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
                    }
                }
              else
                {
                  s = 0;
                  if(tmp1 <= 0)
                    {
                      t = 1;
                      sqr_dist = a11 + 2*b1 + c;
                    }
                  else if(b1 >= 0)
                    {
                      t = 0;
                      sqr_dist = c;
                    }
                  else
                    {
                      t = -b1/a11;
                      sqr_dist = b1*t + c;
                    }
                }
            }
          else if(t < 0)  // region 6
            {
              tmp0 = a01 + b1;
              tmp1 = a00 + b0;
              if(tmp1 > tmp0)
                {
                  numer = tmp1 - tmp0;
                  denom = a00 - 2*a01 + a11;
                  if(numer >= denom)
                    {
                      t = 1;
                      s = 0;
                      sqr_dist = a11 + 2*b1 + c;
                    }
                  else
                    {
                      t = numer/denom;
                      s = 1 - t;
                      sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
                    }
                }
              else
                {
                  t = 0;
                  if(tmp1 <= 0)
                    {
                      s = 1;
                      sqr_dist = a00 + 2*b0 + c;
                    }
                  else if(b0 >= 0)
                    {
                      s = 0;
                      sqr_dist = c;
                    }
                  else
                    {
                      s = -b0/a00;
                      sqr_dist = b0*s + c;
                    }
                }
            }
          else  // region 1
            {
              numer = a11 + b1 - a01 - b0;
              if(numer <= 0)
                {
                  s = 0;
                  t = 1;
                  sqr_dist = a11 + 2*b1 + c;
                }
              else
                {
                  denom = a00 - 2*a01 + a11;
                  if(numer >= denom)
                    {
                      s = 1;
                      t = 0;
                      sqr_dist = a00 + 2*b0 + c;
                    }
                  else
                    {
                      s = numer/denom;
                      t = 1 - s;
                      sqr_dist = s*(a00*s + a01*t + 2*b0) + t*(a01*s + a11*t + 2*b1) + c;
                    }
                }
            }
        }

      // Account for numerical round-off error.
      if(sqr_dist < 0)
        sqr_dist = 0;

      if(sqr_dist > radius * radius)
        {
          if(dist) *dist = std::sqrt(sqr_dist) - radius;
          return true;
        }
      else
        {
          if(dist) *dist = -1;
          return false;
        }
    }


    inline bool sphereTriangleDistance
      (const Sphere& sp, const Transform3f& tf, const Vec3f& P1,
       const Vec3f& P2, const Vec3f& P3, FCL_REAL* dist, Vec3f* p1, Vec3f* p2)
    {
      if(p1 || p2)
        {
          const Vec3f & o = tf.getTranslation();
          Project::ProjectResult result;
          result = Project::projectTriangle(P1, P2, P3, o);
          if(result.sqr_distance > sp.radius * sp.radius)
            {
              if(dist) *dist = std::sqrt(result.sqr_distance) - sp.radius;
              Vec3f project_p = P1 * result.parameterization[0] + P2 * result.parameterization[1] + P3 * result.parameterization[2];
              Vec3f dir = o - project_p;
              dir.normalize();
              if(p1) { *p1 = o - dir * sp.radius; }
              if(p2) *p2 = project_p;
              return true;
            }
          else
            return false;
        }
      else
        {
          return sphereTriangleDistance(sp, tf, P1, P2, P3, dist);
        }
    }


    inline bool sphereTriangleDistance
      (const Sphere& sp, const Transform3f& tf1, const Vec3f& P1,
       const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
       FCL_REAL* dist, Vec3f* p1, Vec3f* p2)
    {
      bool res = details::sphereTriangleDistance(sp, tf1, tf2.transform(P1), tf2.transform(P2), tf2.transform(P3), dist, p1, p2);
      return res;
    }



    struct HPP_FCL_LOCAL ContactPoint
    {
      Vec3f normal;
      Vec3f point;
      FCL_REAL depth;
    ContactPoint(const Vec3f& n, const Vec3f& p, FCL_REAL d) : normal(n), point(p), depth(d) {}
    };


    static inline void lineClosestApproach(const Vec3f& pa, const Vec3f& ua,
                                           const Vec3f& pb, const Vec3f& ub,
                                           FCL_REAL* alpha, FCL_REAL* beta)
    {
      Vec3f p = pb - pa;
      FCL_REAL uaub = ua.dot(ub);
      FCL_REAL q1 = ua.dot(p);
      FCL_REAL q2 = -ub.dot(p);
      FCL_REAL d = 1 - uaub * uaub;
      if(d <= (FCL_REAL)(0.0001f))
        {
          *alpha = 0;
          *beta = 0;
        }
      else
        {
          d = 1 / d;
          *alpha = (q1 + uaub * q2) * d;
          *beta = (uaub * q1 + q2) * d;
        }
    }

    // find all the intersection points between the 2D rectangle with vertices
    // at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
    // (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
    //
    // the intersection points are returned as x,y pairs in the 'ret' array.
    // the number of intersection points is returned by the function (this will
    // be in the range 0 to 8).
    static int intersectRectQuad2(FCL_REAL h[2], FCL_REAL p[8], FCL_REAL ret[16])
    {
      // q (and r) contain nq (and nr) coordinate points for the current (and
      // chopped) polygons
      int nq = 4, nr = 0;
      FCL_REAL buffer[16];
      FCL_REAL* q = p;
      FCL_REAL* r = ret;
      for(int dir = 0; dir <= 1; ++dir)
        {
          // direction notation: xy[0] = x axis, xy[1] = y axis
          for(int sign = -1; sign <= 1; sign += 2)
            {
              // chop q along the line xy[dir] = sign*h[dir]
              FCL_REAL* pq = q;
              FCL_REAL* pr = r;
              nr = 0;
              for(int i = nq; i > 0; --i)
                {
                  // go through all points in q and all lines between adjacent points
                  if(sign * pq[dir] < h[dir])
                    {
                      // this point is inside the chopping line
                      pr[0] = pq[0];
                      pr[1] = pq[1];
                      pr += 2;
                      nr++;
                      if(nr & 8)
                        {
                          q = r;
                          goto done;
                        }
                    }
                  FCL_REAL* nextq = (i > 1) ? pq+2 : q;
                  if((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir]))
                    {
                      // this line crosses the chopping line
                      pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
                        (nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
                      pr[dir] = sign*h[dir];
                      pr += 2;
                      nr++;
                      if(nr & 8)
                        {
                          q = r;
                          goto done;
                        }
                    }
                  pq += 2;
                }
              q = r;
              r = (q == ret) ? buffer : ret;
              nq = nr;
            }
        }

    done:
      if(q != ret) memcpy(ret, q, nr*2*sizeof(FCL_REAL));
      return nr;
    }

    // given n points in the plane (array p, of size 2*n), generate m points that
    // best represent the whole set. the definition of 'best' here is not
    // predetermined - the idea is to select points that give good box-box
    // collision detection behavior. the chosen point indexes are returned in the
    // array iret (of size m). 'i0' is always the first entry in the array.
    // n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
    // in the range [0..n-1].
    static inline void cullPoints2(int n, FCL_REAL p[], int m, int i0, int iret[])
    {
      // compute the centroid of the polygon in cx,cy
      FCL_REAL a, cx, cy, q;
      switch(n)
        {
        case 1:
          cx = p[0];
          cy = p[1];
          break;
        case 2:
          cx = 0.5 * (p[0] + p[2]);
          cy = 0.5 * (p[1] + p[3]);
          break;
        default:
          a = 0;
          cx = 0;
          cy = 0;
          for(int i = 0; i < n-1; ++i)
            {
              q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
              a += q;
              cx += q*(p[i*2]+p[i*2+2]);
              cy += q*(p[i*2+1]+p[i*2+3]);
            }
          q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
          if(std::abs(a+q) > std::numeric_limits<FCL_REAL>::epsilon())
            a = 1/(3*(a+q));
          else
            a= 1e18f;

          cx = a*(cx + q*(p[n*2-2]+p[0]));
          cy = a*(cy + q*(p[n*2-1]+p[1]));
        }


      // compute the angle of each point w.r.t. the centroid
      FCL_REAL A[8];
      for(int i = 0; i < n; ++i)
        A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

      // search for points that have angles closest to A[i0] + i*(2*pi/m).
      int avail[8];
      for(int i = 0; i < n; ++i) avail[i] = 1;
      avail[i0] = 0;
      iret[0] = i0;
      iret++;
      const double pi = boost::math::constants::pi<FCL_REAL>();
      for(int j = 1; j < m; ++j)
        {
          a = j*(2*pi/m) + A[i0];
          if (a > pi) a -= 2*pi;
          FCL_REAL maxdiff= 1e9, diff;

          *iret = i0;	// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0
          for(int i = 0; i < n; ++i)
            {
              if(avail[i])
                {
                  diff = std::abs(A[i]-a);
                  if(diff > pi) diff = 2*pi - diff;
                  if(diff < maxdiff)
                    {
                      maxdiff = diff;
                      *iret = i;
                    }
                }
            }
          avail[*iret] = 0;
          iret++;
        }
    }



    inline int boxBox2(const Vec3f& halfSide1, const Matrix3f& R1, const Vec3f& T1,
                       const Vec3f& halfSide2, const Matrix3f& R2, const Vec3f& T2,
                       Vec3f& normal, FCL_REAL* depth, int* return_code,
                       int maxc, std::vector<ContactPoint>& contacts)
    {
      const FCL_REAL fudge_factor = FCL_REAL(1.05);
      Vec3f normalC;
      FCL_REAL s, s2, l;
      int invert_normal, code;

      Vec3f p (T2 - T1); // get vector from centers of box 1 to box 2, relative to box 1
      Vec3f pp (R1.transpose() * p); // get pp = p relative to body 1

      // get side lengths / 2
      const Vec3f& A (halfSide1);
      const Vec3f& B (halfSide2);

      // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
      Matrix3f R (R1.transpose() * R2);
      Matrix3f Q (R.cwiseAbs());


      // for all 15 possible separating axes:
      //   * see if the axis separates the boxes. if so, return 0.
      //   * find the depth of the penetration along the separating axis (s2)
      //   * if this is the largest depth so far, record it.
      // the normal vector will be set to the separating axis with the smallest
      // depth. note: normalR is set to point to a column of R1 or R2 if that is
      // the smallest depth normal so far. otherwise normalR is 0 and normalC is
      // set to a vector relative to body 1. invert_normal is 1 if the sign of
      // the normal should be flipped.

      int best_col_id = -1;
      const Matrix3f* normalR = 0;
      FCL_REAL tmp = 0;

      s = - std::numeric_limits<FCL_REAL>::max();
      invert_normal = 0;
      code = 0;

      // separating axis = u1, u2, u3
      tmp = pp[0];
      s2 = std::abs(tmp) - (Q.row(0).dot(B) + A[0]);
      if(s2 > 0) { *return_code = 0; return 0; }
      if(s2 > s)
        {
          s = s2;
          best_col_id = 0;
          normalR = &R1;
          invert_normal = (tmp < 0);
          code = 1;
        }

      tmp = pp[1];
      s2 = std::abs(tmp) - (Q.row(1).dot(B) + A[1]);
      if(s2 > 0) { *return_code = 0; return 0; }
      if(s2 > s)
        {
          s = s2;
          best_col_id = 1;
          normalR = &R1;
          invert_normal = (tmp < 0);
          code = 2;
        }

      tmp = pp[2];
      s2 = std::abs(tmp) - (Q.row(2).dot(B) + A[2]);
      if(s2 > 0) { *return_code = 0; return 0; }
      if(s2 > s)
        {
          s = s2;
          best_col_id = 2;
          normalR = &R1;
          invert_normal = (tmp < 0);
          code = 3;
        }

      // separating axis = v1, v2, v3
      tmp = R2.col(0).dot(p);
      s2 = std::abs(tmp) - (Q.col(0).dot(A) + B[0]);
      if(s2 > 0) { *return_code = 0; return 0; }
      if(s2 > s)
        {
          s = s2;
          best_col_id = 0;
          normalR = &R2;
          invert_normal = (tmp < 0);
          code = 4;
        }

      tmp = R2.col(1).dot(p);
      s2 = std::abs(tmp) - (Q.col(1).dot(A) + B[1]);
      if(s2 > 0) { *return_code = 0; return 0; }
      if(s2 > s)
        {
          s = s2;
          best_col_id = 1;
          normalR = &R2;
          invert_normal = (tmp < 0);
          code = 5;
        }

      tmp = R2.col(2).dot(p);
      s2 =  std::abs(tmp) - (Q.col(2).dot(A) + B[2]);
      if(s2 > 0) { *return_code = 0; return 0; }
      if(s2 > s)
        {
          s = s2;
          best_col_id = 2;
          normalR = &R2;
          invert_normal = (tmp < 0);
          code = 6;
        }


      FCL_REAL fudge2(1.0e-6);
      Q.array() += fudge2;

      Vec3f n;
      static const FCL_REAL eps = std::numeric_limits<FCL_REAL>::epsilon();

      // separating axis = u1 x (v1,v2,v3)
      tmp = pp[2] * R(1, 0) - pp[1] * R(2, 0);
      s2 = std::abs(tmp) - (A[1] * Q(2, 0) + A[2] * Q(1, 0) + B[1] * Q(0, 2) + B[2] * Q(0, 1));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(0, -R(2, 0), R(1, 0));
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 7;
            }
        }

      tmp = pp[2] * R(1, 1) - pp[1] * R(2, 1);
      s2 = std::abs(tmp) - (A[1] * Q(2, 1) + A[2] * Q(1, 1) + B[0] * Q(0, 2) + B[2] * Q(0, 0));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(0, -R(2, 1), R(1, 1));
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 8;
            }
        }

      tmp = pp[2] * R(1, 2) - pp[1] * R(2, 2);
      s2 = std::abs(tmp) - (A[1] * Q(2, 2) + A[2] * Q(1, 2) + B[0] * Q(0, 1) + B[1] * Q(0, 0));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(0, -R(2, 2), R(1, 2));
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 9;
            }
        }

      // separating axis = u2 x (v1,v2,v3)
      tmp = pp[0] * R(2, 0) - pp[2] * R(0, 0);
      s2 = std::abs(tmp) - (A[0] * Q(2, 0) + A[2] * Q(0, 0) + B[1] * Q(1, 2) + B[2] * Q(1, 1));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(R(2, 0), 0, -R(0, 0));
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 10;
            }
        }

      tmp = pp[0] * R(2, 1) - pp[2] * R(0, 1);
      s2 = std::abs(tmp) - (A[0] * Q(2, 1) + A[2] * Q(0, 1) + B[0] * Q(1, 2) + B[2] * Q(1, 0));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(R(2, 1), 0, -R(0, 1));
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 11;
            }
        }

      tmp = pp[0] * R(2, 2) - pp[2] * R(0, 2);
      s2 = std::abs(tmp) - (A[0] * Q(2, 2) + A[2] * Q(0, 2) + B[0] * Q(1, 1) + B[1] * Q(1, 0));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(R(2, 2), 0, -R(0, 2));
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 12;
            }
        }

      // separating axis = u3 x (v1,v2,v3)
      tmp = pp[1] * R(0, 0) - pp[0] * R(1, 0);
      s2 = std::abs(tmp) - (A[0] * Q(1, 0) + A[1] * Q(0, 0) + B[1] * Q(2, 2) + B[2] * Q(2, 1));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(-R(1, 0), R(0, 0), 0);
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 13;
            }
        }

      tmp = pp[1] * R(0, 1) - pp[0] * R(1, 1);
      s2 = std::abs(tmp) - (A[0] * Q(1, 1) + A[1] * Q(0, 1) + B[0] * Q(2, 2) + B[2] * Q(2, 0));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(-R(1, 1), R(0, 1), 0);
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 14;
            }
        }

      tmp = pp[1] * R(0, 2) - pp[0] * R(1, 2);
      s2 = std::abs(tmp) - (A[0] * Q(1, 2) + A[1] * Q(0, 2) + B[0] * Q(2, 1) + B[1] * Q(2, 0));
      if(s2 > 0) { *return_code = 0; return 0; }
      n = Vec3f(-R(1, 2), R(0, 2), 0);
      l = n.norm();
      if(l > eps)
        {
          s2 /= l;
          if(s2 * fudge_factor > s)
            {
              s = s2;
              best_col_id = -1;
              normalC = n / l;
              invert_normal = (tmp < 0);
              code = 15;
            }
        }



      if (!code) { *return_code = code; return 0; }

      // if we get to this point, the boxes interpenetrate. compute the normal
      // in global coordinates.
      if(best_col_id != -1)
        normal = normalR->col(best_col_id);
      else
        normal = R1 * normalC;

      if(invert_normal)
        normal = -normal;

      *depth = -s; // s is negative when the boxes are in collision

      // compute contact point(s)

      if(code > 6)
        {
          // an edge from box 1 touches an edge from box 2.
          // find a point pa on the intersecting edge of box 1
          Vec3f pa(T1);
          FCL_REAL sign;

          for(int j = 0; j < 3; ++j)
            {
              sign = (R1.col(j).dot(normal) > 0) ? 1 : -1;
              pa += R1.col(j) * (A[j] * sign);
            }

          // find a point pb on the intersecting edge of box 2
          Vec3f pb(T2);

          for(int j = 0; j < 3; ++j)
            {
              sign = (R2.col(j).dot(normal) > 0) ? -1 : 1;
              pb += R2.col(j) * (B[j] * sign);
            }

          FCL_REAL alpha, beta;
          Vec3f ua(R1.col((code-7)/3));
          Vec3f ub(R2.col((code-7)%3));

          lineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
          pa += ua * alpha;
          pb += ub * beta;


          // Vec3f pointInWorld((pa + pb) * 0.5);
          // contacts.push_back(ContactPoint(-normal, pointInWorld, -*depth));
          contacts.push_back(ContactPoint(-normal,pb,-*depth));
          *return_code = code;

          return 1;
        }

      // okay, we have a face-something intersection (because the separating
      // axis is perpendicular to a face). define face 'a' to be the reference
      // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
      // the incident face (the closest face of the other box).

      const Matrix3f *Ra, *Rb;
      const Vec3f *pa, *pb, *Sa, *Sb;

      if(code <= 3)
        {
          Ra = &R1;
          Rb = &R2;
          pa = &T1;
          pb = &T2;
          Sa = &A;
          Sb = &B;
        }
      else
        {
          Ra = &R2;
          Rb = &R1;
          pa = &T2;
          pb = &T1;
          Sa = &B;
          Sb = &A;
        }

      // nr = normal vector of reference face dotted with axes of incident box.
      // anr = absolute values of nr.
      Vec3f normal2, nr, anr;
      if(code <= 3)
        normal2 = normal;
      else
        normal2 = -normal;

      nr = Rb->transpose() * normal2;
      anr = nr.cwiseAbs();

      // find the largest compontent of anr: this corresponds to the normal
      // for the indident face. the other axis numbers of the indicent face
      // are stored in a1,a2.
      int lanr, a1, a2;
      if(anr[1] > anr[0])
        {
          if(anr[1] > anr[2])
            {
              a1 = 0;
              lanr = 1;
              a2 = 2;
            }
          else
            {
              a1 = 0;
              a2 = 1;
              lanr = 2;
            }
        }
      else
        {
          if(anr[0] > anr[2])
            {
              lanr = 0;
              a1 = 1;
              a2 = 2;
            }
          else
            {
              a1 = 0;
              a2 = 1;
              lanr = 2;
            }
        }

      // compute center point of incident face, in reference-face coordinates
      Vec3f center;
      if(nr[lanr] < 0)
        center = (*pb) - (*pa) + Rb->col(lanr) * ((*Sb)[lanr]);
      else
        center = (*pb) - (*pa) - Rb->col(lanr) * ((*Sb)[lanr]);

      // find the normal and non-normal axis numbers of the reference box
      int codeN, code1, code2;
      if(code <= 3)
        codeN = code-1;
      else codeN = code-4;

      if(codeN == 0)
        {
          code1 = 1;
          code2 = 2;
        }
      else if(codeN == 1)
        {
          code1 = 0;
          code2 = 2;
        }
      else
        {
          code1 = 0;
          code2 = 1;
        }

      // find the four corners of the incident face, in reference-face coordinates
      FCL_REAL quad[8]; // 2D coordinate of incident face (x,y pairs)
      FCL_REAL c1, c2, m11, m12, m21, m22;
      c1 = Ra->col(code1).dot(center);
      c2 = Ra->col(code2).dot(center);
      // optimize this? - we have already computed this data above, but it is not
      // stored in an easy-to-index format. for now it's quicker just to recompute
      // the four dot products.
      Vec3f tempRac = Ra->col(code1);
      m11 = Rb->col(a1).dot(tempRac);
      m12 = Rb->col(a2).dot(tempRac);
      tempRac = Ra->col(code2);
      m21 = Rb->col(a1).dot(tempRac);
      m22 = Rb->col(a2).dot(tempRac);

      FCL_REAL k1 = m11 * (*Sb)[a1];
      FCL_REAL k2 = m21 * (*Sb)[a1];
      FCL_REAL k3 = m12 * (*Sb)[a2];
      FCL_REAL k4 = m22 * (*Sb)[a2];
      quad[0] = c1 - k1 - k3;
      quad[1] = c2 - k2 - k4;
      quad[2] = c1 - k1 + k3;
      quad[3] = c2 - k2 + k4;
      quad[4] = c1 + k1 + k3;
      quad[5] = c2 + k2 + k4;
      quad[6] = c1 + k1 - k3;
      quad[7] = c2 + k2 - k4;

      // find the size of the reference face
      FCL_REAL rect[2];
      rect[0] = (*Sa)[code1];
      rect[1] = (*Sa)[code2];

      // intersect the incident and reference faces
      FCL_REAL ret[16];
      int n_intersect = intersectRectQuad2(rect, quad, ret);
      if(n_intersect < 1) { *return_code = code; return 0; } // this should never happen

      // convert the intersection points into reference-face coordinates,
      // and compute the contact position and depth for each point. only keep
      // those points that have a positive (penetrating) depth. delete points in
      // the 'ret' array as necessary so that 'point' and 'ret' correspond.
      Vec3f points[8]; // penetrating contact points
      FCL_REAL dep[8]; // depths for those points
      FCL_REAL det1 = 1.f/(m11*m22 - m12*m21);
      m11 *= det1;
      m12 *= det1;
      m21 *= det1;
      m22 *= det1;
      int cnum = 0;	// number of penetrating contact points found
      for(int j = 0; j < n_intersect; ++j)
        {
          FCL_REAL k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
          FCL_REAL k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
          points[cnum] = center + Rb->col(a1) * k1 + Rb->col(a2) * k2;
          dep[cnum] = (*Sa)[codeN] - normal2.dot(points[cnum]);
          if(dep[cnum] >= 0)
            {
              ret[cnum*2] = ret[j*2];
              ret[cnum*2+1] = ret[j*2+1];
              cnum++;
            }
        }
      if(cnum < 1) { *return_code = code; return 0; } // this should never happen

      // we can't generate more contacts than we actually have
      if(maxc > cnum) maxc = cnum;
      if(maxc < 1) maxc = 1;

      if(cnum <= maxc)
        {
          if(code<4)
            {
              // we have less contacts than we need, so we use them all
              for(int j = 0; j < cnum; ++j)
                {
                  Vec3f pointInWorld = points[j] + (*pa);
                  contacts.push_back(ContactPoint(-normal, pointInWorld, -dep[j]));
                }
            }
          else
            {
              // we have less contacts than we need, so we use them all
              for(int j = 0; j < cnum; ++j)
                {
                  Vec3f pointInWorld = points[j] + (*pa) - normal * dep[j];
                  contacts.push_back(ContactPoint(-normal, pointInWorld, -dep[j]));
                }
            }
        }
      else
        {
          // we have more contacts than are wanted, some of them must be culled.
          // find the deepest point, it is always the first contact.
          int i1 = 0;
          FCL_REAL maxdepth = dep[0];
          for(int i = 1; i < cnum; ++i)
            {
              if(dep[i] > maxdepth)
                {
                  maxdepth = dep[i];
                  i1 = i;
                }
            }

          int iret[8];
          cullPoints2(cnum, ret, maxc, i1, iret);

          for(int j = 0; j < maxc; ++j)
            {
              Vec3f posInWorld = points[iret[j]] + (*pa);
              if(code < 4)
                contacts.push_back(ContactPoint(-normal, posInWorld, -dep[iret[j]]));
              else
                contacts.push_back(ContactPoint(-normal, posInWorld - normal * dep[iret[j]], -dep[iret[j]]));
            }
          cnum = maxc;
        }

      *return_code = code;
      return cnum;
    }

    inline bool compareContactPoints
      (const ContactPoint& c1,const ContactPoint& c2)
    {
      return c1.depth < c2.depth;
    } // Accending order, assuming penetration depth is a negative number!

    inline bool boxBoxIntersect(const Box& s1, const Transform3f& tf1,
                                const Box& s2, const Transform3f& tf2,
                                Vec3f* contact_points,
                                FCL_REAL* penetration_depth_, Vec3f* normal_)
    {
      std::vector<ContactPoint> contacts;
      int return_code;
      Vec3f normal;
      FCL_REAL depth;
      /* int cnum = */ boxBox2(s1.halfSide, tf1.getRotation(), tf1.getTranslation(),
                               s2.halfSide, tf2.getRotation(), tf2.getTranslation(),
                               normal, &depth, &return_code,
                               4, contacts);

      if(normal_) *normal_ = normal;
      if(penetration_depth_) *penetration_depth_ = depth;

      if(contact_points)
        {
          if(contacts.size() > 0)
            {
              std::sort(contacts.begin(), contacts.end(), compareContactPoints);
              *contact_points = contacts[0].point;
              if(penetration_depth_) *penetration_depth_ = -contacts[0].depth;
            }
        }

      return return_code != 0;
    }

    template<typename T>
      inline T halfspaceIntersectTolerance()
      {
        return 0;
      }

    template<>
      inline float halfspaceIntersectTolerance()
      {
        return 0.0001f;
      }

    template<>
      inline double halfspaceIntersectTolerance()
      {
        return 0.0000001;
      }

    inline bool sphereHalfspaceIntersect
      (const Sphere& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Halfspace new_s2 = transform(s2, tf2);
      const Vec3f& center = tf1.getTranslation();
      distance = new_s2.signedDistance(center) - s1.radius;
      if(distance <= 0)
      {
        normal = -new_s2.n; // pointing from s1 to s2
        p1 = p2 = center - new_s2.n * s1.radius - (distance * 0.5) * new_s2.n;
        return true;
      }
      else {
        p1 = center - s1.radius * new_s2.n;
        p2 = p1 - distance * new_s2.n;
        return false;
      }
    }

    /// @brief box half space, a, b, c  = +/- edge size
    /// n^T * (R(o + a v1 + b v2 + c v3) + T) <= d
    /// so (R^T n) (a v1 + b v2 + c v3) + n * T <= d
    /// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c
    /// the max value of left side is d - n * T + |(R^T n) (a v1 + b v2 + c v3)|, check that is enough
    inline bool boxHalfspaceIntersect
      (const Box& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2)
    {
      Halfspace new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      Vec3f Q (R.transpose() * new_s2.n);

      FCL_REAL depth = Q.cwiseProduct(s1.halfSide).lpNorm<1>() - new_s2.signedDistance(T);
      return (depth >= 0);
    }


    inline bool boxHalfspaceIntersect
      (const Box& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2, Vec3f& normal)
    {
      Halfspace new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      // Q: plane normal expressed in box frame
      const Vec3f Q (R.transpose() * new_s2.n);
      // A: scalar products of each side with normal
      const Vec3f A (Q.cwiseProduct(s1.halfSide));

      distance = new_s2.signedDistance(T) - A.lpNorm<1>();
      if(distance > 0) {
        p1.noalias() = T + R * (A.array() > 0).select (s1.halfSide, - s1.halfSide);
        p2.noalias() = p1 - distance * new_s2.n;
        return false;
      }

      /// find deepest point
      Vec3f p(T);
      int sign = 0;

      if(std::abs(Q[0] - 1) < halfspaceIntersectTolerance<FCL_REAL>() || std::abs(Q[0] + 1) < halfspaceIntersectTolerance<FCL_REAL>())
        {
          sign = (A[0] > 0) ? -1 : 1;
          p += R.col(0) * (s1.halfSide[0] * sign);
        }
      else if(std::abs(Q[1] - 1) < halfspaceIntersectTolerance<FCL_REAL>() || std::abs(Q[1] + 1) < halfspaceIntersectTolerance<FCL_REAL>())
        {
          sign = (A[1] > 0) ? -1 : 1;
          p += R.col(1) * (s1.halfSide[1] * sign);
        }
      else if(std::abs(Q[2] - 1) < halfspaceIntersectTolerance<FCL_REAL>() || std::abs(Q[2] + 1) < halfspaceIntersectTolerance<FCL_REAL>())
        {
          sign = (A[2] > 0) ? -1 : 1;
          p += R.col(2) * (s1.halfSide[2] * sign);
        }
      else
        {
          p.noalias() += R * (A.array() > 0).select (-s1.halfSide, s1.halfSide);
        }

      /// compute the contact point from the deepest point
      normal = -new_s2.n;
      p1 = p2 = p - new_s2.n * (distance * 0.5);

      return true;
    }

    inline bool capsuleHalfspaceIntersect
      (const Capsule& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Halfspace new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      Vec3f dir_z = R.col(2);

      FCL_REAL cosa = dir_z.dot(new_s2.n);
      if(std::abs(cosa) < halfspaceIntersectTolerance<FCL_REAL>())
        {
          // Capsule parallel to plane
          FCL_REAL signed_dist = new_s2.signedDistance(T);
          distance = signed_dist - s1.radius;
          if(distance > 0) {
            p1 = T - s1.radius * new_s2.n;
            p2 = p1 - distance * new_s2.n;
            return false;
          }

          normal = -new_s2.n;
          p1 = p2 = T + new_s2.n * (-0.5 * distance - s1.radius);
          return true;
        }
      else
        {
          int sign = (cosa > 0) ? -1 : 1;
          // closest capsule vertex to halfspace if no collision,
          // or deeper inside halspace if collision
          Vec3f p = T + dir_z * (s1.halfLength * sign);

          FCL_REAL signed_dist = new_s2.signedDistance(p);
          distance = signed_dist - s1.radius;
          if(distance > 0) {
            p1 = T - s1.radius * new_s2.n;
            p2 = p1 - distance * new_s2.n;
            return false;
          }
          normal = -new_s2.n;
          // deepest point
          Vec3f c = p - new_s2.n * s1.radius;
          p1 = p2 = c - (0.5 * distance) * new_s2.n;
          return true;
        }
    }


    inline bool cylinderHalfspaceIntersect
      (const Cylinder& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Halfspace new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      Vec3f dir_z = R.col(2);
      FCL_REAL cosa = dir_z.dot(new_s2.n);

      if(cosa < halfspaceIntersectTolerance<FCL_REAL>())
        {
          FCL_REAL signed_dist = new_s2.signedDistance(T);
          distance = signed_dist - s1.radius;
          if(distance > 0) {
            // TODO: compute closest points
            p1 = p2 = Vec3f(0, 0, 0);
            return false;
          }

          normal = -new_s2.n;
          p1 = p2 = T - (0.5 * distance + s1.radius) * new_s2.n;
          return true;
        }
      else
        {
          Vec3f C = dir_z * cosa - new_s2.n;
          if(std::abs(cosa + 1) < halfspaceIntersectTolerance<FCL_REAL>() ||
             std::abs(cosa - 1) < halfspaceIntersectTolerance<FCL_REAL>())
            C = Vec3f(0, 0, 0);
          else
            {
              FCL_REAL s = C.norm();
              s = s1.radius / s;
              C *= s;
            }

          int sign = (cosa > 0) ? -1 : 1;
          // deepest point
          Vec3f p = T + dir_z * (s1.halfLength * sign) + C;
          distance = new_s2.signedDistance(p);
          if(distance > 0) {
            // TODO: compute closest points
            p1 = p2 = Vec3f(0, 0, 0);
            return false;
          }
          else
            {
              normal = -new_s2.n;
              p1 = p2 = p - (0.5 * distance) * new_s2.n;
              return true;
            }
        }
    }


    inline bool coneHalfspaceIntersect
      (const Cone& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Halfspace new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      Vec3f dir_z = R.col(2);
      FCL_REAL cosa = dir_z.dot(new_s2.n);

      if(cosa < halfspaceIntersectTolerance<FCL_REAL>())
        {
          FCL_REAL signed_dist = new_s2.signedDistance(T);
          distance = signed_dist - s1.radius;
          if(distance > 0) {
            p1 = p2 = Vec3f (0, 0, 0);
            return false;
          }
          else
            {
              normal = -new_s2.n;
              p1 = p2 = T - dir_z * (s1.halfLength) -
                new_s2.n * (0.5 * distance + s1.radius);
              return true;
            }
        }
      else
        {
          Vec3f C = dir_z * cosa - new_s2.n;
          if(std::abs(cosa + 1) < halfspaceIntersectTolerance<FCL_REAL>() ||
             std::abs(cosa - 1) < halfspaceIntersectTolerance<FCL_REAL>())
            C = Vec3f(0, 0, 0);
          else
            {
              FCL_REAL s = C.norm();
              s = s1.radius / s;
              C *= s;
            }

          Vec3f a1 = T + dir_z * (s1.halfLength);
          Vec3f a2 = T - dir_z * (s1.halfLength) + C;

          FCL_REAL d1 = new_s2.signedDistance(a1);
          FCL_REAL d2 = new_s2.signedDistance(a2);

          if(d1 > 0 && d2 > 0) return false;
          else
            {
              distance = std::min(d1, d2);
              normal = -new_s2.n;
              p1 = p2 = ((d1 < d2) ? a1 : a2) - (0.5 * distance) * new_s2.n;
              return true;
            }
        }
    }

    inline bool convexHalfspaceIntersect
      (const ConvexBase& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2,
       Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal)
    {
      Halfspace new_s2 = transform(s2, tf2);

      Vec3f v;
      FCL_REAL depth = std::numeric_limits<FCL_REAL>::max();

      for(int i = 0; i < s1.num_points; ++i)
        {
          Vec3f p = tf1.transform(s1.points[i]);

          FCL_REAL d = new_s2.signedDistance(p);
          if(d < depth)
            {
              depth = d;
              v = p;
            }
        }

      if(depth <= 0)
        {
          if(contact_points) *contact_points = v - new_s2.n * (0.5 * depth);
          if(penetration_depth) *penetration_depth = depth;
          if(normal) *normal = -new_s2.n;
          return true;
        }
      else
        return false;
    }

    // Intersection between half-space and triangle
    // Half-space is in pose tf1,
    // Triangle is in pose tf2
    // Computes distance and closest points (p1, p2) if no collision,
    //          contact point (p1 = p2), normal and penetration depth (-distance)
    //          if collision
    inline bool halfspaceTriangleIntersect
      (const Halfspace& s1, const Transform3f& tf1, const Vec3f& P1,
       const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2, Vec3f& normal)
    {
      Halfspace new_s1 = transform(s1, tf1);

      Vec3f v = tf2.transform(P1);
      FCL_REAL depth = new_s1.signedDistance(v);

      Vec3f p = tf2.transform(P2);
      FCL_REAL d = new_s1.signedDistance(p);
      if(d < depth)
        {
          depth = d;
          v = p;
        }

      p = tf2.transform(P3);
      d = new_s1.signedDistance(p);
      if(d < depth)
        {
          depth = d;
          v = p;
        }
      // v is the vertex with minimal projection abscissa (depth) on normal to
      //plane,
      distance = depth;
      if(depth <= 0)
        {
          normal = new_s1.n;
          p1 = p2 = v - (0.5 * depth) * new_s1.n;
          return true;
        }
      else
        {
          p1 = v - depth * new_s1.n;
          p2 = v;
          return false;
        }
    }


    /// @brief return whether plane collides with halfspace
    /// if the separation plane of the halfspace is parallel with the plane
    ///     return code 1, if the plane's normal is the same with halfspace's normal and plane is inside halfspace, also return plane in pl
    ///     return code 2, if the plane's normal is oppositie to the halfspace's normal and plane is inside halfspace, also return plane in pl
    ///     plane is outside halfspace, collision-free
    /// if not parallel
    ///     return the intersection ray, return code 3. ray origin is p and direction is d
    inline bool planeHalfspaceIntersect
      (const Plane& s1, const Transform3f& tf1,
       const Halfspace& s2, const Transform3f& tf2, Plane& pl, Vec3f& p,
       Vec3f& d, FCL_REAL& penetration_depth, int& ret)
    {
      Plane new_s1 = transform(s1, tf1);
      Halfspace new_s2 = transform(s2, tf2);

      ret = 0;

      penetration_depth = std::numeric_limits<FCL_REAL>::max();
      Vec3f dir = (new_s1.n).cross(new_s2.n);
      FCL_REAL dir_norm = dir.squaredNorm();
      if(dir_norm < std::numeric_limits<FCL_REAL>::epsilon()) // parallel
        {
          if((new_s1.n).dot(new_s2.n) > 0)
            {
              penetration_depth = new_s2.d - new_s1.d;
              if(penetration_depth < 0)
                return false;
              else
                {
                  ret = 1;
                  pl = new_s1;
                  return true;
                }
            }
          else
            {
              penetration_depth = -(new_s1.d + new_s2.d);
              if(penetration_depth < 0)
                return false;
              else
                {
                  ret = 2;
                  pl = new_s1;
                  return true;
                }
            }
        }

      Vec3f n = new_s2.n * new_s1.d - new_s1.n * new_s2.d;
      Vec3f origin = n.cross(dir);
      origin *= (1.0 / dir_norm);

      p = origin;
      d = dir;
      ret = 3;

      return true;
    }

    ///@ brief return whether two halfspace intersect
    /// if the separation planes of the two halfspaces are parallel
    ///    return code 1, if two halfspaces' normal are same and s1 is in s2, also return s1 in s;
    ///    return code 2, if two halfspaces' normal are same and s2 is in s1, also return s2 in s;
    ///    return code 3, if two halfspaces' normal are opposite and s1 and s2 are into each other;
    ///    collision free, if two halfspaces' are separate;
    /// if the separation planes of the two halfspaces are not parallel, return intersection ray, return code 4. ray origin is p and direction is d
    /// collision free return code 0
    inline bool halfspaceIntersect(const Halfspace& s1, const Transform3f& tf1,
                                   const Halfspace& s2, const Transform3f& tf2,
                                   Vec3f& p, Vec3f& d,
                                   Halfspace& s,
                                   FCL_REAL& penetration_depth, int& ret)
    {
      Halfspace new_s1 = transform(s1, tf1);
      Halfspace new_s2 = transform(s2, tf2);

      ret = 0;

      penetration_depth = std::numeric_limits<FCL_REAL>::max();
      Vec3f dir = (new_s1.n).cross(new_s2.n);
      FCL_REAL dir_norm = dir.squaredNorm();
      if(dir_norm < std::numeric_limits<FCL_REAL>::epsilon()) // parallel
        {
          if((new_s1.n).dot(new_s2.n) > 0)
            {
              if(new_s1.d < new_s2.d) // s1 is inside s2
                {
                  ret = 1;
                  s = new_s1;
                }
              else // s2 is inside s1
                {
                  ret = 2;
                  s = new_s2;
                }
              return true;
            }
          else
            {
              penetration_depth = -(new_s1.d + new_s2.d);
              if(penetration_depth < 0) // not collision
                return false;
              else // in each other
                {
                  ret = 3;
                  return true;
                }
            }
        }

      Vec3f n = new_s2.n * new_s1.d - new_s1.n * new_s2.d;
      Vec3f origin = n.cross(dir);
      origin *= (1.0 / dir_norm);

      p = origin;
      d = dir;
      ret = 4;

      return true;
    }

    /// @param p1 closest (or most penetrating) point on the Halfspace,
    /// @param p2 closest (or most penetrating) point on the shape,
    /// @param normal the halfspace normal.
    /// @return true if the distance is negative (the shape overlaps).
    inline bool halfspaceDistance(const Halfspace& h, const Transform3f& tf1,
                                  const ShapeBase& s, const Transform3f& tf2,
                                  FCL_REAL& dist, Vec3f& p1, Vec3f& p2,
                                  Vec3f& normal)
    {
      Vec3f n_w = tf1.getRotation() * h.n;
      Vec3f n_2 (tf2.getRotation().transpose() * n_w);
      int hint = 0;
      p2 = getSupport(&s, -n_2, true, hint);
      p2 = tf2.transform(p2);

      dist = (p2 - tf1.getTranslation()).dot(n_w) - h.d;
      p1 = p2 - dist * n_w;
      normal = n_w;

      return dist <= 0;
    }

    template<typename T>
      inline T planeIntersectTolerance()
      {
        return 0;
      }

    template<>
      inline double planeIntersectTolerance<double>()
      {
        return 0.0000001;
      }

    template<>
      float inline planeIntersectTolerance<float>()
      {
        return 0.0001f;
      }

    inline bool spherePlaneIntersect
      (const Sphere& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    //  Vec3f& contact_points, FCL_REAL& penetration_depth, Vec3f* normal)
    {
      Plane new_s2 = transform(s2, tf2);

      const Vec3f& center = tf1.getTranslation();
      FCL_REAL signed_dist = new_s2.signedDistance(center);
      distance = std::abs(signed_dist) - s1.radius;
      if(distance <= 0)
      {
        if (signed_dist > 0)
          normal = -new_s2.n;
        else
          normal = new_s2.n;
        p1 = p2 = center - new_s2.n * signed_dist;
        return true;
      }
      else
      {
        if (signed_dist > 0)
        {
          p1 = center - s1.radius * new_s2.n;
          p2 = center - signed_dist * new_s2.n;
        }
        else
        {
          p1 = center + s1.radius * new_s2.n;
          p2 = center + signed_dist * new_s2.n;
        }
        return false;
      }
    }

    /// @brief box half space, a, b, c  = +/- edge size
    /// n^T * (R(o + a v1 + b v2 + c v3) + T) ~ d
    /// so (R^T n) (a v1 + b v2 + c v3) + n * T ~ d
    /// check whether d - n * T - (R^T n) (a v1 + b v2 + c v3) >= 0 for some a, b, c and <=0 for some a, b, c
    /// so need to check whether |d - n * T| <= |(R^T n)(a v1 + b v2 + c v3)|, the reason is as follows:
    /// (R^T n) (a v1 + b v2 + c v3) can get |(R^T n) (a v1 + b v2 + c v3)| for one a, b, c.
    /// if |d - n * T| <= |(R^T n)(a v1 + b v2 + c v3)| then can get both positive and negative value on the right side.
    inline bool boxPlaneIntersect(const Box& s1, const Transform3f& tf1,
                                  const Plane& s2, const Transform3f& tf2,
                                  FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
                                  Vec3f& normal)
    //                            Vec3f* contact_points,
    //                            FCL_REAL* penetration_depth, Vec3f* normal)
    {
      static const FCL_REAL eps (sqrt (std::numeric_limits<FCL_REAL>::epsilon()));
      const Plane new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      const Vec3f Q (R.transpose() * new_s2.n);
      const Vec3f A (Q.cwiseProduct(s1.halfSide));

      const FCL_REAL signed_dist = new_s2.signedDistance(T);
      distance = std::abs(signed_dist) - A.lpNorm<1>();
      if(distance > 0) {
        // Is the box above or below the plane
        const bool positive = signed_dist > 0;
        // Set p1 at the center of the box
        p1 = T;
        for (Vec3f::Index i=0; i<3; ++i) {
          // scalar product between box axis and plane normal
          FCL_REAL alpha ((positive ? 1 : -1 ) * R.col (i).dot (new_s2.n));
          if (alpha > eps) {
            p1 -= R.col (i) * s1.halfSide [i];
          } else if (alpha < -eps) {
            p1 += R.col (i) * s1.halfSide [i];
          }
        }
        p2 = p1 - ( positive ? distance : -distance) * new_s2.n;
        assert (new_s2.distance (p2) < 3 *eps);
        return false;
      }

      // find the deepest point
      Vec3f p = T;

      // when center is on the positive side of the plane, use a, b, c
      // make (R^T n) (a v1 + b v2 + c v3) the minimum
      // otherwise, use a, b, c make (R^T n) (a v1 + b v2 + c v3) the maximum
      int sign = (signed_dist > 0) ? 1 : -1;

      if(std::abs(Q[0] - 1) < planeIntersectTolerance<FCL_REAL>() ||
         std::abs(Q[0] + 1) < planeIntersectTolerance<FCL_REAL>())
        {
          int sign2 = (A[0] > 0) ? -sign : sign;
          p.noalias() += R.col(0) * (s1.halfSide[0] * sign2);
        }
      else if(std::abs(Q[1] - 1) < planeIntersectTolerance<FCL_REAL>() ||
              std::abs(Q[1] + 1) < planeIntersectTolerance<FCL_REAL>())
        {
          int sign2 = (A[1] > 0) ? -sign : sign;
          p.noalias() += R.col(1) * (s1.halfSide[1] * sign2);
        }
      else if(std::abs(Q[2] - 1) < planeIntersectTolerance<FCL_REAL>() ||
              std::abs(Q[2] + 1) < planeIntersectTolerance<FCL_REAL>())
        {
          int sign2 = (A[2] > 0) ? -sign : sign;
          p.noalias() += R.col(2) * (s1.halfSide[2] * sign2);
        }
      else
        {
          Vec3f tmp(sign * R * s1.halfSide);
          p.noalias() += (A.array() > 0).select (- tmp, tmp);
        }

      // compute the contact point by project the deepest point onto the plane
      if (signed_dist > 0) normal = -new_s2.n; else normal = new_s2.n;
      p1 = p2.noalias() = p - new_s2.n * new_s2.signedDistance(p);

      return true;
    }

    /// Taken from book Real Time Collision Detection, from Christer Ericson
    /// @param pb the closest point to the sphere center on the box surface
    /// @param ps when colliding, matches pb, which is inside the sphere.
    ///           when not colliding, the closest point on the sphere
    /// @param normal direction of motion of the box
    /// @return true if the distance is negative (the shape overlaps).
    inline bool boxSphereDistance(const Box   & b, const Transform3f& tfb,
                                  const Sphere& s, const Transform3f& tfs,
                                  FCL_REAL& dist, Vec3f& pb, Vec3f& ps,
                                  Vec3f& normal)
    {
      const Vec3f& os = tfs.getTranslation();
      const Vec3f& ob = tfb.getTranslation();
      const Matrix3f& Rb = tfb.getRotation();

      pb = ob;

      bool outside = false;
      const Vec3f os_in_b_frame (Rb.transpose() * (os - ob));
      int axis = -1;
      FCL_REAL min_d = std::numeric_limits<FCL_REAL>::max();
      for (int i = 0; i < 3; ++i) {
        FCL_REAL facedist;
        if        (os_in_b_frame(i) < - b.halfSide(i)) { // outside
          pb.noalias() -= b.halfSide(i) * Rb.col(i);
          outside = true;
        } else if (os_in_b_frame(i) >   b.halfSide(i)) { // outside
          pb.noalias() += b.halfSide(i) * Rb.col(i);
          outside = true;
        } else {
          pb.noalias() += os_in_b_frame(i) * Rb.col(i);
          if (!outside && (facedist = b.halfSide(i) - std::fabs(os_in_b_frame(i))) < min_d) {
            axis = i;
            min_d = facedist;
          }
        }
      }
      normal.noalias() = pb - os;
      FCL_REAL pdist = normal.norm();
      if (outside) { // pb is on the box
        dist =   pdist - s.radius;
        normal /= - pdist;
      } else { // pb is inside the box
        if (os_in_b_frame(axis) >= 0 ) normal =  Rb.col(axis);
        else                           normal = -Rb.col(axis);
        dist = - min_d - s.radius;
      }
      if (!outside || dist <= 0) {
        ps = pb;
        return true;
      } else {
        ps = os - s.radius * normal;
        return false;
      }
    }

    inline bool capsulePlaneIntersect
      (const Capsule& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Plane new_s2 = transform(s2, tf2);

      // position orientation of capsule
      const Matrix3f& R1 = tf1.getRotation();
      const Vec3f& T1 = tf1.getTranslation();

      Vec3f dir_z = R1.col(2);

      // ends of capsule inner segment
      Vec3f a1 = T1 + dir_z * s1.halfLength;
      Vec3f a2 = T1 - dir_z * s1.halfLength;

      FCL_REAL d1 = new_s2.signedDistance(a1);
      FCL_REAL d2 = new_s2.signedDistance(a2);

      FCL_REAL abs_d1 = std::abs(d1);
      FCL_REAL abs_d2 = std::abs(d2);

      // two end points on different side of the plane
      // the contact point is the intersect of axis with the plane
      // the normal is the direction to avoid intersection
      // the depth is the minimum distance to resolve the collision
      if(d1 * d2 < -planeIntersectTolerance<FCL_REAL>())
      {
        if(abs_d1 < abs_d2)
        {
          distance = -abs_d1 - s1.radius;
          p1 = p2 = a1 * (abs_d2 / (abs_d1 + abs_d2)) +
            a2 * (abs_d1 / (abs_d1 + abs_d2));
          if (d1 < 0) normal = -new_s2.n; else normal = new_s2.n;
        }
        else
        {
          distance = -abs_d2 - s1.radius;
          p1 = p2 = a1 * (abs_d2 / (abs_d1 + abs_d2)) +
            a2 * (abs_d1 / (abs_d1 + abs_d2));
          if (d2 < 0) normal = -new_s2.n; else normal = new_s2.n;
        }
        assert (!p1.hasNaN () && !p2.hasNaN ());
        return true;
      }

      if(abs_d1 > s1.radius && abs_d2 > s1.radius) {
        // Here both capsule ends are on the same side of the plane
        if (d1 > 0) normal = new_s2.n; else normal = -new_s2.n;
        if (abs_d1 < abs_d2) {
          distance = abs_d1 - s1.radius;
          p1 = a1 - s1.radius * normal;
          p2 = p1 - distance * normal;
        } else {
          distance = abs_d2 - s1.radius;
          p1 = a2 - s1.radius * normal;
          p2 = p1 - distance * normal;
        }
        assert (!p1.hasNaN () && !p2.hasNaN ());
        return false;
      }
      else
      {
        // Both capsule ends are on the same side of the plane, but one
        // is closer than the capsule radius, hence collision
        distance = std::min(abs_d1, abs_d2) - s1.radius;

        if(abs_d1 <= s1.radius && abs_d2 <= s1.radius)
        {
          Vec3f c1 = a1 - new_s2.n * d1;
          Vec3f c2 = a2 - new_s2.n * d2;
          p1 = p2 = (c1 + c2) * 0.5;
        }
        else if(abs_d1 <= s1.radius)
        {
          Vec3f c = a1 - new_s2.n * d1;
          p1 = p2 = c;
        }
        else if(abs_d2 <= s1.radius)
        {
          Vec3f c = a2 - new_s2.n * d2;
          p1 = p2 = c;
        } else {
          assert (false);
        }

        if (d1 < 0) normal = new_s2.n; else normal = -new_s2.n;
        assert (!p1.hasNaN () && !p2.hasNaN ());
        return true;
      }
      assert (false);
    }

    /// @brief cylinder-plane intersect
    /// n^T (R (r * cosa * v1 + r * sina * v2 + h * v3) + T) ~ d
    /// need one point to be positive and one to be negative
    /// (n^T * v3) * h + n * T -d + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) ~ 0
    /// (n^T * v3) * h + r * (cosa * (n^T * R * v1) + sina * (n^T * R * v2)) + n * T - d ~ 0
    inline bool cylinderPlaneIntersect
      (const Cylinder& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Plane new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      Vec3f dir_z = R.col(2);
      FCL_REAL cosa = dir_z.dot(new_s2.n);

      if(std::abs(cosa) < planeIntersectTolerance<FCL_REAL>())
      {
        FCL_REAL d = new_s2.signedDistance(T);
        distance = std::abs(d) - s1.radius;
        if(distance > 0) return false;
        else
        {
          if (d < 0) normal = new_s2.n;
          else normal = -new_s2.n;
          p1 = p2 = T - new_s2.n * d;
          return true;
        }
      }
      else
      {
        Vec3f C = dir_z * cosa - new_s2.n;
        if(std::abs(cosa + 1) < planeIntersectTolerance<FCL_REAL>() ||
           std::abs(cosa - 1) < planeIntersectTolerance<FCL_REAL>())
          C = Vec3f(0, 0, 0);
        else
        {
          FCL_REAL s = C.norm();
          s = s1.radius / s;
          C *= s;
        }

        Vec3f a1 = T + dir_z * (s1.halfLength);
        Vec3f a2 = T - dir_z * (s1.halfLength);

        Vec3f c1, c2;
        if(cosa > 0)
        {
          c1 = a1 - C;
          c2 = a2 + C;
        }
        else
        {
          c1 = a1 + C;
          c2 = a2 - C;
        }

        FCL_REAL d1 = new_s2.signedDistance(c1);
        FCL_REAL d2 = new_s2.signedDistance(c2);

        if(d1 * d2 <= 0)
        {
          FCL_REAL abs_d1 = std::abs(d1);
          FCL_REAL abs_d2 = std::abs(d2);

          if(abs_d1 > abs_d2)
          {
            distance = -abs_d2;
            p1 = p2 = c2 - new_s2.n * d2;
            if (d2 < 0) normal = -new_s2.n; else normal = new_s2.n;
          }
          else
          {
            distance = -abs_d1;
            p1 = p2 = c1 - new_s2.n * d1;
            if (d1 < 0) normal = -new_s2.n;
            else normal = new_s2.n;
          }
          return true;
        }
        else
          return false;
      }
    }

    inline bool conePlaneIntersect
      (const Cone& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
       Vec3f& normal)
    {
      Plane new_s2 = transform(s2, tf2);

      const Matrix3f& R = tf1.getRotation();
      const Vec3f& T = tf1.getTranslation();

      Vec3f dir_z = R.col(2);
      FCL_REAL cosa = dir_z.dot(new_s2.n);

      if(std::abs(cosa) < planeIntersectTolerance<FCL_REAL>())
        {
          FCL_REAL d = new_s2.signedDistance(T);
          distance = std::abs(d) - s1.radius;
          if(distance > 0) {
            p1 = p2 = Vec3f (0,0,0);
            return false;
          }
          else
            {
              if (d < 0) normal = new_s2.n; else normal = -new_s2.n;
              p1 = p2 = T - dir_z * (s1.halfLength) +
                dir_z * (- distance / s1.radius * s1.halfLength) - new_s2.n * d;
              return true;
            }
        }
      else
        {
          Vec3f C = dir_z * cosa - new_s2.n;
          if(std::abs(cosa + 1) < planeIntersectTolerance<FCL_REAL>() ||
             std::abs(cosa - 1) < planeIntersectTolerance<FCL_REAL>())
            C = Vec3f(0, 0, 0);
          else
            {
              FCL_REAL s = C.norm();
              s = s1.radius / s;
              C *= s;
            }

          Vec3f c[3];
          c[0] = T + dir_z * (s1.halfLength);
          c[1] = T - dir_z * (s1.halfLength) + C;
          c[2] = T - dir_z * (s1.halfLength) - C;

          FCL_REAL d[3];
          d[0] = new_s2.signedDistance(c[0]);
          d[1] = new_s2.signedDistance(c[1]);
          d[2] = new_s2.signedDistance(c[2]);

          if((d[0] >= 0 && d[1] >= 0 && d[2] >= 0) ||
             (d[0] <= 0 && d[1] <= 0 && d[2] <= 0))
            return false;
          else
            {
              bool positive[3];
              for(std::size_t i = 0; i < 3; ++i)
                positive[i] = (d[i] >= 0);

              int n_positive = 0;
              FCL_REAL d_positive = 0, d_negative = 0;
              for(std::size_t i = 0; i < 3; ++i)
                {
                  if(positive[i])
                    {
                      n_positive++;
                      if(d_positive <= d[i]) d_positive = d[i];
                    }
                  else
                    {
                      if(d_negative <= -d[i]) d_negative = -d[i];
                    }
                }

              distance = -std::min(d_positive, d_negative);
              if (d_positive > d_negative) normal = -new_s2.n;
              else normal = new_s2.n;
              Vec3f p[2];
              Vec3f q;

              FCL_REAL p_d[2];
              FCL_REAL q_d(0);

              if(n_positive == 2)
              {
                for(std::size_t i = 0, j = 0; i < 3; ++i)
                {
                  if(positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
                  else { q = c[i]; q_d = d[i]; }
                }

                Vec3f t1 = (-p[0] * q_d + q * p_d[0]) / (-q_d + p_d[0]);
                Vec3f t2 = (-p[1] * q_d + q * p_d[1]) / (-q_d + p_d[1]);
                p1 = p2 = (t1 + t2) * 0.5;
              }
              else
              {
                for(std::size_t i = 0, j = 0; i < 3; ++i)
                {
                  if(!positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
                  else { q = c[i]; q_d = d[i]; }
                }

                Vec3f t1 = (p[0] * q_d - q * p_d[0]) / (q_d - p_d[0]);
                Vec3f t2 = (p[1] * q_d - q * p_d[1]) / (q_d - p_d[1]);
                p1 = p2 = (t1 + t2) * 0.5;
              }
            }
          return true;
        }
    }

    inline bool convexPlaneIntersect
      (const ConvexBase& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal)
    {
      Plane new_s2 = transform(s2, tf2);

      Vec3f v_min, v_max;
      FCL_REAL d_min = std::numeric_limits<FCL_REAL>::max(), d_max = -std::numeric_limits<FCL_REAL>::max();

      for(int i = 0; i < s1.num_points; ++i)
        {
          Vec3f p = tf1.transform(s1.points[i]);

          FCL_REAL d = new_s2.signedDistance(p);

          if(d < d_min) { d_min = d; v_min = p; }
          if(d > d_max) { d_max = d; v_max = p; }
        }

      if(d_min * d_max > 0) return false;
      else
        {
          if(d_min + d_max > 0)
            {
              if(penetration_depth) *penetration_depth = -d_min;
              if(normal) *normal = -new_s2.n;
              if(contact_points) *contact_points = v_min - new_s2.n * d_min;
            }
          else
            {
              if(penetration_depth) *penetration_depth = d_max;
              if(normal) *normal = new_s2.n;
              if(contact_points) *contact_points = v_max - new_s2.n * d_max;
            }
          return true;
        }
    }



    inline bool planeTriangleIntersect
      (const Plane& s1, const Transform3f& tf1, const Vec3f& P1,
       const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
       FCL_REAL& distance, Vec3f& p1, Vec3f& p2, Vec3f& normal)
    {
      Plane new_s1 = transform(s1, tf1);

      Vec3f c[3];
      c[0] = tf2.transform(P1);
      c[1] = tf2.transform(P2);
      c[2] = tf2.transform(P3);

      FCL_REAL d[3];
      d[0] = new_s1.signedDistance(c[0]);
      d[1] = new_s1.signedDistance(c[1]);
      d[2] = new_s1.signedDistance(c[2]);
      int imin;
      if (d[0] >= 0 && d[1] >= 0 && d[2] >= 0)
        {
          if (d[0] < d[1])
            if (d[0] < d[2]) {
              imin = 0;
            }
            else { // d [2] <= d[0] < d [1]
              imin = 2;
            }
          else { // d[1] <= d[0]
            if (d[2] < d[1]) {
              imin = 2;
            } else { // d[1] <= d[2]
              imin = 1;
            }
          }
          distance = d[imin];
          p2 = c[imin];
          p1 = c[imin] - d[imin] * new_s1.n;
          return false;
        }
      if (d[0] <= 0 && d[1] <= 0 && d[2] <= 0)
        {
          if (d[0] > d[1])
            if (d[0] > d[2]) {
              imin = 0;
            }
            else { // d [2] >= d[0] > d [1]
              imin = 2;
            }
          else { // d[1] >= d[0]
            if (d[2] > d[1]) {
              imin = 2;
            } else { // d[1] >= d[2]
              imin = 1;
            }
          }
          distance = -d[imin];
          p2 = c[imin];
          p1 = c[imin] - d[imin] * new_s1.n;
          return false;
        }
      bool positive[3];
      for(std::size_t i = 0; i < 3; ++i)
        positive[i] = (d[i] > 0);

      int n_positive = 0;
      FCL_REAL d_positive = 0, d_negative = 0;
      for(std::size_t i = 0; i < 3; ++i)
        {
          if(positive[i])
            {
              n_positive++;
              if(d_positive <= d[i]) d_positive = d[i];
            }
          else
            {
              if(d_negative <= -d[i]) d_negative = -d[i];
            }
        }

      distance = -std::min(d_positive, d_negative);
      if (d_positive > d_negative)
        {
          normal = new_s1.n;
        } else
        {
          normal = -new_s1.n;
        }
      Vec3f p[2];
      Vec3f q;

      FCL_REAL p_d[2];
      FCL_REAL q_d(0);

      if(n_positive == 2)
        {
          for(std::size_t i = 0, j = 0; i < 3; ++i)
            {
              if(positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
              else { q = c[i]; q_d = d[i]; }
            }

          Vec3f t1 = (-p[0] * q_d + q * p_d[0]) / (-q_d + p_d[0]);
          Vec3f t2 = (-p[1] * q_d + q * p_d[1]) / (-q_d + p_d[1]);
          p1 = p2 = (t1 + t2) * 0.5;
        }
      else
        {
          for(std::size_t i = 0, j = 0; i < 3; ++i)
            {
              if(!positive[i]) { p[j] = c[i]; p_d[j] = d[i]; j++; }
              else { q = c[i]; q_d = d[i]; }
            }

          Vec3f t1 = (p[0] * q_d - q * p_d[0]) / (q_d - p_d[0]);
          Vec3f t2 = (p[1] * q_d - q * p_d[1]) / (q_d - p_d[1]);
          p1 = p2 = (t1 + t2) * 0.5;
        }
      return true;
    }

    inline bool halfspacePlaneIntersect
      (const Halfspace& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       Plane& pl, Vec3f& p, Vec3f& d, FCL_REAL& penetration_depth, int& ret)
    {
      return planeHalfspaceIntersect(s2, tf2, s1, tf1, pl, p, d, penetration_depth, ret);
    }

    inline bool planeIntersect
      (const Plane& s1, const Transform3f& tf1,
       const Plane& s2, const Transform3f& tf2,
       Vec3f* /*contact_points*/, FCL_REAL* /*penetration_depth*/, Vec3f* /*normal*/)
    {
      Plane new_s1 = transform(s1, tf1);
      Plane new_s2 = transform(s2, tf2);

      FCL_REAL a = (new_s1.n).dot(new_s2.n);
      if(a == 1 && new_s1.d != new_s2.d)
        return false;
      if(a == -1 && new_s1.d != -new_s2.d)
        return false;

      return true;
    }

    /// See the prototype below
    inline FCL_REAL computePenetration
      (const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
       const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
       Vec3f& normal)
    {
      Vec3f u ((P2-P1).cross (P3-P1));
      normal = u.normalized ();
      FCL_REAL depth1 ((P1-Q1).dot (normal));
      FCL_REAL depth2 ((P1-Q2).dot (normal));
      FCL_REAL depth3 ((P1-Q3).dot (normal));
      return std::max (depth1, std::max (depth2, depth3));
    }

    // Compute penetration distance and normal of two triangles in collision
    // Normal is normal of triangle 1 (P1, P2, P3), penetration depth is the
    // minimal distance (Q1, Q2, Q3) should be translated along the normal so
    // that the triangles are collision free.
    //
    // Note that we compute here an upper bound of the penetration distance,
    // not the exact value.
    inline FCL_REAL computePenetration
      (const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
       const Vec3f& Q1, const Vec3f& Q2, const Vec3f& Q3,
       const Transform3f& tf1, const Transform3f& tf2, Vec3f& normal)
    {
      Vec3f globalP1 (tf1.transform (P1));
      Vec3f globalP2 (tf1.transform (P2));
      Vec3f globalP3 (tf1.transform (P3));
      Vec3f globalQ1 (tf2.transform (Q1));
      Vec3f globalQ2 (tf2.transform (Q2));
      Vec3f globalQ3 (tf2.transform (Q3));
      return computePenetration (globalP1, globalP2, globalP3,
          globalQ1, globalQ2, globalQ3, normal);
    }
  } // details
} // namespace fcl
} // namespace hpp

#endif // HPP_FCL_SRC_NARROWPHASE_DETAILS_H
