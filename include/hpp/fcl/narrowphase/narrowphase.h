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

#ifndef HPP_FCL_NARROWPHASE_H
#define HPP_FCL_NARROWPHASE_H

#include <hpp/fcl/narrowphase/gjk.h>

namespace hpp
{
namespace fcl
{

  /// @brief collision and distance solver based on GJK algorithm implemented in fcl (rewritten the code from the GJK in bullet)
  struct HPP_FCL_DLLAPI GJKSolver
  {
    /// @brief intersection checking between two shapes
    template<typename S1, typename S2>
    bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                        const S2& s2, const Transform3f& tf2,
                        FCL_REAL& distance_lower_bound,
                        bool enable_penetration,
                        Vec3f* contact_points, Vec3f* normal) const
    {
      Vec3f guess(1, 0, 0);
      support_func_guess_t support_hint;
      if(enable_cached_guess) {
        guess = cached_guess;
        support_hint = support_func_cached_guess;
      } else
        support_hint.setZero();
    
      details::MinkowskiDiff shape;
      shape.set (&s1, &s2, tf1, tf2);
  
      details::GJK gjk((unsigned int )gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, guess, support_hint);
      if(enable_cached_guess) {
        cached_guess = gjk.getGuessFromSimplex();
        support_func_cached_guess = gjk.support_hint;
      }
    
      Vec3f w0, w1;
      switch(gjk_status) {
        case details::GJK::Inside:
          if (!enable_penetration && contact_points == NULL && normal == NULL)
            return true;
          if (gjk.hasPenetrationInformation(shape)) {
            gjk.getClosestPoints (shape, w0, w1);
            distance_lower_bound = gjk.distance;
            if(normal) *normal = tf1.getRotation() * (w0 - w1).normalized();
            if(contact_points) *contact_points = tf1.transform((w0 + w1) / 2);
            return true;
          } else {
            details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            if(epa_status & details::EPA::Valid
                || epa_status == details::EPA::OutOfFaces    // Warnings
                || epa_status == details::EPA::OutOfVertices // Warnings
                )
            {
              epa.getClosestPoints (shape, w0, w1);
              distance_lower_bound = -epa.depth;
              if(normal) *normal = tf1.getRotation() * epa.normal;
              if(contact_points) *contact_points = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
              return true;
            }
            distance_lower_bound = -(std::numeric_limits<FCL_REAL>::max)();
            // EPA failed but we know there is a collision so we should
            return true;
          }
          break;
        case details::GJK::Valid:
          distance_lower_bound = gjk.distance;
          break;
        default:
          ;
      }

      return false;
    }

    //// @brief intersection checking between one shape and a triangle with transformation
    /// @return true if the shape are colliding.
    template<typename S>
    bool shapeTriangleInteraction
    (const S& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const
    {
      bool col;
      // Express everything in frame 1
      const Transform3f tf_1M2 (tf1.inverseTimes(tf2));
      TriangleP tri(
          tf_1M2.transform (P1),
          tf_1M2.transform (P2),
          tf_1M2.transform (P3));

      Vec3f guess(1, 0, 0);
      support_func_guess_t support_hint;
      if(enable_cached_guess) {
        guess = cached_guess;
        support_hint = support_func_cached_guess;
      } else
        support_hint.setZero();

      details::MinkowskiDiff shape;
      shape.set (&s, &tri);
  
      details::GJK gjk((unsigned int )gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, guess, support_hint);
      if(enable_cached_guess) {
        cached_guess = gjk.getGuessFromSimplex();
        support_func_cached_guess = gjk.support_hint;
      }

      Vec3f w0, w1;
      switch(gjk_status) {
        case details::GJK::Inside:
          col = true;
          if (gjk.hasPenetrationInformation(shape)) {
            gjk.getClosestPoints (shape, w0, w1);
            distance = gjk.distance;
            normal = tf1.getRotation() * (w0 - w1).normalized();
            p1 = p2 = tf1.transform((w0 + w1) / 2);
          } else {
            details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            if(epa_status & details::EPA::Valid
                || epa_status == details::EPA::OutOfFaces    // Warnings
                || epa_status == details::EPA::OutOfVertices // Warnings
                )
            {
              epa.getClosestPoints (shape, w0, w1);
              distance = -epa.depth;
              normal = tf1.getRotation() * epa.normal;
              p1 = p2 = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
              assert (distance <= 1e-6);
            } else {
              distance = -(std::numeric_limits<FCL_REAL>::max)();
              gjk.getClosestPoints (shape, w0, w1);
              p1 = p2 = tf1.transform (w0);
            }
          }
          break;
        case details::GJK::Valid:
        case details::GJK::Failed:
          col = false;

          gjk.getClosestPoints (shape, p1, p2);
          // TODO On degenerated case, the closest point may be wrong
          // (i.e. an object face normal is colinear to gjk.ray
          // assert (distance == (w0 - w1).norm());
          distance = gjk.distance;

          p1 = tf1.transform (p1);
          p2 = tf1.transform (p2);
          assert (distance > 0);
          break;
        default:
          assert (false && "should not reach type part.");
          return true;
        }
      return col;
    }

    /// @brief distance computation between two shapes
    template<typename S1, typename S2>
    bool shapeDistance(const S1& s1, const Transform3f& tf1,
                       const S2& s2, const Transform3f& tf2,
                       FCL_REAL& distance, Vec3f& p1, Vec3f& p2,
                       Vec3f& normal) const
    {
#ifndef NDEBUG
      FCL_REAL eps (sqrt(std::numeric_limits<FCL_REAL>::epsilon()));
#endif
      Vec3f guess(1, 0, 0);
      support_func_guess_t support_hint;
      if(enable_cached_guess) {
        guess = cached_guess;
        support_hint = support_func_cached_guess;
      } else
        support_hint.setZero();

      details::MinkowskiDiff shape;
      shape.set (&s1, &s2, tf1, tf2);

      details::GJK gjk((unsigned int) gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, guess, support_hint);
      if(enable_cached_guess) {
        cached_guess = gjk.getGuessFromSimplex();
        support_func_cached_guess = gjk.support_hint;
      }

      if(gjk_status == details::GJK::Failed)
      {
        // TODO: understand why GJK fails between cylinder and box
        assert (distance * distance < sqrt (eps));
        Vec3f w0, w1;
        gjk.getClosestPoints (shape, w0, w1);
        distance = 0;
        p1 = tf1.transform (w0);
        p2 = tf1.transform (w1);
        normal = Vec3f (0,0,0);
        return false;
      }
      else if(gjk_status == details::GJK::Valid)
        {
          gjk.getClosestPoints (shape, p1, p2);
          // TODO On degenerated case, the closest point may be wrong
          // (i.e. an object face normal is colinear to gjk.ray
          // assert (distance == (w0 - w1).norm());
          distance = gjk.distance;

          normal = (tf1.getRotation() * gjk.ray).normalized();
          p1 = tf1.transform (p1);
          p2 = tf1.transform (p2);
          return true;
        }
      else
        {
          assert (gjk_status == details::GJK::Inside);
          if (gjk.hasPenetrationInformation (shape)) {
            gjk.getClosestPoints (shape, p1, p2);
            distance = gjk.distance;
            // Return contact points in case of collision
            //p1 = tf1.transform (p1);
            //p2 = tf1.transform (p2);
            normal = (tf1.getRotation() * (p1 - p2)).normalized();
            p1 = tf1.transform(p1);
            p2 = tf1.transform(p2);
          } else {
            details::EPA epa(epa_max_face_num, epa_max_vertex_num,
                             epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            if(epa_status & details::EPA::Valid
                || epa_status == details::EPA::OutOfFaces    // Warnings
                || epa_status == details::EPA::OutOfVertices // Warnings
                )
            {
              Vec3f w0, w1;
              epa.getClosestPoints (shape, w0, w1);
              assert (epa.depth >= -eps);
              distance = (std::min) (0., -epa.depth);
              normal = tf1.getRotation() * epa.normal;
              p1 = tf1.transform(w0);
              p2 = tf1.transform(w1);
              return false;
            }
            distance = -(std::numeric_limits<FCL_REAL>::max)();
            gjk.getClosestPoints (shape, p1, p2);
            p1 = tf1.transform(p1);
            p2 = tf1.transform(p2);
          }
          return false;
        }
    }

    /// @brief default setting for GJK algorithm
    GJKSolver()
    {
      gjk_max_iterations = 128;
      gjk_tolerance = 1e-6;
      epa_max_face_num = 128;
      epa_max_vertex_num = 64;
      epa_max_iterations = 255;
      epa_tolerance = 1e-6;
      enable_cached_guess = false;
      cached_guess = Vec3f(1, 0, 0);
      support_func_cached_guess = support_func_guess_t::Zero();
    }

    void enableCachedGuess(bool if_enable) const
    {
      enable_cached_guess = if_enable;
    }

    void setCachedGuess(const Vec3f& guess) const
    {
      cached_guess = guess;
    }

    Vec3f getCachedGuess() const
    {
      return cached_guess;
    }

    /// @brief maximum number of simplex face used in EPA algorithm
    unsigned int epa_max_face_num;

    /// @brief maximum number of simplex vertex used in EPA algorithm
    unsigned int epa_max_vertex_num;

    /// @brief maximum number of iterations used for EPA iterations
    unsigned int epa_max_iterations;

    /// @brief the threshold used in EPA to stop iteration
    FCL_REAL epa_tolerance;

    /// @brief the threshold used in GJK to stop iteration
    FCL_REAL gjk_tolerance;

    /// @brief maximum number of iterations used for GJK iterations
    FCL_REAL gjk_max_iterations;

    /// @brief Whether smart guess can be provided
    mutable bool enable_cached_guess;

    /// @brief smart guess
    mutable Vec3f cached_guess;

    /// @brief smart guess for the support function
    mutable support_func_guess_t support_func_cached_guess;
  };

  template<>
  HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction
  (const Sphere& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
   const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
   Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  template<>
  HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction
  (const Halfspace& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
   const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
   Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  template<>
  HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction
  (const Plane& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
   const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
   Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

#define SHAPE_INTERSECT_SPECIALIZATION_BASE(S1,S2) \
template<> \
HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<S1, S2> \
 (const S1 &s1, const Transform3f& tf1, \
  const S2 &s2, const Transform3f& tf2, \
  FCL_REAL& distance_lower_bound, \
  bool, \
  Vec3f* contact_points, Vec3f* normal) const

#define SHAPE_INTERSECT_SPECIALIZATION(S1,S2) \
  SHAPE_INTERSECT_SPECIALIZATION_BASE(S1,S2); \
  SHAPE_INTERSECT_SPECIALIZATION_BASE(S2,S1)

  SHAPE_INTERSECT_SPECIALIZATION(Sphere,Capsule);
  SHAPE_INTERSECT_SPECIALIZATION_BASE(Sphere,Sphere);
  SHAPE_INTERSECT_SPECIALIZATION(Sphere,Box);
  SHAPE_INTERSECT_SPECIALIZATION(Sphere,Halfspace);
  SHAPE_INTERSECT_SPECIALIZATION(Sphere,Plane);

  SHAPE_INTERSECT_SPECIALIZATION(Halfspace,Box);
  SHAPE_INTERSECT_SPECIALIZATION(Halfspace,Capsule);
  SHAPE_INTERSECT_SPECIALIZATION(Halfspace,Cylinder);
  SHAPE_INTERSECT_SPECIALIZATION(Halfspace,Cone);
  SHAPE_INTERSECT_SPECIALIZATION(Halfspace,Plane);

  SHAPE_INTERSECT_SPECIALIZATION(Plane,Box);
  SHAPE_INTERSECT_SPECIALIZATION(Plane,Capsule);
  SHAPE_INTERSECT_SPECIALIZATION(Plane,Cylinder);
  SHAPE_INTERSECT_SPECIALIZATION(Plane,Cone);

#undef SHAPE_INTERSECT_SPECIALIZATION
#undef SHAPE_INTERSECT_SPECIALIZATION_BASE

#define SHAPE_DISTANCE_SPECIALIZATION_BASE(S1,S2) \
template<> \
HPP_FCL_DLLAPI bool GJKSolver::shapeDistance<S1, S2> \
 (const S1& s1, const Transform3f& tf1, \
  const S2& s2, const Transform3f& tf2, \
  FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const

#define SHAPE_DISTANCE_SPECIALIZATION(S1,S2) \
  SHAPE_DISTANCE_SPECIALIZATION_BASE(S1,S2); \
  SHAPE_DISTANCE_SPECIALIZATION_BASE(S2,S1)

  SHAPE_DISTANCE_SPECIALIZATION(Sphere,Capsule);
  SHAPE_DISTANCE_SPECIALIZATION(Sphere,Box);
  SHAPE_DISTANCE_SPECIALIZATION(Sphere,Cylinder);
  SHAPE_DISTANCE_SPECIALIZATION_BASE(Sphere,Sphere);
  SHAPE_DISTANCE_SPECIALIZATION_BASE(Capsule,Capsule);
  SHAPE_DISTANCE_SPECIALIZATION_BASE(TriangleP,TriangleP);

#undef SHAPE_DISTANCE_SPECIALIZATION
#undef SHAPE_DISTANCE_SPECIALIZATION_BASE

#if !(__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc99-extensions"
#endif
  /// \name Shape intersection specializations
  /// \{

// param doc is the doxygen detailled description (should be enclosed in /** */
// and contain no dot for some obscure reasons).
#define HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape1,Shape2,doc)                     \
  /** @brief Fast implementation for Shape1-Shape2 collision. */               \
  doc                                                                          \
  template<>                                                                   \
    HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<Shape1, Shape2>                             \
    (const Shape1& s1, const Transform3f& tf1,                                 \
     const Shape2& s2, const Transform3f& tf2,                                 \
     FCL_REAL& distance_lower_bound, bool enable_penetration,                  \
     Vec3f* contact_points, Vec3f* normal) const
#define HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Shape,doc)                        \
  HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape,Shape,doc)
#define HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Shape1,Shape2,doc)                \
  HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape1,Shape2,doc);                          \
  HPP_FCL_DECLARE_SHAPE_INTERSECT(Shape2,Shape1,doc)

  HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Sphere,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Sphere, Capsule,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Sphere, Halfspace,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Sphere, Plane,);

  template<>
  HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<Box, Sphere>
  (const Box& s1, const Transform3f& tf1,
   const Sphere& s2, const Transform3f& tf2,
   FCL_REAL& distance_lower_bound, bool enable_penetration,
   Vec3f* contact_points, Vec3f* normal) const;

#ifdef IS_DOXYGEN // for doxygen only
  /** \todo currently disabled and to re-enable it, API of function
   *  \ref obbDisjointAndLowerBoundDistance should be modified.
   *  */
  template<>
  HPP_FCL_DLLAPI bool GJKSolver::shapeIntersect<Box, Box>
    (const Box& s1, const Transform3f& tf1,
     const Box& s2, const Transform3f& tf2,
     FCL_REAL& distance_lower_bound, bool enable_penetration,
     Vec3f* contact_points, Vec3f* normal) const;
#endif
  //HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Box,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Box, Halfspace,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Box, Plane,);

  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Capsule, Halfspace,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Capsule, Plane,);

  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cylinder, Halfspace,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cylinder, Plane,);

  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cone, Halfspace,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Cone, Plane,);

  HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Halfspace,);

  HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF(Plane,);
  HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR(Plane, Halfspace,);

#undef HPP_FCL_DECLARE_SHAPE_INTERSECT
#undef HPP_FCL_DECLARE_SHAPE_INTERSECT_SELF
#undef HPP_FCL_DECLARE_SHAPE_INTERSECT_PAIR

  /// \}

  /// \name Shape triangle interaction specializations
  /// \{

// param doc is the doxygen detailled description (should be enclosed in /** */
// and contain no dot for some obscure reasons).
#define HPP_FCL_DECLARE_SHAPE_TRIANGLE(Shape,doc)                              \
  /** @brief Fast implementation for Shape-Triangle interaction. */            \
  doc                                                                          \
  template<> HPP_FCL_DLLAPI bool GJKSolver::shapeTriangleInteraction<Shape>                   \
    (const Shape& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2, \
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,              \
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const

  HPP_FCL_DECLARE_SHAPE_TRIANGLE(Sphere,);
  HPP_FCL_DECLARE_SHAPE_TRIANGLE(Halfspace,);
  HPP_FCL_DECLARE_SHAPE_TRIANGLE(Plane,);

#undef HPP_FCL_DECLARE_SHAPE_TRIANGLE

  /// \}

  /// \name Shape distance specializations
  /// \{

// param doc is the doxygen detailled description (should be enclosed in /** */
// and contain no dot for some obscure reasons).
#define HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape1,Shape2,doc)                      \
  /** @brief Fast implementation for Shape1-Shape2 distance. */                \
  doc                                                                          \
  template<>                                                                   \
    bool HPP_FCL_DLLAPI GJKSolver::shapeDistance<Shape1, Shape2>                              \
    (const Shape1& s1, const Transform3f& tf1,                                 \
     const Shape2& s2, const Transform3f& tf2,                                 \
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const
#define HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(Shape,doc)                         \
  HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape,Shape,doc)
#define HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Shape1,Shape2,doc)                 \
  HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape1,Shape2,doc);                           \
  HPP_FCL_DECLARE_SHAPE_DISTANCE(Shape2,Shape1,doc)

  HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Sphere, Box,);
  HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Sphere, Capsule,);
  HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR(Sphere, Cylinder,);
  HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(Sphere,);

  HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(Capsule,
      /** Closest points are based on two line-segments. */
      );

  HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF(TriangleP,
      /** Do not run EPA algorithm to compute penetration depth. Use a dedicated method. */
      );

#undef HPP_FCL_DECLARE_SHAPE_DISTANCE
#undef HPP_FCL_DECLARE_SHAPE_DISTANCE_SELF
#undef HPP_FCL_DECLARE_SHAPE_DISTANCE_PAIR

  /// \}
#if !(__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#pragma GCC diagnostic pop
#endif
}

} // namespace hpp

#endif
