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
  struct GJKSolver_indep
  {
    /// @brief intersection checking between two shapes
    template<typename S1, typename S2>
      bool shapeIntersect(const S1& s1, const Transform3f& tf1,
                          const S2& s2, const Transform3f& tf2,
                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const
    {
      Vec3f guess(1, 0, 0);
      if(enable_cached_guess) guess = cached_guess;
    
      details::MinkowskiDiff shape;
      shape.shapes[0] = &s1;
      shape.shapes[1] = &s2;
      shape.toshape1 = tf2.getRotation().transpose() * tf1.getRotation();
      shape.toshape0 = tf1.inverseTimes(tf2);
  
      details::GJK gjk((unsigned int )gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
      if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();
    
      switch(gjk_status)
        {
        case details::GJK::Inside:
          {
            details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            if(epa_status != details::EPA::Failed)
              {
                Vec3f w0 (Vec3f::Zero());
                for(size_t i = 0; i < epa.result.rank; ++i)
                  {
                    w0 += shape.support(epa.result.vertex[i]->d, 0) *
                      epa.result.coefficient[i];
                  }
                if(penetration_depth) *penetration_depth = -epa.depth;
                if(normal) *normal = tf2.getRotation() * epa.normal;
                if(contact_points) *contact_points = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
                return true;
              }
            else return false;
          }
          break;
        default:
          ;
        }

      return false;
    }

    //// @brief intersection checking between one shape and a triangle with transformation
    /// \return true if the shape are colliding.
    template<typename S>
    bool shapeTriangleInteraction
    (const S& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const
    {
      bool col;
      TriangleP tri(P1, P2, P3);

      Vec3f guess(1, 0, 0);
      if(enable_cached_guess) guess = cached_guess;

      details::MinkowskiDiff shape;
      shape.shapes[0] = &s;
      shape.shapes[1] = &tri;
      shape.toshape1 = tf2.getRotation().transpose() * tf1.getRotation();
      shape.toshape0 = tf1.inverseTimes(tf2);
  
      details::GJK gjk((unsigned int )gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
      if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

      switch(gjk_status)
        {
        case details::GJK::Inside:
          {
            col = true;
            details::EPA epa(epa_max_face_num, epa_max_vertex_num, epa_max_iterations, epa_tolerance);
            details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
            assert (epa_status != details::EPA::Failed); (void) epa_status;
            Vec3f w0 (Vec3f::Zero());
            for(size_t i = 0; i < epa.result.rank; ++i)
              {
                w0 += shape.support(epa.result.vertex[i]->d, 0) *
                  epa.result.coefficient[i];
              }
            distance = -epa.depth;
            normal = -epa.normal;
            p1 = p2 = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
            assert (distance <= 1e-6);
            break;
          }
        case details::GJK::Valid:
        case details::GJK::Failed:
          {
            col = false;
            Vec3f w0 (Vec3f::Zero()), w1 (Vec3f::Zero());
            for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
              {
                FCL_REAL p = gjk.getSimplex()->coefficient[i];
                w0 += shape.support(gjk.getSimplex()->vertex[i]->d, 0) * p;
                w1 += shape.support(-gjk.getSimplex()->vertex[i]->d, 1) * p;
              }
            distance = (w0 - w1).norm();
            p1 = tf1.transform (w0);
            p2 = tf1.transform (w1);
            assert (distance > 0);
          }
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
      bool compute_normal (true);
      Vec3f guess(1, 0, 0);
      if(enable_cached_guess) guess = cached_guess;

      details::MinkowskiDiff shape;
      shape.shapes[0] = &s1;
      shape.shapes[1] = &s2;
      shape.toshape1 = tf2.getRotation().transpose() * tf1.getRotation();
      shape.toshape0 = tf1.inverseTimes(tf2);

      details::GJK gjk((unsigned int) gjk_max_iterations, gjk_tolerance);
      details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
      if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

      if(gjk_status == details::GJK::Failed)
      {
        // TODO: understand why GJK fails between cylinder and box
        assert (distance * distance < sqrt (eps));
        Vec3f w0 (Vec3f::Zero()), w1 (Vec3f::Zero());
        for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
        {
          FCL_REAL p = gjk.getSimplex()->coefficient[i];
          w0 += shape.support(gjk.getSimplex()->vertex[i]->d, 0) * p;
          w1 += shape.support(-gjk.getSimplex()->vertex[i]->d, 1) * p;
        }
        distance = 0;
        p1 = p2 = tf1.transform (.5* (w0 + w1));
        normal = Vec3f (0,0,0);
        return false;
      }
      else if(gjk_status == details::GJK::Valid)
        {
          Vec3f w0 (Vec3f::Zero()), w1 (Vec3f::Zero());
          for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
            {
              FCL_REAL p = gjk.getSimplex()->coefficient[i];
              w0 += shape.support(gjk.getSimplex()->vertex[i]->d, 0) * p;
              w1 += shape.support(-gjk.getSimplex()->vertex[i]->d, 1) * p;
            }

          distance = (w0 - w1).norm();

          p1 = tf1.transform (w0);
          p2 = tf1.transform (w1);
          return true;
        }
      else
        {
          assert (gjk_status == details::GJK::Inside);
          if (compute_normal)
            {
              details::EPA epa(epa_max_face_num, epa_max_vertex_num,
                               epa_max_iterations, epa_tolerance);
              details::EPA::Status epa_status = epa.evaluate(gjk, -guess);
              if(epa_status != details::EPA::Failed)
                {
                  Vec3f w0 (Vec3f::Zero());
                  for(size_t i = 0; i < epa.result.rank; ++i)
                    {
                      w0 += shape.support(epa.result.vertex[i]->d, 0) *
                        epa.result.coefficient[i];
                    }
                  assert (epa.depth >= -eps);
                  distance = std::min (0., -epa.depth);
                  normal = tf2.getRotation() * epa.normal;
                  p1 = p2 = tf1.transform(w0 - epa.normal*(epa.depth *0.5));
                }
            }
          else
            {
              Vec3f w0 (Vec3f::Zero()), w1 (Vec3f::Zero());
              for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
                {
                  FCL_REAL p = gjk.getSimplex()->coefficient[i];
                  w0 += shape.support(gjk.getSimplex()->vertex[i]->d, 0) * p;
                  w1 += shape.support(-gjk.getSimplex()->vertex[i]->d, 1) * p;
                }
              distance = 0;

              p1 = tf1.transform (w0);
              p2 = tf1.transform (w1);
            }
          return false;
        }
    }

    /// @brief default setting for GJK algorithm
    GJKSolver_indep()
    {
      gjk_max_iterations = 128;
      gjk_tolerance = 1e-6;
      epa_max_face_num = 128;
      epa_max_vertex_num = 64;
      epa_max_iterations = 255;
      epa_tolerance = 1e-6;
      enable_cached_guess = false;
      cached_guess = Vec3f(1, 0, 0);
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
  };

  /// @brief Fast implementation for sphere-capsule collision
  template<>
    bool GJKSolver_indep::shapeIntersect<Sphere, Capsule>(const Sphere& s1, const Transform3f& tf1,
                                                          const Capsule& s2, const Transform3f& tf2,
                                                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Capsule, Sphere>(const Capsule &s1, const Transform3f& tf1,
                                                          const Sphere &s2, const Transform3f& tf2,
                                                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  /// @brief Fast implementation for sphere-sphere collision
  template<>
    bool GJKSolver_indep::shapeIntersect<Sphere, Sphere>(const Sphere& s1, const Transform3f& tf1,
                                                         const Sphere& s2, const Transform3f& tf2,
                                                         Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  /// @brief Fast implementation for box-box collision
  template<>
    bool GJKSolver_indep::shapeIntersect<Box, Box>(const Box& s1, const Transform3f& tf1,
                                                   const Box& s2, const Transform3f& tf2,
                                                   Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Sphere, Halfspace>(const Sphere& s1, const Transform3f& tf1,
                                                            const Halfspace& s2, const Transform3f& tf2,
                                                            Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Sphere>(const Halfspace& s1, const Transform3f& tf1,
                                                            const Sphere& s2, const Transform3f& tf2,
                                                            Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Box, Halfspace>(const Box& s1, const Transform3f& tf1,
                                                         const Halfspace& s2, const Transform3f& tf2,
                                                         Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Box>(const Halfspace& s1, const Transform3f& tf1,
                                                         const Box& s2, const Transform3f& tf2,
                                                         Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Capsule, Halfspace>(const Capsule& s1, const Transform3f& tf1,
                                                             const Halfspace& s2, const Transform3f& tf2,
                                                             Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Capsule>(const Halfspace& s1, const Transform3f& tf1,
                                                             const Capsule& s2, const Transform3f& tf2,
                                                             Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Cylinder, Halfspace>(const Cylinder& s1, const Transform3f& tf1,
                                                              const Halfspace& s2, const Transform3f& tf2,
                                                              Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Cylinder>(const Halfspace& s1, const Transform3f& tf1,
                                                              const Cylinder& s2, const Transform3f& tf2,
                                                              Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Cone, Halfspace>(const Cone& s1, const Transform3f& tf1,
                                                          const Halfspace& s2, const Transform3f& tf2,
                                                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Cone>(const Halfspace& s1, const Transform3f& tf1,
                                                          const Cone& s2, const Transform3f& tf2,
                                                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Halfspace>(const Halfspace& s1, const Transform3f& tf1,
                                                               const Halfspace& s2, const Transform3f& tf2,
                                                               Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Halfspace>(const Plane& s1, const Transform3f& tf1,
                                                           const Halfspace& s2, const Transform3f& tf2,
                                                           Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Halfspace, Plane>(const Halfspace& s1, const Transform3f& tf1,
                                                           const Plane& s2, const Transform3f& tf2,
                                                           Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Sphere, Plane>(const Sphere& s1, const Transform3f& tf1,
                                                        const Plane& s2, const Transform3f& tf2,
                                                        Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Sphere>(const Plane& s1, const Transform3f& tf1,
                                                        const Sphere& s2, const Transform3f& tf2,
                                                        Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Box, Plane>(const Box& s1, const Transform3f& tf1,
                                                     const Plane& s2, const Transform3f& tf2,
                                                     Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Box>(const Plane& s1, const Transform3f& tf1,
                                                     const Box& s2, const Transform3f& tf2,
                                                     Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Capsule, Plane>(const Capsule& s1, const Transform3f& tf1,
                                                         const Plane& s2, const Transform3f& tf2,
                                                         Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Capsule>(const Plane& s1, const Transform3f& tf1,
                                                         const Capsule& s2, const Transform3f& tf2,
                                                         Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Cylinder, Plane>(const Cylinder& s1, const Transform3f& tf1,
                                                          const Plane& s2, const Transform3f& tf2,
                                                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Cylinder>(const Plane& s1, const Transform3f& tf1,
                                                          const Cylinder& s2, const Transform3f& tf2,
                                                          Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Cone, Plane>(const Cone& s1, const Transform3f& tf1,
                                                      const Plane& s2, const Transform3f& tf2,
                                                      Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Cone>(const Plane& s1, const Transform3f& tf1,
                                                      const Cone& s2, const Transform3f& tf2,
                                                      Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  template<>
    bool GJKSolver_indep::shapeIntersect<Plane, Plane>(const Plane& s1, const Transform3f& tf1,
                                                       const Plane& s2, const Transform3f& tf2,
                                                       Vec3f* contact_points, FCL_REAL* penetration_depth, Vec3f* normal) const;

  /// @brief Fast implementation for sphere-triangle collision
  template<>
    bool GJKSolver_indep::shapeTriangleInteraction
    (const Sphere& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  template<>
    bool GJKSolver_indep::shapeTriangleInteraction
    (const Halfspace& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  template<>
    bool GJKSolver_indep::shapeTriangleInteraction
    (const Plane& s, const Transform3f& tf1, const Vec3f& P1, const Vec3f& P2,
     const Vec3f& P3, const Transform3f& tf2, FCL_REAL& distance,
     Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  /// @brief Fast implementation for sphere-capsule distance
  template<>
    bool GJKSolver_indep::shapeDistance<Sphere, Capsule>
    (const Sphere& s1, const Transform3f& tf1,
     const Capsule& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  template<>
    bool GJKSolver_indep::shapeDistance<Capsule, Sphere>
    (const Capsule& s1, const Transform3f& tf1,
     const Sphere& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  /// @brief Fast implementation for sphere-cylinder distance
  template<>
    bool GJKSolver_indep::shapeDistance<Sphere, Cylinder>
    (const Sphere& s1, const Transform3f& tf1,
     const Cylinder& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  template<>
    bool GJKSolver_indep::shapeDistance<Cylinder, Sphere>
    (const Cylinder& s1, const Transform3f& tf1,
     const Sphere& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  /// @brief Fast implementation for sphere-sphere distance
  template<>
    bool GJKSolver_indep::shapeDistance<Sphere, Sphere>
    (const Sphere& s1, const Transform3f& tf1,
     const Sphere& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  // @brief Computation of the distance result for capsule capsule. Closest points are based on two line-segments.
  template<>
    bool GJKSolver_indep::shapeDistance<Capsule, Capsule>
    (const Capsule& s1, const Transform3f& tf1,
     const Capsule& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

  // Distance computation between two triangles
  //
  // Do not run EPA algorithm to compute penetration depth, use a dedicated
  // method.
  template<>
    bool GJKSolver_indep::shapeDistance<TriangleP, TriangleP>
    (const TriangleP& s1, const Transform3f& tf1,
     const TriangleP& s2, const Transform3f& tf2,
     FCL_REAL& dist, Vec3f& p1, Vec3f& p2, Vec3f& normal) const;

}

} // namespace hpp

#endif
