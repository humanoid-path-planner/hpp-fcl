// Geometric Tools, LLC
// Copyright (c) 1998-2011
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// Modified by Florent Lamiraux 2014

#include <cmath>
#include <limits>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <../src/distance_func_matrix.h>

#define CLAMP(value, l, u) fmax(fmin(value, u), l)

// Note that partial specialization of template functions is not allowed.
// Therefore, two implementations with the default narrow phase solvers are
// provided. If another narrow phase solver were to be used, the default
// template implementation would be called, unless the function is also
// specialized for this new type.
//
// One solution would be to make narrow phase solvers derive from an abstract
// class and specialize the template for this abstract class.
namespace hpp
{
namespace fcl {
  class GJKSolver;


  // Compute the distance between C1 and C2 by computing the distance
  // between the two segments supporting the capsules.
  // Match algorithm of Real-Time Collision Detection, Christer Ericson - Closest Point of Two Line Segments
  template <>
  FCL_REAL ShapeShapeDistance <Capsule, Capsule> (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver*, const DistanceRequest& request,
   DistanceResult& result)
  {
    const Capsule* capsule1 = static_cast <const Capsule*> (o1);
    const Capsule* capsule2 = static_cast <const Capsule*> (o2);

    FCL_REAL EPSILON = std::numeric_limits<FCL_REAL>::epsilon () * 100;

    // We assume that capsules are centered at the origin.
    const fcl::Vec3f& c1 = tf1.getTranslation ();
    const fcl::Vec3f& c2 = tf2.getTranslation ();
    // We assume that capsules are oriented along z-axis.
    FCL_REAL halfLength1 = capsule1->halfLength;
    FCL_REAL halfLength2 = capsule2->halfLength;
    FCL_REAL radius1 = capsule1->radius;
    FCL_REAL radius2 = capsule2->radius;
    // direction of capsules
    // ||d1|| = 2 * halfLength1
    const fcl::Vec3f& d1 = 2 * halfLength1 * tf1.getRotation().col(2);
    const fcl::Vec3f& d2 = 2 * halfLength2 * tf2.getRotation().col(2);

    // Starting point of the segments
    // p1 + d1 is the end point of the segment
    const fcl::Vec3f& p1 = c1 - d1 / 2;
    const fcl::Vec3f& p2 = c2 - d2 / 2;
    const fcl::Vec3f& r = p1-p2;
    FCL_REAL a = d1.dot(d1);
    FCL_REAL b = d1.dot(d2);
    FCL_REAL c = d1.dot(r);
    FCL_REAL e = d2.dot(d2);
    FCL_REAL f = d2.dot(r);
    // S1 is parametrized by the equation p1 + s * d1
    // S2 is parametrized by the equation p2 + t * d2
    FCL_REAL s = 0.0;
    FCL_REAL t = 0.0;

    if (a <= EPSILON && e <= EPSILON)
    {
      // Check if the segments degenerate into points
      s = t = 0.0;
      FCL_REAL distance = (p1-p2).norm();
      Vec3f normal = (p1 - p2) / distance;
      distance = distance - (radius1 + radius2);
      result.min_distance = distance;
      if (request.enable_nearest_points)
      {
        result.nearest_points[0] = p1 - radius1 * normal;
        result.nearest_points[1] = p2 + radius2 * normal;
      }
      return distance;
    }
    else if (a <= EPSILON)
    {
      // First segment is degenerated
      s = 0.0;
      t = CLAMP(f / e, 0.0, 1.0);
    }
    else if (e <= EPSILON)
    {
      // Second segment is degenerated
      s = CLAMP(-c / a, 0.0, 1.0);
      t = 0.0;
    }
    else
    {
      // Always non-negative, equal 0 if the segments are parallel
      FCL_REAL denom = a*e-b*b;

      if (denom != 0.0)
      {
        s = CLAMP((b*f-c*e) / denom, 0.0, 1.0);
      }
      else
      {
        s = 0.0;
      }

      t = (b*s + f) / e;
      if (t < 0.0)
      {
        t = 0.0;
        s = CLAMP(-c / a, 0.0, 1.0);
      }
      else if (t > 1.0)
      {
        t = 1.0;
        s = CLAMP((b - c)/a, 0.0, 1.0);
      }
    }

    // witness points achieving the distance between the two segments
    const Vec3f& w1 = p1 + s * d1;
    const Vec3f& w2 = p2 + t * d2;
    FCL_REAL distance = (w1 - w2).norm();
    Vec3f normal = (w1 - w2) / distance;

    // capsule spcecific distance computation
    distance = distance - (radius1 + radius2);
    result.min_distance = distance;
    // witness points for the capsules
    if (request.enable_nearest_points)
    {
      result.nearest_points[0] = w1 - radius1 * normal;
      result.nearest_points[1] = w2 + radius2 * normal;
    }
    return distance;
  }

  template <>
  std::size_t ShapeShapeCollide <Capsule, Capsule>
  (const CollisionGeometry* o1, const Transform3f& tf1,
   const CollisionGeometry* o2, const Transform3f& tf2,
   const GJKSolver*, const CollisionRequest& request,
   CollisionResult& result)
  {
    GJKSolver* unused = 0x0;
    DistanceResult distanceResult;
    DistanceRequest distanceRequest (request.enable_contact);

    FCL_REAL distance = ShapeShapeDistance <Capsule, Capsule>
      (o1, tf1, o2, tf2, unused, distanceRequest, distanceResult);

    if (distance > 0)
    {
      return 0;
    }
    else
    {
      Contact contact (o1, o2, -1, -1);
      const Vec3f& p1 = distanceResult.nearest_points [0];
      const Vec3f& p2 = distanceResult.nearest_points [1];
      contact.pos = 0.5 * (p1+p2);
      contact.normal = (p2-p1)/(p2-p1).norm ();
      result.addContact(contact);
      return 1;
    }
  }

} // namespace fcl

} // namespace hpp
