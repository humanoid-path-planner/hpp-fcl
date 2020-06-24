/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS and AIST
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** \author Florent Lamiraux */


#define BOOST_TEST_MODULE FCL_GEOMETRIC_SHAPES
#include <boost/test/included/unit_test.hpp>

#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE ((x + 1.0), (1.0), (eps))

#include <cmath>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include "utility.h"

BOOST_AUTO_TEST_CASE(distance_capsule_box)
{
  typedef boost::shared_ptr <hpp::fcl::CollisionGeometry> CollisionGeometryPtr_t;
  // Capsule of radius 2 and of height 4
  CollisionGeometryPtr_t capsuleGeometry (new hpp::fcl::Capsule (2., 4.));
  // Box of size 1 by 2 by 4
  CollisionGeometryPtr_t boxGeometry (new hpp::fcl::Box (1., 2., 4.));

  // Enable computation of nearest points
  hpp::fcl::DistanceRequest distanceRequest (true, 0, 0);
  hpp::fcl::DistanceResult distanceResult;
  
  hpp::fcl::Transform3f tf1 (hpp::fcl::Vec3f (3., 0, 0));
  hpp::fcl::Transform3f tf2;
  hpp::fcl::CollisionObject capsule (capsuleGeometry, tf1);
  hpp::fcl::CollisionObject box (boxGeometry, tf2);

  // test distance
  hpp::fcl::distance (&capsule, &box, distanceRequest, distanceResult);
  // Nearest point on capsule
  hpp::fcl::Vec3f o1 (distanceResult.nearest_points [0]);
  // Nearest point on box
  hpp::fcl::Vec3f o2 (distanceResult.nearest_points [1]);
  BOOST_CHECK_CLOSE (distanceResult.min_distance, 0.5, 1e-1);
  BOOST_CHECK_CLOSE (o1 [0], 1.0, 1e-1);
  CHECK_CLOSE_TO_0 (o1 [1], 1e-1);
  BOOST_CHECK_CLOSE (o2 [0], 0.5, 1e-1);
  CHECK_CLOSE_TO_0 (o2 [1], 1e-1);

  // Move capsule above box
  tf1 = hpp::fcl::Transform3f (hpp::fcl::Vec3f (0., 0., 8.));
  capsule.setTransform (tf1);

  // test distance
  distanceResult.clear ();
  hpp::fcl::distance (&capsule, &box, distanceRequest, distanceResult);
  o1 = distanceResult.nearest_points [0];
  o2 = distanceResult.nearest_points [1];

  BOOST_CHECK_CLOSE (distanceResult.min_distance, 2.0, 1e-1);
  CHECK_CLOSE_TO_0 (o1 [0], 1e-1);
  CHECK_CLOSE_TO_0 (o1 [1], 1e-1);
  BOOST_CHECK_CLOSE (o1 [2], 4.0, 1e-1);

  CHECK_CLOSE_TO_0 (o2 [0], 1e-1);
  CHECK_CLOSE_TO_0 (o2 [1], 1e-1);
  BOOST_CHECK_CLOSE (o2 [2],  2.0, 1e-1);

  // Rotate capsule around y axis by pi/2 and move it behind box
  tf1.setTranslation (hpp::fcl::Vec3f (-10., 0., 0.));
  tf1.setQuatRotation (hpp::fcl::makeQuat (sqrt(2)/2, 0, sqrt(2)/2, 0));
  capsule.setTransform (tf1);

  // test distance
  distanceResult.clear ();
  hpp::fcl::distance (&capsule, &box, distanceRequest, distanceResult);
  o1 = distanceResult.nearest_points [0];
  o2 = distanceResult.nearest_points [1];

  BOOST_CHECK_CLOSE (distanceResult.min_distance, 5.5, 1e-1);
  BOOST_CHECK_CLOSE (o1 [0], -6, 1e-2);
  CHECK_CLOSE_TO_0 (o1 [1], 1e-1);
  CHECK_CLOSE_TO_0 (o1 [2], 1e-1);
  BOOST_CHECK_CLOSE (o2 [0], -0.5, 1e-2);
  CHECK_CLOSE_TO_0 (o2 [1], 1e-1);
  CHECK_CLOSE_TO_0 (o2 [2], 1e-1);
}
