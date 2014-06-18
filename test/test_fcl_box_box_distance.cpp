/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CNRS.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Florent Lamiraux <florent@laas.fr> */

#define BOOST_TEST_MODULE "FCL_BOX_BOX"
#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE ((x + 1.0), (1.0), (eps))

#include <boost/test/unit_test.hpp>

#include <cmath>
#include <fcl/distance.h>
#include <fcl/math/transform.h>
#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>

typedef boost::shared_ptr <fcl::CollisionGeometry> CollisionGeometryPtr_t;

using fcl::Transform3f;
using fcl::Quaternion3f;
using fcl::Vec3f;
using fcl::CollisionObject;
using fcl::DistanceResult;
using fcl::DistanceRequest;
using fcl::GST_INDEP;

BOOST_AUTO_TEST_CASE(distance_box_box_1)
{
  CollisionGeometryPtr_t s1 (new fcl::Box (6, 10, 2));
  CollisionGeometryPtr_t s2 (new fcl::Box (2, 2, 2));

  Transform3f tf1;
  Transform3f tf2 (Vec3f(25, 20, 5));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
  DistanceResult distanceResult;

  fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;
  double dx = 25 - 3 - 1;
  double dy = 20 - 5 - 1;
  double dz = 5 - 1 - 1;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];
  BOOST_CHECK_CLOSE(distanceResult.min_distance, 
		    sqrt (dx*dx + dy*dy + dz*dz), 1e-4);

  BOOST_CHECK_CLOSE (p1 [0], 3, 1e-6);
  BOOST_CHECK_CLOSE (p1 [1], 5, 1e-6);
  BOOST_CHECK_CLOSE (p1 [2], 1, 1e-6);
  BOOST_CHECK_CLOSE (p2 [0], 24, 1e-6);
  BOOST_CHECK_CLOSE (p2 [1], 19, 1e-6);
  BOOST_CHECK_CLOSE (p2 [2], 4, 1e-6);
}

BOOST_AUTO_TEST_CASE(distance_box_box_2)
{
  CollisionGeometryPtr_t s1 (new fcl::Box (6, 10, 2));
  CollisionGeometryPtr_t s2 (new fcl::Box (2, 2, 2));
  static double pi = M_PI;
  Transform3f tf1;
  Transform3f tf2 (Quaternion3f (cos (pi/8), sin(pi/8)/sqrt(3),
				 sin(pi/8)/sqrt(3), sin(pi/8)/sqrt(3)),
		   Vec3f(0, 0, 10));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
  DistanceResult distanceResult;

  fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];
  double distance = -1.62123444 + 10 - 1;
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  BOOST_CHECK_CLOSE (p1 [0], 0.60947571, 1e-4);
  BOOST_CHECK_CLOSE (p1 [1], 0.01175873, 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], 1, 1e-6);
  BOOST_CHECK_CLOSE (p2 [0], 0.60947571, 1e-4);
  BOOST_CHECK_CLOSE (p2 [1], 0.01175873, 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], -1.62123444 + 10, 1e-4);
}

BOOST_AUTO_TEST_CASE(distance_box_box_3)
{
  CollisionGeometryPtr_t s1 (new fcl::Box (1, 1, 1));
  CollisionGeometryPtr_t s2 (new fcl::Box (1, 1, 1));
  static double pi = M_PI;
  Transform3f tf1 (Quaternion3f (cos (pi/8), 0, 0, sin (pi/8)),
		   Vec3f (-2, 1, .5));
  Transform3f tf2 (Quaternion3f (cos (pi/8), 0, sin(pi/8),0),
		   Vec3f (2, .5, .5));

  CollisionObject o1 (s1, tf1);
  CollisionObject o2 (s2, tf2);

  // Enable computation of nearest points
  DistanceRequest distanceRequest (true, 0, 0, GST_INDEP);
  DistanceResult distanceResult;

  fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  const Vec3f& p1 = distanceResult.nearest_points [0];
  const Vec3f& p2 = distanceResult.nearest_points [1];
  double distance = 4 - sqrt (2);
  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  const Vec3f p1Ref (sqrt(2)/2 - 2, 1, .5);
  const Vec3f p2Ref (2 - sqrt(2)/2, 1, .5);
  BOOST_CHECK_CLOSE (p1 [0], p1Ref [0], 1e-4);
  BOOST_CHECK_CLOSE (p1 [1], p1Ref [1], 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], p1Ref [2], 1e-4);
  BOOST_CHECK_CLOSE (p2 [0], p2Ref [0], 1e-4);
  BOOST_CHECK_CLOSE (p2 [1], p2Ref [1], 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], p2Ref [2], 1e-4);

  // Apply the same global transform to both objects and recompute
  Transform3f tf3 (Quaternion3f (0.435952844074,-0.718287018243,
				 0.310622451066, 0.444435113443),
		   Vec3f (4, 5, 6));
  tf1 = tf3*tf1;
  tf2 = tf3*tf2;
  o1 = CollisionObject (s1, tf1);
  o2 = CollisionObject (s2, tf2);

  distanceResult.clear ();
  fcl::distance (&o1, &o2, distanceRequest, distanceResult);

  std::cerr << "Applied transformations on two boxes" << std::endl;
  std::cerr << " T1 = " << tf1.getTranslation() << std::endl
	    << " R1 = " << tf1.getRotation () << std::endl
	    << " T2 = " << tf2.getTranslation() << std::endl
	    << " R2 = " << tf2.getRotation () << std::endl;
  std::cerr << "Closest points: p1 = " << distanceResult.nearest_points [0]
	    << ", p2 = " << distanceResult.nearest_points [1]
	    << ", distance = " << distanceResult.min_distance << std::endl;

  BOOST_CHECK_CLOSE(distanceResult.min_distance, distance, 1e-4);

  const Vec3f p1Moved = tf3.transform (p1Ref);
  const Vec3f p2Moved = tf3.transform (p2Ref);
  BOOST_CHECK_CLOSE (p1 [0], p1Moved [0], 1e-4);
  BOOST_CHECK_CLOSE (p1 [1], p1Moved [1], 1e-4);
  BOOST_CHECK_CLOSE (p1 [2], p1Moved [2], 1e-4);
  BOOST_CHECK_CLOSE (p2 [0], p2Moved [0], 1e-4);
  BOOST_CHECK_CLOSE (p2 [1], p2Moved [1], 1e-4);
  BOOST_CHECK_CLOSE (p2 [2], p2Moved [2], 1e-4);
  
}
