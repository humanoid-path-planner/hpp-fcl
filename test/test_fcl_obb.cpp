/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS
 *  Author: Florent Lamiraux
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
 *   * Neither the name of CNRS nor the names of its
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

#define BOOST_TEST_MODULE FCL_DISTANCE_OBB
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#define CHECK_CLOSE_TO_0(x, eps) BOOST_CHECK_CLOSE ((x + 1.0), (1.0), (eps))

#include <boost/test/included/unit_test.hpp>
#include <hpp/fcl/narrowphase/narrowphase.h>

#include "../src/BV/OBB.h"
#include "../src/distance_func_matrix.h"

using hpp::fcl::FCL_REAL;

struct Sample
{
  hpp::fcl::Matrix3f R;
  hpp::fcl::Vec3f T;
  hpp::fcl::Vec3f extent1;
  hpp::fcl::Vec3f extent2;
};

struct Result
{
  bool overlap;
  FCL_REAL distance;
};

BOOST_AUTO_TEST_CASE(obb_overlap)
{
  static const size_t nbSamples = 10000;
  Sample sample [nbSamples];
  FCL_REAL range = 2;
  FCL_REAL sumDistance = 0, sumDistanceLowerBound = 0;
  for (std::size_t i=0; i<nbSamples; ++i) {
    // sample unit quaternion
    FCL_REAL u1 = (FCL_REAL) rand() / RAND_MAX;
    FCL_REAL u2 = (FCL_REAL) rand() / RAND_MAX;
    FCL_REAL u3 = (FCL_REAL) rand() / RAND_MAX;

    FCL_REAL q1 = sqrt (1-u1)*sin(2*M_PI*u2);
    FCL_REAL q2 = sqrt (1-u1)*cos(2*M_PI*u2);
    FCL_REAL q3 = sqrt (u1) * sin(2*M_PI*u3);
    FCL_REAL q4 = sqrt (u1) * cos(2*M_PI*u3);
    hpp::fcl::Quaternion3f (q1, q2, q3, q4).toRotation (sample [i].R);

    // sample translation
    sample [i].T = hpp::fcl::Vec3f ((FCL_REAL) range * rand() / RAND_MAX,
			       (FCL_REAL) range * rand() / RAND_MAX,
			       (FCL_REAL) range * rand() / RAND_MAX);

    // sample extents
    sample [i].extent1 = hpp::fcl::Vec3f ((FCL_REAL) rand() / RAND_MAX,
				     (FCL_REAL) rand() / RAND_MAX,
				     (FCL_REAL) rand() / RAND_MAX);

    sample [i].extent2 = hpp::fcl::Vec3f ((FCL_REAL) rand() / RAND_MAX,
				     (FCL_REAL) rand() / RAND_MAX,
				     (FCL_REAL) rand() / RAND_MAX);
  }

  // Compute result for each function
  Result resultDistance [nbSamples];
  Result resultDisjoint [nbSamples];
  Result resultDisjointAndLowerBound [nbSamples];

  hpp::fcl::Transform3f tf1;
  hpp::fcl::Transform3f tf2;
  hpp::fcl::Box box1 (0, 0, 0);
  hpp::fcl::Box box2 (0, 0, 0);
  hpp::fcl::DistanceRequest request (false, 0, 0, hpp::fcl::GST_INDEP);
  hpp::fcl::DistanceResult result;
  FCL_REAL distance;
  FCL_REAL squaredDistance;
  timeval t0, t1;
  hpp::fcl::GJKSolver_indep gjkSolver;

  // ShapeShapeDistance
  gettimeofday (&t0, NULL);
  for (std::size_t i=0; i<nbSamples; ++i) {
    box1.side = 2*sample [i].extent1;
    box2.side = 2*sample [i].extent2;
    tf2.setTransform (sample [i].R, sample [i].T);
    resultDistance [i].distance =
      hpp::fcl::ShapeShapeDistance<hpp::fcl::Box, hpp::fcl::Box, hpp::fcl::GJKSolver_indep>
      (&box1, tf1, &box2, tf2, &gjkSolver, request, result);
    resultDistance [i].overlap = (resultDistance [i].distance < 0);
    if (resultDistance [i].distance < 0) {
      resultDistance [i].distance = 0;
    }
    result.clear ();
  }    
  gettimeofday (&t1, NULL);
  double t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec + 0.);
  std::cout << "Time for " << nbSamples
	    << " calls to ShapeShapeDistance (in seconds): "
	    << t << std::endl;

  // obbDisjointAndLowerBoundDistance
  gettimeofday (&t0, NULL);
  for (std::size_t i=0; i<nbSamples; ++i) {
    resultDisjointAndLowerBound [i].overlap =
      !obbDisjointAndLowerBoundDistance (sample [i].R, sample [i].T,
					 sample [i].extent1,
					 sample [i].extent2,
					 squaredDistance);
    resultDisjointAndLowerBound [i].distance = sqrt (squaredDistance);
  }    
  gettimeofday (&t1, NULL);
  t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec + 0.);
  std::cout << "Time for " << nbSamples
	    << " calls to obbDisjointAndLowerBoundDistance"
	    << " (in seconds): "
	    << t << std::endl;

  gettimeofday (&t0, NULL);
  for (std::size_t i=0; i<nbSamples; ++i) {
    resultDisjoint [i].overlap =
      !obbDisjoint (sample [i].R, sample [i].T,
		    sample [i].extent1,
		    sample [i].extent2);
    resultDisjoint [i].distance = 0;
  }    
  gettimeofday (&t1, NULL);
  t = (t1.tv_sec - t0.tv_sec) + 1e-6*(t1.tv_usec - t0.tv_usec + 0.);
  std::cout << "Time for " << nbSamples << " calls to obbDisjoint"
	    << " (in seconds): "
	    << t << std::endl;

  // Test correctness of results
  bool res1, res2, res3;
  for (std::size_t i=0; i<nbSamples; ++i) {
    res1 = resultDisjoint [i].overlap;
    res2 = resultDisjointAndLowerBound [i].overlap;
    res3 = resultDistance [i].overlap;
    sumDistanceLowerBound += resultDisjointAndLowerBound [i].distance;
    sumDistance += resultDistance [i].distance;

#if 0
    std::cout << "ShapeShapeDistance:    overlap: "
	      << resultDistance [i].overlap
	      << ", distance: " << resultDistance [i].distance << std::endl;
    std::cout << "disjoint:              overlap: "
	      << resultDisjoint [i].overlap
	      << ", distance: " << resultDisjoint [i].distance << std::endl;
    std::cout << "disjointAndLowerBound: overlap: "
	      << resultDisjointAndLowerBound [i].overlap
	      << ", distance: " << resultDisjointAndLowerBound [i].distance
	      << std::endl << std::endl;
#endif
    BOOST_CHECK (res1 == res2);
    BOOST_CHECK (res1 == res3);
    BOOST_CHECK ((!res1 && resultDisjointAndLowerBound [i].distance > 0) ||
		 (res1 && resultDisjointAndLowerBound [i].distance == 0));
    BOOST_CHECK (resultDisjointAndLowerBound [i].distance <=
		 resultDistance [i].distance);
  }
  std::cout << "Sum distances ShapeShapeDistance:               "
	    << sumDistance << std::endl;
  std::cout << "Sum distances obbDisjointAndLowerBoundDistance: "
	    << sumDistanceLowerBound << std::endl;
}
