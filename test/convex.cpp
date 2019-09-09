/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, LAAS-CNRS
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

/** \author Joseph Mirabel */


#define BOOST_TEST_MODULE FCL_GEOMETRIC_SHAPES
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <hpp/fcl/shape/geometric_shapes.h>

using namespace hpp::fcl;

BOOST_AUTO_TEST_CASE(convex)
{
  FCL_REAL l = 1, w = 1, d = 1;

  Vec3f pts[8];
  pts[0] = Vec3f( l, w, d);
  pts[1] = Vec3f( l, w,-d);
  pts[2] = Vec3f( l,-w, d);
  pts[3] = Vec3f( l,-w,-d);
  pts[4] = Vec3f(-l, w, d);
  pts[5] = Vec3f(-l, w,-d);
  pts[6] = Vec3f(-l,-w, d);
  pts[7] = Vec3f(-l,-w,-d);
  std::vector<int> polygons;
  polygons.push_back(4);
  polygons.push_back(0);
  polygons.push_back(2);
  polygons.push_back(3);
  polygons.push_back(1);

  polygons.push_back(4);
  polygons.push_back(2);
  polygons.push_back(6);
  polygons.push_back(7);
  polygons.push_back(3);

  polygons.push_back(4);
  polygons.push_back(4);
  polygons.push_back(5);
  polygons.push_back(7);
  polygons.push_back(6);

  polygons.push_back(4);
  polygons.push_back(0);
  polygons.push_back(1);
  polygons.push_back(5);
  polygons.push_back(4);

  polygons.push_back(4);
  polygons.push_back(1);
  polygons.push_back(3);
  polygons.push_back(7);
  polygons.push_back(5);

  polygons.push_back(4);
  polygons.push_back(0);
  polygons.push_back(2);
  polygons.push_back(6);
  polygons.push_back(4);

  Convex box (
      pts, // points
      8, // num points
      polygons.data(),
      6 // number of polygons
      );

  // Check neighbors
  for (int i = 0; i < 8; ++i) {
    BOOST_CHECK_EQUAL(box.neighbors[i].count(), 3);
  }
  BOOST_CHECK_EQUAL(box.neighbors[0][0], 1);
  BOOST_CHECK_EQUAL(box.neighbors[0][1], 2);
  BOOST_CHECK_EQUAL(box.neighbors[0][2], 4);

  BOOST_CHECK_EQUAL(box.neighbors[1][0], 0);
  BOOST_CHECK_EQUAL(box.neighbors[1][1], 3);
  BOOST_CHECK_EQUAL(box.neighbors[1][2], 5);

  BOOST_CHECK_EQUAL(box.neighbors[2][0], 0);
  BOOST_CHECK_EQUAL(box.neighbors[2][1], 3);
  BOOST_CHECK_EQUAL(box.neighbors[2][2], 6);

  BOOST_CHECK_EQUAL(box.neighbors[3][0], 1);
  BOOST_CHECK_EQUAL(box.neighbors[3][1], 2);
  BOOST_CHECK_EQUAL(box.neighbors[3][2], 7);

  BOOST_CHECK_EQUAL(box.neighbors[4][0], 0);
  BOOST_CHECK_EQUAL(box.neighbors[4][1], 5);
  BOOST_CHECK_EQUAL(box.neighbors[4][2], 6);

  BOOST_CHECK_EQUAL(box.neighbors[5][0], 1);
  BOOST_CHECK_EQUAL(box.neighbors[5][1], 4);
  BOOST_CHECK_EQUAL(box.neighbors[5][2], 7);

  BOOST_CHECK_EQUAL(box.neighbors[6][0], 2);
  BOOST_CHECK_EQUAL(box.neighbors[6][1], 4);
  BOOST_CHECK_EQUAL(box.neighbors[6][2], 7);

  BOOST_CHECK_EQUAL(box.neighbors[7][0], 3);
  BOOST_CHECK_EQUAL(box.neighbors[7][1], 5);
  BOOST_CHECK_EQUAL(box.neighbors[7][2], 6);
}
