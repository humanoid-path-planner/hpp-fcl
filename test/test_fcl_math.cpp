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


#define BOOST_TEST_MODULE "FCL_MATH"
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/math/vec_3f.h>
#include <hpp/fcl/math/matrix_3f.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/broadphase/morton.h>

using namespace fcl;

BOOST_AUTO_TEST_CASE(vec_test_eigen_vec64)
{
  Vec3f v1(1.0, 2.0, 3.0);
  BOOST_CHECK(v1[0] == 1.0);
  BOOST_CHECK(v1[1] == 2.0);
  BOOST_CHECK(v1[2] == 3.0);

  Vec3f v2 = v1;
  Vec3f v3(3.3, 4.3, 5.3);
  v1 += v3;
  BOOST_CHECK(isEqual(v1, v2 + v3));
  v1 -= v3;
  BOOST_CHECK(isEqual(v1, v2));
  v1 -= v3;
  BOOST_CHECK(isEqual(v1, v2 - v3));
  v1 += v3;

  v1.array() *= v3.array();
  BOOST_CHECK(isEqual(v1, v2.cwiseProduct(v3)));
  v1.array() /= v3.array();
  BOOST_CHECK(isEqual(v1, v2));
  v1.array() /= v3.array();
  BOOST_CHECK(isEqual(v1, v2.cwiseQuotient(v3)));
  v1.array() *= v3.array();

  v1 *= 2.0;
  BOOST_CHECK(isEqual(v1, v2 * 2.0));
  v1 /= 2.0;
  BOOST_CHECK(isEqual(v1, v2));
  v1 /= 2.0;
  BOOST_CHECK(isEqual(v1, v2 / 2.0));
  v1 *= 2.0;

  v1.array() += 2.0;
  BOOST_CHECK(isEqual(v1, (v2.array() + 2.0).matrix()));
  v1.array() -= 2.0;
  BOOST_CHECK(isEqual(v1, v2));
  v1.array() -= 2.0;
  BOOST_CHECK(isEqual(v1, (v2.array() - 2.0).matrix()));
  v1.array() += 2.0;

  BOOST_CHECK(isEqual((-Vec3f(1.0, 2.0, 3.0)), Vec3f(-1.0, -2.0, -3.0)));

  v1 = Vec3f(1.0, 2.0, 3.0);
  v2 = Vec3f(3.0, 4.0, 5.0);
  BOOST_CHECK(isEqual((v1.cross(v2)), Vec3f(-2.0, 4.0, -2.0)));
  BOOST_CHECK(std::abs(v1.dot(v2) - 26) < 1e-5);

  v1 = Vec3f(3.0, 4.0, 5.0);
  BOOST_CHECK(std::abs(v1.squaredNorm() - 50) < 1e-5);
  BOOST_CHECK(std::abs(v1.norm() - sqrt(50)) < 1e-5);
  BOOST_CHECK(isEqual(v1.normalized(), v1 / v1.norm()));


  v1 = Vec3f(1.0, 2.0, 3.0);
  v2 = Vec3f(3.0, 4.0, 5.0);
  BOOST_CHECK(isEqual(v1.cross(v2), Vec3f(-2.0, 4.0, -2.0)));
  BOOST_CHECK(v1.dot(v2) == 26);
}

BOOST_AUTO_TEST_CASE(morton)
{
  AABB bbox(Vec3f(0, 0, 0), Vec3f(1000, 1000, 1000));
  morton_functor<boost::dynamic_bitset<> > F1(bbox, 10);
  morton_functor<boost::dynamic_bitset<> > F2(bbox, 20);
  morton_functor<FCL_UINT64> F3(bbox);
  morton_functor<FCL_UINT32> F4(bbox);

  Vec3f p(254, 873, 674);
  std::cout << std::hex << F1(p).to_ulong() << std::endl;
  std::cout << std::hex << F2(p).to_ulong() << std::endl;
  std::cout << std::hex << F3(p) << std::endl;
  std::cout << std::hex << F4(p) << std::endl;
  
}

Vec3f rotate (Vec3f input, FCL_REAL w, Vec3f vec) {
  return 2*vec.dot(input)*vec + (w*w - vec.dot(vec))*input + 2*w*vec.cross(input);
}

BOOST_AUTO_TEST_CASE(quaternion)
{
  Quaternion3f q1, q2, q3;
  q2.fromAxisAngle(Vec3f(0,0,1), M_PI/2);
  q3 = q2.inverse();

  Vec3f v(1,-1,0);

  BOOST_CHECK(isEqual(v, q1.transform(v)));
  BOOST_CHECK(isEqual(Vec3f(1,1,0), q2.transform(v)));
  BOOST_CHECK(isEqual(rotate(v, q3.w(), Vec3f(q3.x(), q3.y(), q3.z())), q3.transform(v)));
}

BOOST_AUTO_TEST_CASE(transform)
{
  Quaternion3f q;
  q.fromAxisAngle(Vec3f(0,0,1), M_PI/2);
  Vec3f T (0,1,2);
  Transform3f tf (q, T);

  Vec3f v(1,-1,0);

  BOOST_CHECK(isEqual(q.transform(v).eval() + T, q.transform(v) + T));

  Vec3f rv (q.transform(v));
  // typename Transform3f::transform_return_type<Vec3f>::type output =
    // tf.transform(v);
  // std::cout << rv << std::endl;
  // std::cout << output.lhs() << std::endl;
  BOOST_CHECK(isEqual(rv + T, tf.transform(v)));
}
