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


#define BOOST_TEST_MODULE FCL_MATH
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <hpp/fcl/math/vec_3f.h>
#include <hpp/fcl/math/matrix_3f.h>
#include <hpp/fcl/math/transform.h>

#include <hpp/fcl/intersect.h>

using namespace fcl;

template<typename Derived>
inline Quaternion3f fromAxisAngle(const Eigen::MatrixBase<Derived>& axis, FCL_REAL angle)
{
  return Quaternion3f (Eigen::AngleAxis<double>(angle, axis));
}

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

Vec3f rotate (Vec3f input, FCL_REAL w, Vec3f vec) {
  return 2*vec.dot(input)*vec + (w*w - vec.dot(vec))*input + 2*w*vec.cross(input);
}

BOOST_AUTO_TEST_CASE(quaternion)
{
  Quaternion3f q1 (Quaternion3f::Identity()), q2, q3;
  q2 = fromAxisAngle(Vec3f(0,0,1), M_PI/2);
  q3 = q2.inverse();

  Vec3f v(1,-1,0);

  BOOST_CHECK(isEqual(v, q1 * v));
  BOOST_CHECK(isEqual(Vec3f(1,1,0), q2 * v));
  BOOST_CHECK(isEqual(rotate(v, q3.w(), Vec3f(q3.x(), q3.y(), q3.z())), q3 * v));
}

BOOST_AUTO_TEST_CASE(transform)
{
  Quaternion3f q = fromAxisAngle(Vec3f(0,0,1), M_PI/2);
  Vec3f T (0,1,2);
  Transform3f tf (q, T);

  Vec3f v(1,-1,0);

  BOOST_CHECK(isEqual(q * v + T, q * v + T));

  Vec3f rv (q * v);
  // typename Transform3f::transform_return_type<Vec3f>::type output =
    // tf * v;
  // std::cout << rv << std::endl;
  // std::cout << output.lhs() << std::endl;
  BOOST_CHECK(isEqual(rv + T, tf.transform(v)));
}

BOOST_AUTO_TEST_CASE(intersect_triangle)
{
  std::vector< Vec3f > points(3 * 6);
  points[0] = Vec3f(0,0,0);
  points[1] = Vec3f(1,0,0);
  points[2] = Vec3f(0,1,0);

  // FCL_REAL eps = +1e-16;
  FCL_REAL eps = 0;
  points[3] = Vec3f(0.5,0,eps);
  points[4] = Vec3f(0.5,0,1);
  points[5] = Vec3f(0.5,1,eps);

  eps = -1e-3;
  points[6] = Vec3f(0.5,0,eps);
  points[7] = Vec3f(0.5,0,1);
  points[8] = Vec3f(0.5,1,eps);

  eps = -1e-9;
  points[9]  = Vec3f(0.5,0,eps);
  points[10] = Vec3f(0.5,0,1);
  points[11] = Vec3f(0.5,1,eps);

  points[12] = Vec3f(0.43977451324462891,0.047868609428405762,-0.074923992156982422);
  points[13] = Vec3f(0.409393310546875,0.048755228519439697,-0.083331555128097534);
  points[14] = Vec3f(0.41051089763641357,0.059760168194770813,-0.071275442838668823);

  points[15] = Vec3f(0.43746706770940053,0.04866138334047334,-0.075818714863365125);
  points[16] = Vec3f(0.44251195980451652,0.043831023891018804,-0.074980982849817135);
  points[17] = Vec3f(0.4213840328819074,0.076059133343436849,-0.07361578194185768);

  std::vector < int > pairs(2 * 4);
  pairs[0] = 0;
  pairs[1] = 3;
  pairs[2] = 0;
  pairs[3] = 6;
  pairs[4] = 0;
  pairs[5] = 9;
  pairs[6] = 12;
  pairs[7] = 15;

  for (std::size_t ip = 6; ip < pairs.size(); ip += 2) {
    Vec3f& p1 = points[pairs[ip + 0]    ];
    Vec3f& p2 = points[pairs[ip + 0] + 1];
    Vec3f& p3 = points[pairs[ip + 0] + 2];

    Vec3f& q1 = points[pairs[ip + 1]    ];
    Vec3f& q2 = points[pairs[ip + 1] + 1];
    Vec3f& q3 = points[pairs[ip + 1] + 2];

    FCL_REAL penetration;
    Vec3f normal;
    unsigned int n_contacts;
    Vec3f contacts[2];

    bool intersect =
      Intersect::intersect_Triangle(p1, p2, p3, q1, q2, q3,
          contacts, &n_contacts, &penetration, &normal);

    if (intersect) {
      std::cout << ip << " intersect" << std::endl;
      BOOST_CHECK_MESSAGE (n_contacts > 0, "There shoud be at least 1 contact: " << n_contacts);
    } else {
      BOOST_CHECK_MESSAGE (n_contacts == 0, "There shoud be 0 contact: " << n_contacts);
    }
  }
}
