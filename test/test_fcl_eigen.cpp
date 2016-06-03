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

#define BOOST_TEST_MODULE "FCL_EIGEN"
#include <boost/test/included/unit_test.hpp>

#include <hpp/fcl/config-fcl.hh>
#include <hpp/fcl/eigen/vec_3fx.h>

using namespace fcl;

#define PRINT_VECTOR(a) std::cout << #a": " << a.base().transpose() << std::endl;
#define PRINT_MATRIX(a) std::cout << #a": " << a << std::endl;

BOOST_AUTO_TEST_CASE(fcl_eigen_vec3fx)
{
  typedef Eigen::FclMatrix <double, 1, 0> Vec3f;
  Vec3f a (0, 1, 2);
  Vec3f b (1, 2, 3);

  std::cout << (Vec3f::Base&) a - (Vec3f::Base&) b << std::endl;
  std::cout << a - b << std::endl;
  Vec3f c = a - b;
  std::cout << c << std::endl;
  c.normalize ();

  Vec3f l = (a - b).normalize ();

  std::cout << l << std::endl;
  std::cout << c << std::endl;

  Vec3f d = -b;
  std::cout << d << std::endl;

  d += b;
  std::cout << d << std::endl;

  d += 1;
  std::cout << d << std::endl;

  d *= b;
  std::cout << d << std::endl;

  // std::cout << d * b << std::endl;
  // std::cout << d / b << std::endl;
  // std::cout << d + 3.4 << std::endl;
  // std::cout << d - 3.4 << std::endl;
  // std::cout << d * 2 << std::endl;
  // std::cout << d / 3 << std::endl;
  // std::cout << (d - 3.4).abs().squaredNorm() << std::endl;

  // std::cout << (((d - 3.4).abs() + 1) + 3).squaredNorm() << std::endl;
  PRINT_VECTOR(a)
  PRINT_VECTOR(b)
  PRINT_VECTOR(min(a,b))
  PRINT_VECTOR(max(a,b))
  a.lbound(b);
  PRINT_VECTOR(a)
  std::cout << (a+1).lbound(b) << std::endl;
  std::cout << (a+1).ubound(b) << std::endl;

  std::cout << a.getRow(1) << std::endl;
}

BOOST_AUTO_TEST_CASE(fcl_eigen_matrix3fx)
{
  typedef Eigen::FclMatrix <double, 3, 0> Matrix3fX;
  Matrix3fX a (0, 1, 2, 3, 4, 5, 6, 7, 8);

  PRINT_MATRIX(a);
  a.transpose ();
  PRINT_MATRIX(a);
  a += a;
  PRINT_MATRIX(a);
  a *= a;
  PRINT_MATRIX(a);

  Matrix3fX b = inverse(a);
  a += a + a * b;
  PRINT_MATRIX(a);

  b = inverse(a);
  a.transpose ();
  PRINT_MATRIX(a);
  PRINT_MATRIX(a.transposeTimes (b));
  PRINT_MATRIX(a.timesTranspose (b));
  PRINT_MATRIX(a.tensorTransform (b));
}
