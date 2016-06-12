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

/** \author Jia Pan */


#include <hpp/fcl/math/transform.h>
#include <boost/math/constants/constants.hpp>

namespace fcl
{

void Quaternion3f::fromRotation(const Matrix3f& R)
{
  const int next[3] = {1, 2, 0};

  FCL_REAL trace = R(0, 0) + R(1, 1) + R(2, 2);
  FCL_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[W] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[X] = (R(2, 1) - R(1, 2))*root;
    data[Y] = (R(0, 2) - R(2, 0))*root;
    data[Z] = (R(1, 0) - R(0, 1))*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(R(1, 1) > R(0, 0))
    {
      i = 1;
    }
    if(R(2, 2) > R(i, i))
    {
      i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0);
    FCL_REAL* quat[3] = { &data[X], &data[Y], &data[Z] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[W] = (R(k, j) - R(j, k)) * root;
    *quat[j] = (R(j, i) + R(i, j)) * root;
    *quat[k] = (R(k, i) + R(i, k)) * root;
  }
}

void Quaternion3f::toRotation(Matrix3f& R) const
{
  assert (.99 < data [W]*data [W] + data [X]*data [X] +
	  data [Y]*data [Y] + data [Z]*data [Z]);
  assert (data [W]*data [W] + data [X]*data [X] +
	  data [Y]*data [Y] + data [Z]*data [Z] < 1.01);
  FCL_REAL twoX  = 2.0*data[X];
  FCL_REAL twoY  = 2.0*data[Y];
  FCL_REAL twoZ  = 2.0*data[Z];
  FCL_REAL twoWX = twoX*data[W];
  FCL_REAL twoWY = twoY*data[W];
  FCL_REAL twoWZ = twoZ*data[W];
  FCL_REAL twoXX = twoX*data[X];
  FCL_REAL twoXY = twoY*data[X];
  FCL_REAL twoXZ = twoZ*data[X];
  FCL_REAL twoYY = twoY*data[Y];
  FCL_REAL twoYZ = twoZ*data[Y];
  FCL_REAL twoZZ = twoZ*data[Z];

  R(0,0) = 1.0 - (twoYY + twoZZ);
  R(0,1) = twoXY - twoWZ;
  R(0,2) = twoXZ + twoWY;
  R(1,0) = twoXY + twoWZ;
  R(1,1) = 1.0 - (twoXX + twoZZ);
  R(1,2) = twoYZ - twoWX;
  R(2,0) = twoXZ - twoWY;
  R(2,1) = twoYZ + twoWX;
  R(2,2) = 1.0 - (twoXX + twoYY);
}

void Quaternion3f::fromAxes(const Matrix3f& axes)
{
  // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
  // article "Quaternion Calculus and Fast Animation".

  const int next[3] = {1, 2, 0};

  FCL_REAL trace = axes.trace();
  FCL_REAL root;

  if(trace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    root = sqrt(trace + 1.0);  // 2w
    data[W] = 0.5 * root;
    root = 0.5 / root;  // 1/(4w)
    data[X] = (axes(1,2) - axes(2,1))*root;
    data[Y] = (axes(2,0) - axes(0,2))*root;
    data[Z] = (axes(0,1) - axes(1,0))*root;
  }
  else
  {
    // |w| <= 1/2
    int i = 0;
    if(axes(1,1) > axes(0,0))
    {
      i = 1;
    }
    if(axes(2,2) > axes(i,i))
    {
      i = 2;
    }
    int j = next[i];
    int k = next[j];

    root = sqrt(axes(i,i) - axes(j,j) - axes(k,k) + 1.0);
    FCL_REAL* quat[3] = { &data[X], &data[Y], &data[Z] };
    *quat[i] = 0.5 * root;
    root = 0.5 / root;
    data[W] = (axes(j,k) - axes(k,j)) * root;
    *quat[j] = (axes(i,j) + axes(j,i)) * root;
    *quat[k] = (axes(i,k) + axes(k,i)) * root;
  }
}

void Quaternion3f::toAxes(Matrix3f& axes) const
{
  FCL_REAL twoX  = 2.0*data[X];
  FCL_REAL twoY  = 2.0*data[Y];
  FCL_REAL twoZ  = 2.0*data[Z];
  FCL_REAL twoWX = twoX*data[W];
  FCL_REAL twoWY = twoY*data[W];
  FCL_REAL twoWZ = twoZ*data[W];
  FCL_REAL twoXX = twoX*data[X];
  FCL_REAL twoXY = twoY*data[X];
  FCL_REAL twoXZ = twoZ*data[X];
  FCL_REAL twoYY = twoY*data[Y];
  FCL_REAL twoYZ = twoZ*data[Y];
  FCL_REAL twoZZ = twoZ*data[Z];

  axes << 1.0 - (twoYY + twoZZ), twoXY + twoWZ, twoXZ - twoWY,
          twoXY - twoWZ, 1.0 - (twoXX + twoZZ), twoYZ + twoWX,
          twoXZ + twoWY, twoYZ - twoWX, 1.0 - (twoXX + twoYY);
}

void Quaternion3f::toAxisAngle(Vec3f& axis, FCL_REAL& angle) const
{
  FCL_REAL sqr_length = data.squaredNorm();
  if(sqr_length > 0)
  {
    angle = 2.0 * acos((double)data[W]);
    double inv_length = 1.0 / sqrt(sqr_length);
    axis.noalias() = inv_length * data.segment<3>(X);
  }
  else
  {
    angle = 0;
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
  }
}

const Quaternion3f& Quaternion3f::operator *= (const Quaternion3f& other)
{
  FCL_REAL a = data[W] * other.data[W] - data[X] * other.data[X] - data[Y] * other.data[Y] - data[Z] * other.data[Z];
  FCL_REAL b = data[W] * other.data[X] + data[X] * other.data[W] + data[Y] * other.data[Z] - data[Z] * other.data[Y];
  FCL_REAL c = data[W] * other.data[Y] - data[X] * other.data[Z] + data[Y] * other.data[W] + data[Z] * other.data[X];
  FCL_REAL d = data[W] * other.data[Z] + data[X] * other.data[Y] - data[Y] * other.data[X] + data[Z] * other.data[W];

  data[W] = a;
  data[X] = b;
  data[Y] = c;
  data[Z] = d;
  return *this;
}

Quaternion3f Quaternion3f::operator - () const
{
  return Quaternion3f(-data[W], -data[X], -data[Y], -data[Z]);
}

Quaternion3f Quaternion3f::operator * (FCL_REAL t) const
{
  return Quaternion3f(data[W] * t, data[X] * t, data[Y] * t, data[Z] * t);
}

const Quaternion3f& Quaternion3f::operator *= (FCL_REAL t)
{
  data[W] *= t;
  data[X] *= t;
  data[Y] *= t;
  data[Z] *= t;

  return *this;
}

void Quaternion3f::fromEuler(FCL_REAL a, FCL_REAL b, FCL_REAL c)
{
  Matrix3f R;
  // R.setEulerYPR(a, b, c);
  setEulerYPR(R, a, b, c);

  fromRotation(R);
}

void Quaternion3f::toEuler(FCL_REAL& a, FCL_REAL& b, FCL_REAL& c) const
{
  Matrix3f R;
  toRotation(R);
  a = atan2(R(1, 0), R(0, 0));
  b = asin(-R(2, 0));
  c = atan2(R(2, 1), R(2, 2));

  if(b == boost::math::constants::pi<double>() * 0.5)
  {
    if(a > 0)
      a -= boost::math::constants::pi<double>();
    else 
      a += boost::math::constants::pi<double>();

    if(c > 0)
      c -= boost::math::constants::pi<double>();
    else
      c += boost::math::constants::pi<double>();
  }
}


const Matrix3f& Transform3f::getRotationInternal() const
{
  boost::mutex::scoped_lock slock(const_cast<boost::mutex&>(lock_));
  if(!matrix_set)
  {
    q.toRotation(R);
    matrix_set = true;
  }

  return R;
}


Transform3f inverse(const Transform3f& tf)
{
  Transform3f res(tf);
  return res.inverse();
}

void relativeTransform(const Transform3f& tf1, const Transform3f& tf2,
                       Transform3f& tf)
{
  const Quaternion3f& q1_inv = tf1.getQuatRotation().conj();
  tf = Transform3f(q1_inv * tf2.getQuatRotation(), q1_inv.transform(tf2.getTranslation() - tf1.getTranslation()));
}

void relativeTransform2(const Transform3f& tf1, const Transform3f& tf2,
                       Transform3f& tf)
{
  const Quaternion3f& q1inv = tf1.getQuatRotation().conj();
  const Quaternion3f& q2_q1inv = tf2.getQuatRotation() * q1inv;
  tf = Transform3f(q2_q1inv, tf2.getTranslation() - q2_q1inv.transform(tf1.getTranslation()));
}



}
