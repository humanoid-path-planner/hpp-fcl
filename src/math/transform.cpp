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

namespace hpp
{
namespace fcl
{

const Matrix3f& Transform3f::getRotationInternal() const
{
  boost::mutex::scoped_lock slock(const_cast<boost::mutex&>(lock_));
  if(!matrix_set)
  {
    R = q.toRotationMatrix();
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
  const Quaternion3f& q1_inv = tf1.getQuatRotation().conjugate();
  tf = Transform3f(q1_inv * tf2.getQuatRotation(), q1_inv * (tf2.getTranslation() - tf1.getTranslation()));
}

void relativeTransform2(const Transform3f& tf1, const Transform3f& tf2,
                       Transform3f& tf)
{
  const Quaternion3f& q1inv = tf1.getQuatRotation().conjugate();
  const Quaternion3f& q2_q1inv = tf2.getQuatRotation() * q1inv;
  tf = Transform3f(q2_q1inv, tf2.getTranslation() - q2_q1inv * tf1.getTranslation());
}



}

} // namespace hpp
