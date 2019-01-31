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


#ifndef HPP_FCL_TRANSFORM_H
#define HPP_FCL_TRANSFORM_H

#include <hpp/fcl/math/matrix_3f.h>
#include <boost/thread/mutex.hpp>

namespace hpp
{
namespace fcl
{

typedef Eigen::Quaternion<FCL_REAL> Quaternion3f;
static inline std::ostream& operator << (std::ostream& o, const Quaternion3f& q)
{
  o << "(" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << ")";
  return o;
}

inline bool isQuatIdentity (const Quaternion3f& q)
{
  return (q.w() == 1 || q.w() == -1) && q.x() == 0 && q.y() == 0 && q.z() == 0;
}

inline bool areQuatEquals (const Quaternion3f& q1, const Quaternion3f& q2)
{
  return (q1.w() ==  q2.w() && q1.x() ==  q2.x() && q1.y() ==  q2.y() && q1.z() ==  q2.z())
      || (q1.w() == -q2.w() && q1.x() == -q2.x() && q1.y() == -q2.y() && q1.z() == -q2.z());
}

/// @brief Simple transform class used locally by InterpMotion
class Transform3f
{
  /// @brief Whether matrix cache is set
  mutable bool matrix_set;
  /// @brief Matrix cache
  mutable Matrix3f R;

  /// @brief Tranlation vector
  Vec3f T;

  /// @brief Rotation
  Quaternion3f q;

  const Matrix3f& getRotationInternal() const;
public:

  /// @brief Default transform is no movement
  Transform3f()
  {
    setIdentity(); // set matrix_set true
  }

  /// @brief Construct transform from rotation and translation
  Transform3f(const Matrix3f& R_, const Vec3f& T_) : matrix_set(true),
                                                     R(R_),
                                                     T(T_)
  {
    q = Quaternion3f(R_);
  }

  /// @brief Construct transform from rotation and translation
  Transform3f(const Quaternion3f& q_, const Vec3f& T_) : matrix_set(false),
                                                         T(T_),
                                                         q(q_)
  {
  }

  /// @brief Construct transform from rotation
  Transform3f(const Matrix3f& R_) : matrix_set(true),
                                    R(R_),
                                    T(Vec3f::Zero())
  {
    q = Quaternion3f(R_);
  }

  /// @brief Construct transform from rotation
  Transform3f(const Quaternion3f& q_) : matrix_set(false),
                                        T(Vec3f::Zero()),
                                        q(q_)
  {
  }

  /// @brief Construct transform from translation
  Transform3f(const Vec3f& T_) : matrix_set(true), 
                                 T(T_),
                                 q(Quaternion3f::Identity())
  {
    R.setIdentity();
  }

  /// @brief Construct transform from another transform
  Transform3f(const Transform3f& tf) : matrix_set(tf.matrix_set),
                                       R(tf.R),
                                       T(tf.T),
                                       q(tf.q)
  {
  }

  /// @brief operator = 
  Transform3f& operator = (const Transform3f& tf)
  {
    matrix_set = tf.matrix_set;
    R = tf.R;
    q = tf.q;
    T = tf.T;
    return *this;
  }

  /// @brief get translation
  inline const Vec3f& getTranslation() const
  {
    return T;
  }

  /// @brief get rotation
  inline const Matrix3f& getRotation() const
  {
    if(matrix_set) return R;
    return getRotationInternal();
  }

  /// @brief get quaternion
  inline const Quaternion3f& getQuatRotation() const
  {
    return q;
  }

  /// @brief set transform from rotation and translation
  inline void setTransform(const Matrix3f& R_, const Vec3f& T_)
  {
    R.noalias() = R_;
    T.noalias() = T_;
    q = Quaternion3f(R_);
    matrix_set = true;
  }

  /// @brief set transform from rotation and translation
  inline void setTransform(const Quaternion3f& q_, const Vec3f& T_)
  {
    matrix_set = false;
    q = q_;
    T.noalias() = T_;
  }

  /// @brief set transform from rotation
  template<typename Derived>
  inline void setRotation(const Eigen::MatrixBase<Derived>& R_)
  {
    R.noalias() = R_;
    matrix_set = true;
    q = Quaternion3f(R);
  }

  /// @brief set transform from translation
  template<typename Derived>
  inline void setTranslation(const Eigen::MatrixBase<Derived>& T_)
  {
    T.noalias() = T_;
  }

  /// @brief set transform from rotation
  inline void setQuatRotation(const Quaternion3f& q_)
  {
    matrix_set = false;
    q = q_;
  }

  /// @brief transform a given vector by the transform
  template <typename Derived>
  inline Vec3f transform(const Eigen::MatrixBase<Derived>& v) const
  {
    return q * v + T;
  }

  /// @brief inverse transform
  inline Transform3f& inverse()
  {
    matrix_set = false;
    q = q.conjugate();
    T = q * (-T);
    return *this;
  }

  /// @brief inverse the transform and multiply with another
  inline Transform3f inverseTimes(const Transform3f& other) const
  {
    const Quaternion3f& q_inv = q.conjugate();
    return Transform3f(q_inv * other.q, q_inv * (other.T - T));
  }

  /// @brief multiply with another transform
  inline const Transform3f& operator *= (const Transform3f& other)
  {
    matrix_set = false;
    T += q * other.T;
    q *= other.q;
    return *this;
  }

  /// @brief multiply with another transform
  inline Transform3f operator * (const Transform3f& other) const
  {
    Quaternion3f q_new = q * other.q;
    return Transform3f(q_new, q * other.T + T);
  }

  /// @brief check whether the transform is identity
  inline bool isIdentity() const
  {
    return isQuatIdentity(q) && T.isZero();
  }

  /// @brief set the transform to be identity transform
  inline void setIdentity()
  {
    R.setIdentity();
    T.setZero();
    q.setIdentity();
    matrix_set = true;
  }

  bool operator == (const Transform3f& other) const
  {
    return areQuatEquals(q, other.getQuatRotation()) && (T == other.getTranslation());
  }

  bool operator != (const Transform3f& other) const
  {
    return !(*this == other);
  }

};

/// @brief inverse the transform
Transform3f inverse(const Transform3f& tf);

/// @brief compute the relative transform between two transforms: tf2 = tf1 * tf (relative to the local coordinate system in tf1)
void relativeTransform(const Transform3f& tf1, const Transform3f& tf2,
                       Transform3f& tf);

/// @brief compute the relative transform between two transforms: tf2 = tf * tf1 (relative to the global coordinate system)
void relativeTransform2(const Transform3f& tf1, const Transform3f& tf2,
                        Transform3f& tf);


}

} // namespace hpp

#endif
