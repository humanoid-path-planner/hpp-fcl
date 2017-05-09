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


#ifndef FCL_TRANSFORM_H
#define FCL_TRANSFORM_H

#include <hpp/fcl/math/matrix_3f.h>
#include <boost/thread/mutex.hpp>

namespace fcl
{

  /*
/// @brief Quaternion used locally by InterpMotion
class Quaternion3f
{
private:
  typedef Eigen::Matrix<FCL_REAL, 4, 1, Eigen::DontAlign> Vec4f;
  typedef typename Vec4f::     FixedSegmentReturnType<3>::Type XYZ_t;
  typedef typename Vec4f::ConstFixedSegmentReturnType<3>::Type XYZConst_t;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  enum {
    W = 0,
    X = 1,
    Y = 2,
    Z = 3
  };
  /// @brief Default quaternion is identity rotation
  Quaternion3f()
  {
    data[W] = 1;
    data[X] = 0;
    data[Y] = 0;
    data[Z] = 0;
  }

  /// @brief Construct quaternion from 4D vector
  template <typename Derived>
  Quaternion3f(const Eigen::MatrixBase<Derived>& other) :
    data (other.derived())
  {}

  Quaternion3f(FCL_REAL w, FCL_REAL x, FCL_REAL y, FCL_REAL z)
  {
    data[W] = w;
    data[X] = x;
    data[Y] = y;
    data[Z] = z;
  }

  template<typename Derived>
  Quaternion3f(FCL_REAL _w, const Eigen::MatrixBase<Derived>& _vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    w() = _w;
    vec().noalias() = _vec;
  }

  /// @brief Whether the rotation is identity
  bool isIdentity() const
  {
    return (data[W] == 1) && (data[X] == 0) && (data[Y] == 0) && (data[Z] == 0);
  }

  /// @brief Matrix to quaternion
  void fromRotation(const Matrix3f& R);

  /// @brief Quaternion to matrix
  void toRotation(Matrix3f& R) const;

  /// @brief Euler to quaternion
  void fromEuler(FCL_REAL a, FCL_REAL b, FCL_REAL c);

  /// @brief Quaternion to Euler
  void toEuler(FCL_REAL& a, FCL_REAL& b, FCL_REAL& c) const;

  /// @brief Axes to quaternion
  void fromAxes(const Matrix3f& axes);

  /// @brief Axes to matrix
  void toAxes(Matrix3f& axis) const;

  /// @brief Axis and angle to quaternion
  template<typename Derived>
  inline void fromAxisAngle(const Eigen::MatrixBase<Derived>& axis, FCL_REAL angle)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    FCL_REAL half_angle = 0.5 * angle;
    FCL_REAL sn = sin((double)half_angle);
    data[W] = cos((double)half_angle);
    data.segment<3>(X).noalias() = sn*axis;
  }

  /// @brief Quaternion to axis and angle
  void toAxisAngle(Vec3f& axis, FCL_REAL& angle) const;

  /// @brief Dot product between quaternions
  inline FCL_REAL dot(const Quaternion3f& other) const
  {
    return data.dot(other.data);
  }

  /// @brief addition
  inline const Eigen::CwiseBinaryOp<
    Eigen::internal::scalar_sum_op <FCL_REAL>, const Vec4f, const Vec4f>
    operator + (const Quaternion3f& other) const
  {
    return Eigen::CwiseBinaryOp< Eigen::internal::scalar_sum_op <FCL_REAL>,
           const Vec4f, const Vec4f>
             (data, other.data);
  }
  inline const Quaternion3f& operator += (const Quaternion3f& other)
  {
    data += other.data;
    return *this;
  }

  /// @brief multiplication
  inline Quaternion3f operator * (const Quaternion3f& other) const
  {
    return Quaternion3f(w() * other.w() - vec().dot(other.vec()),
                        w() * other.vec() + other.w() * vec() + vec().cross(other.vec()));
  }
  const Quaternion3f& operator *= (const Quaternion3f& other);

  /// @brief division
  Quaternion3f operator - () const;

  /// @brief scalar multiplication
  Quaternion3f operator * (FCL_REAL t) const;
  const Quaternion3f& operator *= (FCL_REAL t);

  /// @brief conjugate
  inline Quaternion3f conj() const
  {
    return Quaternion3f (w(), -vec());
  }

  /// @brief inverse
  inline Quaternion3f inverse() const {
    Quaternion3f inv = conj();
    inv.normalize();
    return inv;
  }

  inline void normalize () {
    FCL_REAL n = data.squaredNorm();
    if (n > 0) data *= 1/sqrt(n);
  }

  /// @brief rotate a vector
  template<typename Derived>
  inline const quaternion_transform_return_type<Derived>
  transform(const Eigen::MatrixBase<Derived>& v) const;

  bool operator == (const Quaternion3f& other) const
  {
    return (data == other.data);
  }

  bool operator != (const Quaternion3f& other) const
  {
    return !(*this == other);
  }

  inline FCL_REAL& w() { return data[W]; }
  inline FCL_REAL& x() { return data[X]; }
  inline FCL_REAL& y() { return data[Y]; }
  inline FCL_REAL& z() { return data[Z]; }

  inline const FCL_REAL& w() const { return data[W]; }
  inline const FCL_REAL& x() const { return data[X]; }
  inline const FCL_REAL& y() const { return data[Y]; }
  inline const FCL_REAL& z() const { return data[Z]; }

private:

  FCL_REAL operator [] (std::size_t i) const
  {
    return data[i];
  }

  inline XYZ_t      vec()       { return data.segment<3>(X); }
  inline XYZConst_t vec() const { return data.segment<3>(X); }

  Vec4f data;

  template<typename Derived>
  friend struct quaternion_transform_return_type;
};

template<typename RhsType> struct quaternion_transform_return_type :
  quaternion_transform_return_type_traits<RhsType>::type
{
  typedef quaternion_transform_return_type_traits<RhsType> quat_traits;

  typedef typename quat_traits::type Base;

  EIGEN_GENERIC_PUBLIC_INTERFACE(quaternion_transform_return_type)

  EIGEN_STRONG_INLINE quaternion_transform_return_type(const Quaternion3f& q, const Eigen::MatrixBase<RhsType>& v) :
      Base (2*q.vec().dot(v) * q.vec(), ((q.w()*q.w() - q.vec().dot(q.vec()))*v + 2*q.w()*m_uCrossV)),
      m_uCrossV (q.vec().cross(v))
  {}

  typename quat_traits::Cross_t m_uCrossV;
};

template<typename LhsType, typename RhsType> struct translate_return_type :
  translate_return_type_traits<LhsType, RhsType>::type
{
  typedef translate_return_type_traits<LhsType, RhsType> trans_traits;

  typedef typename trans_traits::type Base;

  EIGEN_GENERIC_PUBLIC_INTERFACE(translate_return_type)

  EIGEN_STRONG_INLINE translate_return_type(const quaternion_transform_return_type<LhsType>& rv, const RhsType& T) :
    Base (rv, T)
  {}
};

template<typename Derived, typename OtherDerived>
const translate_return_type<Derived, OtherDerived>
operator+ (const quaternion_transform_return_type<Derived>& rv,
           const Eigen::MatrixBase<OtherDerived>& T)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived, 3);
  return translate_return_type<Derived, OtherDerived>(rv, T.derived());
}

template<typename Derived>
const quaternion_transform_return_type<Derived>
Quaternion3f::transform (const Eigen::MatrixBase<Derived>& v) const
{
  return quaternion_transform_return_type<Derived> (*this, v.derived());
}

static inline std::ostream& operator << (std::ostream& o, const Quaternion3f& q)
{
  o << "(" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << ")";
  return o;
}
*/

typedef Eigen::Quaternion<FCL_REAL> Quaternion3f;

/// @brief Simple transform class used locally by InterpMotion
class Transform3f
{
  boost::mutex lock_;

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
    q.fromRotation(R_);
  }

  /// @brief Construct transform from rotation and translation
  Transform3f(const Quaternion3f& q_, const Vec3f& T_) : matrix_set(false),
                                                         T(T_),
                                                         q(q_)
  {
  }

  /// @brief Construct transform from rotation
  Transform3f(const Matrix3f& R_) : matrix_set(true), 
                                    R(R_)
  {
    q.fromRotation(R_);
  }

  /// @brief Construct transform from rotation
  Transform3f(const Quaternion3f& q_) : matrix_set(false),
                                        q(q_)
  {
  }

  /// @brief Construct transform from translation
  Transform3f(const Vec3f& T_) : matrix_set(true), 
                                 T(T_)
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
    q.fromRotation(R_);
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
    q.fromRotation(R);
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
    return q.transform(v) + T;
  }

  /// @brief inverse transform
  inline Transform3f& inverse()
  {
    matrix_set = false;
    q = q.conj();
    T = q.transform(-T).eval();
    return *this;
  }

  /// @brief inverse the transform and multiply with another
  inline Transform3f inverseTimes(const Transform3f& other) const
  {
    const Quaternion3f& q_inv = q.conj();
    return Transform3f(q_inv * other.q, q_inv.transform(other.T - T));
  }

  /// @brief multiply with another transform
  inline const Transform3f& operator *= (const Transform3f& other)
  {
    matrix_set = false;
    T += q.transform(other.T).eval();
    q *= other.q;
    return *this;
  }

  /// @brief multiply with another transform
  inline Transform3f operator * (const Transform3f& other) const
  {
    Quaternion3f q_new = q * other.q;
    return Transform3f(q_new, q.transform(other.T) + T);
  }

  /// @brief check whether the transform is identity
  inline bool isIdentity() const
  {
    return q.isIdentity() && T.isZero();
  }

  /// @brief set the transform to be identity transform
  inline void setIdentity()
  {
    R.setIdentity();
    T.setZero();
    q = Quaternion3f();
    matrix_set = true;
  }

  bool operator == (const Transform3f& other) const
  {
    return (q == other.getQuatRotation()) && (T == other.getTranslation());
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

#endif
