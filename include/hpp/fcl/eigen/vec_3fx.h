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

/** \author Joseph Mirabel */

#ifndef FCL_EIGEN_VEC_3F_H
#define FCL_EIGEN_VEC_3F_H

#include <hpp/fcl/config-fcl.hh>
#include <hpp/fcl/data_types.h>

#include <hpp/fcl/eigen/math_details.h>

#include <cmath>
#include <iostream>
#include <limits>


namespace fcl
{

/// @brief Vector3 class wrapper. The core data is in the template parameter class.
template <typename T>
class Vec3fX :
  Eigen::Matrix <T, 3, 1>
{
public:
  typedef Eigen::Matrix <T, 3, 1> Base;

  Vec3fX(void): Base() {}

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
    Vec3fX(const Eigen::MatrixBase<OtherDerived>& other)
    : Base(other)
    {}

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
    Vec3fX& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
      this->Base::operator=(other);
      return *this;
    }

  /// @brief create Vector (x, y, z)
  Vec3fX(T x, T y, T z) : Base(x, y, z) {}

  /// @brief create vector (x, x, x)
  Vec3fX(U x) : Base(Base::Constant (x)) {}

  /// @brief create vector using the internal data type
  // Vec3fX(const T& data_) : data(data_) {}

  // inline U operator [] (size_t i) const { return data[i]; }
  // inline U& operator [] (size_t i) { return data[i]; }

  // inline Vec3fX operator + (const Vec3fX& other) const { return Vec3fX(data + other.data); }
  // inline Vec3fX operator - (const Vec3fX& other) const { return Vec3fX(data - other.data); }
  // inline Vec3fX operator * (const Vec3fX& other) const { return Vec3fX(data * other.data); }
  // inline Vec3fX operator / (const Vec3fX& other) const { return Vec3fX(data / other.data); }
  // inline Vec3fX& operator += (const Vec3fX& other) { data += other.data; return *this; }
  // inline Vec3fX& operator -= (const Vec3fX& other) { data -= other.data; return *this; }
  // inline Vec3fX& operator *= (const Vec3fX& other) { data *= other.data; return *this; }
  // inline Vec3fX& operator /= (const Vec3fX& other) { data /= other.data; return *this; }
  // inline Vec3fX operator + (U t) const { return Vec3fX(data + t); }
  // inline Vec3fX operator - (U t) const { return Vec3fX(data - t); }
  // inline Vec3fX operator * (U t) const { return Vec3fX(data * t); }
  // inline Vec3fX operator / (U t) const { return Vec3fX(data / t); }
  // inline Vec3fX& operator += (U t) { data += t; return *this; }
  // inline Vec3fX& operator -= (U t) { data -= t; return *this; }
  // inline Vec3fX& operator *= (U t) { data *= t; return *this; }
  // inline Vec3fX& operator /= (U t) { data /= t; return *this; }
  // inline Vec3fX operator - () const { return Vec3fX(-data); }
  // inline Vec3fX cross(const Vec3fX& other) const { return Vec3fX(details::cross_prod(data, other.data)); }
  // inline U dot(const Vec3fX& other) const { return details::dot_prod3(data, other.data); }
  inline Vec3fX& normalize()
  {
    T sqr_length = this->squaredNorm();
    if(sqr_length > 0) this->operator/= ((T)sqrt(sqr_length));
    return *this;
  }

  inline Vec3fX& normalize(bool* signal)
  {
    T sqr_length = this->squaredNorm();
    if(sqr_length > 0)
    {
      this->operator/= ((T)sqrt(sqr_length));
      *signal = true;
    }
    else
      *signal = false;
    return *this;
  }

  inline Vec3fX& abs() 
  {
    *this = this->cwiseAbs ();
    return *this;
  }

  inline T length() const { return this->norm(); }
  // inline T norm() const { return sqrt(details::dot_prod3(data, data)); }
  inline T sqrLength() const { return this->squaredNorm(); }
  // inline T squaredNorm() const { return details::dot_prod3(data, data); }
  inline void setValue(T x, T y, T z) { m_storage.data() = { x, y, z } }
  inline void setValue(T x) { this->setConstant (x); }
  // inline void setZero () {data.setValue (0); }
  inline bool equal(const Vec3fX& other, T epsilon = std::numeric_limits<U>::epsilon() * 100) const
  {
    return ((*this - other).cwiseAbs().array () < epsilon).all();
  }
  inline Vec3fX<T>& negate() { *this = -*this; return *this; }

  bool operator == (const Vec3fX& other) const
  {
    return equal(other, 0);
  }

  bool operator != (const Vec3fX& other) const
  {
    return !(*this == other);
  }


  inline Vec3fX<T>& ubound(const Vec3fX<T>& u)
  {
    *this = this->cwiseMin (u);
    return *this;
  }

  inline Vec3fX<T>& lbound(const Vec3fX<T>& l)
  {
    *this = this->cwiseMax (l);
    return *this;
  }

  bool isZero() const
  {
    return (m_storage.data()[0] == 0)
      &&   (m_storage.data()[1] == 0)
      &&   (m_storage.data()[2] == 0);
  }

};

template<typename T>
static inline Vec3fX<T> normalize(const Vec3fX<T>& v)
{
  typename T::meta_type sqr_length = v.squaredNorm ();
  if(sqr_length > 0)
    return v / (typename T::meta_type)sqrt(sqr_length);
  else
    return v;
}

template <typename T>
static inline typename T::meta_type triple(const Vec3fX<T>& x, const Vec3fX<T>& y, const Vec3fX<T>& z)
{
  return x.dot(y.cross(z));
}

template <typename T>
std::ostream& operator << (std::ostream& out, const Vec3fX<T>& x)
{
  out << x[0] << " " << x[1] << " " << x[2];
  return out;
}

template <typename T>
// static inline Vec3fX<T> min(const Vec3fX<T>& x, const Vec3fX<T>& y)
static inline
const Eigen::CwiseBinaryOp <Eigen::internal::scalar_min_op<T>, const Vec3fX<T>, const Vec3fX<T> >
 min(const Vec3fX<T>& x, const Vec3fX<T>& y)
{
  return x.cwiseMin (y);
}

template <typename T>
// static inline Vec3fX<T> max(const Vec3fX<T>& x, const Vec3fX<T>& y)
static inline
const Eigen::CwiseBinaryOp<Eigen::internal::scalar_max_op<T>, const Vec3fX<T>, const Vec3fX<T> >
max(const Vec3fX<T>& x, const Vec3fX<T>& y)
{
  return x.cwiseMax (y);
}

template <typename T>
// static inline Vec3fX<T> abs(const Vec3fX<T>& x)
static inline
const Eigen::CwiseUnaryOp<Eigen::internal::scalar_abs_op<T>, const Vec3fX<T> >
abs(const Vec3fX<T>& x)
{
  return x.cwiseAbs ();
}

template <typename T>
void generateCoordinateSystem(const Vec3fX<T>& w, Vec3fX<T>& u, Vec3fX<T>& v)
{
  typedef typename T::meta_type U;
  U inv_length;
  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = (U)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (U)0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (U)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (U)0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }
}

 // template <typename T>
   // inline Vec3fX <T> operator * (const typename Vec3fX <T>::U& t,
				 // const Vec3fX <T>& v)
   // {
     // return Vec3fX <T> (v.data * t);
   // }


}


#endif
