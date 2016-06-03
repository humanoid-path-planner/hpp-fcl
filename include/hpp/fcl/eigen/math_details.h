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

#ifndef FCL_EIGEN_DETAILS_H
#define FCL_EIGEN_DETAILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace fcl
{

namespace details
{

template <typename T>
struct eigen_wrapper_v3
{
  typedef T meta_type;
  typedef Eigen::Matrix <T, 3, 1> vector_type;

  vector_type v;

  eigen_wrapper_v3() { setValue(0); }

  template <typename Derived>
    eigen_wrapper_v3(const Eigen::MatrixBase <Derived>& value) :
      v (value)
  {}

  eigen_wrapper_v3(T x) :
    v (vector_type::Constant (x))
  {}

  eigen_wrapper_v3(T* x) :
    v (vector_type::Constant (*x))
  {}

  eigen_wrapper_v3(T x, T y, T z) :
    v (x, y, z)
  {}

  inline void setValue(T x, T y, T z)
  {
    v << x, y, z;
  }

  inline void setValue(T x)
  {
    v.setConstant (x);
  }

  inline eigen_wrapper_v3<T>& ubound(const eigen_wrapper_v3<T>& u)
  {
    v = v.cwiseMin (u.v);
    return *this;
  }

  inline eigen_wrapper_v3<T>& lbound(const eigen_wrapper_v3<T>& l)
  {
    v = v.cwiseMax (l.v);
    return *this;
  }

  T operator [] (size_t i) const { return v[i]; }
  T& operator [] (size_t i) { return v[i]; }

  inline eigen_wrapper_v3<T> operator + (const eigen_wrapper_v3<T>& other) const { return eigen_wrapper_v3<T>(v + other.v); }
  inline eigen_wrapper_v3<T> operator - (const eigen_wrapper_v3<T>& other) const { return eigen_wrapper_v3<T>(v - other.v); }
  inline eigen_wrapper_v3<T> operator * (const eigen_wrapper_v3<T>& other) const { return eigen_wrapper_v3<T>(v.cwiseProduct (other.v)); }
  inline eigen_wrapper_v3<T> operator / (const eigen_wrapper_v3<T>& other) const { return (eigen_wrapper_v3<T>(v) /= other); }
  inline eigen_wrapper_v3<T>& operator += (const eigen_wrapper_v3<T>& other) { v += other.v; return *this; }
  inline eigen_wrapper_v3<T>& operator -= (const eigen_wrapper_v3<T>& other) { v -= other.v; return *this; }
  inline eigen_wrapper_v3<T>& operator *= (const eigen_wrapper_v3<T>& other) { v = v.cwiseProduct  (other.v); return *this; }
  inline eigen_wrapper_v3<T>& operator /= (const eigen_wrapper_v3<T>& other) { v = v.cwiseQuotient (other.v); return *this; }
  inline eigen_wrapper_v3<T> operator + (T t) const { return eigen_wrapper_v3<T>((v.array() + t).matrix()); }
  inline eigen_wrapper_v3<T> operator - (T t) const { return eigen_wrapper_v3<T>((v.array() - t).matrix()); }
  inline eigen_wrapper_v3<T> operator * (T t) const { return eigen_wrapper_v3<T>(v * t); }
  inline eigen_wrapper_v3<T> operator / (T t) const { return eigen_wrapper_v3<T>(v / t); }
  inline eigen_wrapper_v3<T>& operator += (T t) { v.array() += t; return *this; }
  inline eigen_wrapper_v3<T>& operator -= (T t) { v.array() -= t; return *this; }
  inline eigen_wrapper_v3<T>& operator *= (T t) { v.array() *= t; return *this; }
  inline eigen_wrapper_v3<T>& operator /= (T t) { v.array() /= t; return *this; }
  inline eigen_wrapper_v3<T> operator - () const { return eigen_wrapper_v3<T>(-v); }
};


template <typename T>
static inline eigen_wrapper_v3<T> cross_prod(const eigen_wrapper_v3<T>& l, const eigen_wrapper_v3<T>& r)
{
  return eigen_wrapper_v3<T>(l.v.cross (r.v));
}

template <typename T>
static inline T dot_prod3(const eigen_wrapper_v3<T>& l, const eigen_wrapper_v3<T>& r)
{
  return l.v.dot(r.v);
}

template <typename T>
static inline eigen_wrapper_v3<T> min(const eigen_wrapper_v3<T>& x, const eigen_wrapper_v3<T>& y)
{
  return eigen_wrapper_v3<T>(x.v.cwiseMin (y.v));
}

template <typename T>
static inline eigen_wrapper_v3<T> max(const eigen_wrapper_v3<T>& x, const eigen_wrapper_v3<T>& y)
{
  return eigen_wrapper_v3<T>(x.v.cwiseMax(y.v));
}

template <typename T>
static inline eigen_wrapper_v3<T> abs(const eigen_wrapper_v3<T>& x)
{
  return eigen_wrapper_v3<T>(x.v.cwiseAbs());
}

template <typename T>
static inline bool equal(const eigen_wrapper_v3<T>& x, const eigen_wrapper_v3<T>& y, T epsilon)
{
  return ((x.v - y.v).cwiseAbs ().array () < epsilon).all();
}

namespace internal {
  template <typename Derived, int Size> struct assign {
    static Eigen::Matrix<typename Derived::Scalar, 4, 1> run (const Eigen::MatrixBase <Derived>& value)
    {
      assert (false);
    }
  };

  template <typename Derived> struct assign <Derived, 3> {
    static Eigen::Matrix<typename Derived::Scalar, 4, 1> run (const Eigen::MatrixBase <Derived>& value)
    {
      return (Eigen::Matrix<typename Derived::Scalar, 4, 1> () << value, 0).finished ();
    }
  };

  template <typename Derived> struct assign <Derived, 4> {
    static Eigen::Matrix<typename Derived::Scalar, 4, 1> run (const Eigen::MatrixBase <Derived>& value)
    {
      return value;
    }
  };
}

template <typename T>
struct eigen_wrapper_v4
{
  typedef T meta_type;
  typedef Eigen::Matrix <T, 4, 1> vector_type;

  vector_type v;

  eigen_wrapper_v4() { setValue(0); }

  template <typename Derived>
    eigen_wrapper_v4(const Eigen::MatrixBase <Derived>& value) :
      v (internal::assign <Derived, Derived::SizeAtCompileTime>::run (value))
  {}

  eigen_wrapper_v4(T x) :
    v (vector_type::Constant (x))
  {
    v[3] = 0;
  }

  eigen_wrapper_v4(T* x) :
    v (vector_type::Constant (*x))
  {
    v[3] = 0;
  }

  eigen_wrapper_v4(T x, T y, T z) :
    v (x, y, z, 0)
  {}

  inline typename vector_type::template FixedSegmentReturnType<3>::Type d () { return v.template head <3> (); }

  inline typename vector_type::template ConstFixedSegmentReturnType<3>::Type d () const { return v.template head <3> (); }

  inline void setValue(T x, T y, T z, T w = 0)
  {
    v << x, y, z, w;
  }

  inline void setValue(T x)
  {
    d().setConstant (x);
    v[3] = 0;
  }

  inline eigen_wrapper_v4<T>& ubound(const eigen_wrapper_v4<T>& u)
  {
    v = v.cwiseMin (u.v);
    return *this;
  }

  inline eigen_wrapper_v4<T>& lbound(const eigen_wrapper_v4<T>& l)
  {
    v = v.cwiseMax (l.v);
    return *this;
  }

  T operator [] (size_t i) const { return v[i]; }
  T& operator [] (size_t i) { return v[i]; }

  inline eigen_wrapper_v4<T> operator + (const eigen_wrapper_v4<T>& other) const { return eigen_wrapper_v4<T>(v + other.v); }
  inline eigen_wrapper_v4<T> operator - (const eigen_wrapper_v4<T>& other) const { return eigen_wrapper_v4<T>(v - other.v); }
  inline eigen_wrapper_v4<T> operator * (const eigen_wrapper_v4<T>& other) const { return eigen_wrapper_v4<T>(v.cwiseProduct (other.v)); }
  inline eigen_wrapper_v4<T> operator / (const eigen_wrapper_v4<T>& other) const { return (eigen_wrapper_v4<T>(v) /= other); }
  inline eigen_wrapper_v4<T>& operator += (const eigen_wrapper_v4<T>& other) { v += other.v; return *this; }
  inline eigen_wrapper_v4<T>& operator -= (const eigen_wrapper_v4<T>& other) { v -= other.v; return *this; }
  inline eigen_wrapper_v4<T>& operator *= (const eigen_wrapper_v4<T>& other) { v = v.cwiseProduct  (other.v); return *this; }
  inline eigen_wrapper_v4<T>& operator /= (const eigen_wrapper_v4<T>& other) { d() = d().cwiseQuotient (other.d()); return *this; }
  inline eigen_wrapper_v4<T> operator + (T t) const { return eigen_wrapper_v4<T>((d().array() + t).matrix()); }
  inline eigen_wrapper_v4<T> operator - (T t) const { return eigen_wrapper_v4<T>((d().array() - t).matrix()); }
  inline eigen_wrapper_v4<T> operator * (T t) const { return eigen_wrapper_v4<T>(v * t); }
  inline eigen_wrapper_v4<T> operator / (T t) const { return eigen_wrapper_v4<T>(v / t); }
  inline eigen_wrapper_v4<T>& operator += (T t) { d().array() += t; return *this; }
  inline eigen_wrapper_v4<T>& operator -= (T t) { d().array() -= t; return *this; }
  inline eigen_wrapper_v4<T>& operator *= (T t) { v.array() *= t; return *this; }
  inline eigen_wrapper_v4<T>& operator /= (T t) { v.array() /= t; return *this; }
  inline eigen_wrapper_v4<T> operator - () const { return eigen_wrapper_v4<T>(-v); }
} __attribute__ ((aligned));


template <typename T>
static inline eigen_wrapper_v4<T> cross_prod(const eigen_wrapper_v4<T>& l, const eigen_wrapper_v4<T>& r)
{
  return eigen_wrapper_v4<T>(l.v.cross3 (r.v));
}

template <typename T>
static inline T dot_prod3(const eigen_wrapper_v4<T>& l, const eigen_wrapper_v4<T>& r)
{
  return l.v.dot(r.v);
}

template <typename T>
static inline eigen_wrapper_v4<T> min(const eigen_wrapper_v4<T>& x, const eigen_wrapper_v4<T>& y)
{
  return eigen_wrapper_v4<T>(x.v.cwiseMin (y.v));
}

template <typename T>
static inline eigen_wrapper_v4<T> max(const eigen_wrapper_v4<T>& x, const eigen_wrapper_v4<T>& y)
{
  return eigen_wrapper_v4<T>(x.v.cwiseMax(y.v));
}

template <typename T>
static inline eigen_wrapper_v4<T> abs(const eigen_wrapper_v4<T>& x)
{
  return eigen_wrapper_v4<T>(x.v.cwiseAbs());
}

template <typename T>
static inline bool equal(const eigen_wrapper_v4<T>& x, const eigen_wrapper_v4<T>& y, T epsilon)
{
  return ((x.v - y.v).cwiseAbs ().array () < epsilon).all();
}

template <typename T>
struct eigen_v3 :
  Eigen::Matrix <T, 3, 1>
{
  typedef T meta_type;
  typedef Eigen::Matrix <T, 3, 1> Base;

  eigen_v3(void): Base () { this->setZero (); }

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
    eigen_v3(const Eigen::MatrixBase<OtherDerived>& other)
    : Base(other)
  {}

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
    eigen_v3& operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
      this->Base::operator=(other);
      return *this;
    }

  eigen_v3(T x) :
    Base (x, x, x)
  {}

  eigen_v3(T* x) :
    Base (*x, *x, *x)
  {}

  eigen_v3(T x, T y, T z) :
    Base (x, y, z)
  {}

  inline void setValue(T x, T y, T z)
  {
    this->operator[] (0) = x;
    this->operator[] (1) = y;
    this->operator[] (2) = z;
  }

  inline void setValue(T x)
  {
    this->setConstant (x);
  }

  template<typename OtherDerived>
  inline eigen_v3<T>& ubound(const Eigen::MatrixBase<OtherDerived>& u)
  {
    *this = this->cwiseMin (u);
    return *this;
  }

  template<typename OtherDerived>
  inline eigen_v3<T>& lbound(const Eigen::MatrixBase<OtherDerived>& l)
  {
    *this = this->cwiseMax (l);
    return *this;
  }

  // T operator [] (size_t i) const { return v[i]; }
  // T& operator [] (size_t i) { return v[i]; }

  // inline eigen_wrapper_v3<T> operator + (const eigen_wrapper_v3<T>& other) const { return eigen_wrapper_v3<T>(v + other.v); }
  // inline eigen_wrapper_v3<T> operator - (const eigen_wrapper_v3<T>& other) const { return eigen_wrapper_v3<T>(v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]); }
  // inline eigen_wrapper_v3<T> operator * (const eigen_wrapper_v3<T>& other) const { return eigen_wrapper_v3<T>(v[0] * other.v[0], v[1] * other.v[1], v[2] * other.v[2]); }
  // inline eigen_v3<T> operator / (const eigen_v3<T>& other) const { return eigen_v3<T>(this->array().cwiseQuotient (other)); }
  inline eigen_v3<T>& operator += (const eigen_v3<T>& other) { return static_cast<eigen_v3<T>&>(this->Base::operator+= (other)); }
  inline eigen_v3<T>& operator -= (const eigen_v3<T>& other) { return static_cast<eigen_v3<T>&>(this->Base::operator-= (other)); }
  inline eigen_v3<T>& operator *= (const eigen_v3<T>& other) { return static_cast<eigen_v3<T>&>(this->array().operator*=(other)); }
  inline eigen_v3<T>& operator /= (const eigen_v3<T>& other) { return static_cast<eigen_v3<T>&>(this->array().operator/=(other)); }
  // inline eigen_wrapper_v4<T>& operator /= (const eigen_wrapper_v4<T>& other) {  = d().cwiseQuotient (other.d()); return *this; }
  // inline eigen_wrapper_v3<T>& operator *= (const eigen_wrapper_v3<T>& other) { return this->Base::operator*= (other); }
  // inline eigen_wrapper_v3<T>& operator /= (const eigen_wrapper_v3<T>& other) { return this->Base::operator/= (other); }
  // inline eigen_wrapper_v3<T> operator + (T t) const { return eigen_wrapper_v3<T>(v + t); }
  // inline eigen_wrapper_v3<T> operator - (T t) const { return eigen_wrapper_v3<T>(v - t); }
  // inline eigen_wrapper_v3<T> operator * (T t) const { return eigen_wrapper_v3<T>(v * t); }
  // inline eigen_wrapper_v3<T> operator / (T t) const { return eigen_wrapper_v3<T>(v / t); }
  inline eigen_v3<T>& operator += (T t) { this->array() += t; return *this; }
  inline eigen_v3<T>& operator -= (T t) { this->array() -= t; return *this; }
  inline eigen_v3<T>& operator *= (T t) { this->array() *= t; return *this; }
  inline eigen_v3<T>& operator /= (T t) { this->array() /= t; return *this; }
  // inline eigen_wrapper_v3<T> operator - () const { return eigen_wrapper_v3<T>(-v); }
};


template <typename T>
static inline eigen_v3<T> cross_prod(const eigen_v3<T>& l, const eigen_v3<T>& r)
{
  return l.cross (r);
}

template <typename T>
static inline T dot_prod3(const eigen_v3<T>& l, const eigen_v3<T>& r)
{
  return l.dot(r);
}

template <typename T>
static inline eigen_v3<T> min(const eigen_v3<T>& x, const eigen_v3<T>& y)
{
  return x.cwiseMin (y);
}

template <typename T>
static inline eigen_v3<T> max(const eigen_v3<T>& x, const eigen_v3<T>& y)
{
  return x.cwiseMax(y);
}

template <typename T>
static inline eigen_v3<T> abs(const eigen_v3<T>& x)
{
  return x.cwiseAbs();
}

template <typename T>
static inline bool equal(const eigen_v3<T>& x, const eigen_v3<T>& y, T epsilon)
{
  return ((x - y).cwiseAbs ().array () < epsilon).all();
}

template<typename T>
struct eigen_wrapper_m3
{
  typedef T meta_type;
  typedef eigen_wrapper_v3<T> vector_type;
  typedef Eigen::Matrix <T, 3, 3, Eigen::RowMajor> matrix_type;
  typedef Eigen::Matrix <T, 3, 1> inner_col_type;
  typedef typename matrix_type::ConstRowXpr ConstRowXpr;

  matrix_type m;
  eigen_wrapper_m3() {};

  template <typename Derived>
    eigen_wrapper_m3(const Eigen::MatrixBase <Derived>& matrix) :
      m (matrix)
  {}

  eigen_wrapper_m3(T xx, T xy, T xz,
                   T yx, T yy, T yz,
                   T zx, T zy, T zz)
  {
    setValue(xx, xy, xz,
             yx, yy, yz,
             zx, zy, zz);
  }

  eigen_wrapper_m3(const vector_type& v1, const vector_type& v2, const vector_type& v3)
  {
    m << v1.v, v2.v, v3.v;
  }

  eigen_wrapper_m3(const eigen_wrapper_m3<T>& other) { m = other.m; }

  inline inner_col_type getColumn(size_t i) const { return m.col (i); }
  inline ConstRowXpr getRow(size_t i) const { return m.row (i); }

  inline T operator() (size_t i, size_t j) const { return m(i, j); }
  inline T& operator() (size_t i, size_t j) { return m(i, j); }

  inline vector_type operator * (const vector_type& v) const { return vector_type(m * v.v); }

  inline eigen_wrapper_m3<T> operator * (const eigen_wrapper_m3<T>& other) const { return eigen_wrapper_m3<T>(m * other.m); }
  inline eigen_wrapper_m3<T> operator + (const eigen_wrapper_m3<T>& other) const { return eigen_wrapper_m3<T>(m + other.m); }
  inline eigen_wrapper_m3<T> operator - (const eigen_wrapper_m3<T>& other) const { return eigen_wrapper_m3<T>(m - other.m); }

  inline eigen_wrapper_m3<T> operator + (T c) const { return eigen_wrapper_m3<T>(m + c); }
  inline eigen_wrapper_m3<T> operator - (T c) const { return eigen_wrapper_m3<T>(m - c); }
  inline eigen_wrapper_m3<T> operator * (T c) const { return eigen_wrapper_m3<T>(m * c); }
  inline eigen_wrapper_m3<T> operator / (T c) const { return eigen_wrapper_m3<T>(m / c); }

  inline eigen_wrapper_m3<T>& operator *= (const eigen_wrapper_m3<T>& other) { m *= other.m; return *this; }
  inline eigen_wrapper_m3<T>& operator += (const eigen_wrapper_m3<T>& other) { m += other.m; return *this; }
  inline eigen_wrapper_m3<T>& operator -= (const eigen_wrapper_m3<T>& other) { m -= other.m; return *this; }

  inline eigen_wrapper_m3<T>& operator += (T c) { m.array() += c; return *this; }
  inline eigen_wrapper_m3<T>& operator -= (T c) { m.array() -= c; return *this; }
  inline eigen_wrapper_m3<T>& operator *= (T c) { m.array() *= c; return *this; }
  inline eigen_wrapper_m3<T>& operator /= (T c) { m.array() /= c; return *this; }


  void setIdentity() { m.setIdentity (); }

  void setZero() { m.setZero(); }

  static const eigen_wrapper_m3<T>& getIdentity()
  {
    static const eigen_wrapper_m3<T> I(matrix_type::Identity ());
    return I;
  }

  T determinant() const { return m.determinant (); }

  eigen_wrapper_m3<T>& transpose()
  {
    m.transposeInPlace ();
    return *this;
  }

  eigen_wrapper_m3<T>& inverse()
  {
    m = m.inverse().eval();
    return *this;
  }

  eigen_wrapper_m3<T> transposeTimes(const eigen_wrapper_m3<T>& other) const
  {
    return eigen_wrapper_m3<T>(m.transpose () * other.m);
  }

  eigen_wrapper_m3<T> timesTranspose(const eigen_wrapper_m3<T>& other) const
  {
    return eigen_wrapper_m3<T>(m * other.transpose ());
  }

  vector_type transposeTimes(const vector_type& other) const
  {
    return vector_type(m.transpose () * other.v);
  }

  inline T transposeDotX(const vector_type& other) const
  {
    return m.col(0).dot(other.v);
  }

  inline T transposeDotY(const vector_type& other) const
  {
    return m.col(1).dot(other.v);
  }

  inline T transposeDotZ(const vector_type& other) const
  {
    return m.col(2).dot(other.v);
  }

  inline T transposeDot(size_t i, const vector_type& other) const
  {
    return m.col (i).dot(other.v);
  }

  inline T dotX(const vector_type& other) const
  {
    return m.row (0).dot (other.v);
  }

  inline T dotY(const vector_type& other) const
  {
    return m.row (1).dot (other.v);
  }

  inline T dotZ(const vector_type& other) const
  {
    return m.row (2).dot (other.v);
  }

  inline T dot(size_t i, const vector_type& other) const
  {
    return m.row (i).dot (other.v);
  }

  inline void setValue(T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz)
  {
    m << xx, xy, xz,
         yx, yy, yz,
         zx, zy, zz;
  }

  inline void setValue(T x) { m.setConstant (x); }
};


template<typename T>
eigen_wrapper_m3<T> abs(const eigen_wrapper_m3<T>& m)
{
  return eigen_wrapper_m3<T>(m.m.cwiseAbs ());
}

template<typename T>
eigen_wrapper_m3<T> transpose(const eigen_wrapper_m3<T>& m)
{
  return eigen_wrapper_m3<T>(m.m.transpose ());
}


template<typename T>
eigen_wrapper_m3<T> inverse(const eigen_wrapper_m3<T>& m)
{
  return eigen_wrapper_m3<T> (m.m.inverse().eval ());
}

template<typename T>
struct eigen_m3 :
  Eigen::Matrix <T, 3, 3>
{
  typedef T meta_type;
  typedef eigen_v3<T> vector_type;
  typedef Eigen::Matrix <T, 3, 3> Base;

  typedef typename Base::ColXpr ColXpr;
  typedef typename Base::ConstColXpr ConstColXpr;
  typedef typename Base::RowXpr RowXpr;
  typedef typename Base::ConstRowXpr ConstRowXpr;

  eigen_m3(void): Base () { }

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
    eigen_m3(const Eigen::MatrixBase<OtherDerived>& other)
    : Base(other)
  {}

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
    eigen_m3& operator= (const Eigen::MatrixBase <OtherDerived>& other)
    {
      this->Base::operator=(other);
      return *this;
    }

  eigen_m3(T xx, T xy, T xz,
           T yx, T yy, T yz,
           T zx, T zy, T zz)
  {
    setValue(xx, xy, xz,
             yx, yy, yz,
             zx, zy, zz);
  }

  eigen_m3(const vector_type& v1, const vector_type& v2, const vector_type& v3)
  {
    this->row(1) = v1;
    this->row(2) = v2;
    this->row(3) = v3;
  }

  inline ColXpr col(size_t i) { return this->Base::col (i); }
  inline RowXpr row(size_t i) { return this->Base::row (i); }
  inline ConstColXpr col(size_t i) const { return this->Base::col (i); }
  inline ConstRowXpr row(size_t i) const { return this->Base::row (i); }

  inline eigen_m3<T>& operator *= (const eigen_m3<T>& other) { return static_cast<eigen_m3<T>&>(this->operator*=(other)); }
  inline eigen_m3<T>& operator += (const eigen_m3<T>& other) { return static_cast<eigen_m3<T>&>(this->operator+=(other)); }
  inline eigen_m3<T>& operator -= (const eigen_m3<T>& other) { return static_cast<eigen_m3<T>&>(this->operator-=(other)); }

  inline eigen_m3<T>& operator += (T c) { return static_cast<eigen_m3<T>&>(this->operator+=(c)); }
  inline eigen_m3<T>& operator -= (T c) { return static_cast<eigen_m3<T>&>(this->operator-=(c)); }
  inline eigen_m3<T>& operator *= (T c) { return static_cast<eigen_m3<T>&>(this->operator*=(c)); }
  inline eigen_m3<T>& operator /= (T c) { return static_cast<eigen_m3<T>&>(this->operator/=(c)); }

  static const eigen_m3<T>& getIdentity()
  {
    static const eigen_m3<T> I(Base::Identity ());
    return I;
  }

  eigen_m3<T>& transpose() { this->transposeInPlace (); return *this; }

  eigen_m3<T>& inverse()
  {
    this->Base::operator=(this->inverse ().eval());
    // m = m.inverse().eval();
    return *this;
  }

  template <typename OtherDerived>
    const typename Eigen::ProductReturnType<eigen_m3<T>, OtherDerived>::Type
    transposeTimes(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->Base::transpose() * other;
  }

  template <typename OtherDerived>
    const typename Eigen::ProductReturnType<eigen_m3<T>, OtherDerived>::Type
    timesTranspose(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->operator* (other.Base::transpose());
  }

  template <typename OtherDerived>
  inline T transposeDotX(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->col(0).dot(other);
  }

  template <typename OtherDerived>
  inline T transposeDotY(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->col(1).dot(other);
  }

  template <typename OtherDerived>
  inline T transposeDotZ(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->col(2).dot(other);
  }

  template <typename OtherDerived>
  inline T transposeDot(size_t i, const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->col(i).dot(other);
  }

  template <typename OtherDerived>
  inline T dotX(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->row(0).dot(other);
  }

  template <typename OtherDerived>
  inline T dotY(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->row(1).dot(other);
  }

  template <typename OtherDerived>
  inline T dotZ(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->row(2).dot(other);
  }

  template <typename OtherDerived>
  inline T dot(size_t i, const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->row(i).dot(other);
  }

  inline void setValue(T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz)
  {
    *this << xx, xy, xz,
             yx, yy, yz,
             zx, zy, zz;
  }

  inline void setValue(T x)
  {
    this->setConstant (x);
  }
};


template<typename T>
eigen_m3<T> abs(const eigen_m3<T>& m)
{
  return eigen_m3<T>(m.cwiseAbs ());
}

template<typename T>
eigen_m3<T> transpose(const eigen_m3<T>& m)
{
  return eigen_m3<T>(m.eigen_m3<T>::Base::transpose ());
}


template<typename T>
eigen_m3<T> inverse(const eigen_m3<T>& m)
{
  return eigen_m3<T> (m.eigen_m3<T>::Base::inverse().eval ());
}

}

}

#endif
