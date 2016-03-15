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

#define FCL_EIGEN_EXPOSE_PARENT_TYPE(Type) typedef typename Base::Type Type;

#define FCL_EIGEN_MAKE_CWISE_BINARY_OP(METHOD,FUNCTOR) \
      template <typename OtherDerived> \
      EIGEN_STRONG_INLINE const FclOp<const CwiseBinaryOp<FUNCTOR<Scalar>, const FCL_EIGEN_CURRENT_CLASS, const OtherDerived> > \
      (METHOD) (const MatrixBase<OtherDerived>& other) const \
      { \
        return FclOp <const CwiseBinaryOp<FUNCTOR<Scalar>, const FCL_EIGEN_CURRENT_CLASS, const OtherDerived> > (*this, other.derived()); \
      }

#define FCL_EIGEN_MAKE_CWISE_UNARY_OP(METHOD,FUNCTOR) \
      EIGEN_STRONG_INLINE const FclOp<const CwiseUnaryOp<FUNCTOR<Scalar>, const FCL_EIGEN_CURRENT_CLASS> > \
      (METHOD) (const Scalar& scalar) const \
      { \
        return FclOp <const CwiseUnaryOp<FUNCTOR<Scalar>, const FCL_EIGEN_CURRENT_CLASS> > (*this, FUNCTOR<Scalar>(scalar)); \
      }

#define FCL_EIGEN_RENAME_PARENT_METHOD(OLD,NEW,RETTYPE) \
      template <typename OtherDerived> \
      EIGEN_STRONG_INLINE RETTYPE (METHOD) () \
      { \
        return (this->Base::METHOD) (); \
      }

#define FCL_EIGEN_MAKE_EXPOSE_PARENT1(METHOD) \
      template <typename OtherDerived> \
      EIGEN_STRONG_INLINE FclMatrix& (METHOD) (const MatrixBase<OtherDerived>& other) \
      { \
        (this->Base::METHOD)(other); return *this; \
      }

#define FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY1(METHOD) \
      template <typename OtherDerived> \
      EIGEN_STRONG_INLINE FclMatrix& (METHOD) (const MatrixBase<OtherDerived>& other) \
      { \
        (this->array().METHOD)(other.derived().array()); return *this; \
      }

#define FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY_SCALAR1(METHOD) \
      EIGEN_STRONG_INLINE FCL_EIGEN_CURRENT_CLASS& (METHOD) (const Scalar& other) \
      { \
        (this->array().METHOD)(other); return *this; \
      }

#define FCL_EIGEN_MAKE_GET_COL_ROW() \
  EIGEN_STRONG_INLINE FclOp<ColXpr> getColumn (size_t i) { return FclOp<ColXpr>(*this, i); } \
  EIGEN_STRONG_INLINE FclOp<RowXpr> getRow    (size_t i) { return FclOp<RowXpr>(*this, i); } \
  EIGEN_STRONG_INLINE FclOp<ConstColXpr> getColumn (size_t i) const { return FclOp<ConstColXpr>(*this, i); } \
  EIGEN_STRONG_INLINE FclOp<ConstRowXpr> getRow    (size_t i) const { return FclOp<ConstRowXpr>(*this, i); }

#define FCL_EIGEN_MATRIX_DOT_AXIS(NAME,axis,index) \
  template<typename OtherDerived> \
  EIGEN_STRONG_INLINE Scalar NAME##axis (const MatrixBase<OtherDerived>& other) const \
  { return this->NAME (index, other); }

#define FCL_EIGEN_MATRIX_DOT(NAME,FUNC) \
  template<typename OtherDerived> \
  EIGEN_STRONG_INLINE Scalar NAME(size_t i, const MatrixBase<OtherDerived>& other) const \
  { \
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(FCL_EIGEN_CURRENT_CLASS, 3, 3); \
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(OtherDerived, 3); \
    return this->FUNC(i).dot(other); \
  } \
  FCL_EIGEN_MATRIX_DOT_AXIS(NAME,X,0)\
  FCL_EIGEN_MATRIX_DOT_AXIS(NAME,Y,1)\
  FCL_EIGEN_MATRIX_DOT_AXIS(NAME,Z,2)

namespace Eigen {
  template <typename T> class FclOp;
  template <typename T, int Cols, int Options> class FclMatrix;

#include <hpp/fcl/eigen/product.h>

  namespace internal {

    template<typename T> struct traits< FclOp<T> > : traits <T> {};
    // template<typename T>
      // struct traits< FclOp<T> >
      // {
        // typedef T Scalar;
        // typedef FclOp<T> This;
        // typedef traits<typename This::Base> traits_base;
        // typedef typename traits_base::StorageKind StorageKind;
        // typedef typename traits_base::Index Index;
        // typedef typename traits_base::XprKind XprKind;
        // enum {
          // RowsAtCompileTime = traits_base::RowsAtCompileTime,
          // ColsAtCompileTime = traits_base::ColsAtCompileTime,
          // MaxRowsAtCompileTime = traits_base::MaxRowsAtCompileTime,
          // MaxColsAtCompileTime = traits_base::MaxColsAtCompileTime,
          // Flags = traits_base::Flags,
          // CoeffReadCost = traits_base::CoeffReadCost
        // };
      // };
    template<typename T, int Cols, int _Options>
      struct traits< FclMatrix<T, Cols, _Options> >
      {
        typedef T Scalar;
        typedef FclMatrix<T, Cols, _Options> This;
        typedef traits<typename This::Base> traits_base;
        typedef typename traits_base::StorageKind StorageKind;
        typedef typename traits_base::Index Index;
        typedef typename traits_base::XprKind XprKind;
        enum {
          RowsAtCompileTime = traits_base::RowsAtCompileTime,
          ColsAtCompileTime = traits_base::ColsAtCompileTime,
          MaxRowsAtCompileTime = traits_base::MaxRowsAtCompileTime,
          MaxColsAtCompileTime = traits_base::MaxColsAtCompileTime,
          Flags = traits_base::Flags,
          CoeffReadCost = traits_base::CoeffReadCost,
          Options = traits_base::Options,
          InnerStrideAtCompileTime = 1,
          OuterStrideAtCompileTime = traits_base::OuterStrideAtCompileTime
        };
      };
  }

  template <typename Derived>
    struct UnaryReturnType {
      typedef typename Derived::Scalar Scalar;
      typedef typename Derived::PlainObject Normalize;
      typedef FclOp <
        const CwiseUnaryOp<internal::scalar_opposite_op<Scalar>, const Derived>
        > Opposite;
      typedef FclOp <
        const CwiseUnaryOp<internal::scalar_abs_op<Scalar>, const Derived>
        > Abs;
    };

  template <typename Derived, typename OtherDerived>
    struct BinaryReturnType {
      typedef typename Derived::Scalar Scalar;
      typedef FclOp <
        const CwiseBinaryOp<internal::scalar_difference_op<Scalar>,
              const Derived, const OtherDerived>
                > Difference;
      // static inline const Difference difference (const Derived& l, const OtherDerived& r)
      // { return Difference (l, r, internal::scalar_difference_op<Scalar>()); }
      typedef FclOp <
        const CwiseBinaryOp<internal::scalar_sum_op<Scalar>,
              const Derived, const OtherDerived>
                > Sum;
      typedef FclOp <
        const CwiseBinaryOp<internal::scalar_min_op<Scalar>,
              const Derived, const OtherDerived>
                > Min;
      typedef FclOp <
        const CwiseBinaryOp<internal::scalar_max_op<Scalar>,
              const Derived, const OtherDerived>
                > Max;
    };

#define FCL_EIGEN_CURRENT_CLASS FclMatrix

/// @brief Vector3 class wrapper. The core data is in the template parameter class.
template <typename T, int Cols, int _Options>
class FclMatrix : public Matrix <T, 3, Cols, _Options>
{
public:
  typedef Matrix <T, 3, Cols, _Options> Base;
  FCL_EIGEN_EXPOSE_PARENT_TYPE(Scalar)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(ColXpr)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(RowXpr)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(ConstColXpr)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(ConstRowXpr)

  FclMatrix(void): Base() {}

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
    FclMatrix(const MatrixBase<OtherDerived>& other)
    : Base(other)
    {}

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
    FclMatrix& operator=(const MatrixBase <OtherDerived>& other)
    {
      this->Base::operator=(other);
      return *this;
    }

  /// @brief create Vector (x, y, z)
  FclMatrix(T x, T y, T z) : Base(x, y, z) {}

  FclMatrix(T xx, T xy, T xz,
            T yx, T yy, T yz,
            T zx, T zy, T zz) : Base()
  {
    setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
  }

  /// @brief create vector (x, x, x)
  FclMatrix(T x) : Base(Base::Constant (x)) {}

  Base& base () { return *this; }
  const Base& base () const { return *this; }

  /// @brief create vector using the internal data type
  // Vec3fX(const T& data_) : data(data_) {}

  // inline U operator [] (size_t i) const { return data[i]; }
  // inline U& operator [] (size_t i) { return data[i]; }

  // FclOp<typename Base::ColXpr> getColumn (size_t i) { return FclOp<typename Base::ColXpr>(*this, i); }
  // FclOp<typename Base::RowXpr> getRow    (size_t i) { return FclOp<typename Base::RowXpr>(*this, i); }
  // FclOp<typename Base::ColXpr> getColumn (size_t i) { return FclOp<typename Base::ColXpr>(*this, i); }
  // FclOp<typename Base::RowXpr> getRow    (size_t i) { return FclOp<typename Base::RowXpr>(*this, i); }
  FCL_EIGEN_MAKE_GET_COL_ROW()
  FCL_EIGEN_MATRIX_DOT(dot,row)
  FCL_EIGEN_MATRIX_DOT(transposeDot,col)

  FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator+,internal::scalar_sum_op)
  FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator-,internal::scalar_difference_op)
  // Map this to scalar product or matrix product depending on the Cols.
  // FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator*,internal::scalar_product_op)
  FCL_EIGEN_MAKE_PRODUCT_OPERATOR()
  FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator/,internal::scalar_quotient_op)
  FCL_EIGEN_MAKE_EXPOSE_PARENT1(operator+=)
  FCL_EIGEN_MAKE_EXPOSE_PARENT1(operator-=)
  FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY1(operator*=)
  FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY1(operator/=)
  FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator+,internal::scalar_add_op)
  // This operator cannot be implement with the macro
  // FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator-,internal::scalar_difference_op)
  EIGEN_STRONG_INLINE const FclOp<const CwiseUnaryOp<internal::scalar_add_op<Scalar>, const FclMatrix> >
  operator- (const Scalar& scalar) const
  {
    return FclOp <const CwiseUnaryOp<internal::scalar_add_op<Scalar>, const FclMatrix> > (*this, internal::scalar_add_op<Scalar>(-scalar));
  }
  FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator*,internal::scalar_multiple_op)
  FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator/,internal::scalar_quotient1_op)
  FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY_SCALAR1(operator+=)
  FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY_SCALAR1(operator-=)
  FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY_SCALAR1(operator*=)
  FCL_EIGEN_MAKE_EXPOSE_PARENT_ARRAY_SCALAR1(operator/=)
  inline const typename UnaryReturnType<FclMatrix>::Opposite operator - () const { return typename UnaryReturnType<FclMatrix>::Opposite(*this); }
  // There is no class for cross 
  // inline Vec3fX cross(const Vec3fX& other) const { return Vec3fX(details::cross_prod(data, other.data)); }
  // inline U dot(const Vec3fX& other) const { return details::dot_prod3(data, other.data); }
  inline FclMatrix& normalize()
  {
    T sqr_length = this->squaredNorm();
    if(sqr_length > 0) this->operator/= ((T)std::sqrt(sqr_length));
    return *this;
  }

  inline FclMatrix& normalize(bool* signal)
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

  inline FclMatrix& abs() 
  {
    *this = this->cwiseAbs ();
    return *this;
  }

  inline T length() const { return this->norm(); }
  // inline T norm() const { return sqrt(details::dot_prod3(data, data)); }
  inline T sqrLength() const { return this->squaredNorm(); }
  // inline T squaredNorm() const { return details::dot_prod3(data, data); }
  inline void setValue(T x, T y, T z) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(FclMatrix, 3);
    this->m_storage.data()[0] = x;
    this->m_storage.data()[1] = y;
    this->m_storage.data()[2] = z;
  }
  inline void setValue(T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(FclMatrix, 3, 3);
    this->operator()(0,0) = xx; this->operator()(0,1) = xy; this->operator()(0,2) = xz;
    this->operator()(1,0) = yx; this->operator()(1,1) = yy; this->operator()(1,2) = yz;
    this->operator()(2,0) = zx; this->operator()(2,1) = zy; this->operator()(2,2) = zz;
  }
  inline void setValue(T x) { this->setConstant (x); }
  // inline void setZero () {data.setValue (0); }
  inline bool equal(const FclMatrix& other, T epsilon = std::numeric_limits<T>::epsilon() * 100) const
  {
    return ((*this - other).cwiseAbs().array () < epsilon).all();
  }
  inline FclMatrix& negate() { *this = -*this; return *this; }

  bool operator == (const FclMatrix& other) const
  {
    return equal(other, 0);
  }

  bool operator != (const FclMatrix& other) const
  {
    return !(*this == other);
  }

  inline FclMatrix& ubound(const FclMatrix& u)
  {
    *this = this->cwiseMin (u);
    return *this;
  }

  inline FclMatrix& lbound(const FclMatrix& l)
  {
    *this = this->cwiseMax (l);
    return *this;
  }

  bool isZero() const
  {
    return (this->m_storage.data()[0] == 0)
      &&   (this->m_storage.data()[1] == 0)
      &&   (this->m_storage.data()[2] == 0);
  }

  FclMatrix& transpose () {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Base, 3, 3);
    this->transposeInPlace();
    return *this;
  }

  FclMatrix& inverse()
  {
    this->Base::operator=(this->Base::inverse ().eval());
    return *this;
  }

  template<typename OtherDerived>
  EIGEN_STRONG_INLINE const typename FclProduct<const FclMatrix,const OtherDerived>::TransposeTimesType
  transposeTimes (const MatrixBase<OtherDerived>& other) const
  {
    const Transpose<const FclMatrix> t (*this);
    return typename FclProduct<const FclMatrix,const OtherDerived>::TransposeTimesType (t, other.derived());
  }

  template<typename OtherDerived>
  EIGEN_STRONG_INLINE const typename FclProduct<const FclMatrix,const OtherDerived>::TimesTransposeType
  timesTranspose (const MatrixBase<OtherDerived>& other) const
  {
    const Transpose<const OtherDerived> t (other.derived());
    return typename FclProduct<const FclMatrix,const OtherDerived>::TimesTransposeType (*this, t);
  }

  template<typename OtherDerived>
  EIGEN_STRONG_INLINE const typename FclProduct<const FclMatrix,const OtherDerived>::TensorTransformType
  tensorTransform(const MatrixBase<OtherDerived>& other) const
  {
    const Transpose<const FclMatrix> t (*this);
    const typename FclProduct<const FclMatrix,const OtherDerived>::ProductType left (*this, other.derived());
    return typename FclProduct<const FclMatrix,const OtherDerived>::TensorTransformType (left, t);
  }
};

#undef FCL_EIGEN_CURRENT_CLASS
#define FCL_EIGEN_CURRENT_CLASS FclOp

template <typename EigenOp>
class FclOp : public EigenOp
{
public:
  typedef typename internal::traits<EigenOp>::Scalar T;
  typedef EigenOp Base;
  FCL_EIGEN_EXPOSE_PARENT_TYPE(Scalar)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(ColXpr)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(RowXpr)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(ConstColXpr)
  FCL_EIGEN_EXPOSE_PARENT_TYPE(ConstRowXpr)

  // template<typename Lhs, typename Rhs, typename BinaryOp>
  // FclOp (const Lhs& lhs, const Rhs& rhs, const BinaryOp& bop)
    // : Base (lhs, rhs, bop) {}

  template<typename Lhs, typename Rhs>
  FclOp (Lhs& lhs, const Rhs& rhs)
    : Base (lhs, rhs) {}

  template<typename XprType>
  FclOp (XprType& xpr)
    : Base (xpr) {}

  Base& base () { return *this; }
  const Base& base () const { return *this; }

  // template<typename Lhs, typename Rhs, typename BinaryOp>
  // FclOp (const CwiseBinaryOp<BinaryOp, Lhs, Rhs>& o)
    // : Base (o.lhs(), o.rhs(), o.functor()) {}

  FCL_EIGEN_MAKE_GET_COL_ROW()
  FCL_EIGEN_MATRIX_DOT(dot,row)
  FCL_EIGEN_MATRIX_DOT(transposeDot,col)

  FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator+,internal::scalar_sum_op)
  FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator-,internal::scalar_difference_op)
  // Map this to scalar product or matrix product depending on the Cols.
  // FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator*,internal::scalar_product_op)
  FCL_EIGEN_MAKE_PRODUCT_OPERATOR()
  FCL_EIGEN_MAKE_CWISE_BINARY_OP(operator/,internal::scalar_quotient_op)
  FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator+,internal::scalar_add_op)
  // This operator cannot be implement with the macro
  // FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator-,internal::scalar_difference_op)
  EIGEN_STRONG_INLINE const FclOp<const CwiseUnaryOp<internal::scalar_add_op<Scalar>, const FclOp> >
  operator- (const Scalar& scalar) const
  {
    return FclOp <const CwiseUnaryOp<internal::scalar_add_op<Scalar>, const FclOp> > (*this, internal::scalar_add_op<Scalar>(-scalar));
  }
  FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator*,internal::scalar_multiple_op)
  FCL_EIGEN_MAKE_CWISE_UNARY_OP(operator/,internal::scalar_quotient1_op)

  inline const typename UnaryReturnType<EigenOp>::Normalize
    normalize() const
  {
    return this->normalized ();
    // T sqr_length = this->squaredNorm();
    // if(sqr_length > 0) CwiseExpose (*this /= ((T)sqrt(sqr_length)));
    // return *this;
  }

  /*
  inline Vec3fX<T> normalize(bool* signal)
  {
    T sqr_length = this->squaredNorm();
    if(sqr_length > 0)
    {
      *this /= ((T)sqrt(sqr_length));
      *signal = true;
    }
    else
      *signal = false;
    return *this;
  }
  // */

  inline const typename UnaryReturnType<EigenOp>::Abs
    abs() const
  {
    return typename UnaryReturnType<EigenOp>::Abs (this->derived());
  }

  inline T length() const { return this->norm(); }
  // inline T norm() const { return sqrt(details::dot_prod3(data, data)); }
  inline T sqrLength() const { return this->squaredNorm(); }
  // inline T squaredNorm() const { return details::dot_prod3(data, data); }
  /* Useless
  inline void setValue(T x, T y, T z) {
    this->m_storage.data()[0] = x;
    this->m_storage.data()[1] = y;
    this->m_storage.data()[2] = z;
  }
  inline void setValue(T x) { this->setConstant (x); }
  inline void setZero () {data.setValue (0); }
  //*/

  template <typename Derived>
  inline bool equal(const Eigen::MatrixBase<Derived>& other, T epsilon = std::numeric_limits<T>::epsilon() * 100) const
  {
    return ((*this - other).cwiseAbs().array () < epsilon).all();
  }

  /*
  inline const typename Eigen::CwiseUnaryOp<Eigen::internal::scalar_opposite_op<T>, const EigenOp>
    negate() { return this->operator-(); }
  // */

  template <typename Derived>
  bool operator == (const Eigen::MatrixBase<Derived>& other) const
  {
    return equal(other, 0);
  }

  template <typename Derived>
  bool operator != (const Eigen::MatrixBase<Derived>& other) const
  {
    return !(*this == other);
  }

  // Not in place.
  FCL_EIGEN_MAKE_CWISE_BINARY_OP(ubound,internal::scalar_min_op)
  FCL_EIGEN_MAKE_CWISE_BINARY_OP(lbound,internal::scalar_max_op)

  bool isZero() const
  {
    return this->isZero ();
    // (this->m_storage.data()[0] == 0)
      // &&   (this->m_storage.data()[1] == 0)
      // &&   (this->m_storage.data()[2] == 0);
  }
  // */

  const FclOp<Transpose<const FclOp> > transpose () const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(EigenOp, 3, 3);
    return FclOp<Transpose<const FclOp> >(*this);
  }

  // const FclOp<internal::inverse_impl<FclOp> > inverse () const { return FclOp<Transpose<FclOp> >(*this); }
};
}

namespace fcl
{
// #if 0
template<typename T, int _Options>
static inline Eigen::FclMatrix<T, 1, _Options> normalize(const Eigen::FclMatrix<T, 1, _Options>& v)
{
  T sqr_length = v.squaredNorm ();
  if(sqr_length > 0)
    return v / (T)sqrt(sqr_length);
  else
    return v;
}

template<typename T, int _Options>
static inline T triple(
    const Eigen::FclMatrix<T, 1, _Options>& x,
    const Eigen::FclMatrix<T, 1, _Options>& y,
    const Eigen::FclMatrix<T, 1, _Options>& z)
{
  return x.dot(y.cross(z));
}

// template<typename T, int _Options>
// std::ostream& operator << (std::ostream& out, const Eigen::FclMatrix<T>& x)
// {
  // out << x[0] << " " << x[1] << " " << x[2];
  // return out;
// }

template<typename T, int _Options>
static inline const typename Eigen::BinaryReturnType<
  const Eigen::FclMatrix<T, 1, _Options>, const Eigen::FclMatrix<T, 1, _Options>
  >::Min
 min(const Eigen::FclMatrix<T, 1, _Options>& x, const Eigen::FclMatrix<T, 1, _Options>& y)
{
  return typename Eigen::BinaryReturnType<const Eigen::FclMatrix<T, 1, _Options>, const Eigen::FclMatrix<T, 1, _Options> >::Min (x, y);
}

template<typename T, int _Options>
static inline const typename Eigen::BinaryReturnType<
  const Eigen::FclMatrix<T, 1, _Options>, const Eigen::FclMatrix<T, 1, _Options>
  >::Max
 max(const Eigen::FclMatrix<T, 1, _Options>& x, const Eigen::FclMatrix<T, 1, _Options>& y)
{
  return typename Eigen::BinaryReturnType<const Eigen::FclMatrix<T, 1, _Options>, const Eigen::FclMatrix<T, 1, _Options> >::Max (x, y);
}

template<typename T, int _Cols, int _Options>
// static inline Vec3fX<T> abs(const Vec3fX<T>& x)
static inline const typename Eigen::UnaryReturnType<const Eigen::FclMatrix<T, _Cols, _Options> >
abs(const Eigen::FclMatrix<T, _Cols, _Options>& x)
{
  return typename Eigen::UnaryReturnType<const Eigen::FclMatrix<T, _Cols, _Options> >::Abs (x);
}

template<typename T, int _Options>
void generateCoordinateSystem(
    const Eigen::FclMatrix<T, 1, _Options>& w,
    const Eigen::FclMatrix<T, 1, _Options>& u,
    const Eigen::FclMatrix<T, 1, _Options>& v)
{
  T inv_length;
  if(std::abs(w[0]) >= std::abs(w[1]))
  {
    inv_length = (T)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = (T)0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = (T)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = (T)0;
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

// #endif

/* ----- Start Matrices ------ */
template<typename Matrix, typename Vector>
void hat(Matrix& mat, const Vector& vec)
{
  mat.setValue(0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0);
}

template<typename Matrix, typename Vector>
void relativeTransform(const Matrix& R1, const Vector& t1,
                       const Matrix& R2, const Vector& t2,
                       Matrix& R, Vector& t)
{
  R = R1.transposeTimes(R2);
  t = R1.transposeTimes(t2 - t1);
}

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the eigen values, vout is the eigen vectors
template<typename Matrix, typename Vector>
void eigen(const Matrix& m, typename Matrix::Scalar dout[3], Vector vout[3])
{
  typedef typename Matrix::Scalar Scalar;
  Matrix R(m);
  int n = 3;
  int j, iq, ip, i;
  Scalar tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  Scalar b[3];
  Scalar z[3];
  Scalar v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Scalar d[3];

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = d[ip] = R(ip, ip);
    z[ip] = 0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += std::abs(R(ip, iq));
    if(sm == 0.0)
    {
      vout[0].setValue(v[0][0], v[0][1], v[0][2]);
      vout[1].setValue(v[1][0], v[1][1], v[1][2]);
      vout[2].setValue(v[2][0], v[2][1], v[2][2]);
      dout[0] = d[0]; dout[1] = d[1]; dout[2] = d[2];
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * std::abs(R(ip, iq));
        if(i > 3 &&
           std::abs(d[ip]) + g == std::abs(d[ip]) &&
           std::abs(d[iq]) + g == std::abs(d[iq]))
          R(ip, iq) = 0.0;
        else if(std::abs(R(ip, iq)) > tresh)
        {
          h = d[iq] - d[ip];
          if(std::abs(h) + g == std::abs(h)) t = (R(ip, iq)) / h;
          else
          {
            theta = 0.5 * h / (R(ip, iq));
            t = 1.0 /(std::abs(theta) + std::sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / std::sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * R(ip, iq);
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          R(ip, iq) = 0.0;
          for(j = 0; j < ip; ++j) { g = R(j, ip); h = R(j, iq); R(j, ip) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = R(ip, j); h = R(j, iq); R(ip, j) = g - s * (h + g * tau); R(j, iq) = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = R(ip, j); h = R(iq, j); R(ip, j) = g - s * (h + g * tau); R(iq, j) = h + s * (g - h * tau); }
          for(j = 0; j < n; ++j) { g = v[j][ip]; h = v[j][iq]; v[j][ip] = g - s * (h + g * tau); v[j][iq] = h + s * (g - h * tau); }
          nrot++;
        }
      }
    }
    for(ip = 0; ip < n; ++ip)
    {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  std::cerr << "eigen: too many iterations in Jacobi transform." << std::endl;

  return;
}

// template<typename >
// Matrix abs(const Matrix& R) 
// {
  // return R.abs());
// }

template<typename T, int _Options>
Eigen::FclOp<Eigen::Transpose<const Eigen::FclMatrix<T,3,_Options> > > transpose(const Eigen::FclMatrix<T, 3, _Options>& R)
{
  return Eigen::FclOp<Eigen::Transpose<const Eigen::FclMatrix<T,3,_Options> > > (R);
}

template<typename T, int _Options>
Eigen::FclMatrix<T,3,_Options> inverse(const Eigen::FclMatrix<T, 3, _Options>& R)
{
  return R.inverse();
}

template<typename Matrix, typename Vector>
typename Matrix::Scalar quadraticForm(const Matrix& R, const Vector& v)
{
  return v.dot(R * v);
}


}


#endif
