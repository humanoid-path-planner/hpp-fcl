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

#ifndef FCL_EIGEN_MATRIX_3F_H
#define FCL_EIGEN_MATRIX_3F_H

namespace fcl
{

/// @brief Matrix2 class wrapper. the core data is in the template parameter class
template<typename T>
class Matrix3fX :
  public Eigen::Matrix <T, 3, 3, Eigen::RowMajor>
{
public:
  typedef Eigen::Matrix <T, 3, 3, Eigen::RowMajor> Base;
  typedef typename Base::ColXpr ColXpr;
  typedef typename Base::ConstColXpr ConstColXpr;
  typedef typename Base::RowXpr RowXpr;
  typedef typename Base::ConstRowXpr ConstRowXpr;

  
  Matrix3fX(void): Base() {}

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
    Matrix3fX(const Eigen::MatrixBase<OtherDerived>& other)
    : Base(other)
    {}

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
    Matrix3fX& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
      this->Base::operator=(other);
      return *this;
    }

  Matrix3fX(T xx, T xy, T xz,
            T yx, T yy, T yz,
            T zx, T zy, T zz)
    : Base((Base () << xx, xy, xz, yx, yy, yz, zx, zy, zz).finished ())
  {}

  template <typename OtherDerived>
  Matrix3fX(const Eigen::MatrixBase<OtherDerived>& v1,
            const Eigen::MatrixBase<OtherDerived>& v2,
            const Eigen::MatrixBase<OtherDerived>& v3)
    : Base((Base () << v1, v2, v3).finished ())
  {}

  inline ColXpr getColumn(size_t i) { return this->col(i); }
  inline RowXpr getRow(size_t i)    { return this->row(i); }
  inline ConstColXpr getColumn(size_t i) const { return this->col(i); }
  inline ConstRowXpr getRow(size_t i)    const { return this->row(i); }

/*
  inline U operator () (size_t i, size_t j) const
  {
    return data(i, j);
  }

  inline U& operator () (size_t i, size_t j)
  {
    return data(i, j);
  }

  inline Vec3fX<S> operator * (const Vec3fX<S>& v) const
  {
    return Vec3fX<S>(data * v.data);
  }

  inline Matrix3fX<T> operator * (const Matrix3fX<T>& m) const
  {
    return Matrix3fX<T>(data * m.data);
  }

  inline Matrix3fX<T> operator + (const Matrix3fX<T>& other) const
  {
    return Matrix3fX<T>(data + other.data);
  }

  inline Matrix3fX<T> operator - (const Matrix3fX<T>& other) const
  {
    return Matrix3fX<T>(data - other.data);
  }

  inline Matrix3fX<T> operator + (U c) const
  {
    return Matrix3fX<T>(data + c);
  }

  inline Matrix3fX<T> operator - (U c) const
  {
    return Matrix3fX<T>(data - c);
  }

  inline Matrix3fX<T> operator * (U c) const
  {
    return Matrix3fX<T>(data * c);
  }

  inline Matrix3fX<T> operator / (U c) const
  {
    return Matrix3fX<T>(data / c);
  }

  inline Matrix3fX<T>& operator *= (const Matrix3fX<T>& other)
  {
    data *= other.data;
    return *this;
  }

  inline Matrix3fX<T>& operator += (const Matrix3fX<T>& other)
  {
    data += other.data;
    return *this;
  }

  inline Matrix3fX<T>& operator -= (const Matrix3fX<T>& other)
  {
    data -= other.data;
    return *this;
  }

  inline Matrix3fX<T>& operator += (U c) 
  {
    data += c;
    return *this;
  }

  inline Matrix3fX<T>& operator -= (U c)
  {
    data -= c;
    return *this;
  }

  inline Matrix3fX<T>& operator *= (U c)
  {
    data *= c;
    return *this;
  }

  inline Matrix3fX<T>& operator /= (U c)
  {
    data /= c;
    return *this;
  }

  inline void setIdentity()
  {
    data.setIdentity();
  }
  // */

  inline bool isIdentity () const
  {
    return
      data (0,0) == 1 && data (0,1) == 0 && data (0,2) == 0 &&
      data (1,0) == 0 && data (1,1) == 1 && data (1,2) == 0 &&
      data (2,0) == 0 && data (2,1) == 0 && data (2,2) == 1;
  }

  /*
  inline void setZero()
  {
    data.setZero();
  }
  // */

  /// @brief Set the matrix from euler angles YPR around ZYX axes
  /// @param eulerX Roll about X axis
  /// @param eulerY Pitch around Y axis
  /// @param eulerZ Yaw aboud Z axis
  ///  
  /// These angles are used to produce a rotation matrix. The euler
  /// angles are applied in ZYX order. I.e a vector is first rotated 
  /// about X then Y and then Z
  inline void setEulerZYX(FCL_REAL eulerX, FCL_REAL eulerY, FCL_REAL eulerZ)
  {
    FCL_REAL ci(cos(eulerX));
    FCL_REAL cj(cos(eulerY));
    FCL_REAL ch(cos(eulerZ));
    FCL_REAL si(sin(eulerX));
    FCL_REAL sj(sin(eulerY));
    FCL_REAL sh(sin(eulerZ));
    FCL_REAL cc = ci * ch;
    FCL_REAL cs = ci * sh;
    FCL_REAL sc = si * ch;
    FCL_REAL ss = si * sh;

    setValue(cj * ch, sj * sc - cs, sj * cc + ss,
             cj * sh, sj * ss + cc, sj * cs - sc, 
             -sj,     cj * si,      cj * ci);

  }

  /// @brief Set the matrix from euler angles using YPR around YXZ respectively
  /// @param yaw Yaw about Y axis
  /// @param pitch Pitch about X axis
  /// @param roll Roll about Z axis 
  void setEulerYPR(FCL_REAL yaw, FCL_REAL pitch, FCL_REAL roll)
  {
    setEulerZYX(roll, pitch, yaw);
  }

  /*
  inline U determinant() const
  {
    return data.determinant();
  }
  // */

  Matrix3fX<T>& transpose() 
  {
    this->transposeInPlace();
    return *this;
  }

  Matrix3fX<T>& inverse()
  {
    *this = this->inverse().eval ();
    return *this;
  }

  Matrix3fX<T>& abs()
  {
    *this = this->cwiseAbs ();
    return *this;
  }

  static const Matrix3fX<T>& getIdentity()
  {
    static const Matrix3fX<T> I((T)1, (T)0, (T)0,
                                (T)0, (T)1, (T)0,
                                (T)0, (T)0, (T)1);
    return I;
  }

  template<typename OtherDerived>
    const typename Eigen::ProductReturnType< Eigen::Transpose <Matrix3fX<T> >,
                                             OtherDerived>::Type
   transposeTimes(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return this->Base::transpose() * other;
  }

  template<typename OtherDerived>
    const typename Eigen::ProductReturnType< Matrix3fX<T>,
                                             Eigen::Transpose <OtherDerived> >
    ::Type timesTranspose(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return *this * other.transpose ();
  }

  /*
  Vec3fX<S> transposeTimes(const Vec3fX<S>& v) const
  {
    return Vec3fX<S>(data.transposeTimes(v.data));
  }
  // */

  template<typename OtherDerived>
    const Eigen::ProductReturnType<
      Eigen::ProductReturnType< Matrix3fX<T>,
                                OtherDerived>::Type,
      Eigen::Transpose < Matrix3fX<T> >
        > ::Type
  tensorTransform(const Eigen::MatrixBase<OtherDerived>& m) const
  {
    return (*this * m) * this->Base::transpose ();
    // Matrix3fX<T> res(*this);
    // res *= m;
    // return res.timesTranspose(*this);
  }

  // (1 0 0)^T (*this)^T v
  template<typename OtherDerived>
  inline T transposeDotX(const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return transposeDot(0, v);
  }

  // (0 1 0)^T (*this)^T v
  template<typename OtherDerived>
  inline T transposeDotY(const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return transposeDot(1, v);
  }

  // (0 0 1)^T (*this)^T v
  template<typename OtherDerived>
  inline T transposeDotZ(const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return transposeDot(2, v);
  }

  // (\delta_{i3})^T (*this)^T v
  template<typename OtherDerived>
  inline T transposeDot(size_t i, const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return this->col(i).dot(v);
  }

  // (1 0 0)^T (*this) v
  template<typename OtherDerived>
  inline T dotX(const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return dot(0, v);
  }

  // (0 1 0)^T (*this) v
  template<typename OtherDerived>
  inline T dotY(const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return dot(1, v);
  }

  // (0 0 1)^T (*this) v
  template<typename OtherDerived>
  inline T dotZ(const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return dot(2, v);
  }

  // (\delta_{i3})^T (*this) v
  template<typename OtherDerived>
  inline T dot(size_t i, const Eigen::MatrixBase<OtherDerived>& v) const
  {
    return this->row(i).dot(v);
  }

  inline void setValue(T xx, T xy, T xz,
                       T yx, T yy, T yz,
                       T zx, T zy, T zz)
  {
    (*this)(0,0) = xx; (*this)(0,1) = xy; (*this)(0,2) = xz;
    (*this)(1,0) = yx; (*this)(1,1) = yy; (*this)(1,2) = yz;
    (*this)(2,0) = zx; (*this)(2,1) = zy; (*this)(2,2) = zz;
  }

  inline void setValue(T x)
  {
    this->setConstant (x);
  }
};

template<typename T, typename OtherDerived>
void hat(Matrix3fX<T>& mat, const Eigen::MatrixBase<OtherDerived>& vec)
{
  mat.setValue(0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0);
}

template<typename T, typename OtherDerived>
void relativeTransform(const Matrix3fX<T>& R1, const Eigen::MatrixBase<OtherDerived>& t1,
                       const Matrix3fX<T>& R2, const Eigen::MatrixBase<OtherDerived>& t2,
                       Matrix3fX<T>& R, Eigen::MatrixBase<OtherDerived>& t)
{
  R = R1.transposeTimes(R2);
  t = R1.transposeTimes(t2 - t1);
}

/// @brief compute the eigen vector and eigen vector of a matrix. dout is the eigen values, vout is the eigen vectors
template<typename T, typename Derived>
void eigen(const Matrix3fX<T>& m, T dout[3], const Eigen::MatrixBase<Derived> vout[3])
{
  Matrix3fX<T> R(m);
  int n = 3;
  int j, iq, ip, i;
  T tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  T b[3];
  T z[3];
  T v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  T d[3];

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

template<typename T>
const typename CwiseUnaryOp<internal::scalar_abs_op<Scalar>, const Matrix3fX<T> >
abs(const Matrix3fX<T>& R) 
{
  return R.cwiseAbs ();
}

template<typename T>
typename Eigen::Transpose <Matrix3fX<T> > transpose(const Matrix3fX<T>& R)
{
  return R.Matrix3fX<T>::Base::transpose ();
}

template<typename T>
Matrix3fX<T> inverse(const Matrix3fX<T>& R)
{
  return R.Matrix3fX<T>::Base::inverse ().eval ();
}

template<typename T, typename Derived>
T quadraticForm(const Matrix3fX<T>& R, const Eigen::MatrixBase<Derived>& v)
{
  return v.dot(R * v);
}

}



#endif
