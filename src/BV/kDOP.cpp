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

#include <hpp/fcl/BV/kDOP.h>
#include <limits>
#include <iostream>

#include <hpp/fcl/collision_data.h>

namespace hpp
{
namespace fcl
{

/// @brief Find the smaller and larger one of two values
inline void minmax(FCL_REAL a, FCL_REAL b, FCL_REAL& minv, FCL_REAL& maxv)
{
  if(a > b)
  {
    minv = b;
    maxv = a;
  }
  else
  {
    minv = a;
    maxv = b;
  }
}
/// @brief Merge the interval [minv, maxv] and value p/
inline void minmax(FCL_REAL p, FCL_REAL& minv, FCL_REAL& maxv)
{
  if(p > maxv) maxv = p;
  if(p < minv) minv = p;
}


/// @brief Compute the distances to planes with normals from KDOP vectors except those of AABB face planes
template<short N>
void getDistances(const Vec3f& p, FCL_REAL* d) {}

/// @brief Specification of getDistances
template<>
inline void getDistances<5>(const Vec3f& p, FCL_REAL* d)
{
  d[0] = p[0] + p[1];
  d[1] = p[0] + p[2];
  d[2] = p[1] + p[2];
  d[3] = p[0] - p[1];
  d[4] = p[0] - p[2];
}

template<>
inline void getDistances<6>(const Vec3f& p, FCL_REAL* d)
{
  d[0] = p[0] + p[1];
  d[1] = p[0] + p[2];
  d[2] = p[1] + p[2];
  d[3] = p[0] - p[1];
  d[4] = p[0] - p[2];
  d[5] = p[1] - p[2];
}

template<>
inline void getDistances<9>(const Vec3f& p, FCL_REAL* d)
{
  d[0] = p[0] + p[1];
  d[1] = p[0] + p[2];
  d[2] = p[1] + p[2];
  d[3] = p[0] - p[1];
  d[4] = p[0] - p[2];
  d[5] = p[1] - p[2];
  d[6] = p[0] + p[1] - p[2];
  d[7] = p[0] + p[2] - p[1];
  d[8] = p[1] + p[2] - p[0];
}

template<short N>
KDOP<N>::KDOP()
{
  FCL_REAL real_max = std::numeric_limits<FCL_REAL>::max();
  dist_.template head<N/2>().setConstant( real_max);
  dist_.template tail<N/2>().setConstant(-real_max);
}

template<short N>
KDOP<N>::KDOP(const Vec3f& v)
{
  for(short i = 0; i < 3; ++i)
  {
    dist_[i] = dist_[N / 2 + i] = v[i];
  }

  FCL_REAL d[(N - 6) / 2];
  getDistances<(N - 6) / 2>(v, d);
  for(short i = 0; i < (N - 6) / 2; ++i)
  {
    dist_[3 + i] = dist_[3 + i + N / 2] = d[i];
  }
}

template<short N>
KDOP<N>::KDOP(const Vec3f& a, const Vec3f& b)
{
  for(short i = 0; i < 3; ++i)
  {
    minmax(a[i], b[i], dist_[i], dist_[i + N / 2]);
  }

  FCL_REAL ad[(N - 6) / 2], bd[(N - 6) / 2];
  getDistances<(N - 6) / 2>(a, ad);
  getDistances<(N - 6) / 2>(b, bd);
  for(short i = 0; i < (N - 6) / 2; ++i)
  {
    minmax(ad[i], bd[i], dist_[3 + i], dist_[3 + i + N / 2]);
  }
}
  
template<short N>
bool KDOP<N>::overlap(const KDOP<N>& other) const
{
  if ((dist_.template head<N/2>() > other.dist_.template tail<N/2>()).any()) return false;
  if ((dist_.template tail<N/2>() < other.dist_.template head<N/2>()).any()) return false;
  return true;
}

template<short N>
bool KDOP<N>::overlap(const KDOP<N>& other, const CollisionRequest& request,
    FCL_REAL& sqrDistLowerBound) const
{
  const FCL_REAL breakDistance (request.break_distance + request.security_margin);

  FCL_REAL a = (dist_.template head<N/2>() - other.dist_.template tail<N/2>()).minCoeff();
  if (a > breakDistance) {
    sqrDistLowerBound = a*a;
    return false;
  }

  FCL_REAL b = (other.dist_.template head<N/2>() - dist_.template tail<N/2>()).minCoeff();
  if (b > breakDistance) {
    sqrDistLowerBound = b*b;
    return false;
  }

  sqrDistLowerBound = std::min(a, b);
  return true;
}

template<short N>
bool KDOP<N>::inside(const Vec3f& p) const
{
  if ((p.array() < dist_.template head<3>()).any()) return false;
  if ((p.array() > dist_.template segment<3>(N/2)).any()) return false;

  enum { P = ((N-6)/2) };
  Eigen::Array<FCL_REAL, P, 1> d;
  getDistances<P>(p, d.data());

  if ((d < dist_.template segment<P>(3    )).any()) return false;
  if ((d > dist_.template segment<P>(3+N/2)).any()) return false;

  return true;
}

template<short N>
KDOP<N>& KDOP<N>::operator += (const Vec3f& p)
{
  for(short i = 0; i < 3; ++i)
  {
    minmax(p[i], dist_[i], dist_[N / 2 + i]);
  }
    
  FCL_REAL pd[(N - 6) / 2];
  getDistances<(N - 6) / 2>(p, pd);
  for(short i = 0; i < (N - 6) / 2; ++i)
  {
    minmax(pd[i], dist_[3 + i], dist_[3 + N / 2 + i]);
  }

  return *this;
}

template<short N>
KDOP<N>& KDOP<N>::operator += (const KDOP<N>& other)
{
  for(short i = 0; i < N / 2; ++i)
  {
    dist_[i] = std::min(other.dist_[i], dist_[i]);
    dist_[i + N / 2] = std::max(other.dist_[i + N / 2], dist_[i + N / 2]);
  }
  return *this;
}

template<short N>
KDOP<N> KDOP<N>::operator + (const KDOP<N>& other) const
{
  KDOP<N> res(*this);
  return res += other;
}


template<short N>
FCL_REAL KDOP<N>::distance(const KDOP<N>& /*other*/, Vec3f* /*P*/, Vec3f* /*Q*/) const
{
  std::cerr << "KDOP distance not implemented!" << std::endl;
  return 0.0;
}


template<short N>
KDOP<N> translate(const KDOP<N>& bv, const Vec3f& t)
{
  KDOP<N> res(bv);
  for(short i = 0; i < 3; ++i)
  {
    res.dist(i) += t[i];
    res.dist(short(N / 2 + i)) += t[i];
  }

  FCL_REAL d[(N - 6) / 2];
  getDistances<(N - 6) / 2>(t, d);
  for(short i = 0; i < (N - 6) / 2; ++i)
  {
    res.dist(short(3 + i)) += d[i];
    res.dist(short(3 + i + N / 2)) += d[i];
  }

  return res;
}


template class KDOP<16>;
template class KDOP<18>;
template class KDOP<24>;

template KDOP<16> translate<16>(const KDOP<16>&, const Vec3f&);
template KDOP<18> translate<18>(const KDOP<18>&, const Vec3f&);
template KDOP<24> translate<24>(const KDOP<24>&, const Vec3f&);

}

} // namespace hpp
