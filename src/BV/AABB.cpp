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

#include <hpp/fcl/BV/AABB.h>

#include <limits>
#include <hpp/fcl/collision_data.h>

namespace hpp
{
namespace fcl
{

AABB::AABB() : min_(Vec3f::Constant(std::numeric_limits<FCL_REAL>::max())),
               max_(Vec3f::Constant(-std::numeric_limits<FCL_REAL>::max()))
{
}

bool AABB::overlap(const AABB& other, const CollisionRequest& request,
                    FCL_REAL& sqrDistLowerBound) const
{
  const FCL_REAL breakDistance (request.break_distance + request.security_margin);
  const FCL_REAL breakDistance2 = breakDistance * breakDistance;

  sqrDistLowerBound = (min_ - other.max_).array().max(0).matrix().squaredNorm();
  if(sqrDistLowerBound > breakDistance2) return false;

  sqrDistLowerBound = (other.min_ - max_).array().max(0).matrix().squaredNorm();
  if(sqrDistLowerBound > breakDistance2) return false;

  return true;
}

FCL_REAL AABB::distance(const AABB& other, Vec3f* P, Vec3f* Q) const
{
  FCL_REAL result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const FCL_REAL& amin = min_[i];
    const FCL_REAL& amax = max_[i];
    const FCL_REAL& bmin = other.min_[i];
    const FCL_REAL& bmax = other.max_[i];
    
    if(amin > bmax)
    {
      FCL_REAL delta = bmax - amin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amin;
        (*Q)[i] = bmax;
      }
    }
    else if(bmin > amax)
    {
      FCL_REAL delta = amax - bmin;
      result += delta * delta;
      if(P && Q)
      {
        (*P)[i] = amax;
        (*Q)[i] = bmin;
      }
    }
    else
    {
      if(P && Q)
      {
        if(bmin >= amin)
        {
          FCL_REAL t = 0.5 * (amax + bmin);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
        else
        {
          FCL_REAL t = 0.5 * (amin + bmax);
          (*P)[i] = t;
          (*Q)[i] = t;
        }
      }
    }
  }

  return std::sqrt(result);
}

FCL_REAL AABB::distance(const AABB& other) const
{
  FCL_REAL result = 0;
  for(std::size_t i = 0; i < 3; ++i)
  {
    const FCL_REAL& amin = min_[i];
    const FCL_REAL& amax = max_[i];
    const FCL_REAL& bmin = other.min_[i];
    const FCL_REAL& bmax = other.max_[i];
    
    if(amin > bmax)
    {
      FCL_REAL delta = bmax - amin;
      result += delta * delta;
    }
    else if(bmin > amax)
    {
      FCL_REAL delta = amax - bmin;
      result += delta * delta;
    }
  }

  return std::sqrt(result);
}

bool overlap(const Matrix3f& R0, const Vec3f& T0, const AABB& b1, const AABB& b2)
{
  AABB bb1 (translate (rotate (b1, R0), T0));
  return bb1.overlap (b2);
}

bool overlap(const Matrix3f& R0, const Vec3f& T0, const AABB& b1,
	     const AABB& b2, const CollisionRequest& request,
             FCL_REAL& sqrDistLowerBound)
{
  AABB bb1 (translate (rotate (b1, R0), T0));
  return bb1.overlap (b2, request, sqrDistLowerBound);
}

}

} // namespace hpp
