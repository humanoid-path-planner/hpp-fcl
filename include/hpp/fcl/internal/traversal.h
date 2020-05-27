/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, LAAS CNRS
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

#ifndef HPP_FCL_TRAVERSAL_DETAILS_TRAVERSAL_H
#define HPP_FCL_TRAVERSAL_DETAILS_TRAVERSAL_H

/// @cond INTERNAL

namespace hpp
{
namespace fcl
{

enum {
  RelativeTransformationIsIdentity = 1
};

namespace details
{
  template <bool enabled>
  struct HPP_FCL_DLLAPI RelativeTransformation
  {
    RelativeTransformation () : R (Matrix3f::Identity()) {}

    const Matrix3f& _R () const { return R; }
    const Vec3f   & _T () const { return T; }

    Matrix3f R;
    Vec3f T;
  };

  template <>
  struct HPP_FCL_DLLAPI RelativeTransformation <false>
  {
    static const Matrix3f& _R () { throw std::logic_error ("should never reach this point"); }
    static const Vec3f   & _T () { throw std::logic_error ("should never reach this point"); }
  };
} // namespace details

}

} // namespace hpp

/// @endcond

#endif
