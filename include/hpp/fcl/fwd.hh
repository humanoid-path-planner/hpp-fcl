//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2014, CNRS-LAAS
//  Author: Florent Lamiraux
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of CNRS-LAAS nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//

#ifndef HPP_FCL_FWD_HH
#define HPP_FCL_FWD_HH

#include <boost/shared_ptr.hpp>
#include <sstream>

#include <hpp/fcl/config.hh>

#if _WIN32
  #define HPP_FCL_PRETTY_FUNCTION __FUNCSIG__
#else
  #define HPP_FCL_PRETTY_FUNCTION __PRETTY_FUNCTION__
#endif

#define HPP_FCL_UNUSED_VARIABLE(var) (void)(var)

#define HPP_FCL_THROW_PRETTY(message,exception) \
{ \
std::stringstream ss; \
ss << "From file: " << __FILE__ << "\n"; \
ss << "in function: " << HPP_FCL_PRETTY_FUNCTION << "\n"; \
ss << "at line: " << __LINE__ << "\n"; \
ss << "message: " << message << "\n"; \
throw exception(ss.str()); \
}

#if (__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
  #define HPP_FCL_WITH_CXX11_SUPPORT
#endif

namespace hpp {
namespace fcl {
  using boost::shared_ptr;

  class CollisionObject;
  typedef shared_ptr <CollisionObject> CollisionObjectPtr_t;
  typedef shared_ptr < const CollisionObject> CollisionObjectConstPtr_t;
  class CollisionGeometry;
  typedef shared_ptr <CollisionGeometry> CollisionGeometryPtr_t;
  typedef shared_ptr <const CollisionGeometry>
  CollisionGeometryConstPtr_t;
  class Transform3f;

  class AABB;

  class BVHModelBase;
  typedef shared_ptr<BVHModelBase> BVHModelPtr_t;

  class OcTree;
  typedef shared_ptr<OcTree> OcTreePtr_t;
  typedef shared_ptr<const OcTree> OcTreeConstPtr_t;
}
} // namespace hpp

#endif // HPP_FCL_FWD_HH
