//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2014, CNRS-LAAS
//  Copyright (c) 2022-2024, Inria
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

#include <cassert>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <hpp/fcl/config.hh>
#include <hpp/fcl/deprecated.hh>
#include <hpp/fcl/warning.hh>

#if _WIN32
#define HPP_FCL_PRETTY_FUNCTION __FUNCSIG__
#else
#define HPP_FCL_PRETTY_FUNCTION __PRETTY_FUNCTION__
#endif

#define HPP_FCL_UNUSED_VARIABLE(var) (void)(var)

#ifdef NDEBUG
#define HPP_FCL_ONLY_USED_FOR_DEBUG(var) HPP_FCL_UNUSED_VARIABLE(var)
#else
#define HPP_FCL_ONLY_USED_FOR_DEBUG(var)
#endif

#define HPP_FCL_THROW_PRETTY(message, exception)              \
  {                                                           \
    std::stringstream ss;                                     \
    ss << "From file: " << __FILE__ << "\n";                  \
    ss << "in function: " << HPP_FCL_PRETTY_FUNCTION << "\n"; \
    ss << "at line: " << __LINE__ << "\n";                    \
    ss << "message: " << message << "\n";                     \
    throw exception(ss.str());                                \
  }

#ifdef HPP_FCL_TURN_ASSERT_INTO_EXCEPTION
#define HPP_FCL_ASSERT(check, message, exception) \
  do {                                            \
    if (!(check)) {                               \
      HPP_FCL_THROW_PRETTY(message, exception);   \
    }                                             \
  } while (0)
#else
#define HPP_FCL_ASSERT(check, message, exception) \
  {                                               \
    HPP_FCL_UNUSED_VARIABLE(exception(message));  \
    assert((check) && message);                   \
  }
#endif

#if (__cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1600))
#define HPP_FCL_WITH_CXX11_SUPPORT
#endif

#if defined(__GNUC__) || defined(__clang__)
#define HPP_FCL_COMPILER_DIAGNOSTIC_PUSH _Pragma("GCC diagnostic push")
#define HPP_FCL_COMPILER_DIAGNOSTIC_POP _Pragma("GCC diagnostic pop")
#define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#if !defined(__has_warning) || __has_warning("-Wmaybe-uninitialized")
  #define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED \
    _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
#else
  #define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
#endif
#elif defined(WIN32)
#define HPP_FCL_COMPILER_DIAGNOSTIC_PUSH _Pragma("warning(push)")
#define HPP_FCL_COMPILER_DIAGNOSTIC_POP _Pragma("warning(pop)")
#define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS \
  _Pragma("warning(disable : 4996)")
#define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED \
  _Pragma("warning(disable : 4700)")
#else
#define HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
#define HPP_FCL_COMPILER_DIAGNOSTIC_POP
#define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
#define HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_MAYBE_UNINITIALIZED
#endif  // __GNUC__

namespace hpp {
namespace fcl {
using std::dynamic_pointer_cast;
using std::make_shared;
using std::shared_ptr;

class CollisionObject;
typedef shared_ptr<CollisionObject> CollisionObjectPtr_t;
typedef shared_ptr<const CollisionObject> CollisionObjectConstPtr_t;
class CollisionGeometry;
typedef shared_ptr<CollisionGeometry> CollisionGeometryPtr_t;
typedef shared_ptr<const CollisionGeometry> CollisionGeometryConstPtr_t;
class Transform3f;

class AABB;

class BVHModelBase;
typedef shared_ptr<BVHModelBase> BVHModelPtr_t;

class OcTree;
typedef shared_ptr<OcTree> OcTreePtr_t;
typedef shared_ptr<const OcTree> OcTreeConstPtr_t;
}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_FWD_HH
