//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2022 NRIA
//  Author: Justin Carpentier
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
//   * Neither the name of CNRS-LAAS. nor the names of its
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

#ifndef HPP_FCL_PYTHON_BROADPHASE_BROADPHASE_DYNAMIC_AABB_TREE_HH
#define HPP_FCL_PYTHON_BROADPHASE_BROADPHASE_DYNAMIC_AABB_TREE_HH

#include <eigenpy/eigenpy.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>

#include "../fcl.hh"

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
//#include "doxygen_autodoc/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#endif

namespace hpp { namespace fcl {

struct DynamicAABBTreeCollisionManagerWrapper
{
  static void expose()
  {
    bp::class_<DynamicAABBTreeCollisionManager, bp::bases<BroadPhaseCollisionManager> >("DynamicAABBTreeCollisionManager",bp::no_init)
    .def(dv::init<DynamicAABBTreeCollisionManager>())
    ;
  }
}; // BroadPhaseCollisionManagerWrapper

}}

#endif // ifndef HPP_FCL_PYTHON_BROADPHASE_BROADPHASE_DYNAMIC_AABB_TREE_HH
