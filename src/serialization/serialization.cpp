/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
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

/** \author Justin Carpentier */

#include "hpp/fcl/serialization/fwd.h"

using namespace hpp::fcl;

#include "hpp/fcl/serialization/transform.h"
#include "hpp/fcl/serialization/collision_data.h"
#include "hpp/fcl/serialization/geometric_shapes.h"
#include "hpp/fcl/serialization/convex.h"
#include "hpp/fcl/serialization/hfield.h"
#include "hpp/fcl/serialization/BVH_model.h"
#ifdef HPP_FCL_HAS_OCTOMAP
#include "hpp/fcl/serialization/octree.h"
#endif

HPP_FCL_SERIALIZATION_DEFINE_EXPORT(CollisionRequest)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(CollisionResult)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(DistanceRequest)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(DistanceResult)

HPP_FCL_SERIALIZATION_DEFINE_EXPORT(ShapeBase)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(CollisionGeometry)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(TriangleP)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Box)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Sphere)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Ellipsoid)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Capsule)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Cone)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Cylinder)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Halfspace)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Plane)

#define EXPORT_AND_CAST(Derived, Base)               \
  HPP_FCL_SERIALIZATION_CAST_REGISTER(Derived, Base) \
  HPP_FCL_SERIALIZATION_DEFINE_EXPORT(Derived)       \
  /**/

HPP_FCL_SERIALIZATION_CAST_REGISTER(ConvexBase, CollisionGeometry)
EXPORT_AND_CAST(Convex<Triangle>, ConvexBase)
EXPORT_AND_CAST(Convex<Quadrilateral>, ConvexBase)

HPP_FCL_SERIALIZATION_DEFINE_EXPORT(HeightField<AABB>)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(HeightField<OBB>)
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(HeightField<OBBRSS>)

HPP_FCL_SERIALIZATION_CAST_REGISTER(BVHModelBase, CollisionGeometry)

EXPORT_AND_CAST(BVHModel<AABB>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<OBB>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<RSS>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<OBBRSS>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<kIOS>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<KDOP<16>>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<KDOP<18>>, BVHModelBase)
EXPORT_AND_CAST(BVHModel<KDOP<24>>, BVHModelBase)

#ifdef HPP_FCL_HAS_OCTOMAP
HPP_FCL_SERIALIZATION_DEFINE_EXPORT(OcTree)
#endif
