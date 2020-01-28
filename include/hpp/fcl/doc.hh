//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2014 CNRS-LAAS
//  Author: Florent Lamiraux
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

namespace hpp
{
namespace fcl
{

/// \mainpage
/// \anchor hpp_fcl_documentation
///
/// \section hpp_fcl_introduction Introduction
///
/// hpp-fcl is a modified version the FCL libraries.
///
/// It is a library for collision detection and distance computation between
/// various types of geometric shapes reprensented either by
/// \li basic shapes (hpp::fcl::ShapeBase) like box, sphere, cylinders, ...
/// \li or by bounding volume hierarchies of various types (hpp::fcl::BVHModel)
///
/// \par Using hpp-fcl
///
/// The main entry points to the library are functions
/// \li hpp::fcl::collide(const CollisionObject*, const CollisionObject*, const CollisionRequest&, CollisionResult&)
/// \li hpp::fcl::distance(const CollisionObject*, const CollisionObject*, const DistanceRequest&, DistanceResult&)
///
/// \section hpp_fcl_collision_and_distance_lower_bound_computation Collision detection and distance lower bound
///
/// Collision queries can return a distance lower bound between the two objects,
/// which is computationally cheaper than computing the real distance.
/// The following figure shows the returned result in
/// CollisionResult::distance_lower_bound and DistanceResult::min_distance,
/// with respect to the objects real distance.
///
/// \image html doc/distance_computation.png
///
/// The two parameters refer to CollisionRequest::security_margin and
/// CollisionRequest::break_distance.
/// \note In the green hatched area, the distance lower bound is not known. It
/// is only guaranted that it will be inferior to
/// <em>distance - security_margin</em> and superior to \em break_distance.

} // namespace fcl
} // namespace hpp
