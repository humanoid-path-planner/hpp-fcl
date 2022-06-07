/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2021, INRIA
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

#ifndef HPP_FCL_COLLISION_H
#define HPP_FCL_COLLISION_H

#include <hpp/fcl/data_types.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_func_matrix.h>
#include <hpp/fcl/timings.h>

namespace hpp {
namespace fcl {

/// @brief Main collision interface: given two collision objects, and the
/// requirements for contacts, including num of max contacts, whether perform
/// exhaustive collision (i.e., returning returning all the contact points),
/// whether return detailed contact information (i.e., normal, contact point,
/// depth; otherwise only contact primitive id is returned), this function
/// performs the collision between them.
/// Return value is the number of contacts generated between the two objects.
HPP_FCL_DLLAPI std::size_t collide(const CollisionObject* o1,
                                   const CollisionObject* o2,
                                   const CollisionRequest& request,
                                   CollisionResult& result);

/// @copydoc collide(const CollisionObject*, const CollisionObject*, const
/// CollisionRequest&, CollisionResult&)
HPP_FCL_DLLAPI std::size_t collide(const CollisionGeometry* o1,
                                   const Transform3f& tf1,
                                   const CollisionGeometry* o2,
                                   const Transform3f& tf2,
                                   const CollisionRequest& request,
                                   CollisionResult& result);

/// @copydoc collide(const CollisionObject*, const CollisionObject*, const
/// CollisionRequest&, CollisionResult&) \note this function update the initial
/// guess of \c request if requested.
///       See QueryRequest::updateGuess
inline std::size_t collide(const CollisionObject* o1, const CollisionObject* o2,
                           CollisionRequest& request, CollisionResult& result) {
  std::size_t res = collide(o1, o2, (const CollisionRequest&)request, result);
  request.updateGuess(result);
  return res;
}

/// @copydoc collide(const CollisionObject*, const CollisionObject*, const
/// CollisionRequest&, CollisionResult&) \note this function update the initial
/// guess of \c request if requested.
///       See QueryRequest::updateGuess
inline std::size_t collide(const CollisionGeometry* o1, const Transform3f& tf1,
                           const CollisionGeometry* o2, const Transform3f& tf2,
                           CollisionRequest& request, CollisionResult& result) {
  std::size_t res =
      collide(o1, tf1, o2, tf2, (const CollisionRequest&)request, result);
  request.updateGuess(result);
  return res;
}

/// @brief This class reduces the cost of identifying the geometry pair.
/// This is mostly useful for repeated shape-shape queries.
///
/// \code
///   ComputeCollision calc_collision (o1, o2);
///   std::size_t ncontacts = calc_collision(tf1, tf2, request, result);
/// \endcode
class HPP_FCL_DLLAPI ComputeCollision {
 public:
  /// @brief Default constructor from two Collision Geometries.
  ComputeCollision(const CollisionGeometry* o1, const CollisionGeometry* o2);

  std::size_t operator()(const Transform3f& tf1, const Transform3f& tf2,
                         const CollisionRequest& request,
                         CollisionResult& result) const;

  inline std::size_t operator()(const Transform3f& tf1, const Transform3f& tf2,
                                CollisionRequest& request,
                                CollisionResult& result) const {
    std::size_t res = operator()(tf1, tf2, (const CollisionRequest&)request,
                                 result);
    request.updateGuess(result);
    return res;
  }

  bool operator==(const ComputeCollision& other) const {
    return o1 == other.o1 && o2 == other.o2 && solver == other.solver;
  }

  bool operator!=(const ComputeCollision& other) const {
    return !(*this == other);
  }

  virtual ~ComputeCollision(){};

 protected:
  // These pointers are made mutable to let the derived classes to update
  // their values when updating the collision geometry (e.g. creating a new
  // one). This feature should be used carefully to avoid any mis usage (e.g,
  // changing the type of the collision geometry should be avoided).
  mutable const CollisionGeometry* o1;
  mutable const CollisionGeometry* o2;

  mutable GJKSolver solver;

  CollisionFunctionMatrix::CollisionFunc func;
  bool swap_geoms;

  virtual std::size_t run(const Transform3f& tf1, const Transform3f& tf2,
                          const CollisionRequest& request,
                          CollisionResult& result) const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fcl
}  // namespace hpp

#endif
