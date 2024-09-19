/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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

/** @author Jia Pan */

#ifndef COAL_BROAD_PHASE_INTERVAL_TREE_H
#define COAL_BROAD_PHASE_INTERVAL_TREE_H

#include <deque>
#include <map>

#include "coal/broadphase/broadphase_collision_manager.h"
#include "coal/broadphase/detail/interval_tree.h"

namespace coal {

/// @brief Collision manager based on interval tree
class COAL_DLLAPI IntervalTreeCollisionManager
    : public BroadPhaseCollisionManager {
 public:
  typedef BroadPhaseCollisionManager Base;
  using Base::getObjects;

  IntervalTreeCollisionManager();

  ~IntervalTreeCollisionManager();

  /// @brief remove one object from the manager
  void registerObject(CollisionObject* obj);

  /// @brief add one object to the manager
  void unregisterObject(CollisionObject* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  virtual void update();

  /// @brief update the manager by explicitly given the object updated
  void update(CollisionObject* updated_obj);

  /// @brief update the manager by explicitly given the set of objects update
  void update(const std::vector<CollisionObject*>& updated_objs);

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<CollisionObject*>& objs) const;

  /// @brief perform collision test between one object and all the objects
  /// belonging to the manager
  void collide(CollisionObject* obj, CollisionCallBackBase* callback) const;

  /// @brief perform distance computation between one object and all the objects
  /// belonging to the manager
  void distance(CollisionObject* obj, DistanceCallBackBase* callback) const;

  /// @brief perform collision test for the objects belonging to the manager
  /// (i.e., N^2 self collision)
  void collide(CollisionCallBackBase* callback) const;

  /// @brief perform distance test for the objects belonging to the manager
  /// (i.e., N^2 self distance)
  void distance(DistanceCallBackBase* callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseCollisionManager* other_manager,
               CollisionCallBackBase* callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseCollisionManager* other_manager,
                DistanceCallBackBase* callback) const;

  /// @brief whether the manager is empty
  bool empty() const;

  /// @brief the number of objects managed by the manager
  size_t size() const;

 protected:
  /// @brief SAP end point
  /// @brief SAP end point
  struct COAL_DLLAPI EndPoint {
    /// @brief object related with the end point
    CollisionObject* obj;

    /// @brief end point value
    CoalScalar value;

    /// @brief tag for whether it is a lower bound or higher bound of an
    /// interval, 0 for lo, and 1 for hi
    char minmax;

    bool operator<(const EndPoint& p) const;
  };

  /// @brief Extention interval tree's interval to SAP interval, adding more
  /// information
  struct COAL_DLLAPI SAPInterval : public detail::SimpleInterval {
    CollisionObject* obj;

    SAPInterval(CoalScalar low_, CoalScalar high_, CollisionObject* obj_);
  };

  bool checkColl(
      typename std::deque<detail::SimpleInterval*>::const_iterator pos_start,
      typename std::deque<detail::SimpleInterval*>::const_iterator pos_end,
      CollisionObject* obj, CollisionCallBackBase* callback) const;

  bool checkDist(
      typename std::deque<detail::SimpleInterval*>::const_iterator pos_start,
      typename std::deque<detail::SimpleInterval*>::const_iterator pos_end,
      CollisionObject* obj, DistanceCallBackBase* callback,
      CoalScalar& min_dist) const;

  bool collide_(CollisionObject* obj, CollisionCallBackBase* callback) const;

  bool distance_(CollisionObject* obj, DistanceCallBackBase* callback,
                 CoalScalar& min_dist) const;

  /// @brief vector stores all the end points
  std::vector<EndPoint> endpoints[3];

  /// @brief  interval tree manages the intervals
  detail::IntervalTree* interval_trees[3];

  std::map<CollisionObject*, SAPInterval*> obj_interval_maps[3];

  /// @brief tag for whether the interval tree is maintained suitably
  bool setup_;
};

}  // namespace coal

#endif
