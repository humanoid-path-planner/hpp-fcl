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

#ifndef HPP_FCL_BROAD_PHASE_BRUTE_FORCE_INL_H
#define HPP_FCL_BROAD_PHASE_BRUTE_FORCE_INL_H

#include "hpp/fcl/broadphase/broadphase_bruteforce.h"

#include <iterator>
#include <algorithm>

namespace hpp {
namespace fcl {

//==============================================================================
inline
NaiveCollisionManager::NaiveCollisionManager()
{
  // Do nothing
}

//==============================================================================
inline
void NaiveCollisionManager::registerObjects(const std::vector<CollisionObject*>& other_objs)
{
  std::copy(other_objs.begin(), other_objs.end(), std::back_inserter(objs));
}

//==============================================================================
inline
void NaiveCollisionManager::unregisterObject(CollisionObject* obj)
{
  objs.remove(obj);
}

//==============================================================================
inline
void NaiveCollisionManager::registerObject(CollisionObject* obj)
{
  objs.push_back(obj);
}

//==============================================================================
inline
void NaiveCollisionManager::setup()
{
  // Do nothing
}

//==============================================================================
inline
void NaiveCollisionManager::update()
{
  // Do nothing
}

//==============================================================================
inline
void NaiveCollisionManager::clear()
{
  objs.clear();
}

//==============================================================================
inline
void NaiveCollisionManager::getObjects(std::vector<CollisionObject*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

//==============================================================================
inline
void NaiveCollisionManager::collide(CollisionObject* obj, void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  for(auto* obj2 : objs)
  {
    if(callback(obj, obj2, cdata))
      return;
  }
}

//==============================================================================
inline
void NaiveCollisionManager::distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(auto* obj2 : objs)
  {
    if(obj->getAABB().distance(obj2->getAABB()) < min_dist)
    {
      if(callback(obj, obj2, cdata, min_dist))
        return;
    }
  }
}

//==============================================================================
inline
void NaiveCollisionManager::collide(void* cdata, CollisionCallBack callback) const
{
  if(size() == 0) return;

  for(typename std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end = objs.end();
      it1 != end; ++it1)
  {
    typename std::list<CollisionObject*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().overlap((*it2)->getAABB()))
      {
        if(callback(*it1, *it2, cdata))
          return;
      }
    }
  }
}

//==============================================================================
inline
void NaiveCollisionManager::distance(void* cdata, DistanceCallBack callback) const
{
  if(size() == 0) return;

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(typename std::list<CollisionObject*>::const_iterator it1 = objs.begin(), end = objs.end(); it1 != end; ++it1)
  {
    typename std::list<CollisionObject*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().distance((*it2)->getAABB()) < min_dist)
      {
        if(callback(*it1, *it2, cdata, min_dist))
          return;
      }
    }
  }
}

//==============================================================================
inline
void NaiveCollisionManager::collide(BroadPhaseCollisionManager* other_manager_, void* cdata, CollisionCallBack callback) const
{
  NaiveCollisionManager* other_manager = static_cast<NaiveCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    collide(cdata, callback);
    return;
  }

  for(auto* obj1 : objs)
  {
    for(auto* obj2 : other_manager->objs)
    {
      if(obj1->getAABB().overlap(obj2->getAABB()))
      {
        if(callback(obj1, obj2, cdata))
          return;
      }
    }
  }
}

//==============================================================================
inline
void NaiveCollisionManager::distance(BroadPhaseCollisionManager* other_manager_, void* cdata, DistanceCallBack callback) const
{
  NaiveCollisionManager* other_manager = static_cast<NaiveCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(auto* obj1 : objs)
  {
    for(auto* obj2 : other_manager->objs)
    {
      if(obj1->getAABB().distance(obj2->getAABB()) < min_dist)
      {
        if(callback(obj1, obj2, cdata, min_dist))
          return;
      }
    }
  }
}

//==============================================================================
inline
bool NaiveCollisionManager::empty() const
{
  return objs.empty();
}

//==============================================================================
inline
size_t NaiveCollisionManager::size() const
{
  return objs.size();
}

} // namespace fcl

} // namespace hpp

#endif
