/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2016, CNRS - LAAS
 *  Copyright (c) 2020, INRIA
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

#ifndef HPP_FCL_MESH_LOADER_LOADER_H
#define HPP_FCL_MESH_LOADER_LOADER_H

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/config.hh>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/collision_object.h>

#include <map>
#include <ctime>

namespace hpp {
namespace fcl {
/// Base class for building polyhedron from files.
/// This class builds a new object for each file.
class HPP_FCL_DLLAPI MeshLoader {
 public:
  virtual ~MeshLoader() {}

  virtual BVHModelPtr_t load(const std::string& filename,
                             const Vec3f& scale = Vec3f::Ones());

  /// Create an OcTree from a file in binary octomap format.
  /// \todo add OctreePtr_t
  virtual CollisionGeometryPtr_t loadOctree(const std::string& filename);

  MeshLoader(const NODE_TYPE& bvType = BV_OBBRSS) : bvType_(bvType) {}

 private:
  const NODE_TYPE bvType_;
};

/// Class for building polyhedron from files with cache mechanism.
/// This class builds a new object for each different file.
/// If method CachedMeshLoader::load is called twice with the same arguments,
/// the second call returns the result of the first call.
class HPP_FCL_DLLAPI CachedMeshLoader : public MeshLoader {
 public:
  virtual ~CachedMeshLoader() {}

  CachedMeshLoader(const NODE_TYPE& bvType = BV_OBBRSS) : MeshLoader(bvType) {}

  virtual BVHModelPtr_t load(const std::string& filename, const Vec3f& scale);

  struct HPP_FCL_DLLAPI Key {
    std::string filename;
    Vec3f scale;

    Key(const std::string& f, const Vec3f& s) : filename(f), scale(s) {}

    bool operator<(const CachedMeshLoader::Key& b) const;
  };
  struct HPP_FCL_DLLAPI Value {
    BVHModelPtr_t model;
    std::time_t mtime;
  };
  typedef std::map<Key, Value> Cache_t;

  const Cache_t& cache() const { return cache_; }

 private:
  Cache_t cache_;
};
}  // namespace fcl

}  // namespace hpp

#endif  // FCL_MESH_LOADER_LOADER_H
