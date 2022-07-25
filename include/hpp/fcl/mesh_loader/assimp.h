/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2016-2019, CNRS - LAAS
 *  Copyright (c) 2019, INRIA
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

#ifndef HPP_FCL_MESH_LOADER_ASSIMP_H
#define HPP_FCL_MESH_LOADER_ASSIMP_H

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/config.hh>
#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>

struct aiScene;
namespace Assimp {
class Importer;
}

namespace hpp {
namespace fcl {

namespace internal {

struct HPP_FCL_DLLAPI TriangleAndVertices {
  std::vector<fcl::Vec3f> vertices_;
  std::vector<fcl::Triangle> triangles_;
};

struct HPP_FCL_DLLAPI Loader {
  Loader();
  ~Loader();

  void load(const std::string& resource_path);

  Assimp::Importer* importer;
  aiScene const* scene;
};

/**
 * @brief      Recursive procedure for building a mesh
 *
 * @param[in]  scale           Scale to apply when reading the ressource
 * @param[in]  scene           Pointer to the assimp scene
 * @param[in]  vertices_offset Current number of vertices in the model
 * @param      tv              Triangles and Vertices of the mesh submodels
 */
HPP_FCL_DLLAPI void buildMesh(const fcl::Vec3f& scale, const aiScene* scene,
                              unsigned vertices_offset,
                              TriangleAndVertices& tv);

/**
 * @brief      Convert an assimp scene to a mesh
 *
 * @param[in]  scale  Scale to apply when reading the ressource
 * @param[in]  scene  Pointer to the assimp scene
 * @param[out] mesh  The mesh that must be built
 */
template <class BoundingVolume>
inline void meshFromAssimpScene(
    const fcl::Vec3f& scale, const aiScene* scene,
    const shared_ptr<BVHModel<BoundingVolume> >& mesh) {
  TriangleAndVertices tv;

  int res = mesh->beginModel();

  if (res != fcl::BVH_OK) {
    HPP_FCL_THROW_PRETTY("fcl BVHReturnCode = " << res, std::runtime_error);
  }

  buildMesh(scale, scene, (unsigned)mesh->num_vertices, tv);
  mesh->addSubModel(tv.vertices_, tv.triangles_);

  mesh->endModel();
}

}  // namespace internal

/**
 * @brief      Read a mesh file and convert it to a polyhedral mesh
 *
 * @param[in]  resource_path  Path to the ressource mesh file to be read
 * @param[in]  scale          Scale to apply when reading the ressource
 * @param[out] polyhedron     The resulted polyhedron
 */
template <class BoundingVolume>
inline void loadPolyhedronFromResource(
    const std::string& resource_path, const fcl::Vec3f& scale,
    const shared_ptr<BVHModel<BoundingVolume> >& polyhedron) {
  internal::Loader scene;
  scene.load(resource_path);

  internal::meshFromAssimpScene(scale, scene.scene, polyhedron);
}

}  // namespace fcl
}  // namespace hpp

#endif  // HPP_FCL_MESH_LOADER_ASSIMP_H
