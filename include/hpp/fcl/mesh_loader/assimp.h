/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  Copyright (c) 2016, CNRS - LAAS
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

#ifdef HPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
  #include <assimp/DefaultLogger.hpp>
  #include <assimp/IOStream.hpp>
  #include <assimp/IOSystem.hpp>
  #include <assimp/scene.h>
  #include <assimp/Importer.hpp>
  #include <assimp/postprocess.h>
#else
  #include <assimp/DefaultLogger.h>
  #include <assimp/assimp.hpp>
  #include <assimp/IOStream.h>
  #include <assimp/IOSystem.h>
  #include <assimp/aiScene.h>
  #include <assimp/aiPostProcess.h>
#endif

#include <hpp/fcl/BV/OBBRSS.h>
#include <hpp/fcl/BVH/BVH_model.h>

namespace hpp
{
namespace fcl
{
  

struct TriangleAndVertices
{
  std::vector <fcl::Vec3f> vertices_;
  std::vector <fcl::Triangle> triangles_;
};

/**
 * @brief      Recursive procedure for building a mesh
 *
 * @param[in]  scale           Scale to apply when reading the ressource
 * @param[in]  scene           Pointer to the assimp scene
 * @param[in]  node            Current node of the scene
 * @param[in]  vertices_offset Current number of vertices in the model
 * @param      tv              Triangles and Vertices of the mesh submodels
 */
unsigned buildMesh (const fcl::Vec3f & scale,
                    const aiScene* scene,
                    const aiNode* node,
                    unsigned vertices_offset,
                    TriangleAndVertices & tv);

/**
 * @brief      Convert an assimp scene to a mesh
 *
 * @param[in]  name   File (ressource) transformed into an assimp scene in loa
 * @param[in]  scale  Scale to apply when reading the ressource
 * @param[in]  scene  Pointer to the assimp scene
 * @param[out] mesh  The mesh that must be built
 */
template<class BoundingVolume>   
inline void meshFromAssimpScene(const std::string & name,
                         const fcl::Vec3f & scale,
                         const aiScene* scene,
                         const boost::shared_ptr < BVHModel<BoundingVolume> > & mesh)
{
  TriangleAndVertices tv;
  
  if (!scene->HasMeshes())
    throw std::invalid_argument (std::string ("No meshes found in file ")+name);
  
  int res = mesh->beginModel ();
  
  if (res != fcl::BVH_OK)
  {
    std::ostringstream error;
    error << "fcl BVHReturnCode = " << res;
    throw std::runtime_error (error.str ());
  }
    
  buildMesh (scale, scene, scene->mRootNode, 
      (unsigned) mesh->num_vertices, tv);
  mesh->addSubModel (tv.vertices_, tv.triangles_);
    
  mesh->endModel ();
}

/**
 * @brief      Read a mesh file and convert it to a polyhedral mesh
 *
 * @param[in]  resource_path  Path to the ressource mesh file to be read
 * @param[in]  scale          Scale to apply when reading the ressource
 * @param[out] polyhedron     The resulted polyhedron
 */
template<class BoundingVolume>
inline void loadPolyhedronFromResource (const std::string & resource_path,
                                 const fcl::Vec3f & scale,
                                 const boost::shared_ptr < BVHModel<BoundingVolume> > & polyhedron)
{
  Assimp::Importer importer;
  // set list of ignored parameters (parameters used for rendering)
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
      aiComponent_TANGENTS_AND_BITANGENTS|
      aiComponent_COLORS |
      aiComponent_BONEWEIGHTS |
      aiComponent_ANIMATIONS |
      aiComponent_LIGHTS |
      aiComponent_CAMERAS|
      aiComponent_TEXTURES |
      aiComponent_TEXCOORDS |
      aiComponent_MATERIALS |
      aiComponent_NORMALS
      );

  const aiScene* scene = importer.ReadFile(resource_path.c_str(),
      aiProcess_SortByPType |
      aiProcess_Triangulate |
      aiProcess_RemoveComponent |
      aiProcess_ImproveCacheLocality |
      // TODO: I (Joseph Mirabel) have no idea whether degenerated triangles are
      // properly handled. Enabling aiProcess_FindDegenerates would throw an
      // exception when that happens. Is it too conservative ?
      // aiProcess_FindDegenerates |
      aiProcess_JoinIdenticalVertices
      );

  if (!scene)
  {
    const std::string exception_message (std::string ("Could not load resource ") + resource_path + std::string("\n") +
                                         importer.GetErrorString () + std::string("\n") +
                                         "Hint: the mesh directory may be wrong.");
    throw std::invalid_argument(exception_message);
  }
  
  meshFromAssimpScene (resource_path, scale, scene, polyhedron);
}

}

} // namespace hpp

#endif // FCL_MESH_LOADER_ASSIMP_H
