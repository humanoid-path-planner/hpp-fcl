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

#ifndef FCL_MESH_LOADER_ASSIMP_H
#define FCL_MESH_LOADER_ASSIMP_H

#ifdef FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
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
  void clear()
  {
    vertices_.clear ();
    triangles_.clear ();
  }
  std::vector <fcl::Vec3f> vertices_;
  std::vector <fcl::Triangle> triangles_;
};

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
                         const boost::shared_ptr < BVHModel<BoundingVolume> > & mesh) throw (std::invalid_argument)
{
  TriangleAndVertices tv;
  
  if (!scene->HasMeshes())
    throw std::invalid_argument (std::string ("No meshes found in file ")+name);
  
  std::vector<unsigned> subMeshIndexes;
  int res = mesh->beginModel ();
  
  if (res != fcl::BVH_OK)
  {
    std::ostringstream error;
    error << "fcl BVHReturnCode = " << res;
    throw std::runtime_error (error.str ());
  }
    
  tv.clear();
    
  buildMesh (scale, scene, scene->mRootNode, subMeshIndexes, mesh, tv);
  mesh->addSubModel (tv.vertices_, tv.triangles_);
    
  mesh->endModel ();
}

/**
 * @brief      Recursive procedure for building a mesh
 *
 * @param[in]  scale           Scale to apply when reading the ressource
 * @param[in]  scene           Pointer to the assimp scene
 * @param[in]  node            Current node of the scene
 * @param      subMeshIndexes  Submesh triangles indexes interval
 * @param[in]  mesh            The mesh that must be built
 * @param      tv              Triangles and Vertices of the mesh submodels
 */
template<class BoundingVolume>
inline void buildMesh (const fcl::Vec3f & scale,
                const aiScene* scene,
                const aiNode* node,
                std::vector<unsigned> & subMeshIndexes,
                const boost::shared_ptr < BVHModel<BoundingVolume> > & mesh,
                TriangleAndVertices & tv)
{
  if (!node) return;
  
  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
    {
      transform = pnode->mTransformation * transform;
    }
    pnode = pnode->mParent;
  }
  
  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    
    unsigned oldNbPoints = (unsigned) mesh->num_vertices;
    unsigned oldNbTriangles = (unsigned) mesh->num_tris;
    
    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
    {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      tv.vertices_.push_back (fcl::Vec3f (p.x * scale[0],
                                          p.y * scale[1],
                                          p.z * scale[2]));
    }
    
    // add the indices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
    {
      aiFace& face = input_mesh->mFaces[j];
      if (face.mNumIndices != 3) {
        std::stringstream ss;
#ifdef FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES
        ss << "Mesh " << input_mesh->mName.C_Str() << " has a face with "
           << face.mNumIndices << " vertices. This is not supported\n";
        ss << "Node name is: " << node->mName.C_Str() << "\n";
#endif
        ss << "Mesh index: " << i << "\n";
        ss << "Face index: " << j << "\n";
        throw std::invalid_argument (ss.str());
      }
      tv.triangles_.push_back (fcl::Triangle( oldNbPoints + face.mIndices[0],
                                             oldNbPoints + face.mIndices[1],
                                             oldNbPoints + face.mIndices[2]));
    }
    
    // Save submesh triangles indexes interval.
    if (subMeshIndexes.size () == 0)
    {
      subMeshIndexes.push_back (0);
    }
    
    subMeshIndexes.push_back (oldNbTriangles + input_mesh->mNumFaces);
  }
  
  for (uint32_t i=0; i < node->mNumChildren; ++i)
  {
    buildMesh(scale, scene, node->mChildren[i], subMeshIndexes, mesh, tv);
  }
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
                                 const boost::shared_ptr < BVHModel<BoundingVolume> > & polyhedron) throw (std::invalid_argument)
{
  Assimp::Importer importer;
  // // set list of ignored parameters (parameters used for rendering)
  //    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
  //                     aiComponent_TANGENTS_AND_BITANGENTS|
  //                     aiComponent_COLORS |
  //                     aiComponent_BONEWEIGHTS |
  //                     aiComponent_ANIMATIONS |
  //                     aiComponent_LIGHTS |
  //                     aiComponent_CAMERAS|
  //                     aiComponent_TEXTURES |
  //                     aiComponent_TEXCOORDS |
  //                     aiComponent_MATERIALS |
  //                     aiComponent_NORMALS
  //                 );

  const aiScene* scene = importer.ReadFile(resource_path.c_str(), aiProcess_SortByPType| aiProcess_GenNormals|
                                           aiProcess_Triangulate|aiProcess_GenUVCoords|
                                           aiProcess_FlipUVs);
  // const aiScene* scene = importer.ReadFile(resource_path, aiProcess_SortByPType|
  //                                          aiProcess_Triangulate | aiProcess_RemoveComponent |
  //                                          aiProcess_JoinIdenticalVertices);

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
