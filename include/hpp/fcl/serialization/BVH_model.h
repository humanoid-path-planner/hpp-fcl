//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
#define HPP_FCL_SERIALIZATION_BVH_MODEL_H

#include "hpp/fcl/BVH/BVH_model.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/collision_object.h"

namespace boost
{
  namespace serialization
  {
  
    template <class Archive>
    void serialize(Archive & ar,
                   hpp::fcl::Triangle & triangle,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("p0",triangle[0]);
      ar & make_nvp("p1",triangle[1]);
      ar & make_nvp("p2",triangle[2]);
    }
  
    template <class Archive>
    void save(Archive & ar,
              const hpp::fcl::BVHModelBase & bvh_model,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      if(bvh_model.build_state != BVH_BUILD_STATE_PROCESSED && bvh_model.build_state != BVH_BUILD_STATE_UPDATED)
      {
        throw std::invalid_argument("The BVH model is not in a BVH_BUILD_STATE_PROCESSED or BVH_BUILD_STATE_UPDATED state. Could not be serialized.");
      }
      
      ar & make_nvp("base",boost::serialization::base_object<hpp::fcl::CollisionGeometry>(bvh_model));
      ar & make_nvp("num_tris",bvh_model.num_tris);
      ar & make_nvp("num_vertices",bvh_model.num_vertices);
      ar & make_nvp("tri_indices",make_array(bvh_model.tri_indices,bvh_model.num_tris));
      ar & make_nvp("vertices",make_array(bvh_model.vertices,bvh_model.num_vertices));
      ar & make_nvp("build_state",bvh_model.build_state);
      
      ar & make_nvp("num_tris_allocated",bvh_model.num_tris);
      ar & make_nvp("num_vertices_allocated",bvh_model.num_vertices);
      
      if(bvh_model.prev_vertices)
      {
        const bool has_prev_vertices = true;
        ar << make_nvp("has_prev_vertices",has_prev_vertices);
        ar << make_nvp("prev_vertices",make_array(bvh_model.prev_vertices,bvh_model.num_vertices));
      }
      else
      {
        const bool has_prev_vertices = false;
        ar & make_nvp("has_prev_vertices",has_prev_vertices);
      }
      
      if(bvh_model.convex)
      {
        const bool has_convex = true;
        ar << make_nvp("has_convex",has_convex);
      }
      else
      {
        const bool has_convex = false;
        ar << make_nvp("has_convex",has_convex);
      }
    }
    
    template <class Archive>
    void load(Archive & ar,
              hpp::fcl::BVHModelBase & bvh_model,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      
      ar >> make_nvp("base",boost::serialization::base_object<hpp::fcl::CollisionGeometry>(bvh_model));
      ar >> make_nvp("num_tris",bvh_model.num_tris);
      if(bvh_model.num_tris > 0)
        bvh_model.tri_indices = new Triangle[bvh_model.num_tris];
      ar >> make_nvp("num_vertices",bvh_model.num_vertices);
      if(bvh_model.num_vertices > 0)
        bvh_model.vertices = new Vec3f[bvh_model.num_vertices];
      ar >> make_nvp("tri_indices",make_array(bvh_model.tri_indices,bvh_model.num_tris));
      ar >> make_nvp("vertices",make_array(bvh_model.vertices,bvh_model.num_vertices));
      ar >> make_nvp("build_state",bvh_model.build_state);
      
      ar & make_nvp("num_tris_allocated",bvh_model.num_tris);
      ar & make_nvp("num_vertices_allocated",bvh_model.num_vertices);
      
      bool has_prev_vertices;
      ar >> make_nvp("has_prev_vertices",has_prev_vertices);
      if(has_prev_vertices)
      {
        bvh_model.prev_vertices = new Vec3f[bvh_model.num_vertices];
        ar >> make_nvp("prev_vertices",make_array(bvh_model.prev_vertices,bvh_model.num_vertices));
      }
      
      bool has_convex = true;
      ar >> make_nvp("has_convex",has_convex);
    }
    
    HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::BVHModelBase)
  
  }
}

#endif // ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
