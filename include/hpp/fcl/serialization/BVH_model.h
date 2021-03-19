//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
#define HPP_FCL_SERIALIZATION_BVH_MODEL_H

#include <boost/serialization/split_free.hpp>

#include "hpp/fcl/BVH/BVH_model.h"
#include "hpp/fcl/serialization/fwd.h"

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
      ar & make_nvp("num_tris",bvh_model.num_tris);
      ar & make_nvp("num_vertices",bvh_model.num_vertices);
      ar & make_nvp("tri_indices",make_array(bvh_model.tri_indices,bvh_model.num_tris));
      ar & make_nvp("vertices",make_array(bvh_model.vertices,bvh_model.num_vertices));
      ar & make_nvp("build_state",bvh_model.build_state);
      
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
    }
    
    template <class Archive>
    void load(Archive & ar,
              hpp::fcl::BVHModelBase & bvh_model,
              const unsigned int /*version*/)
    {
      using namespace hpp::fcl;
      ar >> make_nvp("num_tris",bvh_model.num_tris);
      if(bvh_model.num_tris > 0)
        bvh_model.tri_indices = new Triangle[bvh_model.num_tris];
      ar >> make_nvp("num_vertices",bvh_model.num_vertices);
      if(bvh_model.num_vertices > 0)
        bvh_model.vertices = new Vec3f[bvh_model.num_vertices];
      ar >> make_nvp("tri_indices",make_array(bvh_model.tri_indices,bvh_model.num_tris));
      ar >> make_nvp("vertices",make_array(bvh_model.vertices,bvh_model.num_vertices));
      ar >> make_nvp("build_state",bvh_model.build_state);
      
      bool has_prev_vertices;
      ar >> make_nvp("has_prev_vertices",has_prev_vertices);
      if(has_prev_vertices)
      {
        bvh_model.prev_vertices = new Vec3f[bvh_model.num_vertices];
        ar >> make_nvp("prev_vertices",make_array(bvh_model.prev_vertices,bvh_model.num_vertices));
      }
    }
    
    HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::BVHModelBase)
  
  }
}

#endif // ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
