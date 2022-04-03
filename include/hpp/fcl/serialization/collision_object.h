//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_COLLISION_OBJECT_H
#define HPP_FCL_SERIALIZATION_COLLISION_OBJECT_H

#include "hpp/fcl/collision_object.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/AABB.h"

namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const hpp::fcl::CollisionGeometry& collision_geometry,
          const unsigned int /*version*/) {
  ar& make_nvp("aabb_center", collision_geometry.aabb_center);
  ar& make_nvp("aabb_radius", collision_geometry.aabb_radius);
  ar& make_nvp("aabb_local", collision_geometry.aabb_local);
  ar& make_nvp("cost_density", collision_geometry.cost_density);
  ar& make_nvp("threshold_occupied", collision_geometry.threshold_occupied);
  ar& make_nvp("threshold_free", collision_geometry.threshold_free);
}

template <class Archive>
void load(Archive& ar, hpp::fcl::CollisionGeometry& collision_geometry,
          const unsigned int /*version*/) {
  ar >> make_nvp("aabb_center", collision_geometry.aabb_center);
  ar >> make_nvp("aabb_radius", collision_geometry.aabb_radius);
  ar >> make_nvp("aabb_local", collision_geometry.aabb_local);
  ar >> make_nvp("cost_density", collision_geometry.cost_density);
  ar >> make_nvp("threshold_occupied", collision_geometry.threshold_occupied);
  ar >> make_nvp("threshold_free", collision_geometry.threshold_free);
  collision_geometry.user_data = NULL;  // no way to recover this
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::CollisionGeometry)

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_COLLISION_OBJECT_H
