//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_AABB_H
#define HPP_FCL_SERIALIZATION_AABB_H

#include "hpp/fcl/BV/AABB.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, hpp::fcl::AABB& aabb,
               const unsigned int /*version*/) {
  ar& make_nvp("min_", aabb.min_);
  ar& make_nvp("max_", aabb.max_);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_AABB_H
