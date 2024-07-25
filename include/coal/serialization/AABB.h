//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_AABB_H
#define COAL_SERIALIZATION_AABB_H

#include "coal/BV/AABB.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::AABB& aabb, const unsigned int /*version*/) {
  ar& make_nvp("min_", aabb.min_);
  ar& make_nvp("max_", aabb.max_);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_AABB_H
