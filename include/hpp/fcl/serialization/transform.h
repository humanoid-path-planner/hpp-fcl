//
// Copyright (c) 2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_TRANSFORM_H
#define HPP_FCL_SERIALIZATION_TRANSFORM_H

#include "hpp/fcl/math/transform.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Transform3f& tf,
               const unsigned int /*version*/) {
  ar& make_nvp("R", tf.rotation());
  ar& make_nvp("T", tf.translation());
}

}  // namespace serialization
}  // namespace boost

#endif  // HPP_FCL_SERIALIZATION_TRANSFORM_H
