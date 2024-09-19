//
// Copyright (c) 2024 INRIA
//

#ifndef COAL_SERIALIZATION_TRANSFORM_H
#define COAL_SERIALIZATION_TRANSFORM_H

#include "coal/math/transform.h"
#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::Transform3s& tf,
               const unsigned int /*version*/) {
  ar& make_nvp("R", tf.rotation());
  ar& make_nvp("T", tf.translation());
}

}  // namespace serialization
}  // namespace boost

#endif  // COAL_SERIALIZATION_TRANSFORM_H
