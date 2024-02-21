//
// Copyright (c) 2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_TRANSFORM_H
#define HPP_FCL_SERIALIZATION_TRANSFORM_H

#include "hpp/fcl/math/transform.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::Transform3f)

template <class Archive>
void save(Archive& ar, const hpp::fcl::Transform3f& tf,
          const unsigned int /*version*/) {
  ar& make_nvp("R", tf.getRotation());
  ar& make_nvp("T", tf.getTranslation());
}

template <class Archive>
void load(Archive& ar, hpp::fcl::Transform3f& tf,
          const unsigned int /*version*/) {
  hpp::fcl::Matrix3f R;
  ar >> make_nvp("R", R);
  tf.setRotation(R);
  hpp::fcl::Vec3f T;
  ar >> make_nvp("T", T);
  tf.setTranslation(T);
}

}  // namespace serialization
}  // namespace boost

#endif  // HPP_FCL_SERIALIZATION_TRANSFORM_H
