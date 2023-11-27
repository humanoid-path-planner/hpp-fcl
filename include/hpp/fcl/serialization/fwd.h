//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_FWD_H
#define HPP_FCL_SERIALIZATION_FWD_H

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "hpp/fcl/serialization/eigen.h"

#define HPP_FCL_SERIALIZATION_SPLIT(Type)                                \
  template <class Archive>                                               \
  void serialize(Archive& ar, Type& value, const unsigned int version) { \
    split_free(ar, value, version);                                      \
  }

namespace hpp {
namespace fcl {
namespace serialization {
template <typename T>
struct register_type {
  template <class Archive>
  static void on(Archive& /*ar*/) {}
};
}  // namespace serialization
}  // namespace fcl
}  // namespace hpp

#endif  // ifndef HPP_FCL_SERIALIZATION_FWD_H
