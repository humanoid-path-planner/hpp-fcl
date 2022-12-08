//
// Copyright (c) 2021-2022 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_TRIANGLE_H
#define HPP_FCL_SERIALIZATION_TRIANGLE_H

#include "hpp/fcl/data_types.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, hpp::fcl::Triangle &triangle,
               const unsigned int /*version*/) {
  ar &make_nvp("p0", triangle[0]);
  ar &make_nvp("p1", triangle[1]);
  ar &make_nvp("p2", triangle[2]);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_TRIANGLE_H
