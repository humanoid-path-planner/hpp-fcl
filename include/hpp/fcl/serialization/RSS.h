//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_RSS_H
#define HPP_FCL_SERIALIZATION_RSS_H

#include "hpp/fcl/BV/RSS.h"

#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, hpp::fcl::RSS& bv, const unsigned int /*version*/) {
  ar& make_nvp("axes", bv.axes);
  ar& make_nvp("Tr", bv.Tr);
  ar& make_nvp("length", make_array(bv.length, 2));
  ar& make_nvp("radius", bv.radius);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_RSS_H
