//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_RSS_H
#define COAL_SERIALIZATION_RSS_H

#include "coal/BV/RSS.h"

#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::RSS& bv, const unsigned int /*version*/) {
  ar& make_nvp("axes", bv.axes);
  ar& make_nvp("Tr", bv.Tr);
  ar& make_nvp("length", make_array(bv.length, 2));
  ar& make_nvp("radius", bv.radius);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_RSS_H
