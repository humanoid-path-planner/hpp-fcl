//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_OBB_H
#define COAL_SERIALIZATION_OBB_H

#include "hpp/fcl/BV/OBB.h"

#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::OBB& bv, const unsigned int /*version*/) {
  ar& make_nvp("axes", bv.axes);
  ar& make_nvp("To", bv.To);
  ar& make_nvp("extent", bv.extent);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_OBB_H
