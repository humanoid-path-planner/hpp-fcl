//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_OBBRSS_H
#define COAL_SERIALIZATION_OBBRSS_H

#include "hpp/fcl/BV/OBBRSS.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/OBB.h"
#include "hpp/fcl/serialization/RSS.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::OBBRSS& bv, const unsigned int /*version*/) {
  ar& make_nvp("obb", bv.obb);
  ar& make_nvp("rss", bv.rss);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_OBBRSS_H
