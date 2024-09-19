//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_OBBRSS_H
#define COAL_SERIALIZATION_OBBRSS_H

#include "coal/BV/OBBRSS.h"

#include "coal/serialization/fwd.h"
#include "coal/serialization/OBB.h"
#include "coal/serialization/RSS.h"

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
