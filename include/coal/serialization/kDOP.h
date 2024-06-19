//
// Copyright (c) 2024 INRIA
//

#ifndef COAL_SERIALIZATION_kDOP_H
#define COAL_SERIALIZATION_kDOP_H

#include "coal/BV/kDOP.h"

#include "coal/serialization/fwd.h"

namespace boost {
namespace serialization {

namespace internal {
template <short N>
struct KDOPAccessor : coal::KDOP<N> {
  typedef coal::KDOP<N> Base;
  using Base::dist_;
};
}  // namespace internal

template <class Archive, short N>
void serialize(Archive& ar, coal::KDOP<N>& bv_,
               const unsigned int /*version*/) {
  typedef internal::KDOPAccessor<N> Accessor;
  Accessor& access = reinterpret_cast<Accessor&>(bv_);
  ar& make_nvp("distances", make_array(access.dist_.data(), N));
}

}  // namespace serialization
}  // namespace boost

#endif  // COAL_SERIALIZATION_kDOP_H
