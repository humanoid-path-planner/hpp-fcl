//
// Copyright (c) 2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_kDOP_H
#define HPP_FCL_SERIALIZATION_kDOP_H

#include "hpp/fcl/BV/kDOP.h"

#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

namespace internal {
template <short N>
struct KDOPAccessor : hpp::fcl::KDOP<N> {
  typedef hpp::fcl::KDOP<N> Base;
  using Base::dist_;
};
}  // namespace internal

template <class Archive, short N>
void serialize(Archive& ar, hpp::fcl::KDOP<N>& bv_,
               const unsigned int /*version*/) {
  typedef internal::KDOPAccessor<N> Accessor;
  Accessor& access = reinterpret_cast<Accessor&>(bv_);
  ar& make_nvp("distances", make_array(access.dist_.data(), N));
}

}  // namespace serialization
}  // namespace boost

#endif  // HPP_FCL_SERIALIZATION_kDOP_H
