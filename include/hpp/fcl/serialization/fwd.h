//
// Copyright (c) 2021-2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_FWD_H
#define HPP_FCL_SERIALIZATION_FWD_H

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

#include "hpp/fcl/serialization/eigen.h"

#define HPP_FCL_SERIALIZATION_SPLIT(Type)                                \
  template <class Archive>                                               \
  void serialize(Archive& ar, Type& value, const unsigned int version) { \
    split_free(ar, value, version);                                      \
  }

#define HPP_FCL_SERIALIZATION_DECLARE_EXPORT(T) \
  BOOST_CLASS_EXPORT_KEY(T)                     \
  namespace boost {                             \
  namespace archive {                           \
  namespace detail {                            \
  namespace extra_detail {                      \
  template <>                                   \
  struct init_guid<T> {                         \
    static guid_initializer<T> const& g;        \
  };                                            \
  }                                             \
  }                                             \
  }                                             \
  }                                             \
  /**/

#define HPP_FCL_SERIALIZATION_DEFINE_EXPORT(T)          \
  namespace boost {                                     \
  namespace archive {                                   \
  namespace detail {                                    \
  namespace extra_detail {                              \
  guid_initializer<T> const& init_guid<T>::g =          \
      ::boost::serialization::singleton<                \
          guid_initializer<T> >::get_mutable_instance() \
          .export_guid();                               \
  }                                                     \
  }                                                     \
  }                                                     \
  }                                                     \
  /**/

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
