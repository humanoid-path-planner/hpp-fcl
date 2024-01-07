//
// Copyright (c) 2021-2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_FWD_H
#define HPP_FCL_SERIALIZATION_FWD_H

#include <type_traits>

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
namespace detail {

template <class Derived, class Base>
struct init_cast_register {};

template <class Derived, class Base>
struct cast_register_initializer {
  void init(std::true_type) const {
    boost::serialization::void_cast_register<Derived, Base>();
  }

  void init(std::false_type) const {}

  cast_register_initializer const& init() const {
    BOOST_STATIC_WARNING((std::is_base_of<Base, Derived>::value));
    init(std::is_base_of<Base, Derived>());
    return *this;
  }
};

}  // namespace detail

template <typename T>
struct register_type {
  template <class Archive>
  static void on(Archive& /*ar*/) {}
};
}  // namespace serialization
}  // namespace fcl
}  // namespace hpp

#define HPP_FCL_SERIALIZATION_CAST_REGISTER(Derived, Base)                   \
  namespace hpp {                                                            \
  namespace fcl {                                                            \
  namespace serialization {                                                  \
  namespace detail {                                                         \
  template <>                                                                \
  struct init_cast_register<Derived, Base> {                                 \
    static cast_register_initializer<Derived, Base> const& g;                \
  };                                                                         \
  cast_register_initializer<Derived, Base> const& init_cast_register<        \
      Derived, Base>::g =                                                    \
      ::boost::serialization::singleton<                                     \
          cast_register_initializer<Derived, Base> >::get_mutable_instance() \
          .init();                                                           \
  }                                                                          \
  }                                                                          \
  }                                                                          \
  }

#endif  // ifndef HPP_FCL_SERIALIZATION_FWD_H
