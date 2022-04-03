//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_BV_NODE_H
#define HPP_FCL_SERIALIZATION_BV_NODE_H

#include "hpp/fcl/BV/BV_node.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/OBBRSS.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, hpp::fcl::BVNodeBase& node,
               const unsigned int /*version*/) {
  ar& make_nvp("first_child", node.first_child);
  ar& make_nvp("first_primitive", node.first_primitive);
  ar& make_nvp("num_primitives", node.num_primitives);
}

template <class Archive, typename BV>
void serialize(Archive& ar, hpp::fcl::BVNode<BV>& node,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::BVNodeBase>(node));
  ar& make_nvp("bv", node.bv);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_BV_NODE_H
