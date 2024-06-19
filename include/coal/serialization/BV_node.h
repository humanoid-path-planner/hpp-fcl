//
// Copyright (c) 2021 INRIA
//

#ifndef COAL_SERIALIZATION_BV_NODE_H
#define COAL_SERIALIZATION_BV_NODE_H

#include "coal/BV/BV_node.h"

#include "coal/serialization/fwd.h"
#include "coal/serialization/OBBRSS.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, coal::BVNodeBase& node,
               const unsigned int /*version*/) {
  ar& make_nvp("first_child", node.first_child);
  ar& make_nvp("first_primitive", node.first_primitive);
  ar& make_nvp("num_primitives", node.num_primitives);
}

template <class Archive, typename BV>
void serialize(Archive& ar, coal::BVNode<BV>& node,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<coal::BVNodeBase>(node));
  ar& make_nvp("bv", node.bv);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef COAL_SERIALIZATION_BV_NODE_H
