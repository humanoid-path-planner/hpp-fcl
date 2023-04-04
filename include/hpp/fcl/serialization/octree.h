//
// Copyright (c) 2023 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_OCTREE_H
#define HPP_FCL_SERIALIZATION_OCTREE_H

#include <sstream>
#include <iostream>

#include <boost/serialization/string.hpp>

#include "hpp/fcl/octree.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

namespace internal {
struct OcTreeAccessor : hpp::fcl::OcTree {
  typedef hpp::fcl::OcTree Base;
  using Base::default_occupancy;
  using Base::free_threshold;
  using Base::occupancy_threshold;
  using Base::tree;
};
}  // namespace internal

template <class Archive>
void save(Archive &ar, const hpp::fcl::OcTree &octree,
          const unsigned int /*version*/) {
  typedef internal::OcTreeAccessor Accessor;
  const Accessor &access = reinterpret_cast<const Accessor &>(octree);

  ar << make_nvp("base", base_object<hpp::fcl::CollisionGeometry>(octree));
  ar << make_nvp("default_occupancy", access.default_occupancy);
  ar << make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar << make_nvp("free_threshold", access.free_threshold);

  const double resolution = octree.getResolution();
  ar << make_nvp("resolution", resolution);

  std::ostringstream stream;
  access.tree->write(stream);
  //  std::cout << "stream:" << std::endl;
  //  std::cout << stream.str() << std::endl;
  //  stream.seekg(0, std::ios::end);
  //  const int stream_size = stream.tellg();
  //  ar << make_nvp("tree_size",stream_size);
  const std::string stream_str = stream.str();
  ar << make_nvp("tree_data", stream_str);
}

template <class Archive>
void load(Archive &ar, hpp::fcl::OcTree &octree,
          const unsigned int /*version*/) {
  typedef internal::OcTreeAccessor Accessor;
  Accessor &access = reinterpret_cast<Accessor &>(octree);

  ar >> make_nvp("base", base_object<hpp::fcl::CollisionGeometry>(octree));
  ar >> make_nvp("default_occupancy", access.default_occupancy);
  ar >> make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar >> make_nvp("free_threshold", access.free_threshold);
  double resolution;
  ar >> make_nvp("resolution", resolution);

  std::string stream_str;
  ar >> make_nvp("tree_data", stream_str);
  std::istringstream stream(stream_str);

  //  std::shared_ptr<const octomap::OcTree> new_octree(new
  //  octomap::OcTree(resolution));

  octomap::AbstractOcTree *new_tree = octomap::AbstractOcTree::read(stream);
  //  const_cast<octomap::OcTree &>(*new_octree).readBinaryData(stream);
  access.tree = std::shared_ptr<const octomap::OcTree>(
      dynamic_cast<octomap::OcTree *>(new_tree));
  //  const_cast<octomap::OcTree &>(*access.tree) = new_octree.get();

  //  const_cast<octomap::OcTree &>(*access.tree).clear();
  //  const_cast<octomap::OcTree &>(*access.tree).readBinaryData(stream);
}

template <class Archive>
void serialize(Archive &ar, hpp::fcl::OcTree &octree,
               const unsigned int version) {
  split_free(ar, octree, version);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_OCTREE_H
