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

  std::ostringstream stream;
  access.tree->write(stream);
  const std::string stream_str = stream.str();
  ar << make_nvp("tree_data", stream_str);

  // const double resolution = octree.getResolution();
  // ar << make_nvp("resolution", resolution);

  ar << make_nvp("base", base_object<hpp::fcl::CollisionGeometry>(octree));
  ar << make_nvp("default_occupancy", access.default_occupancy);
  ar << make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar << make_nvp("free_threshold", access.free_threshold);
}

template <class Archive>
void load_construct_data(Archive &ar, hpp::fcl::OcTree *octree_ptr,
                         const unsigned int /*version*/) {
  std::string stream_str;
  ar >> make_nvp("tree_data", stream_str);
  std::istringstream stream(stream_str);

  octomap::AbstractOcTree *tree_raw_ptr = octomap::AbstractOcTree::read(stream);
  std::shared_ptr<const octomap::OcTree> tree_ptr(
      dynamic_cast<octomap::OcTree *>(tree_raw_ptr));

  new (octree_ptr) hpp::fcl::OcTree(tree_ptr);
  hpp::fcl::OcTree &octree = *octree_ptr;
  typedef internal::OcTreeAccessor Accessor;
  Accessor &access = reinterpret_cast<Accessor &>(octree);

  // double resolution;
  // ar >> make_nvp("resolution", resolution);

  ar >> make_nvp("base", base_object<hpp::fcl::CollisionGeometry>(octree));
  ar >> make_nvp("default_occupancy", access.default_occupancy);
  ar >> make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar >> make_nvp("free_threshold", access.free_threshold);
}

template <class Archive>
void load(Archive &ar, hpp::fcl::OcTree &octree,
          const unsigned int /*version*/) {
  typedef internal::OcTreeAccessor Accessor;
  Accessor &access = reinterpret_cast<Accessor &>(octree);

  std::string stream_str;
  ar >> make_nvp("tree_data", stream_str);
  std::istringstream stream(stream_str);

  octomap::AbstractOcTree *new_tree = octomap::AbstractOcTree::read(stream);
  access.tree = std::shared_ptr<const octomap::OcTree>(
      dynamic_cast<octomap::OcTree *>(new_tree));

  // double resolution;
  // ar >> make_nvp("resolution", resolution);

  ar >> make_nvp("base", base_object<hpp::fcl::CollisionGeometry>(octree));
  ar >> make_nvp("default_occupancy", access.default_occupancy);
  ar >> make_nvp("occupancy_threshold", access.occupancy_threshold);
  ar >> make_nvp("free_threshold", access.free_threshold);
}

template <class Archive>
void serialize(Archive &ar, hpp::fcl::OcTree &octree,
               const unsigned int version) {
  split_free(ar, octree, version);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_OCTREE_H
