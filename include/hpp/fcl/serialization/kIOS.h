//
// Copyright (c) 2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_kIOS_H
#define HPP_FCL_SERIALIZATION_kIOS_H

#include "hpp/fcl/BV/kIOS.h"

#include "hpp/fcl/serialization/OBB.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, hpp::fcl::kIOS& bv, const unsigned int version) {
  split_free(ar, bv, version);
}

template <class Archive>
void save(Archive& ar, const hpp::fcl::kIOS& bv,
          const unsigned int /*version*/) {
  // Number of spheres in kIOS is never larger than kIOS::kios_max_num_spheres
  ar& make_nvp("num_spheres", bv.num_spheres);

  std::array<hpp::fcl::Vec3f, hpp::fcl::kIOS::max_num_spheres> centers{};
  std::array<hpp::fcl::FCL_REAL, hpp::fcl::kIOS::max_num_spheres> radii;
  for (std::size_t i = 0; i < hpp::fcl::kIOS::max_num_spheres; ++i) {
    centers[i] = bv.spheres[i].o;
    radii[i] = bv.spheres[i].r;
  }
  ar& make_nvp("centers", make_array(centers.data(), centers.size()));
  ar& make_nvp("radii", make_array(radii.data(), radii.size()));

  ar& make_nvp("obb", bv.obb);
}

template <class Archive>
void load(Archive& ar, hpp::fcl::kIOS& bv, const unsigned int /*version*/) {
  ar >> make_nvp("num_spheres", bv.num_spheres);

  std::array<hpp::fcl::Vec3f, hpp::fcl::kIOS::max_num_spheres> centers;
  std::array<hpp::fcl::FCL_REAL, hpp::fcl::kIOS::max_num_spheres> radii;
  ar >> make_nvp("centers", make_array(centers.data(), centers.size()));
  ar >> make_nvp("radii", make_array(radii.data(), radii.size()));
  for (std::size_t i = 0; i < hpp::fcl::kIOS::max_num_spheres; ++i) {
    bv.spheres[i].o = centers[i];
    bv.spheres[i].r = radii[i];
  }

  ar >> make_nvp("obb", bv.obb);
}

}  // namespace serialization
}  // namespace boost

#endif  // HPP_FCL_SERIALIZATION_kIOS_H
