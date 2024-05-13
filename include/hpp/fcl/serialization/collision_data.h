//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_COLLISION_DATA_H
#define HPP_FCL_SERIALIZATION_COLLISION_DATA_H

#include "hpp/fcl/collision_data.h"
#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/transform.h"

namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const hpp::fcl::Contact& contact,
          const unsigned int /*version*/) {
  ar& make_nvp("b1", contact.b1);
  ar& make_nvp("b2", contact.b2);
  ar& make_nvp("normal", contact.normal);
  ar& make_nvp("nearest_points", contact.nearest_points);
  ar& make_nvp("pos", contact.pos);
  ar& make_nvp("penetration_depth", contact.penetration_depth);
}

template <class Archive>
void load(Archive& ar, hpp::fcl::Contact& contact,
          const unsigned int /*version*/) {
  ar >> make_nvp("b1", contact.b1);
  ar >> make_nvp("b2", contact.b2);
  ar >> make_nvp("normal", contact.normal);
  std::array<hpp::fcl::Vec3f, 2> nearest_points;
  ar >> make_nvp("nearest_points", nearest_points);
  contact.nearest_points[0] = nearest_points[0];
  contact.nearest_points[1] = nearest_points[1];
  ar >> make_nvp("pos", contact.pos);
  ar >> make_nvp("penetration_depth", contact.penetration_depth);
  contact.o1 = NULL;
  contact.o2 = NULL;
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::Contact)

template <class Archive>
void serialize(Archive& ar, hpp::fcl::QueryRequest& query_request,
               const unsigned int /*version*/) {
  ar& make_nvp("gjk_initial_guess", query_request.gjk_initial_guess);
  // TODO: use gjk_initial_guess instead
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  ar& make_nvp("enable_cached_gjk_guess",
               query_request.enable_cached_gjk_guess);
  HPP_FCL_COMPILER_DIAGNOSTIC_POP
  ar& make_nvp("cached_gjk_guess", query_request.cached_gjk_guess);
  ar& make_nvp("cached_support_func_guess",
               query_request.cached_support_func_guess);
  ar& make_nvp("gjk_max_iterations", query_request.gjk_max_iterations);
  ar& make_nvp("gjk_tolerance", query_request.gjk_tolerance);
  ar& make_nvp("gjk_variant", query_request.gjk_variant);
  ar& make_nvp("gjk_convergence_criterion",
               query_request.gjk_convergence_criterion);
  ar& make_nvp("gjk_convergence_criterion_type",
               query_request.gjk_convergence_criterion_type);
  ar& make_nvp("epa_max_iterations", query_request.epa_max_iterations);
  ar& make_nvp("epa_tolerance", query_request.epa_tolerance);
  ar& make_nvp("collision_distance_threshold",
               query_request.collision_distance_threshold);
  ar& make_nvp("enable_timings", query_request.enable_timings);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::QueryResult& query_result,
               const unsigned int /*version*/) {
  ar& make_nvp("cached_gjk_guess", query_result.cached_gjk_guess);
  ar& make_nvp("cached_support_func_guess",
               query_result.cached_support_func_guess);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::CollisionRequest& collision_request,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::QueryRequest>(
                   collision_request));
  ar& make_nvp("num_max_contacts", collision_request.num_max_contacts);
  ar& make_nvp("enable_contact", collision_request.enable_contact);
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  ar& make_nvp("enable_distance_lower_bound",
               collision_request.enable_distance_lower_bound);
  HPP_FCL_COMPILER_DIAGNOSTIC_POP
  ar& make_nvp("security_margin", collision_request.security_margin);
  ar& make_nvp("break_distance", collision_request.break_distance);
  ar& make_nvp("distance_upper_bound", collision_request.distance_upper_bound);
}

template <class Archive>
void save(Archive& ar, const hpp::fcl::CollisionResult& collision_result,
          const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::QueryResult>(
                           collision_result));
  ar& make_nvp("contacts", collision_result.getContacts());
  ar& make_nvp("distance_lower_bound", collision_result.distance_lower_bound);
  ar& make_nvp("nearest_points", collision_result.nearest_points);
  ar& make_nvp("normal", collision_result.normal);
}

template <class Archive>
void load(Archive& ar, hpp::fcl::CollisionResult& collision_result,
          const unsigned int /*version*/) {
  ar >>
      make_nvp("base", boost::serialization::base_object<hpp::fcl::QueryResult>(
                           collision_result));
  std::vector<hpp::fcl::Contact> contacts;
  ar >> make_nvp("contacts", contacts);
  collision_result.clear();
  for (size_t k = 0; k < contacts.size(); ++k)
    collision_result.addContact(contacts[k]);
  ar >> make_nvp("distance_lower_bound", collision_result.distance_lower_bound);
  std::array<hpp::fcl::Vec3f, 2> nearest_points;
  ar >> make_nvp("nearest_points", nearest_points);
  collision_result.nearest_points[0] = nearest_points[0];
  collision_result.nearest_points[1] = nearest_points[1];
  ar >> make_nvp("normal", collision_result.normal);
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::CollisionResult)

template <class Archive>
void serialize(Archive& ar, hpp::fcl::DistanceRequest& distance_request,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::QueryRequest>(
                   distance_request));
  HPP_FCL_COMPILER_DIAGNOSTIC_PUSH
  HPP_FCL_COMPILER_DIAGNOSTIC_IGNORED_DEPRECECATED_DECLARATIONS
  ar& make_nvp("enable_nearest_points", distance_request.enable_nearest_points);
  HPP_FCL_COMPILER_DIAGNOSTIC_POP
  ar& make_nvp("enable_signed_distance",
               distance_request.enable_signed_distance);
  ar& make_nvp("rel_err", distance_request.rel_err);
  ar& make_nvp("abs_err", distance_request.abs_err);
}

template <class Archive>
void save(Archive& ar, const hpp::fcl::DistanceResult& distance_result,
          const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::QueryResult>(
                           distance_result));
  ar& make_nvp("min_distance", distance_result.min_distance);
  ar& make_nvp("nearest_points", distance_result.nearest_points);
  ar& make_nvp("normal", distance_result.normal);
  ar& make_nvp("b1", distance_result.b1);
  ar& make_nvp("b2", distance_result.b2);
}

template <class Archive>
void load(Archive& ar, hpp::fcl::DistanceResult& distance_result,
          const unsigned int /*version*/) {
  ar >>
      make_nvp("base", boost::serialization::base_object<hpp::fcl::QueryResult>(
                           distance_result));
  ar >> make_nvp("min_distance", distance_result.min_distance);
  std::array<hpp::fcl::Vec3f, 2> nearest_points;
  ar >> make_nvp("nearest_points", nearest_points);
  distance_result.nearest_points[0] = nearest_points[0];
  distance_result.nearest_points[1] = nearest_points[1];
  ar >> make_nvp("normal", distance_result.normal);
  ar >> make_nvp("b1", distance_result.b1);
  ar >> make_nvp("b2", distance_result.b2);
  distance_result.o1 = NULL;
  distance_result.o2 = NULL;
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::DistanceResult)

template <class Archive>
void serialize(Archive& ar, hpp::fcl::ContactPatch& contact_patch,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;
  typedef Eigen::Matrix<FCL_REAL, 2, Eigen::Dynamic> PolygonPoints;

  size_t patch_size = contact_patch.size();
  ar& make_nvp("patch_size", patch_size);
  if (patch_size > 0) {
    if (Archive::is_loading::value) {
      contact_patch.points().resize(patch_size);
    }
    Eigen::Map<PolygonPoints> points_map(
        reinterpret_cast<FCL_REAL*>(contact_patch.points().data()), 2,
        patch_size);
    ar& make_nvp("points", points_map);
  }

  ar& make_nvp("penetration_depth", contact_patch.penetration_depth);
  ar& make_nvp("direction", contact_patch.direction);
  ar& make_nvp("tf", contact_patch.tf);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::ContactPatchRequest& request,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;

  ar& make_nvp("max_num_patch", request.max_num_patch);

  size_t num_samples_curved_shapes = request.getNumSamplesCurvedShapes();
  FCL_REAL patch_tolerance = request.getPatchTolerance();
  ar& make_nvp("num_samples_curved_shapes", num_samples_curved_shapes);
  ar& make_nvp("patch_tolerance", num_samples_curved_shapes);

  if (Archive::is_loading::value) {
    request.setNumSamplesCurvedShapes(num_samples_curved_shapes);
    request.setPatchTolerance(patch_tolerance);
  }
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::ContactPatchResult& result,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;

  size_t num_patches = result.numContactPatches();
  ar& make_nvp("num_patches", num_patches);

  std::vector<ContactPatch> patches;
  patches.resize(num_patches);
  if (Archive::is_loading::value) {
    ar& make_nvp("patches", patches);

    const ContactPatchRequest request(num_patches);
    result.set(request);
    for (size_t i = 0; i < num_patches; ++i) {
      ContactPatch& patch = result.getUnusedContactPatch();
      patch = patches[i];
    }
  } else {
    patches.clear();
    for (size_t i = 0; i < num_patches; ++i) {
      patches.emplace_back(result.getContactPatch(i));
    }
    ar& make_nvp("patches", patches);
  }
}

}  // namespace serialization
}  // namespace boost

HPP_FCL_SERIALIZATION_DECLARE_EXPORT(::hpp::fcl::CollisionRequest)
HPP_FCL_SERIALIZATION_DECLARE_EXPORT(::hpp::fcl::CollisionResult)
HPP_FCL_SERIALIZATION_DECLARE_EXPORT(::hpp::fcl::DistanceRequest)
HPP_FCL_SERIALIZATION_DECLARE_EXPORT(::hpp::fcl::DistanceResult)

#endif  // ifndef HPP_FCL_SERIALIZATION_COLLISION_DATA_H
