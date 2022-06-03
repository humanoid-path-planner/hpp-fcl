//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_COLLISION_DATA_H
#define HPP_FCL_SERIALIZATION_COLLISION_DATA_H

#include "hpp/fcl/collision_data.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const hpp::fcl::Contact& contact,
          const unsigned int /*version*/) {
  ar& make_nvp("b1", contact.b1);
  ar& make_nvp("b2", contact.b2);
  ar& make_nvp("normal", contact.normal);
  ar& make_nvp("pos", contact.pos);
  ar& make_nvp("penetration_depth", contact.penetration_depth);
}

template <class Archive>
void load(Archive& ar, hpp::fcl::Contact& contact,
          const unsigned int /*version*/) {
  ar >> make_nvp("b1", contact.b1);
  ar >> make_nvp("b2", contact.b2);
  ar >> make_nvp("normal", contact.normal);
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
  ar& make_nvp("enable_distance_lower_bound",
               collision_request.enable_distance_lower_bound);
  ar& make_nvp("security_margin", collision_request.security_margin);
  ar& make_nvp("break_distance", collision_request.break_distance);
}

template <class Archive>
void save(Archive& ar, const hpp::fcl::CollisionResult& collision_result,
          const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::QueryResult>(
                           collision_result));
  ar& make_nvp("contacts", collision_result.getContacts());
  ar& make_nvp("distance_lower_bound", collision_result.distance_lower_bound);
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
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::CollisionResult)

template <class Archive>
void serialize(Archive& ar, hpp::fcl::DistanceRequest& distance_request,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::QueryRequest>(
                   distance_request));
  ar& make_nvp("enable_nearest_points", distance_request.enable_nearest_points);
  ar& make_nvp("rel_err", distance_request.rel_err);
  ar& make_nvp("abs_err", distance_request.abs_err);
}

template <class Archive>
void save(Archive& ar, const hpp::fcl::DistanceResult& distance_result,
          const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::QueryResult>(
                           distance_result));
  ar& make_nvp("min_distance", distance_result.min_distance);
  ar& make_nvp("nearest_points", make_array(distance_result.nearest_points, 2));
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
  ar >>
      make_nvp("nearest_points", make_array(distance_result.nearest_points, 2));
  ar >> make_nvp("normal", distance_result.normal);
  ar >> make_nvp("b1", distance_result.b1);
  ar >> make_nvp("b2", distance_result.b2);
  distance_result.o1 = NULL;
  distance_result.o2 = NULL;
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::DistanceResult)

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_COLLISION_DATA_H
