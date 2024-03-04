//
// Copyright (c) 2022-2024 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_CONVEX_H
#define HPP_FCL_SERIALIZATION_CONVEX_H

#include "hpp/fcl/shape/convex.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/geometric_shapes.h"
#include "hpp/fcl/serialization/memory.h"
#include "hpp/fcl/serialization/triangle.h"
#include "hpp/fcl/serialization/quadrilateral.h"

namespace boost {
namespace serialization {

namespace internal {
struct ConvexBaseAccessor : hpp::fcl::ConvexBase {
  typedef hpp::fcl::ConvexBase Base;
};

}  // namespace internal

template <class Archive>
void serialize(Archive& ar, hpp::fcl::ConvexBase& convex_base,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;

  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::ShapeBase>(
                           convex_base));

  const unsigned int num_points_previous = convex_base.num_points;
  ar& make_nvp("num_points", convex_base.num_points);

  const unsigned int num_normals_and_offsets_previous =
      convex_base.num_normals_and_offsets;
  ar& make_nvp("num_normals_and_offsets", convex_base.num_normals_and_offsets);

  const int num_warm_start_supports_previous =
      static_cast<int>(convex_base.support_warm_starts.points.size());
  assert(num_warm_start_supports_previous ==
         static_cast<int>(convex_base.support_warm_starts.indices.size()));
  int num_warm_start_supports = num_warm_start_supports_previous;
  ar& make_nvp("num_warm_start_supports", num_warm_start_supports);

  if (Archive::is_loading::value) {
    if (num_points_previous != convex_base.num_points) {
      convex_base.points.reset();
      if (convex_base.num_points > 0)
        convex_base.points.reset(
            new std::vector<Vec3f>(convex_base.num_points));
    }

    if (num_normals_and_offsets_previous !=
        convex_base.num_normals_and_offsets) {
      convex_base.normals.reset();
      convex_base.offsets.reset();
      if (convex_base.num_normals_and_offsets > 0) {
        convex_base.normals.reset(
            new std::vector<Vec3f>(convex_base.num_normals_and_offsets));
        convex_base.offsets.reset(
            new std::vector<FCL_REAL>(convex_base.num_normals_and_offsets));
      }
    }

    if (num_warm_start_supports_previous != num_warm_start_supports) {
      convex_base.support_warm_starts.points.resize(
          static_cast<size_t>(num_warm_start_supports));
      convex_base.support_warm_starts.indices.resize(
          static_cast<size_t>(num_warm_start_supports));
    }
  }

  typedef Eigen::Matrix<FCL_REAL, 3, Eigen::Dynamic> MatrixPoints;
  if (convex_base.num_points > 0) {
    Eigen::Map<MatrixPoints> points_map(
        reinterpret_cast<FCL_REAL*>(convex_base.points->data()), 3,
        convex_base.num_points);
    ar& make_nvp("points", points_map);
  }

  typedef Eigen::Matrix<FCL_REAL, 1, Eigen::Dynamic> VecOfReals;
  if (convex_base.num_normals_and_offsets > 0) {
    Eigen::Map<MatrixPoints> normals_map(
        reinterpret_cast<FCL_REAL*>(convex_base.normals->data()), 3,
        convex_base.num_normals_and_offsets);
    ar& make_nvp("normals", normals_map);

    Eigen::Map<VecOfReals> offsets_map(
        reinterpret_cast<FCL_REAL*>(convex_base.offsets->data()), 1,
        convex_base.num_normals_and_offsets);
    ar& make_nvp("offsets", offsets_map);
  }

  typedef Eigen::Matrix<int, 1, Eigen::Dynamic> VecOfInts;
  if (num_warm_start_supports > 0) {
    Eigen::Map<MatrixPoints> warm_start_support_points_map(
        reinterpret_cast<FCL_REAL*>(
            convex_base.support_warm_starts.points.data()),
        3, num_warm_start_supports);
    ar& make_nvp("warm_start_support_points", warm_start_support_points_map);

    Eigen::Map<VecOfInts> warm_start_support_indices_map(
        reinterpret_cast<int*>(convex_base.support_warm_starts.indices.data()),
        1, num_warm_start_supports);
    ar& make_nvp("warm_start_support_indices", warm_start_support_indices_map);
  }

  ar& make_nvp("center", convex_base.center);
  // We don't save neighbors as they will be computed directly by calling
  // fillNeighbors.
}

namespace internal {
template <typename PolygonT>
struct ConvexAccessor : hpp::fcl::Convex<PolygonT> {
  typedef hpp::fcl::Convex<PolygonT> Base;
  using Base::fillNeighbors;
};

}  // namespace internal

template <class Archive, typename PolygonT>
void serialize(Archive& ar, hpp::fcl::Convex<PolygonT>& convex_,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;
  typedef internal::ConvexAccessor<PolygonT> Accessor;

  Accessor& convex = reinterpret_cast<Accessor&>(convex_);
  ar& make_nvp("base", boost::serialization::base_object<ConvexBase>(convex_));

  const unsigned int num_polygons_previous = convex.num_polygons;
  ar& make_nvp("num_polygons", convex.num_polygons);

  if (Archive::is_loading::value) {
    if (num_polygons_previous != convex.num_polygons) {
      convex.polygons.reset(new std::vector<PolygonT>(convex.num_polygons));
    }
  }

  ar& make_array<PolygonT>(convex.polygons->data(), convex.num_polygons);

  if (Archive::is_loading::value) convex.fillNeighbors();
}

}  // namespace serialization
}  // namespace boost

HPP_FCL_SERIALIZATION_DECLARE_EXPORT(hpp::fcl::Convex<hpp::fcl::Triangle>)
HPP_FCL_SERIALIZATION_DECLARE_EXPORT(hpp::fcl::Convex<hpp::fcl::Quadrilateral>)

namespace hpp {
namespace fcl {

// namespace internal {
// template <typename BV>
// struct memory_footprint_evaluator< ::hpp::fcl::BVHModel<BV> > {
//   static size_t run(const ::hpp::fcl::BVHModel<BV> &bvh_model) {
//     return static_cast<size_t>(bvh_model.memUsage(false));
//   }
// };
// }  // namespace internal

}  // namespace fcl
}  // namespace hpp

#endif  // ifndef HPP_FCL_SERIALIZATION_CONVEX_H
