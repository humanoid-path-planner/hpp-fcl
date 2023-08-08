//
// Copyright (c) 2022 INRIA
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
void serialize(Archive &ar, hpp::fcl::ConvexBase &convex_base,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;

  ar &make_nvp("base", boost::serialization::base_object<hpp::fcl::ShapeBase>(
                           convex_base));
  const unsigned int num_points_previous = convex_base.num_points;
  ar &make_nvp("num_points", convex_base.num_points);

  if (Archive::is_loading::value) {
    if (num_points_previous != convex_base.num_points) {
      convex_base.points.reset(new Vec3f[convex_base.num_points]);
    }
  }

  {
    typedef Eigen::Matrix<FCL_REAL, 3, Eigen::Dynamic> MatrixPoints;
    Eigen::Map<MatrixPoints> points_map(
        reinterpret_cast<double *>(convex_base.points.get()), 3,
        convex_base.num_points);
    ar &make_nvp("points", points_map);
  }

  ar &make_nvp("center", convex_base.center);
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
void serialize(Archive &ar, hpp::fcl::Convex<PolygonT> &convex_,
               const unsigned int /*version*/) {
  using namespace hpp::fcl;
  typedef internal::ConvexAccessor<PolygonT> Accessor;

  Accessor &convex = reinterpret_cast<Accessor &>(convex_);
  ar &make_nvp("base", boost::serialization::base_object<ConvexBase>(convex));

  const unsigned int num_polygons_previous = convex.num_polygons;
  ar &make_nvp("num_polygons", convex.num_polygons);

  if (Archive::is_loading::value) {
    if (num_polygons_previous != convex.num_polygons) {
      convex.polygons.reset(new PolygonT[convex.num_polygons]);
    }
  }

  ar &make_array<PolygonT>(convex.polygons.get(), convex.num_polygons);

  if (Archive::is_loading::value) convex.fillNeighbors();
}

}  // namespace serialization
}  // namespace boost

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
