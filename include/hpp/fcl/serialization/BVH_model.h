//
// Copyright (c) 2021-2022 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
#define HPP_FCL_SERIALIZATION_BVH_MODEL_H

#include "hpp/fcl/BVH/BVH_model.h"

#include "hpp/fcl/serialization/fwd.h"
#include "hpp/fcl/serialization/BV_node.h"
#include "hpp/fcl/serialization/BV_splitter.h"
#include "hpp/fcl/serialization/collision_object.h"
#include "hpp/fcl/serialization/memory.h"
#include "hpp/fcl/serialization/triangle.h"

namespace boost {
namespace serialization {

namespace internal {
struct BVHModelBaseAccessor : hpp::fcl::BVHModelBase {
  typedef hpp::fcl::BVHModelBase Base;
  using Base::num_tris_allocated;
  using Base::num_vertices_allocated;
};
}  // namespace internal

template <class Archive>
void save(Archive &ar, const hpp::fcl::BVHModelBase &bvh_model,
          const unsigned int /*version*/) {
  using namespace hpp::fcl;
  if (!(bvh_model.build_state == BVH_BUILD_STATE_PROCESSED ||
        bvh_model.build_state == BVH_BUILD_STATE_UPDATED) &&
      (bvh_model.getModelType() == BVH_MODEL_TRIANGLES)) {
    throw std::invalid_argument(
        "The BVH model is not in a BVH_BUILD_STATE_PROCESSED or "
        "BVH_BUILD_STATE_UPDATED state.\n"
        "The BVHModel could not be serialized.");
  }

  ar &make_nvp("base",
               boost::serialization::base_object<hpp::fcl::CollisionGeometry>(
                   bvh_model));

  ar &make_nvp("num_vertices", bvh_model.num_vertices);
  if (bvh_model.num_vertices > 0) {
    typedef Eigen::Matrix<FCL_REAL, 3, Eigen::Dynamic> AsVertixMatrix;
    const Eigen::Map<const AsVertixMatrix> vertices_map(
        reinterpret_cast<const double *>(bvh_model.vertices), 3,
        bvh_model.num_vertices);
    ar &make_nvp("vertices", vertices_map);
  }

  ar &make_nvp("num_tris", bvh_model.num_tris);
  if (bvh_model.num_tris > 0) {
    typedef Eigen::Matrix<Triangle::index_type, 3, Eigen::Dynamic>
        AsTriangleMatrix;
    const Eigen::Map<const AsTriangleMatrix> tri_indices_map(
        reinterpret_cast<const Triangle::index_type *>(bvh_model.tri_indices),
        3, bvh_model.num_tris);
    ar &make_nvp("tri_indices", tri_indices_map);
  }
  ar &make_nvp("build_state", bvh_model.build_state);

  if (bvh_model.prev_vertices) {
    const bool has_prev_vertices = true;
    ar << make_nvp("has_prev_vertices", has_prev_vertices);
    typedef Eigen::Matrix<FCL_REAL, 3, Eigen::Dynamic> AsVertixMatrix;
    const Eigen::Map<const AsVertixMatrix> prev_vertices_map(
        reinterpret_cast<const double *>(bvh_model.prev_vertices), 3,
        bvh_model.num_vertices);
    ar &make_nvp("prev_vertices", prev_vertices_map);
  } else {
    const bool has_prev_vertices = false;
    ar &make_nvp("has_prev_vertices", has_prev_vertices);
  }

  //      if(bvh_model.convex)
  //      {
  //        const bool has_convex = true;
  //        ar << make_nvp("has_convex",has_convex);
  //      }
  //      else
  //      {
  //        const bool has_convex = false;
  //        ar << make_nvp("has_convex",has_convex);
  //      }
}

template <class Archive>
void load(Archive &ar, hpp::fcl::BVHModelBase &bvh_model,
          const unsigned int /*version*/) {
  using namespace hpp::fcl;

  ar >> make_nvp("base",
                 boost::serialization::base_object<hpp::fcl::CollisionGeometry>(
                     bvh_model));

  unsigned int num_vertices;
  ar >> make_nvp("num_vertices", num_vertices);
  if (num_vertices != bvh_model.num_vertices) {
    delete[] bvh_model.vertices;
    bvh_model.vertices = NULL;
    bvh_model.num_vertices = num_vertices;
    if (num_vertices > 0) bvh_model.vertices = new Vec3f[num_vertices];
  }
  if (num_vertices > 0) {
    typedef Eigen::Matrix<FCL_REAL, 3, Eigen::Dynamic> AsVertixMatrix;
    Eigen::Map<AsVertixMatrix> vertices_map(
        reinterpret_cast<double *>(bvh_model.vertices), 3,
        bvh_model.num_vertices);
    ar >> make_nvp("vertices", vertices_map);
  } else
    bvh_model.vertices = NULL;

  unsigned int num_tris;
  ar >> make_nvp("num_tris", num_tris);

  if (num_tris != bvh_model.num_tris) {
    delete[] bvh_model.tri_indices;
    bvh_model.tri_indices = NULL;
    bvh_model.num_tris = num_tris;
    if (num_tris > 0) bvh_model.tri_indices = new Triangle[num_tris];
  }
  if (num_tris > 0) {
    typedef Eigen::Matrix<Triangle::index_type, 3, Eigen::Dynamic>
        AsTriangleMatrix;
    Eigen::Map<AsTriangleMatrix> tri_indices_map(
        reinterpret_cast<Triangle::index_type *>(bvh_model.tri_indices), 3,
        bvh_model.num_tris);
    ar &make_nvp("tri_indices", tri_indices_map);
  } else
    bvh_model.tri_indices = NULL;

  ar >> make_nvp("build_state", bvh_model.build_state);

  typedef internal::BVHModelBaseAccessor Accessor;
  reinterpret_cast<Accessor &>(bvh_model).num_tris_allocated = num_tris;
  reinterpret_cast<Accessor &>(bvh_model).num_vertices_allocated = num_vertices;

  bool has_prev_vertices;
  ar >> make_nvp("has_prev_vertices", has_prev_vertices);
  if (has_prev_vertices) {
    if (num_vertices != bvh_model.num_vertices) {
      delete[] bvh_model.prev_vertices;
      bvh_model.prev_vertices = NULL;
      if (num_vertices > 0) bvh_model.prev_vertices = new Vec3f[num_vertices];
    }
    if (num_vertices > 0) {
      typedef Eigen::Matrix<FCL_REAL, 3, Eigen::Dynamic> AsVertixMatrix;
      Eigen::Map<AsVertixMatrix> prev_vertices_map(
          reinterpret_cast<double *>(bvh_model.prev_vertices), 3,
          bvh_model.num_vertices);
      ar &make_nvp("prev_vertices", prev_vertices_map);
    }
  } else
    bvh_model.prev_vertices = NULL;

  //      bool has_convex = true;
  //      ar >> make_nvp("has_convex",has_convex);
}

HPP_FCL_SERIALIZATION_SPLIT(hpp::fcl::BVHModelBase)

namespace internal {
template <typename BV>
struct BVHModelAccessor : hpp::fcl::BVHModel<BV> {
  typedef hpp::fcl::BVHModel<BV> Base;
  using Base::bvs;
  using Base::num_bvs;
  using Base::num_bvs_allocated;
  using Base::primitive_indices;
};
}  // namespace internal

template <class Archive, typename BV>
void serialize(Archive &ar, hpp::fcl::BVHModel<BV> &bvh_model,
               const unsigned int version) {
  split_free(ar, bvh_model, version);
}

template <class Archive, typename BV>
void save(Archive &ar, const hpp::fcl::BVHModel<BV> &bvh_model_,
          const unsigned int /*version*/) {
  using namespace hpp::fcl;
  typedef internal::BVHModelAccessor<BV> Accessor;
  typedef BVNode<BV> Node;

  const Accessor &bvh_model = reinterpret_cast<const Accessor &>(bvh_model_);
  ar &make_nvp("base",
               boost::serialization::base_object<BVHModelBase>(bvh_model));

  //      if(bvh_model.primitive_indices)
  //      {
  //        const bool with_primitive_indices = true;
  //        ar & make_nvp("with_primitive_indices",with_primitive_indices);
  //
  //        int num_primitives = 0;
  //        switch(bvh_model.getModelType())
  //        {
  //          case BVH_MODEL_TRIANGLES:
  //            num_primitives = bvh_model.num_tris;
  //            break;
  //          case BVH_MODEL_POINTCLOUD:
  //            num_primitives = bvh_model.num_vertices;
  //            break;
  //          default:
  //            ;
  //        }
  //
  //        ar & make_nvp("num_primitives",num_primitives);
  //        if(num_primitives > 0)
  //        {
  //          typedef Eigen::Matrix<unsigned int,1,Eigen::Dynamic>
  //          AsPrimitiveIndexVector; const Eigen::Map<const
  //          AsPrimitiveIndexVector>
  //          primitive_indices_map(reinterpret_cast<const unsigned int
  //          *>(bvh_model.primitive_indices),1,num_primitives); ar &
  //          make_nvp("primitive_indices",primitive_indices_map);
  ////          ar &
  /// make_nvp("primitive_indices",make_array(bvh_model.primitive_indices,num_primitives));
  //        }
  //      }
  //      else
  //      {
  //        const bool with_primitive_indices = false;
  //        ar & make_nvp("with_primitive_indices",with_primitive_indices);
  //      }
  //

  if (bvh_model.bvs) {
    const bool with_bvs = true;
    ar &make_nvp("with_bvs", with_bvs);
    ar &make_nvp("num_bvs", bvh_model.num_bvs);
    ar &make_nvp(
        "bvs",
        make_array(
            reinterpret_cast<const char *>(bvh_model.bvs),
            sizeof(Node) *
                (std::size_t)bvh_model.num_bvs));  // Assuming BVs are POD.
  } else {
    const bool with_bvs = false;
    ar &make_nvp("with_bvs", with_bvs);
  }
}

template <class Archive, typename BV>
void load(Archive &ar, hpp::fcl::BVHModel<BV> &bvh_model_,
          const unsigned int /*version*/) {
  using namespace hpp::fcl;
  typedef internal::BVHModelAccessor<BV> Accessor;
  typedef BVNode<BV> Node;

  Accessor &bvh_model = reinterpret_cast<Accessor &>(bvh_model_);

  ar >> make_nvp("base",
                 boost::serialization::base_object<BVHModelBase>(bvh_model));

  //      bool with_primitive_indices;
  //      ar >> make_nvp("with_primitive_indices",with_primitive_indices);
  //      if(with_primitive_indices)
  //      {
  //        int num_primitives;
  //        ar >> make_nvp("num_primitives",num_primitives);
  //
  //        delete[] bvh_model.primitive_indices;
  //        if(num_primitives > 0)
  //        {
  //          bvh_model.primitive_indices = new unsigned int[num_primitives];
  //          ar &
  //          make_nvp("primitive_indices",make_array(bvh_model.primitive_indices,num_primitives));
  //        }
  //        else
  //          bvh_model.primitive_indices = NULL;
  //      }

  bool with_bvs;
  ar >> make_nvp("with_bvs", with_bvs);
  if (with_bvs) {
    unsigned int num_bvs;
    ar >> make_nvp("num_bvs", num_bvs);

    if (num_bvs != bvh_model.num_bvs) {
      delete[] bvh_model.bvs;
      bvh_model.bvs = NULL;
      bvh_model.num_bvs = num_bvs;
      if (num_bvs > 0) bvh_model.bvs = new BVNode<BV>[num_bvs];
    }
    if (num_bvs > 0) {
      ar >> make_nvp("bvs", make_array(reinterpret_cast<char *>(bvh_model.bvs),
                                       sizeof(Node) * (std::size_t)num_bvs));
    } else
      bvh_model.bvs = NULL;
  }
}

}  // namespace serialization
}  // namespace boost

namespace hpp {
namespace fcl {

namespace internal {
template <typename BV>
struct memory_footprint_evaluator< ::hpp::fcl::BVHModel<BV> > {
  static size_t run(const ::hpp::fcl::BVHModel<BV> &bvh_model) {
    return static_cast<size_t>(bvh_model.memUsage(false));
  }
};
}  // namespace internal

}  // namespace fcl
}  // namespace hpp

#endif  // ifndef HPP_FCL_SERIALIZATION_BVH_MODEL_H
