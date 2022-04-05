//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_SERIALIZATION_GEOMETRIC_SHAPES_H
#define HPP_FCL_SERIALIZATION_GEOMETRIC_SHAPES_H

#include "hpp/fcl/shape/geometric_shapes.h"
#include "hpp/fcl/serialization/fwd.h"

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, hpp::fcl::ShapeBase& shape_base,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::CollisionGeometry>(
                   shape_base));
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::TriangleP& triangle,
               const unsigned int /*version*/) {
  ar& make_nvp(
      "base", boost::serialization::base_object<hpp::fcl::ShapeBase>(triangle));
  ar& make_nvp("a", triangle.a);
  ar& make_nvp("b", triangle.b);
  ar& make_nvp("c", triangle.c);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Box& box,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::ShapeBase>(box));
  ar& make_nvp("halfSide", box.halfSide);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Sphere& sphere,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::ShapeBase>(sphere));
  ar& make_nvp("radius", sphere.radius);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Ellipsoid& ellipsoid,
               const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::ShapeBase>(
                           ellipsoid));
  ar& make_nvp("radii", ellipsoid.radii);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Capsule& capsule,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::ShapeBase>(capsule));
  ar& make_nvp("radius", capsule.radius);
  ar& make_nvp("halfLength", capsule.halfLength);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Cone& cone,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::ShapeBase>(cone));
  ar& make_nvp("radius", cone.radius);
  ar& make_nvp("halfLength", cone.halfLength);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Cylinder& cylinder,
               const unsigned int /*version*/) {
  ar& make_nvp(
      "base", boost::serialization::base_object<hpp::fcl::ShapeBase>(cylinder));
  ar& make_nvp("radius", cylinder.radius);
  ar& make_nvp("halfLength", cylinder.halfLength);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Halfspace& half_space,
               const unsigned int /*version*/) {
  ar& make_nvp("base", boost::serialization::base_object<hpp::fcl::ShapeBase>(
                           half_space));
  ar& make_nvp("n", half_space.n);
  ar& make_nvp("d", half_space.d);
}

template <class Archive>
void serialize(Archive& ar, hpp::fcl::Plane& plane,
               const unsigned int /*version*/) {
  ar& make_nvp("base",
               boost::serialization::base_object<hpp::fcl::ShapeBase>(plane));
  ar& make_nvp("n", plane.n);
  ar& make_nvp("d", plane.d);
}

}  // namespace serialization
}  // namespace boost

#endif  // ifndef HPP_FCL_SERIALIZATION_GEOMETRIC_SHAPES_H
