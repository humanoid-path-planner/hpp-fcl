//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2019 CNRS-LAAS
//  Author: Joseph Mirabel
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of CNRS-LAAS. nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include <boost/python.hpp>

#include "fcl.hh"

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/BVH/BVH_model.h>

using namespace boost::python;

using namespace hpp::fcl;
using boost::shared_ptr;
using boost::noncopyable;

struct BVHModelBaseWrapper
{
  static Vec3f vertices (const BVHModelBase& bvh, int i)
  {
    if (i >= bvh.num_vertices) throw std::out_of_range("index is out of range");
    return bvh.vertices[i];
  }

  static Triangle tri_indices (const BVHModelBase& bvh, int i)
  {
    if (i >= bvh.num_tris) throw std::out_of_range("index is out of range");
    return bvh.tri_indices[i];
  }
};

template <typename BV>
void exposeBVHModel (const std::string& bvname)
{
  typedef BVHModel<BV> BVHModel_t;

  std::string type = "BVHModel" + bvname;
  class_ <BVHModel_t, bases<BVHModelBase>, shared_ptr<BVHModel_t> >
    (type.c_str(), init<>())
    ;
}

struct ConvexBaseWrapper
{
  static Vec3f points (const ConvexBase& convex, int i)
  {
    if (i >= convex.num_points) throw std::out_of_range("index is out of range");
    return convex.points[i];
  }

  static list neighbors (const ConvexBase& convex, int i)
  {
    if (i >= convex.num_points) throw std::out_of_range("index is out of range");
    list n;
    for (unsigned char j = 0; j < convex.neighbors[i].count(); ++j)
      n.append (convex.neighbors[i][j]);
    return n;
  }
};

template <typename PolygonT>
struct ConvexWrapper
{
  typedef Convex<PolygonT> Convex_t;

  static PolygonT polygons (const Convex_t& convex, int i)
  {
    if (i >= convex.num_polygons) throw std::out_of_range("index is out of range");
    return convex.polygons[i];
  }
};

void exposeShapes ()
{
  class_ <ShapeBase, bases<CollisionGeometry>, shared_ptr<ShapeBase>, noncopyable>
    ("ShapeBase", no_init)
    //.def ("getObjectType", &CollisionGeometry::getObjectType)
    ;

  class_ <Box, bases<ShapeBase>, shared_ptr<Box> >
    ("Box", init<>())
    .def (init<FCL_REAL,FCL_REAL,FCL_REAL>())
    .def (init<Vec3f>())
    .def_readwrite ("halfSide", &Box::halfSide)
    ;

  class_ <Capsule, bases<ShapeBase>, shared_ptr<Capsule> >
    ("Capsule", init<FCL_REAL, FCL_REAL>())
    .def_readwrite ("radius", &Capsule::radius)
    .def_readwrite ("lz", &Capsule::lz)
    ;

  class_ <Cone, bases<ShapeBase>, shared_ptr<Cone> >
    ("Cone", init<FCL_REAL, FCL_REAL>())
    .def_readwrite ("radius", &Cone::radius)
    .def_readwrite ("lz", &Cone::lz)
    ;

  class_ <ConvexBase, bases<ShapeBase>, shared_ptr<ConvexBase >, noncopyable>
    ("ConvexBase", no_init)
    .def_readonly ("center", &ConvexBase::center)
    .def_readonly ("num_points", &ConvexBase::num_points)
    .def ("points", &ConvexBaseWrapper::points)
    .def ("neighbors", &ConvexBaseWrapper::neighbors)
    ;

  class_ <Convex<Triangle>, bases<ConvexBase>, shared_ptr<Convex<Triangle> >, noncopyable>
    ("Convex", no_init)
    .def_readonly ("num_polygons", &Convex<Triangle>::num_polygons)
    .def ("polygons", &ConvexWrapper<Triangle>::polygons)
    ;

  class_ <Cylinder, bases<ShapeBase>, shared_ptr<Cylinder> >
    ("Cylinder", init<FCL_REAL, FCL_REAL>())
    .def_readwrite ("radius", &Cylinder::radius)
    .def_readwrite ("lz", &Cylinder::lz)
    ;

  class_ <Halfspace, bases<ShapeBase>, shared_ptr<Halfspace> >
    ("Halfspace", "The half-space is defined by {x | n * x < d}.", init<const Vec3f&, FCL_REAL>())
    .def (init<FCL_REAL,FCL_REAL,FCL_REAL,FCL_REAL>())
    .def (init<>())
    .def_readwrite ("n", &Halfspace::n)
    .def_readwrite ("d", &Halfspace::d)
    ;

  class_ <Plane, bases<ShapeBase>, shared_ptr<Plane> >
    ("Plane", "The plane is defined by {x | n * x = d}.", init<const Vec3f&, FCL_REAL>())
    .def (init<FCL_REAL,FCL_REAL,FCL_REAL,FCL_REAL>())
    .def (init<>())
    .def_readwrite ("n", &Plane::n)
    .def_readwrite ("d", &Plane::d)
    ;

  class_ <Sphere, bases<ShapeBase>, shared_ptr<Sphere> >
    ("Sphere", init<FCL_REAL>())
    .def_readwrite ("radius", &Sphere::radius)
    ;

  class_ <TriangleP, bases<ShapeBase>, shared_ptr<TriangleP> >
    ("TriangleP", init<const Vec3f&, const Vec3f&, const Vec3f&>())
    .def_readwrite ("a", &TriangleP::a)
    .def_readwrite ("b", &TriangleP::b)
    .def_readwrite ("c", &TriangleP::c)
    ;

}

void exposeCollisionGeometries ()
{
  enum_<OBJECT_TYPE>("OBJECT_TYPE")
    .value ("OT_UNKNOWN", OT_UNKNOWN)
    .value ("OT_BVH"    , OT_BVH)
    .value ("OT_GEOM"   , OT_GEOM)
    .value ("OT_OCTREE" , OT_OCTREE)
    ;
  enum_<NODE_TYPE>("NODE_TYPE")
    .value ("BV_UNKNOWN", BV_UNKNOWN)
    .value ("BV_AABB"  , BV_AABB)
    .value ("BV_OBB"   , BV_OBB)
    .value ("BV_RSS"   , BV_RSS)
    .value ("BV_kIOS"  , BV_kIOS)
    .value ("BV_OBBRSS", BV_OBBRSS)
    .value ("BV_KDOP16", BV_KDOP16)
    .value ("BV_KDOP18", BV_KDOP18)
    .value ("BV_KDOP24", BV_KDOP24)
    .value ("GEOM_BOX"      , GEOM_BOX)
    .value ("GEOM_SPHERE"   , GEOM_SPHERE)
    .value ("GEOM_CAPSULE"  , GEOM_CAPSULE)
    .value ("GEOM_CONE"     , GEOM_CONE)
    .value ("GEOM_CYLINDER" , GEOM_CYLINDER)
    .value ("GEOM_CONVEX"   , GEOM_CONVEX)
    .value ("GEOM_PLANE"    , GEOM_PLANE)
    .value ("GEOM_HALFSPACE", GEOM_HALFSPACE)
    .value ("GEOM_TRIANGLE" , GEOM_TRIANGLE)
    .value ("GEOM_OCTREE"   , GEOM_OCTREE)
    ;

  class_ <CollisionGeometry, CollisionGeometryPtr_t, noncopyable>
    ("CollisionGeometry", no_init)
    .def ("getObjectType", &CollisionGeometry::getObjectType)
    .def ("getNodeType", &CollisionGeometry::getNodeType)

    .def ("computeLocalAABB", &CollisionGeometry::computeLocalAABB)

    .def ("computeCOM", &CollisionGeometry::computeCOM)
    .def ("computeMomentofInertia", &CollisionGeometry::computeMomentofInertia)
    .def ("computeVolume", &CollisionGeometry::computeVolume)
    .def ("computeMomentofInertiaRelatedToCOM", &CollisionGeometry::computeMomentofInertiaRelatedToCOM)
    ;

  exposeShapes();

  class_ <BVHModelBase, bases<CollisionGeometry>, BVHModelPtr_t, noncopyable>
    ("BVHModelBase", no_init)
    .def ("vertices", &BVHModelBaseWrapper::vertices)
    .def ("tri_indices", &BVHModelBaseWrapper::tri_indices)
    .def_readonly ("num_vertices", &BVHModelBase::num_vertices)
    .def_readonly ("num_tris", &BVHModelBase::num_tris)

    .def_readonly ("convex", &BVHModelBase::convex)

    .def ("buildConvexRepresentation", &BVHModelBase::buildConvexRepresentation)
    ;
  exposeBVHModel<OBB    >("OBB"    );
  exposeBVHModel<OBBRSS >("OBBRSS" );
}
