//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2019 CNRS-LAAS INRIA
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
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <eigenpy/eigenpy.hpp>

#include "fcl.hh"

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/distance.h>

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#include "doxygen_autodoc/hpp/fcl/collision_data.h"
#endif

#include "../doc/python/doxygen.hh"
#include "../doc/python/doxygen-boost.hh"

#define DEF_RW_CLASS_ATTRIB(CLASS, ATTRIB)                                     \
  def_readwrite (#ATTRIB, &CLASS::ATTRIB,                                      \
      doxygen::class_attrib_doc<CLASS>(#ATTRIB))
#define DEF_RO_CLASS_ATTRIB(CLASS, ATTRIB)                                     \
  def_readonly (#ATTRIB, &CLASS::ATTRIB,                                       \
      doxygen::class_attrib_doc<CLASS>(#ATTRIB))
#define DEF_CLASS_FUNC(CLASS, ATTRIB)                                          \
  def (#ATTRIB, &CLASS::ATTRIB, doxygen::member_func_doc(&CLASS::ATTRIB))
#define DEF_CLASS_FUNC2(CLASS, ATTRIB,policy)                                  \
  def (#ATTRIB, &CLASS::ATTRIB, doxygen::member_func_doc(&CLASS::ATTRIB),policy)

using namespace boost::python;

using namespace hpp::fcl;

namespace dv = doxygen::visitor;

struct DistanceRequestWrapper
{
  static Vec3f getNearestPoint1(const DistanceResult & res) { return res.nearest_points[0]; }
  static Vec3f getNearestPoint2(const DistanceResult & res) { return res.nearest_points[1]; }
};

void exposeDistanceAPI ()
{
  if(!eigenpy::register_symbolic_link_to_registered_type<DistanceRequest>())
  {
    class_ <DistanceRequest, bases<QueryRequest> > ("DistanceRequest",
        doxygen::class_doc<DistanceRequest>(),
        init<optional<bool,FCL_REAL,FCL_REAL> >((arg("self"),
                                                 arg("enable_nearest_points"),
                                                 arg("rel_err"),
                                                 arg("abs_err")),"Constructor"))
      .DEF_RW_CLASS_ATTRIB (DistanceRequest, enable_nearest_points)
      .DEF_RW_CLASS_ATTRIB (DistanceRequest, rel_err)
      .DEF_RW_CLASS_ATTRIB (DistanceRequest, abs_err)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<DistanceRequest> >())
  {
    class_< std::vector<DistanceRequest> >("StdVec_DistanceRequest")
      .def(vector_indexing_suite< std::vector<DistanceRequest> >())
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<DistanceResult>())
  {
    class_ <DistanceResult, bases<QueryResult> > ("DistanceResult",
                             doxygen::class_doc<DistanceResult>(),
                             no_init)
      .def (dv::init<DistanceResult>())
      .DEF_RW_CLASS_ATTRIB (DistanceResult, min_distance)
      .DEF_RW_CLASS_ATTRIB(DistanceResult, normal)
      //.def_readwrite ("nearest_points", &DistanceResult::nearest_points)
      .def("getNearestPoint1",&DistanceRequestWrapper::getNearestPoint1,
          doxygen::class_attrib_doc<DistanceResult>("nearest_points"))
      .def("getNearestPoint2",&DistanceRequestWrapper::getNearestPoint2,
          doxygen::class_attrib_doc<DistanceResult>("nearest_points"))
      .DEF_RO_CLASS_ATTRIB (DistanceResult, o1)
      .DEF_RO_CLASS_ATTRIB (DistanceResult, o2)
      .DEF_RW_CLASS_ATTRIB (DistanceResult, b1)
      .DEF_RW_CLASS_ATTRIB (DistanceResult, b2)

      .def ("clear", &DistanceResult::clear,
          doxygen::member_func_doc(&DistanceResult::clear))
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<DistanceResult> >())
  {
    class_< std::vector<DistanceResult> >("StdVec_DistanceResult")
      .def(vector_indexing_suite< std::vector<DistanceResult> >())
      ;
  }

  doxygen::def ("distance", static_cast< FCL_REAL (*)(const CollisionObject*, const CollisionObject*,
        const DistanceRequest&, DistanceResult&) > (&distance));
  doxygen::def ("distance", static_cast< FCL_REAL (*)(
        const CollisionGeometry*, const Transform3f&,
        const CollisionGeometry*, const Transform3f&,
        DistanceRequest&, DistanceResult&) > (&distance));

  class_<ComputeDistance> ("ComputeDistance",
      doxygen::class_doc<ComputeDistance>(), no_init)
    .def (dv::init<ComputeDistance, const CollisionGeometry*, const CollisionGeometry*>())
    .def ("__call__", static_cast< FCL_REAL (ComputeDistance::*)(
        const Transform3f&, const Transform3f&,
        DistanceRequest&, DistanceResult&) > (&ComputeDistance::operator()));

}
