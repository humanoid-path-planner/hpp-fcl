//
// Software License Agreement (BSD License)
//
//  Copyright (c) 2019-2020 CNRS-LAAS INRIA
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

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/collision.h>

#include "fcl.hh"

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
  def (dv::member_func(#ATTRIB, &CLASS::ATTRIB))
#define DEF_CLASS_FUNC2(CLASS, ATTRIB,policy)                                  \
  def (#ATTRIB, &CLASS::ATTRIB, doxygen::member_func_doc(&CLASS::ATTRIB),policy)

using namespace boost::python;
using namespace hpp::fcl;

namespace dv = doxygen::visitor;

template<int index> const CollisionGeometry* geto(const Contact& c)
{ return index == 1 ? c.o1 : c.o2; }

void exposeCollisionAPI ()
{
  if(!eigenpy::register_symbolic_link_to_registered_type<CollisionRequestFlag>())
  {
    enum_ <CollisionRequestFlag> ("CollisionRequestFlag")
      .value ("CONTACT", CONTACT)
      .value ("DISTANCE_LOWER_BOUND", DISTANCE_LOWER_BOUND)
      .value ("NO_REQUEST", NO_REQUEST)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<QueryRequest>())
  {
    class_ <QueryRequest> ("QueryRequest",
        doxygen::class_doc<QueryRequest>(), no_init)
      .DEF_RW_CLASS_ATTRIB (QueryRequest, enable_cached_gjk_guess    )
      .DEF_RW_CLASS_ATTRIB (QueryRequest, cached_gjk_guess           )
      .DEF_RW_CLASS_ATTRIB (QueryRequest, cached_support_func_guess  )
      .DEF_CLASS_FUNC (QueryRequest, updateGuess)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<CollisionRequest>())
  {
    class_ <CollisionRequest, bases<QueryRequest> > ("CollisionRequest",
        doxygen::class_doc<CollisionRequest>(), no_init)
      .def (dv::init<CollisionRequest>())
      .def (dv::init<CollisionRequest, const CollisionRequestFlag, size_t>())
      .DEF_RW_CLASS_ATTRIB (CollisionRequest, num_max_contacts           )
      .DEF_RW_CLASS_ATTRIB (CollisionRequest, enable_contact             )
      .DEF_RW_CLASS_ATTRIB (CollisionRequest, enable_distance_lower_bound)
      .DEF_RW_CLASS_ATTRIB (CollisionRequest, security_margin            )
      .DEF_RW_CLASS_ATTRIB (CollisionRequest, break_distance             )
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<CollisionRequest> >())
  {
    class_< std::vector<CollisionRequest> >("StdVec_CollisionRequest")
      .def(vector_indexing_suite< std::vector<CollisionRequest> >())
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<Contact>())
  {
    class_ <Contact> ("Contact",
        doxygen::class_doc<Contact>(), init<>(arg("self"),"Default constructor"))
      //.def(init<CollisionGeometryPtr_t, CollisionGeometryPtr_t, int, int>())
      //.def(init<CollisionGeometryPtr_t, CollisionGeometryPtr_t, int, int, Vec3f, Vec3f, FCL_REAL>())
      .add_property ("o1",
          make_function(&geto<1>, return_value_policy<reference_existing_object>()),
          doxygen::class_attrib_doc<Contact>("o1"))
      .add_property ("o2",
          make_function(&geto<2>, return_value_policy<reference_existing_object>()),
          doxygen::class_attrib_doc<Contact>("o2"))
      .DEF_RW_CLASS_ATTRIB (Contact, b1)
      .DEF_RW_CLASS_ATTRIB (Contact, b2)
      .DEF_RW_CLASS_ATTRIB (Contact, normal)
      .DEF_RW_CLASS_ATTRIB (Contact, pos)
      .DEF_RW_CLASS_ATTRIB (Contact, penetration_depth)
      .def (self == self)
      .def (self != self)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<Contact> >())
  {
    class_< std::vector<Contact> >("StdVec_Contact")
      .def(vector_indexing_suite< std::vector<Contact> >())
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type<QueryResult>())
  {
    class_ <QueryResult> ("QueryResult",
        doxygen::class_doc<QueryResult>(), no_init)
      .DEF_RW_CLASS_ATTRIB (QueryResult, cached_gjk_guess         )
      .DEF_RW_CLASS_ATTRIB (QueryResult, cached_support_func_guess)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type< CollisionResult >())
  {
    class_ <CollisionResult, bases<QueryResult> > ("CollisionResult",
        doxygen::class_doc<CollisionResult>(), no_init)
      .def (dv::init<CollisionResult>())
      .DEF_CLASS_FUNC (CollisionResult, isCollision)
      .DEF_CLASS_FUNC (CollisionResult, numContacts)
      .DEF_CLASS_FUNC (CollisionResult, addContact)
      .DEF_CLASS_FUNC (CollisionResult, clear)
      .DEF_CLASS_FUNC2 (CollisionResult, getContact , return_value_policy<copy_const_reference>())
      .DEF_CLASS_FUNC2 (CollisionResult, getContacts, return_internal_reference<>())

      .DEF_RW_CLASS_ATTRIB(CollisionResult, distance_lower_bound)
      ;
  }

  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<CollisionResult> >())
  {
    class_< std::vector<CollisionResult> >("StdVec_CollisionResult")
      .def(vector_indexing_suite< std::vector<CollisionResult> >())
      ;
  }

  doxygen::def ("collide", static_cast< std::size_t (*)(const CollisionObject*, const CollisionObject*,
        const CollisionRequest&, CollisionResult&) > (&collide));
  doxygen::def ("collide", static_cast< std::size_t (*)(
        const CollisionGeometry*, const Transform3f&,
        const CollisionGeometry*, const Transform3f&,
        CollisionRequest&, CollisionResult&) > (&collide));

  class_<ComputeCollision> ("ComputeCollision",
      doxygen::class_doc<ComputeCollision>(), no_init)
    .def (dv::init<ComputeCollision, const CollisionGeometry*, const CollisionGeometry*>())
    .def ("__call__", static_cast< std::size_t (ComputeCollision::*)(
        const Transform3f&, const Transform3f&,
        CollisionRequest&, CollisionResult&) > (&ComputeCollision::operator()));

}
