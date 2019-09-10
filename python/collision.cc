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
#include <hpp/fcl/collision.h>

using namespace boost::python;

using namespace hpp::fcl;
using boost::shared_ptr;
using boost::noncopyable;

void exposeCollisionAPI ()
{
  enum_ <CollisionRequestFlag> ("CollisionRequestFlag")
    .value ("CONTACT", CONTACT)
    .value ("DISTANCE_LOWER_BOUND", DISTANCE_LOWER_BOUND)
    .value ("NO_REQUEST", NO_REQUEST)
    ;

  class_ <CollisionRequest> ("CollisionRequest", init<>())
    .def (init<CollisionRequestFlag, size_t>())

    .def_readwrite ("num_max_contacts"           , &CollisionRequest::num_max_contacts)
    .def_readwrite ("enable_contact"             , &CollisionRequest::enable_contact)
    .def_readwrite ("enable_distance_lower_bound", &CollisionRequest::enable_distance_lower_bound)
    .def_readwrite ("enable_cached_gjk_guess"    , &CollisionRequest::enable_cached_gjk_guess)
    .def_readwrite ("cached_gjk_guess"           , &CollisionRequest::cached_gjk_guess)
    .def_readwrite ("security_margin"            , &CollisionRequest::security_margin)
    .def_readwrite ("break_distance"             , &CollisionRequest::break_distance)
    ;

  class_ <Contact> ("Contact", no_init)
    .def_readonly ("o1", &Contact::o1)
    .def_readonly ("o2", &Contact::o2)
    .def_readwrite ("b1", &Contact::b1)
    .def_readwrite ("b2", &Contact::b2)
    .def_readwrite ("normal", &Contact::normal)
    .def_readwrite ("pos", &Contact::pos)
    .def_readwrite ("penetration_depth", &Contact::penetration_depth)
    .def (self == self)
    ;

  class_ <CollisionResult> ("CollisionResult", init<>())
    .def ("isCollision", &CollisionResult::isCollision)
    .def ("numContacts", &CollisionResult::numContacts)
    .def ("clear", &CollisionResult::clear)
    ;

  def ("collide", static_cast< std::size_t (*)(const CollisionObject*, const CollisionObject*,
        const CollisionRequest&, CollisionResult&) > (&collide));
  def ("collide", static_cast< std::size_t (*)(
        const CollisionGeometry*, const Transform3f&,
        const CollisionGeometry*, const Transform3f&,
        const CollisionRequest&, CollisionResult&) > (&collide));
}
