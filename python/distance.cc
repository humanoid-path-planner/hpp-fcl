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
#include <hpp/fcl/distance.h>

using namespace boost::python;

using namespace hpp::fcl;
using boost::shared_ptr;
using boost::noncopyable;

void exposeDistanceAPI ()
{
  class_ <DistanceRequest> ("DistanceRequest", init<optional<bool,FCL_REAL,FCL_REAL> >())
    .def_readwrite ("enable_nearest_points", &DistanceRequest::enable_nearest_points)
    .def_readwrite ("rel_err"              , &DistanceRequest::rel_err)
    .def_readwrite ("abs_err"              , &DistanceRequest::abs_err)
    ;

  class_ <DistanceResult> ("DistanceResult", init<>())
    .def_readwrite ("min_distance", &DistanceResult::min_distance)
    .def_readwrite ("normal", &DistanceResult::normal)
    //.def_readwrite ("nearest_points", &DistanceResult::nearest_points)
    .def_readonly ("o1", &DistanceResult::o1)
    .def_readonly ("o2", &DistanceResult::o2)
    .def_readwrite ("b1", &DistanceResult::b1)
    .def_readwrite ("b2", &DistanceResult::b2)

    .def ("clear", &DistanceResult::clear)
    ;

  def ("distance", static_cast< FCL_REAL (*)(const CollisionObject*, const CollisionObject*,
        const DistanceRequest&, DistanceResult&) > (&distance));
  def ("distance", static_cast< FCL_REAL (*)(
        const CollisionGeometry*, const Transform3f&,
        const CollisionGeometry*, const Transform3f&,
        const DistanceRequest&, DistanceResult&) > (&distance));
}
