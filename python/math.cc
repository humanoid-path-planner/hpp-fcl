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

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/math/transform.h>

#include "fcl.hh"

using namespace boost::python;

using namespace hpp::fcl;

struct TriangleWrapper
{
  static Triangle::index_type getitem (const Triangle& t, int i)
  {
    if (i >= 3) throw std::out_of_range("index is out of range");
    return t[i];
  }
  static void setitem (Triangle& t, int i, Triangle::index_type v)
  {
    if (i >= 3) throw std::out_of_range("index is out of range");
    t[i] = v;
  }
};

void exposeMaths ()
{
  eigenpy::enableEigenPy();

  if(!eigenpy::register_symbolic_link_to_registered_type<Eigen::Quaterniond>())
    eigenpy::exposeQuaternion();
  if(!eigenpy::register_symbolic_link_to_registered_type<Eigen::AngleAxisd>())
    eigenpy::exposeAngleAxis();

  eigenpy::enableEigenPySpecific<Matrix3f>();
  eigenpy::enableEigenPySpecific<Vec3f   >();

  class_ <Transform3f> ("Transform3f", init<>())
    .def (init<Matrix3f, Vec3f>())
    .def (init<Quaternion3f, Vec3f>())
    .def (init<Matrix3f>())
    .def (init<Quaternion3f>())
    .def (init<Vec3f>())

    .def ("getQuatRotation", &Transform3f::getQuatRotation)
    .def ("getTranslation", &Transform3f::getTranslation, return_value_policy<copy_const_reference>())
    .def ("getRotation", &Transform3f::getRotation, return_value_policy<copy_const_reference>())
    .def ("isIdentity", &Transform3f::setIdentity)

    .def ("setQuatRotation", &Transform3f::setQuatRotation)
    .def ("setTranslation", &Transform3f::setTranslation<Vec3f>)
    .def ("setRotation", &Transform3f::setRotation<Matrix3f>)
    .def ("setTransform", &Transform3f::setTransform<Matrix3f,Vec3f>)
    .def ("setTransform", static_cast<void (Transform3f::*)(const Quaternion3f&, const Vec3f&)>(&Transform3f::setTransform))
    .def ("setIdentity", &Transform3f::setIdentity)

    .def ("transform", &Transform3f::transform<Vec3f>)
    .def ("inverseInPlace", &Transform3f::inverseInPlace, return_internal_reference<>())
    .def ("inverse", &Transform3f::inverse)
    .def ("inverseTimes", &Transform3f::inverseTimes)

    .def (self * self)
    .def (self *= self)
    .def (self == self)
    .def (self != self)
    ;

  class_ <Triangle> ("Triangle", init<>())
    .def (init <Triangle::index_type, Triangle::index_type, Triangle::index_type>())
    .def ("__getitem__", &TriangleWrapper::getitem)
    .def ("__setitem__", &TriangleWrapper::setitem)
    ;
}
