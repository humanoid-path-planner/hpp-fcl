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
#include <eigenpy/geometry.hpp>

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/math/transform.h>

#include "fcl.hh"

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/hpp/fcl/math/transform.h"
#endif

#include "../doc/python/doxygen.hh"
#include "../doc/python/doxygen-boost.hh"

using namespace boost::python;
using namespace hpp::fcl;

namespace dv = doxygen::visitor;

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

  class_ <Transform3f> ("Transform3f", doxygen::class_doc<Transform3f>(), no_init)
    .def (dv::init<Transform3f>())
    .def (dv::init<Transform3f, const Matrix3f::MatrixBase&, const Vec3f::MatrixBase&>())
    .def (dv::init<Transform3f, const Quaternion3f&        , const Vec3f::MatrixBase&>())
    .def (dv::init<Transform3f, const Matrix3f&                                      >())
    .def (dv::init<Transform3f, const Quaternion3f&                                  >())
    .def (dv::init<Transform3f, const Vec3f&                                         >())
    .def (dv::init<Transform3f, const Transform3f&                                   >())

    .def (dv::member_func("getQuatRotation", &Transform3f::getQuatRotation))
    .def ("getTranslation", &Transform3f::getTranslation, doxygen::member_func_doc(&Transform3f::getTranslation), return_value_policy<copy_const_reference>())
    .def ("getRotation", &Transform3f::getRotation, return_value_policy<copy_const_reference>())
    .def (dv::member_func("isIdentity", &Transform3f::isIdentity))

    .def (dv::member_func("setQuatRotation", &Transform3f::setQuatRotation))
    .def ("setTranslation", &Transform3f::setTranslation<Vec3f>)
    .def ("setRotation", &Transform3f::setRotation<Matrix3f>)
    .def (dv::member_func("setTransform", &Transform3f::setTransform<Matrix3f,Vec3f>))
    .def (dv::member_func("setTransform", static_cast<void (Transform3f::*)(const Quaternion3f&, const Vec3f&)>(&Transform3f::setTransform)))
    .def (dv::member_func("setIdentity", &Transform3f::setIdentity))

    .def (dv::member_func("transform", &Transform3f::transform<Vec3f>))
    .def ("inverseInPlace", &Transform3f::inverseInPlace, return_internal_reference<>(), doxygen::member_func_doc(&Transform3f::inverseInPlace))
    .def (dv::member_func("inverse", &Transform3f::inverse))
    .def (dv::member_func("inverseTimes", &Transform3f::inverseTimes))

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

  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<Vec3f> >())
  {
    class_< std::vector<Vec3f> >("StdVec_Vec3f")
      .def(vector_indexing_suite< std::vector<Vec3f> >())
      ;
  }
  if(!eigenpy::register_symbolic_link_to_registered_type< std::vector<Triangle> >())
  {
    class_< std::vector<Triangle> >("StdVec_Triangle")
      .def(vector_indexing_suite< std::vector<Triangle> >())
      ;
  }
}
