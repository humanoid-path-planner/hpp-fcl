//
// Copyright (c) 2022 CNRS INRIA
//

#ifndef HPP_FCL_PYTHON_FWD_HH
#define HPP_FCL_PYTHON_FWD_HH

#include <hpp/fcl/fwd.hh>
#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
namespace doxygen {
using hpp::fcl::shared_ptr;
}
#endif

// Silence a warning about a deprecated use of boost bind by boost python
// at least fo boost 1.73 to 1.75
// ref. https://github.com/stack-of-tasks/tsid/issues/128
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#undef BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "../doc/python/doxygen.hh"
#include "../doc/python/doxygen-boost.hh"

namespace bp = boost::python;
namespace dv = doxygen::visitor;

#define DEF_RW_CLASS_ATTRIB(CLASS, ATTRIB) \
  def_readwrite(#ATTRIB, &CLASS::ATTRIB,   \
                doxygen::class_attrib_doc<CLASS>(#ATTRIB))
#define DEF_RO_CLASS_ATTRIB(CLASS, ATTRIB) \
  def_readonly(#ATTRIB, &CLASS::ATTRIB,    \
               doxygen::class_attrib_doc<CLASS>(#ATTRIB))
#define DEF_CLASS_FUNC(CLASS, ATTRIB) \
  def(dv::member_func(#ATTRIB, &CLASS::ATTRIB))
#define DEF_CLASS_FUNC2(CLASS, ATTRIB, policy) \
  def(#ATTRIB, &CLASS::ATTRIB, doxygen::member_func_doc(&CLASS::ATTRIB), policy)

#endif  // ifndef HPP_FCL_PYTHON_FWD_HH
