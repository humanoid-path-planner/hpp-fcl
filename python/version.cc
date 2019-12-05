//
// Copyright (c) 2019 CNRS INRIA
//

#include "hpp/fcl/config.hh"
#include <boost/python.hpp>
#include <boost/preprocessor/stringize.hpp>

namespace bp = boost::python;

inline bool checkVersionAtLeast(unsigned int major,
                                unsigned int minor,
                                unsigned int patch)
{
  return HPP_FCL_VERSION_AT_LEAST(major, minor, patch);
}

inline bool checkVersionAtMost(unsigned int major,
                               unsigned int minor,
                               unsigned int patch)
{
  return HPP_FCL_VERSION_AT_MOST(major, minor, patch);
}

void exposeVersion()
{
  // Define release numbers of the current hpp-fcl version.
  bp::scope().attr("__version__") = BOOST_PP_STRINGIZE(HPP_FCL_MAJOR_VERSION) "."
    BOOST_PP_STRINGIZE(HPP_FCL_MINOR_VERSION) "."
    BOOST_PP_STRINGIZE(HPP_FCL_PATCH_VERSION);
  bp::scope().attr("__raw_version__") = HPP_FCL_VERSION;
  bp::scope().attr("HPP_FCL_MAJOR_VERSION") = HPP_FCL_MAJOR_VERSION;
  bp::scope().attr("HPP_FCL_MINOR_VERSION") = HPP_FCL_MINOR_VERSION;
  bp::scope().attr("HPP_FCL_PATCH_VERSION") = HPP_FCL_PATCH_VERSION;

  bp::def("checkVersionAtLeast",&checkVersionAtLeast,
          bp::args("major","minor","patch"),
          "Checks if the current version of hpp-fcl is at least"
          " the version provided by the input arguments.");

  bp::def("checkVersionAtMost",&checkVersionAtMost,
          bp::args("major","minor","patch"),
          "Checks if the current version of hpp-fcl is at most"
          " the version provided by the input arguments.");
}
