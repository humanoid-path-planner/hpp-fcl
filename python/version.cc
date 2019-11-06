//
// Copyright (c) 2018 CNRS, INRIA
//

#include "hpp/fcl/config.hh"
#include "hpp/fcl/version.h"
#include <boost/python.hpp>

using namespace hpp::fcl;

namespace bp = boost::python;

BOOST_PYTHON_FUNCTION_OVERLOADS(printVersion_overload, printVersion, 0, 1)

void exposeVersion()
{
  // Define release numbers of the current hpp-fcl version.
  bp::scope().attr("HPP_FCL_MAJOR_VERSION") = HPP_FCL_MAJOR_VERSION;
  bp::scope().attr("HPP_FCL_MINOR_VERSION") = HPP_FCL_MINOR_VERSION;
  bp::scope().attr("HPP_FCL_PATCH_VERSION") = HPP_FCL_PATCH_VERSION;

  bp::def("printVersion",printVersion,
          printVersion_overload(bp::arg("delimiter"),
                                "Returns the current version of hpp-fcl as a string.\n"
                                "The user may specify the delimiter between the different semantic numbers.")
          );

  bp::def("checkVersionAtLeast",&checkVersionAtLeast,
          bp::args("major","minor","patch"),
          "Checks if the current version of hpp-fcl is at least"
          " the version provided by the input arguments.");
}
