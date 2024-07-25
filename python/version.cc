//
// Copyright (c) 2019-2023 CNRS INRIA
//

#include "coal/config.hh"
#include "coal.hh"
#include <boost/preprocessor/stringize.hpp>

namespace bp = boost::python;

inline bool checkVersionAtLeast(int major, int minor, int patch) {
  return COAL_VERSION_AT_LEAST(major, minor, patch);
}

inline bool checkVersionAtMost(int major, int minor, int patch) {
  return COAL_VERSION_AT_MOST(major, minor, patch);
}

void exposeVersion() {
  // Define release numbers of the current Coal version.
  bp::scope().attr("__version__") =
      BOOST_PP_STRINGIZE(COAL_MAJOR_VERSION) "." BOOST_PP_STRINGIZE(COAL_MINOR_VERSION) "." BOOST_PP_STRINGIZE(COAL_PATCH_VERSION);
  bp::scope().attr("__raw_version__") = COAL_VERSION;
  bp::scope().attr("COAL_MAJOR_VERSION") = COAL_MAJOR_VERSION;
  bp::scope().attr("COAL_MINOR_VERSION") = COAL_MINOR_VERSION;
  bp::scope().attr("COAL_PATCH_VERSION") = COAL_PATCH_VERSION;

#if COAL_HAS_QHULL
  bp::scope().attr("WITH_QHULL") = true;
#else
  bp::scope().attr("WITH_QHULL") = false;
#endif

#if COAL_HAS_OCTOMAP
  bp::scope().attr("WITH_OCTOMAP") = true;
#else
  bp::scope().attr("WITH_OCTOMAP") = false;
#endif

  bp::def("checkVersionAtLeast", &checkVersionAtLeast,
          bp::args("major", "minor", "patch"),
          "Checks if the current version of coal is at least"
          " the version provided by the input arguments.");

  bp::def("checkVersionAtMost", &checkVersionAtMost,
          bp::args("major", "minor", "patch"),
          "Checks if the current version of coal is at most"
          " the version provided by the input arguments.");
}
