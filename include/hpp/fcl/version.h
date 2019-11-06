//
// Copyright (c) 2018 CNRS
//

#ifndef HPP_FCL_VERSION_H
#define HPP_FCL_VERSION_H

#include "hpp/fcl/config.hh"

#include <string>
#include <sstream>

namespace hpp {
namespace fcl {
  
  ///
  /// \brief Returns the current version of hpp-fcl as a string using
  ///        the following standard:
  ///        HPP_FCL_MINOR_VERSION.HPP_FCL_MINOR_VERSION.HPP_FCL_PATCH_VERSION
  ///
  inline std::string printVersion(const std::string & delimiter = ".")
  {
    std::ostringstream oss;
    oss
    << HPP_FCL_MAJOR_VERSION << delimiter
    << HPP_FCL_MINOR_VERSION << delimiter
    << HPP_FCL_PATCH_VERSION;
    return oss.str();
  }
  
  ///
  /// \brief Checks if the current version of hpp-fcl is at least the version provided
  ///        by the input arguments.
  ///
  /// \param[in] major_version Major version to check.
  /// \param[in] minor_version Minor version to check.
  /// \param[in] patch_version Patch version to check.
  ///
  /// \returns true if the current version of hpp-fcl is greater than the version provided
  ///        by the input arguments.
  ///
  inline bool checkVersionAtLeast(unsigned int major_version,
                                  unsigned int minor_version,
                                  unsigned int patch_version)
  {
    return
    HPP_FCL_MAJOR_VERSION > major_version
    || (HPP_FCL_MAJOR_VERSION >= major_version
        && (HPP_FCL_MINOR_VERSION > minor_version
            || (HPP_FCL_MINOR_VERSION >= minor_version
                && HPP_FCL_PATCH_VERSION >= patch_version)));
  }
} // namespace fcl
} // namespace hpp

#endif // HPP_FCL_VERSION_H
