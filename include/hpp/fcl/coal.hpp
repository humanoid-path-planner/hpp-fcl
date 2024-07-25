#ifndef HPP_FCL_COAL_HPP
#define HPP_FCL_COAL_HPP

#include <coal/config.hh>
#include <coal/deprecated.hh>

#define COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL

#ifdef _MSC_VER
#pragma message COAL_DEPRECATED_HEADER( \
    "Please update your includes from 'hpp/fcl' to 'coal'")
#else
#warning "Please update your includes from 'hpp/fcl' to 'coal'"
#endif

#define HPP_FCL_VERSION_AT_LEAST(major, minor, patch) \
  COAL_VERSION_AT_LEAST(major, minor, patch)

#endif  // COAL_FWD_HPP
