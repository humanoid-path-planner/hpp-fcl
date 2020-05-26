#
#   Copyright 2020 CNRS
#
#   Author: Guilhem Saurel
#

# Try to find assimp in standard prefixes and in ${assimp_PREFIX}
# Once done this will define
#  assimp_FOUND - System has assimp
#  assimp_INCLUDE_DIR - The assimp include directories
#  assimp_LIBRARY - The libraries needed to use assimp

FIND_PATH(assimp_INCLUDE_DIR
  NAMES assimp/defs.h
  PATHS ${assimp_PREFIX} ${assimp_PREFIX}/include
  )

FIND_LIBRARY(assimp_LIBRARY
  NAMES libassimp.so
  PATHS ${assimp_PREFIX} ${assimp_PREFIX}/lib
  )

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(assimp DEFAULT_MSG assimp_LIBRARY assimp_INCLUDE_DIR)
mark_as_advanced(assimp_INCLUDE_DIR assimp_LIBRARY)
