#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014 CNRS-LAAS
#  Author: Florent Lamiraux, Joseph Mirabel
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of CNRS-LAAS. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 2.8)
set(CXX_DISABLE_WERROR TRUE)
include(cmake/base.cmake)
include(cmake/eigen.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)
include(cmake/hpp.cmake)

set(PROJECT_NAME hpp-fcl)
set(PROJECT_DESCRIPTION
  "HPP fork of FCL -- The Flexible Collision Library"
  )

IF(APPLE)
  SET(CMAKE_MACOSX_RPATH TRUE)
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
     set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  endif("${isSystemDir}" STREQUAL "-1")
ENDIF(APPLE)

OPTION(BUILD_PYTHON_INTERFACE OFF)

setup_hpp_project()

add_required_dependency("eigen3 >= 3.0.0")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# Add a cache variable to allow not compiling and running tests
set (RUN_TESTS TRUE CACHE BOOL "compile and run unit tests")

# Required dependencies
set(BOOST_COMPONENTS thread date_time system)
if (RUN_TESTS)
  set(BOOST_COMPONENTS ${BOOST_COMPONENTS} filesystem unit_test_framework)
endif ()
if (BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  set(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
endif ()

search_for_boost()
# Optional dependencies
add_optional_dependency("octomap >= 1.6")
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
  include_directories(SYSTEM ${OCTOMAP_INCLUDE_DIRS})
  link_directories(${OCTOMAP_LIBRARY_DIRS})
  SET(HPP_FCL_HAVE_OCTOMAP TRUE)
  add_definitions (-DHPP_FCL_HAVE_OCTOMAP)
  string(REPLACE "." ";" VERSION_LIST ${OCTOMAP_VERSION})
  list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
  list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
  list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
  add_definitions (-DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
                   -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
                   -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
  message(STATUS "FCL uses Octomap")
else()
  SET(HPP_FCL_HAVE_OCTOMAP FALSE)
  message(STATUS "FCL does not use Octomap")
endif()

ADD_REQUIRED_DEPENDENCY("assimp >= 2.0")
if(ASSIMP_FOUND)
  if (NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
    add_definitions(-DHPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
endif()

SET(${PROJECT_NAME}_HEADERS
  include/hpp/fcl/collision_data.h
  include/hpp/fcl/profile.h
  include/hpp/fcl/BV/kIOS.h
  include/hpp/fcl/BV/BV.h
  include/hpp/fcl/BV/RSS.h
  include/hpp/fcl/BV/OBBRSS.h
  include/hpp/fcl/BV/BV_node.h
  include/hpp/fcl/BV/AABB.h
  include/hpp/fcl/BV/OBB.h
  include/hpp/fcl/BV/kDOP.h
  include/hpp/fcl/narrowphase/narrowphase.h
  include/hpp/fcl/narrowphase/gjk.h
  include/hpp/fcl/shape/geometric_shape_to_BVH_model.h
  include/hpp/fcl/shape/geometric_shapes.h
  include/hpp/fcl/distance_func_matrix.h
  include/hpp/fcl/collision.h
  include/hpp/fcl/collision_func_matrix.h
  include/hpp/fcl/distance.h
  include/hpp/fcl/math/matrix_3f.h
  include/hpp/fcl/math/vec_3f.h
  include/hpp/fcl/math/types.h
  include/hpp/fcl/math/transform.h
  include/hpp/fcl/data_types.h
  include/hpp/fcl/BVH/BVH_internal.h
  include/hpp/fcl/BVH/BVH_model.h
  include/hpp/fcl/BVH/BVH_front.h
  include/hpp/fcl/BVH/BVH_utility.h
  include/hpp/fcl/collision_object.h
  include/hpp/fcl/collision_utility.h
  include/hpp/fcl/octree.h
  include/hpp/fcl/fwd.hh
  include/hpp/fcl/mesh_loader/assimp.h
  include/hpp/fcl/mesh_loader/loader.h
  )

add_subdirectory(src)
if (BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif ()
if (RUN_TESTS)
  add_subdirectory(test)
endif ()

pkg_config_append_libs("hpp-fcl")
PKG_CONFIG_APPEND_BOOST_LIBS(thread date_time filesystem system)
IF(HPP_FCL_HAVE_OCTOMAP)
  # FCL_HAVE_OCTOMAP kept for backward compatibility reasons.
  PKG_CONFIG_APPEND_CFLAGS(
    "-DHPP_FCL_HAVE_OCTOMAP -DFCL_HAVE_OCTOMAP -DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION}")
ENDIF(HPP_FCL_HAVE_OCTOMAP)

setup_hpp_project_finalize()
