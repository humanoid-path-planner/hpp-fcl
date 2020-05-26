#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, 2020 CNRS-LAAS
#  Author: Florent Lamiraux
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

set(LIBRARY_NAME ${PROJECT_NAME})
set(${LIBRARY_NAME}_SOURCES
  collision.cpp
  distance_func_matrix.cpp
  collision_data.cpp
  collision_node.cpp
  collision_object.cpp
  BV/RSS.cpp
  BV/AABB.cpp
  BV/kIOS.cpp
  BV/kDOP.cpp
  BV/OBBRSS.cpp
  BV/OBB.cpp
  narrowphase/narrowphase.cpp
  narrowphase/gjk.cpp
  narrowphase/details.h
  shape/convex.cpp
  shape/geometric_shapes.cpp
  shape/geometric_shapes_utility.cpp
  distance_box_halfspace.cpp
  distance_box_plane.cpp
  distance_box_sphere.cpp
  distance_capsule_capsule.cpp
  distance_capsule_halfspace.cpp
  distance_capsule_plane.cpp
  distance_cone_halfspace.cpp
  distance_cone_plane.cpp
  distance_cylinder_halfspace.cpp
  distance_cylinder_plane.cpp
  distance_sphere_sphere.cpp
  distance_sphere_cylinder.cpp
  distance_sphere_halfspace.cpp
  distance_sphere_plane.cpp
  intersect.cpp
  math/transform.cpp
  traversal/traversal_recurse.cpp
  profile.cpp
  distance.cpp
  BVH/BVH_utility.cpp
  BVH/BV_fitter.cpp
  BVH/BVH_model.cpp
  BVH/BV_splitter.cpp
  collision_func_matrix.cpp
  collision_utility.cpp
  mesh_loader/assimp.cpp
  mesh_loader/loader.cpp
  )

if(HPP_FCL_HAS_QHULL)
  set(
    libqhullcpp_HEADERS
    ${Qhullcpp_PREFIX}/libqhullcpp/Coordinates.h
    ${Qhullcpp_PREFIX}/libqhullcpp/functionObjects.h
    ${Qhullcpp_PREFIX}/libqhullcpp/PointCoordinates.h
    ${Qhullcpp_PREFIX}/libqhullcpp/Qhull.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullError.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullFacet.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullFacetList.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullFacetSet.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullHyperplane.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullIterator.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullLinkedList.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullPoint.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullPoints.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullPointSet.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullQh.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullRidge.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullSet.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullSets.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullStat.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullVertex.h
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullVertexSet.h
    ${Qhullcpp_PREFIX}/libqhullcpp/RboxPoints.h
    ${Qhullcpp_PREFIX}/libqhullcpp/RoadError.h
    ${Qhullcpp_PREFIX}/libqhullcpp/RoadLogEvent.h
    )

  set(
    libqhullcpp_SOURCES
    ${Qhullcpp_PREFIX}/libqhullcpp/Coordinates.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/PointCoordinates.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/Qhull.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullFacet.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullFacetList.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullFacetSet.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullHyperplane.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullPoint.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullPointSet.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullPoints.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullQh.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullRidge.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullSet.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullStat.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullVertex.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/QhullVertexSet.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/RboxPoints.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/RoadError.cpp
    ${Qhullcpp_PREFIX}/libqhullcpp/RoadLogEvent.cpp
    ${libqhullcpp_HEADERS}
    )

  # TODO We compile qhullcpp because it is not provided in the binary package while
  # the other parts of Qhull are released.
  # Compile Qhull package as found on github leads to a link error (because it generates
  # only a static library). One should be careful that the version of the qhull submodule
  # of this git repo matches the version of qhull provided by the system.
  list(APPEND ${LIBRARY_NAME}_SOURCES ${libqhullcpp_SOURCES})
endif()

SET(PROJECT_HEADERS_FULL_PATH)
FOREACH(header ${${PROJECT_NAME}_HEADERS})
  LIST(APPEND PROJECT_HEADERS_FULL_PATH ${PROJECT_SOURCE_DIR}/${header})
ENDFOREACH()
LIST(APPEND PROJECT_HEADERS_FULL_PATH ${PROJECT_BINARY_DIR}/include/hpp/fcl/config.hh)
add_library(${LIBRARY_NAME}
  SHARED
  ${PROJECT_HEADERS_FULL_PATH}
  ${${LIBRARY_NAME}_SOURCES}
  )

# IDE sources and headers sorting
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)
ADD_HEADER_GROUP(PROJECT_HEADERS_FULL_PATH)

TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  PUBLIC
  ${Boost_THREAD_LIBRARY}
  ${Boost_DATE_TIME_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
#  Boost::thread
#  Boost::date_time
#  Boost::system
  )

if(HPP_FCL_HAS_QHULL)
  target_compile_definitions(${LIBRARY_NAME} PRIVATE -DHPP_FCL_HAS_QHULL)
  target_include_directories(${LIBRARY_NAME} SYSTEM PRIVATE
    ${Qhull_r_INCLUDE_DIR} ${Qhullcpp_PREFIX})
  target_link_libraries(${LIBRARY_NAME} PRIVATE "${Qhull_r_LIBRARY}")
endif()

target_include_directories(${LIBRARY_NAME}
  SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} assimp)
if (NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
  target_compile_definitions(${LIBRARY_NAME} PRIVATE -DHPP_FCL_USE_ASSIMP_UNIFIED_HEADER_NAMES)
  message(STATUS "Assimp version has unified headers")
else()
  message(STATUS "Assimp version does not have unified headers")
endif()
IF(OCTOMAP_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} octomap)
  target_compile_definitions (${LIBRARY_NAME} PUBLIC
    -DHPP_FCL_HAVE_OCTOMAP
    -DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
    -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
    -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION})
ENDIF(OCTOMAP_FOUND)

install(TARGETS ${LIBRARY_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${CMAKE_INSTALL_LIBDIR})
