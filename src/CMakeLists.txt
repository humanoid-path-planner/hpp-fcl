#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014 CNRS-LAAS
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
  ccd/taylor_matrix.cpp
  ccd/taylor_model.cpp
  ccd/interval.cpp
  ccd/interval_vector.cpp
  ccd/conservative_advancement.cpp
  ccd/taylor_vector.cpp
  ccd/interval_matrix.cpp
  ccd/interpolation/interpolation_linear.cpp
  ccd/interpolation/interpolation_factory.cpp
  ccd/interpolation/interpolation.cpp
  ccd/motion.cpp
  collision_data.cpp
  collision_node.cpp
  BV/RSS.cpp
  BV/AABB.cpp
  BV/kIOS.cpp
  BV/kDOP.cpp
  BV/OBBRSS.cpp
  BV/OBB.cpp
  narrowphase/gjk_libccd.cpp
  narrowphase/narrowphase.cpp
  narrowphase/gjk.cpp
  broadphase/broadphase_bruteforce.cpp
  broadphase/interval_tree.cpp
  broadphase/broadphase_SaP.cpp
  broadphase/broadphase_dynamic_AABB_tree.cpp
  broadphase/broadphase_SSaP.cpp
  broadphase/broadphase_dynamic_AABB_tree_array.cpp
  broadphase/broadphase_interval_tree.cpp
  broadphase/broadphase_spatialhash.cpp
  broadphase/hierarchy_tree.cpp
  shape/geometric_shapes.cpp
  shape/geometric_shapes_utility.cpp
  distance_capsule_capsule.cpp
  intersect.cpp
  math/transform.cpp
  math/sampling.cpp
  traversal/traversal_node_setup.cpp
  traversal/traversal_node_bvhs.cpp
  traversal/traversal_recurse.cpp
  traversal/traversal_node_base.cpp
  profile.cpp
  continuous_collision.cpp
  distance.cpp
  BVH/BVH_utility.cpp
  BVH/BV_fitter.cpp
  BVH/BVH_model.cpp
  BVH/BV_splitter.cpp
  collision_func_matrix.cpp
  )

# Declare boost include directories
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} ccd)
IF(OCTOMAP_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} octomap)
ENDIF(OCTOMAP_FOUND)
#PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} flann)


install(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
