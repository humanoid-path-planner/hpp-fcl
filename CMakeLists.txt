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

set(PROJECT_NAME hpp-fcl)
set(PROJECT_DESCRIPTION
  "FCL -- The Flexible Collision Library"
  )
set(PROJECT_URL "http://github.com/humanoid-path-planner/hpp-fcl")

setup_project()

set(FCL_HAVE_SSE FALSE CACHE BOOL "Enable SSE vectorization")

# add_optional_dependency("eigen3 >= 3.0.0")
add_required_dependency("eigen3 >= 3.0.0")
set(FCL_HAVE_EIGEN TRUE CACHE BOOL "Use eigen matrix type when possible")
include_directories(${EIGEN3_INCLUDE_DIRS})

# Required dependencies
add_required_dependency("ccd >= 1.4")
set(BOOST_COMPONENTS thread date_time filesystem system unit_test_framework)
search_for_boost()
# Optional dependencies
add_optional_dependency("octomap >= 1.6")
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
	include_directories(${OCTOMAP_INCLUDE_DIRS})
	link_directories(${OCTOMAP_LIBRARY_DIRS})
	set(FCL_HAVE_OCTOMAP 1)
	message(STATUS "FCL uses Octomap")
else()
	message(STATUS "FCL does not use Octomap")
endif()
# flann package ill defined: comment.
# add_optional_dependency("flann >= 1.7")
# if (${FLANN_FOUND})
#   add_definitions(-DFCL_HAVE_FLANN=1)
# endif()

SET(${PROJECT_NAME}_HEADERS
  include/hpp/fcl/collision_data.h
  include/hpp/fcl/profile.h
  include/hpp/fcl/exception.h
  include/hpp/fcl/ccd/taylor_vector.h
  include/hpp/fcl/ccd/interval_vector.h
  include/hpp/fcl/ccd/simplex.h
  include/hpp/fcl/ccd/support.h
  include/hpp/fcl/ccd/interval_matrix.h
  include/hpp/fcl/ccd/interval.h
  include/hpp/fcl/ccd/interpolation/interpolation_factory.h
  include/hpp/fcl/ccd/interpolation/interpolation_linear.h
  include/hpp/fcl/ccd/interpolation/interpolation.h
  include/hpp/fcl/ccd/conservative_advancement.h
  include/hpp/fcl/ccd/taylor_model.h
  include/hpp/fcl/ccd/taylor_matrix.h
  include/hpp/fcl/ccd/motion_base.h
  include/hpp/fcl/ccd/motion.h
  include/hpp/fcl/deprecated.h
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
  include/hpp/fcl/narrowphase/gjk_libccd.h
  include/hpp/fcl/broadphase/interval_tree.h
  include/hpp/fcl/broadphase/broadphase_spatialhash.h
  include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h
  include/hpp/fcl/broadphase/broadphase_SSaP.h
  include/hpp/fcl/broadphase/broadphase_interval_tree.h
  include/hpp/fcl/broadphase/broadphase.h
  include/hpp/fcl/broadphase/hierarchy_tree.h
  include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h
  include/hpp/fcl/broadphase/broadphase_SaP.h
  include/hpp/fcl/broadphase/broadphase_bruteforce.h
  include/hpp/fcl/broadphase/morton.h
  include/hpp/fcl/broadphase/hash.h
  include/hpp/fcl/learning/classifier.h
  include/hpp/fcl/shape/geometric_shapes_utility.h
  include/hpp/fcl/shape/geometric_shape_to_BVH_model.h
  include/hpp/fcl/shape/geometric_shapes.h
  include/hpp/fcl/simd/simd_intersect.h
  include/hpp/fcl/simd/math_simd_details.h
  include/hpp/fcl/distance_func_matrix.h
  include/hpp/fcl/collision.h
  include/hpp/fcl/collision_node.h
  include/hpp/fcl/collision_func_matrix.h
  include/hpp/fcl/distance.h
  include/hpp/fcl/knn/greedy_kcenters.h
  include/hpp/fcl/knn/nearest_neighbors_GNAT.h
  include/hpp/fcl/knn/nearest_neighbors_flann.h
  include/hpp/fcl/knn/nearest_neighbors_sqrtapprox.h
  include/hpp/fcl/knn/nearest_neighbors_linear.h
  include/hpp/fcl/knn/nearest_neighbors.h
  include/hpp/fcl/continuous_collision.h
  include/hpp/fcl/math/vec_nf.h
  include/hpp/fcl/math/matrix_3f.h
  include/hpp/fcl/math/matrix_3fx.h
  include/hpp/fcl/math/vec_3f.h
  include/hpp/fcl/math/vec_3fx.h
  include/hpp/fcl/math/sampling.h
  include/hpp/fcl/math/math_details.h
  include/hpp/fcl/math/transform.h
  include/hpp/fcl/traversal/traversal_node_shapes.h
  include/hpp/fcl/traversal/traversal_node_setup.h
  include/hpp/fcl/traversal/traversal_recurse.h
  include/hpp/fcl/traversal/traversal_node_octree.h
  include/hpp/fcl/traversal/traversal_node_bvhs.h
  include/hpp/fcl/traversal/traversal_node_bvh_shape.h
  include/hpp/fcl/traversal/traversal_node_base.h
  include/hpp/fcl/data_types.h
  include/hpp/fcl/articulated_model/model.h
  include/hpp/fcl/articulated_model/joint_config.h
  include/hpp/fcl/articulated_model/model_config.h
  include/hpp/fcl/articulated_model/joint.h
  include/hpp/fcl/articulated_model/link.h
  include/hpp/fcl/BVH/BV_splitter.h
  include/hpp/fcl/BVH/BVH_internal.h
  include/hpp/fcl/BVH/BVH_model.h
  include/hpp/fcl/BVH/BV_fitter.h
  include/hpp/fcl/BVH/BVH_front.h
  include/hpp/fcl/BVH/BVH_utility.h
  include/hpp/fcl/intersect.h
  include/hpp/fcl/collision_object.h
  include/hpp/fcl/octree.h
  include/hpp/fcl/fwd.hh
  include/hpp/fcl/eigen/math_details.h
  include/hpp/fcl/eigen/vec_3fx.h
  include/hpp/fcl/eigen/product.h
  include/hpp/fcl/eigen/taylor_operator.h
  include/hpp/fcl/eigen/plugins/ccd/interval-matrix.h
  include/hpp/fcl/eigen/plugins/ccd/interval-vector.h
  )

add_subdirectory(src)
add_subdirectory(test)

pkg_config_append_libs("hpp-fcl")
PKG_CONFIG_APPEND_BOOST_LIBS(thread date_time filesystem system)

config_files(include/hpp/fcl/config-fcl.hh)
install(FILES ${PROJECT_BINARY_DIR}/include/hpp/fcl/config-fcl.hh
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/fcl
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )
setup_project_finalize()
