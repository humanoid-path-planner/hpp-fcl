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

cmake_minimum_required(VERSION 2.8)
set(CXX_DISABLE_WERROR TRUE)
include(cmake/base.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME fcl)
set(PROJECT_DESCRIPTION
  "FCL -- The Flexible Collision Library"
  )
set(PROJECT_URL "http://github.com/humanoid-path-planner/fcl")

setup_project()

# Required dependencies
add_required_dependency("ccd >= 1.4")
set(BOOST_COMPONENTS
  thread date_time filesystem system unit_test_framework REQUIRED
  )
search_for_boost()
# Optional dependencies
add_optional_dependency("octomap >= 1.6")
# flann package ill defined: comment.
# add_optional_dependency("flann >= 1.7")
# if (${FLANN_FOUND})
#   add_definitions(-DFCL_HAVE_FLANN=1)
# endif()

SET(${PROJECT_NAME}_HEADERS
  include/fcl/collision_data.h
  include/fcl/profile.h
  include/fcl/exception.h
  include/fcl/ccd/taylor_vector.h
  include/fcl/ccd/interval_vector.h
  include/fcl/ccd/simplex.h
  include/fcl/ccd/support.h
  include/fcl/ccd/interval_matrix.h
  include/fcl/ccd/interval.h
  include/fcl/ccd/interpolation/interpolation_factory.h
  include/fcl/ccd/interpolation/interpolation_linear.h
  include/fcl/ccd/interpolation/interpolation.h
  include/fcl/ccd/conservative_advancement.h
  include/fcl/ccd/taylor_model.h
  include/fcl/ccd/taylor_matrix.h
  include/fcl/ccd/motion_base.h
  include/fcl/ccd/motion.h
  include/fcl/deprecated.h
  include/fcl/BV/kIOS.h
  include/fcl/BV/BV.h
  include/fcl/BV/RSS.h
  include/fcl/BV/OBBRSS.h
  include/fcl/BV/BV_node.h
  include/fcl/BV/AABB.h
  include/fcl/BV/OBB.h
  include/fcl/BV/kDOP.h
  include/fcl/narrowphase/narrowphase.h
  include/fcl/narrowphase/gjk.h
  include/fcl/narrowphase/gjk_libccd.h
  include/fcl/broadphase/interval_tree.h
  include/fcl/broadphase/broadphase_spatialhash.h
  include/fcl/broadphase/broadphase_dynamic_AABB_tree.h
  include/fcl/broadphase/broadphase_SSaP.h
  include/fcl/broadphase/broadphase_interval_tree.h
  include/fcl/broadphase/broadphase.h
  include/fcl/broadphase/hierarchy_tree.h
  include/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h
  include/fcl/broadphase/broadphase_SaP.h
  include/fcl/broadphase/broadphase_bruteforce.h
  include/fcl/broadphase/morton.h
  include/fcl/broadphase/hash.h
  include/fcl/learning/classifier.h
  include/fcl/shape/geometric_shapes_utility.h
  include/fcl/shape/geometric_shape_to_BVH_model.h
  include/fcl/shape/geometric_shapes.h
  include/fcl/simd/simd_intersect.h
  include/fcl/simd/math_simd_details.h
  include/fcl/distance_func_matrix.h
  include/fcl/collision.h
  include/fcl/collision_node.h
  include/fcl/collision_func_matrix.h
  include/fcl/distance.h
  include/fcl/knn/greedy_kcenters.h
  include/fcl/knn/nearest_neighbors_GNAT.h
  include/fcl/knn/nearest_neighbors_flann.h
  include/fcl/knn/nearest_neighbors_sqrtapprox.h
  include/fcl/knn/nearest_neighbors_linear.h
  include/fcl/knn/nearest_neighbors.h
  include/fcl/continuous_collision.h
  include/fcl/math/vec_nf.h
  include/fcl/math/matrix_3f.h
  include/fcl/math/vec_3f.h
  include/fcl/math/sampling.h
  include/fcl/math/math_details.h
  include/fcl/math/transform.h
  include/fcl/traversal/traversal_node_shapes.h
  include/fcl/traversal/traversal_node_setup.h
  include/fcl/traversal/traversal_recurse.h
  include/fcl/traversal/traversal_node_octree.h
  include/fcl/traversal/traversal_node_bvhs.h
  include/fcl/traversal/traversal_node_bvh_shape.h
  include/fcl/traversal/traversal_node_base.h
  include/fcl/data_types.h
  include/fcl/articulated_model/model.h
  include/fcl/articulated_model/joint_config.h
  include/fcl/articulated_model/model_config.h
  include/fcl/articulated_model/joint.h
  include/fcl/articulated_model/link.h
  include/fcl/BVH/BV_splitter.h
  include/fcl/BVH/BVH_internal.h
  include/fcl/BVH/BVH_model.h
  include/fcl/BVH/BV_fitter.h
  include/fcl/BVH/BVH_front.h
  include/fcl/BVH/BVH_utility.h
  include/fcl/intersect.h
  include/fcl/collision_object.h
  include/fcl/octree.h
  include/fcl/fwd.hh
  )

pkg_config_append_libs("fcl")
add_subdirectory(src)
add_subdirectory(test)

setup_project_finalize()
