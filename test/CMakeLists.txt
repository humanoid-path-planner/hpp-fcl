config_files(fcl_resources/config.h)

macro(add_fcl_template_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ${Boost_LIBRARIES}
    )
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_fcl_template_test)

macro(add_fcl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    hpp-fcl
    ${Boost_LIBRARIES}
    )
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_fcl_test)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})


add_fcl_test(test_fcl_math test_fcl_math.cpp)

add_fcl_test(test_fcl_collision test_fcl_collision.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_distance test_fcl_distance.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_distance_lower_bound test_fcl_distance_lower_bound.cpp
  test_fcl_utility.cpp)
add_fcl_test(test_fcl_geometric_shapes test_fcl_geometric_shapes.cpp test_fcl_utility.cpp)
#add_fcl_test(test_fcl_broadphase test_fcl_broadphase.cpp test_fcl_utility.cpp)
#add_fcl_test(test_fcl_shape_mesh_consistency test_fcl_shape_mesh_consistency.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_frontlist test_fcl_frontlist.cpp test_fcl_utility.cpp)
#add_fcl_test(test_fcl_math test_fcl_math.cpp test_fcl_utility.cpp)

# add_fcl_test(test_fcl_sphere_capsule test_fcl_sphere_capsule.cpp)
add_fcl_test(test_fcl_capsule_capsule test_fcl_capsule_capsule.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_box_box_distance test_fcl_box_box_distance.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_simple test_fcl_simple.cpp)
add_fcl_test(test_fcl_capsule_box_1 test_fcl_capsule_box_1.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_capsule_box_2 test_fcl_capsule_box_2.cpp test_fcl_utility.cpp)
add_fcl_test(test_fcl_obb test_fcl_obb.cpp)

add_fcl_test(test_fcl_bvh_models test_fcl_bvh_models.cpp test_fcl_utility.cpp)

add_fcl_test(test_fcl_profiling test_fcl_profiling.cpp test_fcl_utility.cpp)
PKG_CONFIG_USE_DEPENDENCY(test_fcl_profiling assimp)

add_fcl_test(test_fcl_gjk test_fcl_gjk.cpp)
if(HPP_FCL_HAVE_OCTOMAP)
  add_fcl_test(test_fcl_octree test_fcl_octree.cpp test_fcl_utility.cpp)
endif(HPP_FCL_HAVE_OCTOMAP)

## Benchmark
add_executable(test-benchmark benchmark.cpp test_fcl_utility.cpp)
target_link_libraries(test-benchmark hpp-fcl ${Boost_LIBRARIES})
