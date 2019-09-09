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
    utility
    )
  PKG_CONFIG_USE_DEPENDENCY(${test_name} assimp)
  add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_fcl_test)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_library(utility STATIC utility.cpp)

add_fcl_test(math math.cpp)

add_fcl_test(collision collision.cpp)
add_fcl_test(distance distance.cpp)
add_fcl_test(distance_lower_bound distance_lower_bound.cpp)
add_fcl_test(geometric_shapes geometric_shapes.cpp)
#add_fcl_test(broadphase broadphase.cpp)
#add_fcl_test(shape_mesh_consistency shape_mesh_consistency.cpp)
add_fcl_test(frontlist frontlist.cpp)
#add_fcl_test(math math.cpp)

# add_fcl_test(sphere_capsule sphere_capsule.cpp)
add_fcl_test(capsule_capsule capsule_capsule.cpp)
add_fcl_test(box_box_distance box_box_distance.cpp)
add_fcl_test(simple simple.cpp)
add_fcl_test(capsule_box_1 capsule_box_1.cpp)
add_fcl_test(capsule_box_2 capsule_box_2.cpp)
add_fcl_test(obb obb.cpp)
add_fcl_test(convex convex.cpp)

add_fcl_test(bvh_models bvh_models.cpp)

add_fcl_test(profiling profiling.cpp)
PKG_CONFIG_USE_DEPENDENCY(profiling assimp)

add_fcl_test(gjk gjk.cpp)
if(HPP_FCL_HAVE_OCTOMAP)
  add_fcl_test(octree octree.cpp)
endif(HPP_FCL_HAVE_OCTOMAP)

## Benchmark
add_executable(test-benchmark benchmark.cpp)
target_link_libraries(test-benchmark hpp-fcl ${Boost_LIBRARIES} utility)
