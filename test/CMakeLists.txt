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


add_fcl_test(math math.cpp)

add_fcl_test(collision collision.cpp utility.cpp)
add_fcl_test(distance distance.cpp utility.cpp)
add_fcl_test(distance_lower_bound distance_lower_bound.cpp utility.cpp)
add_fcl_test(geometric_shapes geometric_shapes.cpp utility.cpp)
#add_fcl_test(broadphase broadphase.cpp utility.cpp)
#add_fcl_test(shape_mesh_consistency shape_mesh_consistency.cpp utility.cpp)
add_fcl_test(frontlist frontlist.cpp utility.cpp)
#add_fcl_test(math math.cpp utility.cpp)

# add_fcl_test(sphere_capsule sphere_capsule.cpp)
add_fcl_test(capsule_capsule capsule_capsule.cpp utility.cpp)
add_fcl_test(box_box_distance box_box_distance.cpp utility.cpp)
add_fcl_test(simple simple.cpp)
add_fcl_test(capsule_box_1 capsule_box_1.cpp utility.cpp)
add_fcl_test(capsule_box_2 capsule_box_2.cpp utility.cpp)
add_fcl_test(obb obb.cpp)

add_fcl_test(bvh_models bvh_models.cpp utility.cpp)

add_fcl_test(profiling profiling.cpp utility.cpp)
PKG_CONFIG_USE_DEPENDENCY(profiling assimp)

add_fcl_test(gjk gjk.cpp)
if(HPP_FCL_HAVE_OCTOMAP)
  add_fcl_test(octree octree.cpp utility.cpp)
endif(HPP_FCL_HAVE_OCTOMAP)

## Benchmark
add_executable(test-benchmark benchmark.cpp utility.cpp)
target_link_libraries(test-benchmark hpp-fcl ${Boost_LIBRARIES})
