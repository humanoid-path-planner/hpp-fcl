# Find Boost.UnitTestFramework
FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework filesystem)

config_files(fcl_resources/config.h)

macro(add_fcl_test test_name source)
  ADD_UNIT_TEST(${test_name} ${source})
  target_link_libraries(${test_name}
    PUBLIC
    hpp-fcl
    Boost::filesystem
    utility
    )
  IF(NOT WIN32)
    target_compile_options(${test_name} PRIVATE "-Wno-c99-extensions")
  ENDIF(NOT WIN32)
  if(HPP_FCL_HAS_QHULL)
    target_compile_options(${test_name} PRIVATE -DHPP_FCL_HAS_QHULL)
  endif()
endmacro(add_fcl_test)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(utility STATIC utility.cpp)
target_link_libraries(utility PUBLIC ${PROJECT_NAME})

add_fcl_test(math math.cpp)

add_fcl_test(collision collision.cpp)
add_fcl_test(distance distance.cpp)
add_fcl_test(distance_lower_bound distance_lower_bound.cpp)
add_fcl_test(security_margin security_margin.cpp)
add_fcl_test(geometric_shapes geometric_shapes.cpp)
add_fcl_test(shape_inflation shape_inflation.cpp)
#add_fcl_test(shape_mesh_consistency shape_mesh_consistency.cpp)
add_fcl_test(frontlist frontlist.cpp)
SET_TESTS_PROPERTIES(frontlist PROPERTIES TIMEOUT 7200)

# add_fcl_test(sphere_capsule sphere_capsule.cpp)
add_fcl_test(capsule_capsule capsule_capsule.cpp)
add_fcl_test(box_box_distance box_box_distance.cpp)
add_fcl_test(box_box_collision box_box_collision.cpp)
add_fcl_test(simple simple.cpp)
add_fcl_test(capsule_box_1 capsule_box_1.cpp)
add_fcl_test(capsule_box_2 capsule_box_2.cpp)
add_fcl_test(obb obb.cpp)
add_fcl_test(convex convex.cpp)

add_fcl_test(bvh_models bvh_models.cpp)
add_fcl_test(hfields hfields.cpp)

add_fcl_test(profiling profiling.cpp)

add_fcl_test(gjk gjk.cpp)
add_fcl_test(nesterov_gjk nesterov_gjk.cpp)
add_fcl_test(gjk_convergence_criterion gjk_convergence_criterion.cpp)
if(HPP_FCL_HAS_OCTOMAP)
  add_fcl_test(octree octree.cpp)
endif(HPP_FCL_HAS_OCTOMAP)

add_fcl_test(serialization serialization.cpp)

# Broadphase
add_fcl_test(broadphase broadphase.cpp)
set_tests_properties(broadphase PROPERTIES WILL_FAIL TRUE)
add_fcl_test(broadphase_dynamic_AABB_tree broadphase_dynamic_AABB_tree.cpp)
add_fcl_test(broadphase_collision_1 broadphase_collision_1.cpp)
add_fcl_test(broadphase_collision_2 broadphase_collision_2.cpp)

## Benchmark
add_executable(test-benchmark benchmark.cpp)
target_link_libraries(test-benchmark
  PUBLIC
  utility
  Boost::filesystem
  ${PROJECT_NAME}
  )

## Python tests
IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python_unit)
ENDIF(BUILD_PYTHON_INTERFACE)
