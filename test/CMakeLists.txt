# Find Boost.UnitTestFramework
FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework filesystem chrono timer)

config_files(fcl_resources/config.h)

macro(add_fcl_test test_name source)
  ADD_UNIT_TEST(${test_name} ${source})
  target_link_libraries(${test_name}
    PUBLIC
    hpp-fcl
    Boost::unit_test_framework
    Boost::chrono
    Boost::filesystem
    Boost::timer
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

IF(BUILD_TESTING)
  add_library(utility STATIC utility.cpp)
ELSE()
  add_library(utility STATIC EXCLUDE_FROM_ALL utility.cpp)
ENDIF()
target_link_libraries(utility PUBLIC ${PROJECT_NAME})

add_fcl_test(math math.cpp)

add_fcl_test(collision collision.cpp)
add_fcl_test(distance distance.cpp)
add_fcl_test(distance_lower_bound distance_lower_bound.cpp)
add_fcl_test(geometric_shapes geometric_shapes.cpp)
#add_fcl_test(broadphase broadphase.cpp)
#add_fcl_test(shape_mesh_consistency shape_mesh_consistency.cpp)
add_fcl_test(frontlist frontlist.cpp)
SET_TESTS_PROPERTIES(frontlist PROPERTIES TIMEOUT 7200)
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

add_fcl_test(gjk gjk.cpp)
if(HPP_FCL_HAVE_OCTOMAP)
  add_fcl_test(octree octree.cpp)
endif(HPP_FCL_HAVE_OCTOMAP)

## Benchmark
IF(BUILD_TESTING)
  add_executable(test-benchmark benchmark.cpp)
ELSE()
  add_executable(test-benchmark EXCLUDE_FROM_ALL benchmark.cpp)
ENDIF()
target_link_libraries(test-benchmark 
  PUBLIC 
  utility
  Boost::chrono
  Boost::filesystem
  ${PROJECT_NAME} 
  )

## Python tests
IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python_unit)
ENDIF(BUILD_PYTHON_INTERFACE)
