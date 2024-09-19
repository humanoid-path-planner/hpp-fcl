# This file provide bacward compatiblity for `find_package(hpp-fcl)`.

message(WARNING "Please update your CMake from 'hpp-fcl' to 'coal'")

find_package(coal REQUIRED)

if(CMAKE_VERSION VERSION_LESS "3.18.0")
  if(NOT TARGET hpp-fcl::hpp-fcl)
    add_library(hpp-fcl::hpp-fcl SHARED IMPORTED)
    target_link_libraries(hpp-fcl::hpp-fcl INTERFACE coal::coal)
    get_property(_cfg TARGET coal::coal PROPERTY IMPORTED_CONFIGURATIONS)
    get_property(_loc TARGET coal::coal PROPERTY "IMPORTED_LOCATION_${_cfg}")
    set_property(TARGET hpp-fcl::hpp-fcl PROPERTY IMPORTED_LOCATION "${_loc}")
  endif()
else()
  add_library(hpp-fcl::hpp-fcl ALIAS coal::coal)
endif()
