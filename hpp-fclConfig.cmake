# This file provide bacward compatiblity for `find_package(hpp-fcl)`.

message(WARNING "Please update your CMake from 'hpp-fcl' to 'coal'")

find_package(coal REQUIRED)
add_library(hpp-fcl::hpp-fcl ALIAS coal::coal)
