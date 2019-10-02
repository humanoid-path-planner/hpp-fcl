#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2019 CNRS-LAAS
#  Author: Joseph Mirabel
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

ADD_REQUIRED_DEPENDENCY("eigenpy >= 1.2")

SET(LIBRARY_NAME hppfcl)

INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" ${PYTHON_INCLUDE_DIRS})

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  math.cc
  collision-geometries.cc
  collision.cc
  distance.cc
  fcl.cc)

TARGET_LINK_BOOST_PYTHON(${LIBRARY_NAME})
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} eigenpy)

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_NAME ${LIBRARY_NAME})

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${PYTHON_SITELIB})
