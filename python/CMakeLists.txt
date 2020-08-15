#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2019-2020 CNRS-LAAS INRIA
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

ADD_CUSTOM_TARGET(python)
SET_TARGET_PROPERTIES(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

# Name of the Python library
SET(LIBRARY_NAME hppfcl)

SET(${LIBRARY_NAME}_HEADERS
  fcl.hh
  )

SET(ENABLE_PYTHON_DOXYGEN_AUTODOC TRUE CACHE BOOL "Enable automatic documentation of Python bindings from Doxygen documentation")

IF(    NOT ENABLE_PYTHON_DOXYGEN_AUTODOC
    OR NOT DOXYGEN_FOUND
    OR (APPLE AND PYTHON_VERSION_MAJOR LESS 3)) # Disable documentation on APPLE for python 2.
  SET(ENABLE_DOXYGEN_AUTODOC FALSE)
ELSE()
  SET(ENABLE_DOXYGEN_AUTODOC TRUE)

  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} -c "import lxml"
    RESULT_VARIABLE _pypkg_found
    OUTPUT_QUIET
    ERROR_QUIET)
  IF(_pypkg_found EQUAL 0)
    MESSAGE(STATUS "Found Python package lxml.")
  ELSE()
    SET(ENABLE_DOXYGEN_AUTODOC FALSE)
    MESSAGE(STATUS "Python package lxml not found. Python bindings will not be documented.")
    MESSAGE(STATUS "  You can install it with: pip install lxml")
  ENDIF()

  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} -c "import pylatexenc"
    RESULT_VARIABLE _pypkg_found
    OUTPUT_QUIET
    ERROR_QUIET)
  IF(NOT _pypkg_found EQUAL 0)
    MESSAGE(STATUS "Python package pylatexenc not found.")
    MESSAGE(STATUS "  Formulas in the Python bindings documentation may look ugly.")
    MESSAGE(STATUS "  You can install it with: pip install pylatexenc")
  ENDIF()
  UNSET(_pypkg_found)
ENDIF()
IF(ENABLE_DOXYGEN_AUTODOC)
  ADD_CUSTOM_TARGET(generate_doxygen_cpp_doc
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/python/doxygen_xml_parser.py
    ${CMAKE_BINARY_DIR}/doc/doxygen-xml/index.xml
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc > ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc.log
    BYPRODUCTS
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc.log
    COMMENT "Generating Doxygen C++ documentation"
    )
  ADD_DEPENDENCIES(generate_doxygen_cpp_doc doc)

  LIST(APPEND ${LIBRARY_NAME}_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
    )
ENDIF()

SET(${LIBRARY_NAME}_SOURCES
  version.cc
  math.cc
  collision-geometries.cc
  collision.cc
  distance.cc
  fcl.cc
  gjk.cc
  )

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES} ${${LIBRARY_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}")
IF(WIN32)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${PYTHON_LIBRARY})
ENDIF(WIN32)

ADD_DEPENDENCIES(python ${LIBRARY_NAME})
ADD_HEADER_GROUP(${LIBRARY_NAME}_HEADER)
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)
IF(ENABLE_DOXYGEN_AUTODOC)
  ADD_DEPENDENCIES(${LIBRARY_NAME} generate_doxygen_cpp_doc)
  TARGET_COMPILE_DEFINITIONS(${LIBRARY_NAME} PRIVATE -DHPP_FCL_HAS_DOXYGEN_AUTODOC)
ENDIF()

TARGET_LINK_BOOST_PYTHON(${LIBRARY_NAME} PUBLIC)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC
  ${PROJECT_NAME}
  eigenpy::eigenpy
  Boost::system)

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES
  PREFIX ""
  SUFFIX "${PYTHON_EXT_SUFFIX}"
  LIBRARY_OUTPUT_NAME ${LIBRARY_NAME}
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/${LIBRARY_NAME}"
  )

IF(IS_ABSOLUTE ${PYTHON_SITELIB})
  SET(${LIBRARY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${LIBRARY_NAME})
ELSE()
  SET(${LIBRARY_NAME}_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/${LIBRARY_NAME})
ENDIF()

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${${LIBRARY_NAME}_INSTALL_DIR})

# --- INSTALL SCRIPTS
SET(PYTHON_FILES
  __init__.py
  viewer.py
  )

FOREACH(python ${PYTHON_FILES})
  PYTHON_BUILD(${LIBRARY_NAME} ${python})
  INSTALL(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/${LIBRARY_NAME}/${python}"
    DESTINATION ${${LIBRARY_NAME}_INSTALL_DIR})
ENDFOREACH(python)
