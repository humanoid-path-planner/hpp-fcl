#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2019-2023 CNRS-LAAS INRIA
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

include(${JRL_CMAKE_MODULES}/python-helpers.cmake)
include(${JRL_CMAKE_MODULES}/stubs.cmake)

ADD_CUSTOM_TARGET(python)
SET_TARGET_PROPERTIES(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

# Name of the Python library
SET(PYTHON_LIB_NAME ${PROJECT_NAME}_pywrap)

SET(${PYTHON_LIB_NAME}_HEADERS
  fwd.hh
  coal.hh
  deprecation.hh
  broadphase/fwd.hh
  broadphase/broadphase_collision_manager.hh
  broadphase/broadphase_callbacks.hh
  pickle.hh
  utils/std-pair.hh
  serializable.hh
  )

SET(ENABLE_PYTHON_DOXYGEN_AUTODOC TRUE CACHE BOOL "Enable automatic documentation of Python bindings from Doxygen documentation")

IF(    NOT ENABLE_PYTHON_DOXYGEN_AUTODOC
    OR NOT DOXYGEN_FOUND)
  SET(ENABLE_DOXYGEN_AUTODOC FALSE)
ELSE()
  SET(ENABLE_DOXYGEN_AUTODOC TRUE)

  IF(DOXYGEN_VERSION VERSION_GREATER_EQUAL 1.9.8 AND DOXYGEN_VERSION VERSION_LESS 1.11.0)
    # deactivate python doxygen automoc for doxygen 1.9.8 and 1.10.0,
    # as it incorrectly parse "const" keyword,
    # generating wrong links in C++ doc, and fail generating python doc
    # ref. https://github.com/doxygen/doxygen/issues/10797
    SET(ENABLE_DOXYGEN_AUTODOC FALSE)
    MESSAGE(AUTHOR_WARNING "automoc deactivated because of doxygen 1.10. Please use <1.10 or >=1.11.")
  ENDIF()
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

  LIST(APPEND ${PYTHON_LIB_NAME}_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
    )
ENDIF()

SET(${PYTHON_LIB_NAME}_SOURCES
  version.cc
  math.cc
  collision-geometries.cc
  collision.cc
  contact_patch.cc
  distance.cc
  coal.cc
  gjk.cc
  broadphase/broadphase.cc
  )

IF(COAL_HAS_OCTOMAP)
  LIST(APPEND ${PYTHON_LIB_NAME}_SOURCES octree.cc)
ENDIF(COAL_HAS_OCTOMAP)

ADD_LIBRARY(${PYTHON_LIB_NAME} MODULE ${${PYTHON_LIB_NAME}_SOURCES} ${${PYTHON_LIB_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PYTHON_LIB_NAME} SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(${PYTHON_LIB_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}")
IF(WIN32)
  TARGET_LINK_LIBRARIES(${PYTHON_LIB_NAME} PUBLIC ${PYTHON_LIBRARY})
ENDIF(WIN32)

ADD_DEPENDENCIES(python ${PYTHON_LIB_NAME})
ADD_HEADER_GROUP(${PYTHON_LIB_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PYTHON_LIB_NAME}_SOURCES)
IF(ENABLE_DOXYGEN_AUTODOC)
  ADD_DEPENDENCIES(${PYTHON_LIB_NAME} generate_doxygen_cpp_doc)
  TARGET_COMPILE_DEFINITIONS(${PYTHON_LIB_NAME} PRIVATE -DCOAL_HAS_DOXYGEN_AUTODOC)
ENDIF()

TARGET_LINK_LIBRARIES(${PYTHON_LIB_NAME} PUBLIC
  ${PROJECT_NAME}
  eigenpy::eigenpy
  Boost::system)

SET_TARGET_PROPERTIES(${PYTHON_LIB_NAME} PROPERTIES
  PREFIX ""
  SUFFIX "${PYTHON_EXT_SUFFIX}"
  OUTPUT_NAME "${PYTHON_LIB_NAME}"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/${PROJECT_NAME}"
  # On Windows, shared library are treat as binary
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/${PROJECT_NAME}"
  )

IF(IS_ABSOLUTE ${PYTHON_SITELIB})
  SET(ABSOLUTE_PYTHON_SITELIB ${PYTHON_SITELIB})
ELSE()
  SET(ABSOLUTE_PYTHON_SITELIB ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB})
ENDIF()
SET(${PYTHON_LIB_NAME}_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/${PROJECT_NAME})
IF(UNIX)
  GET_RELATIVE_RPATH(${${PYTHON_LIB_NAME}_INSTALL_DIR} ${PYTHON_LIB_NAME}_INSTALL_RPATH)
  SET_TARGET_PROPERTIES(${PYTHON_LIB_NAME} PROPERTIES INSTALL_RPATH "${${PYTHON_LIB_NAME}_INSTALL_RPATH}")
ENDIF()

INSTALL(TARGETS ${PYTHON_LIB_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${${PYTHON_LIB_NAME}_INSTALL_DIR})

# --- GENERATE STUBS
IF(GENERATE_PYTHON_STUBS)
  LOAD_STUBGEN()
  GENERATE_STUBS(${CMAKE_CURRENT_BINARY_DIR} ${PROJECT_NAME} ${ABSOLUTE_PYTHON_SITELIB} ${PYTHON_LIB_NAME} ${PROJECT_NAME})
ENDIF(GENERATE_PYTHON_STUBS)

# --- INSTALL SCRIPTS
SET(PYTHON_FILES
  __init__.py
  viewer.py
  windows_dll_manager.py
  )

FOREACH(python ${PYTHON_FILES})
  PYTHON_BUILD(${PROJECT_NAME} ${python})
  INSTALL(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/${python}"
    DESTINATION ${${PYTHON_LIB_NAME}_INSTALL_DIR})
ENDFOREACH(python)


if(COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL)
  python_install_on_site(hppfcl __init__.py)
  python_install_on_site(hppfcl viewer.py)
endif()
