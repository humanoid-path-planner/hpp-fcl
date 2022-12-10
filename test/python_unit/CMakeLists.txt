SET(${PROJECT_NAME}_PYTHON_TESTS
  geometric_shapes
  api
  collision
  collision_manager
  pickling
  )

ADD_DEPENDENCIES(build_tests hppfcl)
FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "test/python_unit/${TEST}.py" "python")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
