#################################################
# VAR: SDF_BUILD_TESTS_EXTRA_EXE_SRCS
# Hack: extra sources to build binaries can be supplied to gz_build_tests in
# the variable SDF_BUILD_TESTS_EXTRA_EXE_SRCS. This variable will be clean up
# at the end of the function
#
include_directories(${PROJECT_SOURCE_DIR}/test/gtest/include)
macro (sdf_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})

    add_executable(${BINARY_NAME}
      ${GTEST_SOURCE_file}
      ${SDF_BUILD_TESTS_EXTRA_EXE_SRCS}
    )

    add_dependencies(${BINARY_NAME}
      gtest gtest_main ${PROJECT_LIBRARY_TARGET_NAME}
      )

    link_directories(${IGNITION-MATH_LIBRARY_DIRS})
    target_link_libraries(${BINARY_NAME} PRIVATE
      gtest
      gtest_main
      ${IGNITION-MATH_LIBRARIES}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${tinyxml2_LIBRARIES})

    if (UNIX)
      target_link_libraries(${BINARY_NAME} PRIVATE
        pthread
      )
    elseif(WIN32)
      # Copy in sdformat library
      add_custom_command(TARGET ${BINARY_NAME}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:sdformat${PROJECT_VERSION_MAJOR}>
        $<TARGET_FILE_DIR:${BINARY_NAME}> VERBATIM)

    endif()

    add_test(NAME ${BINARY_NAME} COMMAND
      ${BINARY_NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set (_env_vars)
    set (sdf_paths)

    # Get all the sdf protocol directory names
    file(GLOB dirs RELATIVE "${PROJECT_SOURCE_DIR}/sdf"
         "${PROJECT_SOURCE_DIR}/sdf/*")
    list(SORT dirs)

    # Add each sdf protocol to the sdf_path variable
    foreach(dir ${dirs})
      if (IS_DIRECTORY ${PROJECT_SOURCE_DIR}/sdf/${dir})
        set(sdf_paths "${PROJECT_SOURCE_DIR}/sdf/${dir}:${sdf_paths}")
      endif()
    endforeach()

    # Set the SDF_PATH environment variable
    list(APPEND _env_vars "SDF_PATH=${sdf_paths}")

    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 240
      ENVIRONMENT "${_env_vars}")

    if(PYTHONINTERP_FOUND)
      # Check that the test produced a result and create a failure if it didn't.
      # Guards against crashed and timed out tests.
      add_test(check_${BINARY_NAME} ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
               ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
    endif()

    if(SDFORMAT_RUN_VALGRIND_TESTS AND VALGRIND_PROGRAM)
      add_test(memcheck_${BINARY_NAME} ${VALGRIND_PROGRAM} --leak-check=full
               --error-exitcode=1 --show-leak-kinds=all ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})
    endif()
  endforeach()

  set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS "")
endmacro()

#################################################
# Copied from catkin/cmake/empy.cmake
function(find_python_module module)
  # cribbed from http://www.cmake.org/pipermail/cmake/2011-January/041666.html
  string(TOUPPER ${module} module_upper)
  if(NOT PY_${module_upper})
    if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
      set(${module}_FIND_REQUIRED TRUE)
    endif()
    # A module's location is usually a directory, but for
    # binary modules
    # it's a .so file.
    execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
      RESULT_VARIABLE _${module}_status
      OUTPUT_VARIABLE _${module}_location
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _${module}_status)
      set(PY_${module_upper} ${_${module}_location} CACHE STRING "Location of Python module ${module}")
    endif(NOT _${module}_status)
  endif(NOT PY_${module_upper})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
endfunction(find_python_module)
