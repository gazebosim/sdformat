################################################################################
#APPEND_TO_CACHED_STRING(_string _cacheDesc [items...])
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_STRING _string _cacheDesc)
  FOREACH (newItem ${ARGN})
    SET (${_string} "${${_string}} ${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  ENDFOREACH (newItem ${ARGN})
  #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (APPEND_TO_CACHED_STRING)

################################################################################
# APPEND_TO_CACHED_LIST (_list _cacheDesc [items...]
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_LIST _list _cacheDesc)
  SET (tempList ${${_list}})
  FOREACH (newItem ${ARGN})
    LIST (APPEND tempList ${newItem})
  ENDFOREACH (newItem ${newItem})
  SET (${_list} ${tempList} CACHE INTERNAL ${_cacheDesc} FORCE)
ENDMACRO(APPEND_TO_CACHED_LIST)

#################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
MACRO (LIST_TO_STRING _string _list)
    SET (${_string})
    FOREACH (_item ${_list})
      SET (${_string} "${${_string}} ${_item}")
    ENDFOREACH (_item)
    #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (LIST_TO_STRING)

#################################################
# BUILD ERROR macro
macro (BUILD_ERROR)
  foreach (str ${ARGN})
    SET (msg "\t${str}")
    MESSAGE (STATUS ${msg})
    APPEND_TO_CACHED_LIST(build_errors "build errors" ${msg})
  endforeach ()
endmacro (BUILD_ERROR)

#################################################
# BUILD WARNING macro
macro (BUILD_WARNING)
  foreach (str ${ARGN})
    SET (msg "\t${str}" )
    MESSAGE (STATUS ${msg} )
    APPEND_TO_CACHED_LIST(build_warnings "build warning" ${msg})
  endforeach (str ${ARGN})
endmacro (BUILD_WARNING)

#################################################
macro (sdf_add_library _name)
  set(LIBS_DESTINATION ${PROJECT_BINARY_DIR}/src)
  add_library(${_name} ${ARGN})
  set_target_properties(${_name} PROPERTIES DEFINE_SYMBOL "BUILDING_SDFORMAT_SHARED")
  if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(${_name} PUBLIC SDFORMAT_STATIC_DEFINE)
  endif()
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBS_DESTINATION})
  if (MSVC)
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${LIBS_DESTINATION})
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${LIBS_DESTINATION})
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${LIBS_DESTINATION})
  endif ()
endmacro ()

#################################################
macro (sdf_add_executable _name)
  add_executable(${_name} ${ARGN})
endmacro ()


#################################################
macro (sdf_install_includes _subdir)
  install(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT headers)
endmacro()

#################################################
macro (sdf_install_library _name)
  set_target_properties(${_name} PROPERTIES SOVERSION ${SDF_MAJOR_VERSION} VERSION ${SDF_VERSION_FULL})
  install (
    TARGETS ${_name}
    EXPORT ${_name}
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR}
    COMPONENT shlib)

# Export and install target
export(
  EXPORT ${_name}
  FILE ${PROJECT_BINARY_DIR}/cmake/${sdf_target_output_filename}
  NAMESPACE ${PROJECT_EXPORT_NAME}::)

install(
  EXPORT ${_name}
  DESTINATION ${sdf_config_install_dir}
  FILE ${sdf_target_output_filename}
  NAMESPACE ${PROJECT_EXPORT_NAME}::)

endmacro ()

#################################################
macro (sdf_install_executable _name)
  set_target_properties(${_name} PROPERTIES VERSION ${SDF_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${BIN_INSTALL_DIR})
endmacro ()

#################################################
macro (sdf_setup_unix)
endmacro()

#################################################
macro (sdf_setup_windows)
  # Need for M_PI constant
  add_definitions(-D_USE_MATH_DEFINES -DWINDOWS_LEAN_AND_MEAN)
  # And force linking to MSVC dynamic runtime
  set(CMAKE_C_FLAGS_DEBUG "/MDd ${CMAKE_C_FLAGS_DEBUG}")
  set(CMAKE_C_FLAGS_RELEASE "/MD ${CMAKE_C_FLAGS_RELEASE}")
endmacro()

#################################################
macro (sdf_setup_apple)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined -Wl,dynamic_lookup")
endmacro()

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
      gtest gtest_main ${sdf_target}
      )

    link_directories(${IGNITION-MATH_LIBRARY_DIRS})
    target_link_libraries(${BINARY_NAME} ${tinyxml2_LIBRARIES})

    if (UNIX)
      target_link_libraries(${BINARY_NAME} PRIVATE
        libgtest.a
        libgtest_main.a
        ${sdf_target}
        pthread
        ${IGNITION-MATH_LIBRARIES}
      )
    elseif(WIN32)
      target_link_libraries(${BINARY_NAME} PRIVATE
        gtest.lib
        gtest_main.lib
        ${sdf_target}
        ${IGNITION-MATH_LIBRARIES}
      )

      # Copy in sdformat library
      add_custom_command(TARGET ${BINARY_NAME}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:sdformat${SDF_MAJOR_VERSION}>
        $<TARGET_FILE_DIR:${BINARY_NAME}> VERBATIM)

    endif()

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
      --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

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
# Macro to setup supported compiler warnings
# Based on work of Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST.
include(CheckCXXCompilerFlag)

macro(filter_valid_compiler_warnings)
  foreach(flag ${ARGN})
    CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
    if(${R${flag}})
      set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${flag}")
    endif()
  endforeach()
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
