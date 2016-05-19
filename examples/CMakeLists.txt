cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(SDFormat REQUIRED)

include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(simple simple.cc)
target_link_libraries(simple ${SDFormat_LIBRARIES})
