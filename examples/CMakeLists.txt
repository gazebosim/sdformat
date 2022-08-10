cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(sdformat-examples)

find_package(sdformat13 REQUIRED)

include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(simple simple.cc)
target_link_libraries(simple ${SDFormat_LIBRARIES})

add_executable(dom dom.cc)
target_link_libraries(dom ${SDFormat_LIBRARIES})
