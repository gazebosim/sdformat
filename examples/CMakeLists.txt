cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(sdformat9 REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(simple simple.cc)
target_link_libraries(simple ${SDFormat_LIBRARIES})

add_executable(dom dom.cc)
target_link_libraries(dom ${SDFormat_LIBRARIES})
