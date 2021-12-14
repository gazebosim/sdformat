cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(sdformat10 REQUIRED)
set(SDFORMAT_VER ${sdformat10_VERSION_MAJOR})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_executable(simple simple.cc)
target_link_libraries(simple sdformat${SDFORMAT_VER}::sdformat${SDFORMAT_VER})

add_executable(dom dom.cc)
target_link_libraries(dom sdformat${SDFORMAT_VER}::sdformat${SDFORMAT_VER})
