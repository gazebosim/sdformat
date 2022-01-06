cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(usdSdfConversions)

set(SDF_VER 12)
find_package(sdformat${SDF_VER} COMPONENTS usd REQUIRED)

add_executable(usdConverter usdConverter.cc)

target_link_libraries(usdConverter
  PUBLIC
    sdformat${SDF_VER}::usd
)
