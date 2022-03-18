#!/bin/sh -l

set -x

BUILD_DIR=`pwd`

cd /tmp

# check that we can compile USD from sources (only Focal)
# see https://github.com/ignitionrobotics/sdformat/issues/869
return_code=0
if [ "$(lsb_release -r -s)" != "20.04" ]; then
  return_code=$(($return_code + 1))
fi

mkdir cmake_test
cd cmake_test

echo "cmake_minimum_required(VERSION 3.12)" > CMakeLists.txt

cmake . || return_code=$(($return_code + $?))
if [ $return_code -eq 0 ]
then
  # compile USD from sources
  cd /tmp
  mkdir usd_binaries
  cd usd_binaries

  apt-get install libboost-all-dev libtbb-dev p7zip-full -y

  wget https://github.com/PixarAnimationStudios/USD/archive/refs/tags/v21.11.zip
  unzip v21.11.zip
  cd USD-21.11
  mkdir build
  cd build

  cmake -DCMAKE_INSTALL_PREFIX="/tmp/USD" -DCMAKE_PREFIX_PATH="/tmp/USD" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPXR_PREFER_SAFETY_OVER_SPEED=ON \
    -DPXR_ENABLE_PYTHON_SUPPORT=OFF \
    -DBUILD_SHARED_LIBS=ON  \
    -DTBB_USE_DEBUG_BUILD=OFF  \
    -DPXR_BUILD_DOCUMENTATION=OFF  \
    -DPXR_BUILD_TESTS=OFF  \
    -DPXR_BUILD_EXAMPLES=OFF  \
    -DPXR_BUILD_TUTORIALS=OFF  \
    -DPXR_BUILD_USD_TOOLS=OFF  \
    -DPXR_BUILD_IMAGING=OFF  \
    -DPXR_BUILD_USD_IMAGING=OFF  \
    -DPXR_BUILD_USDVIEW=OFF  \
    -DPXR_BUILD_ALEMBIC_PLUGIN=OFF  \
    -DPXR_BUILD_DRACO_PLUGIN=OFF  \
    -DPXR_ENABLE_MATERIALX_SUPPORT=OFF  \
    -DBoost_INCLUDE_DIR=/usr/include  \
    -DBoost_NO_BOOST_CMAKE=FALSE \
    ..

  make -j$(nproc) install
fi

cd $BUILD_DIR
