#!/bin/sh -l

set -x

BUILD_DIR=`pwd`

cd /tmp
mkdir usd_binaries
cd usd_binaries

apt-get install libboost-all-dev libtbb-dev p7zip-full -y

wget https://github.com/PixarAnimationStudios/USD/archive/refs/tags/v21.11.zip
unzip v21.11.zip
sed -i '2059 i \ \ \ \ requiredDependencies.remove(BOOST)' USD-21.11/build_scripts/build_usd.py
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
  -DBoost_NO_BOOST_CMAKE=On  \
  -DBoost_INCLUDE_DIR=/usr/include  \
  -DBoost_NO_BOOST_CMAKE=FALSE \
  ..

make install

cd $BUILD_DIR
