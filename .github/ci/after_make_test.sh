#!/bin/sh -l

set -x

BUILD_DIR=`pwd`

# Install
make install

# Compile examples
cd ../examples
mkdir build;
cd build;
cmake ..;
make;
./simple ../simple.sdf;

# Compile python bindings
cd $BUILD_DIR/../python
mkdir build;
cd build;
cmake ..;
make;

cd $BUILD_DIR
