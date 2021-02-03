#!/bin/sh -l

set -x

# Install
make install

# Compile examples
curdir=$PWD
cd ../examples
mkdir build;
cd build;
cmake ..;
make;
./simple ../simple.sdf;
cd $curdir
