#!/bin/sh -l

set -x

BUILD_DIR=`pwd`

cd /tmp
mkdir usd_binaries
cd usd_binaries

apt install p7zip-full -y

wget https://developer.download.nvidia.com/USD/usd_binaries/21.05/usd-21-05-usd-linux64_py36-centos_release.7z

7z x usd-21-05-usd-linux64_py36-centos_release.7z
cd $BUILD_DIR
