# sdformat

[![GitHub open issues](https://img.shields.io/github/issues-raw/gazebosim/sdformat.svg)](https://github.com/gazebosim/sdformat/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/gazebosim/sdformat.svg)](https://github.com/gazebosim/sdformat/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

<!--
Note: The branch name in the codecov URL & library version should be updated when forward porting
-->
Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/sdformat/tree/sdf15/graph/badge.svg)](https://codecov.io/gh/gazebosim/sdformat/tree/sdf15)
Ubuntu Jammy  | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=sdformat-ci-sdf15-noble-amd64)](https://build.osrfoundation.org/job/sdformat-ci-sdf15-noble-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=sdformat-ci-sdf15-homebrew-amd64)](https://build.osrfoundation.org/job/sdformat-ci-sdf15-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=sdformat-sdf15-clowin)](https://build.osrfoundation.org/job/sdformat-sdf15-clowin)


SDFormat is an XML file format that describes environments, objects, and robots
in a manner suitable for robotic applications. SDFormat is capable of representing
and describing different physic engines, lighting properties, terrain, static
or dynamic objects, and articulated robots with various sensors, and actuators.
The format of SDFormat is also described by XML, which facilitates updates and
allows conversion from previous versions.

## Documentation

See the [SDFormat Website](http://sdformat.org/) for a more comprehensive
description of the specification, proposals for modifications, developer
information, etc.
This website is published using some information from the
[`sdf_tutorials`](https://github.com/gazebosim/sdf_tutorials) repository.

<!--
TODO(eric.cousineau): Move installation instructions to sdf_tutorials, and link
there?
TODO(eric.cousineau): Move terminology section to sdf_tutorials?
-->

## Terminology

* **SDFormat** - The specification.
    * **SDF** - Synonym for SDFormat, though SDFormat should be preferred, as
      "SDF" is an acronym with other meanings.
* `libsdformat` - The C++ parsing code contained within this repository,
  which can be used to read SDFormat files and return a C++ interface.

[http://sdformat.org/](http://sdformat.org/)

# Installation

We recommend following the Binary Installation instructions to get up and running as quickly and painlessly as possible.

The Source Installation instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Installation

### Ubuntu

On Ubuntu systems, `apt-get` can be used to install `sdformat`:
```sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

sudo apt install libsdformat<#>-dev libsdformat<#>
```

Be sure to replace `<#>` with a number value, such as 14 or 15, depending on
which version you need, or leave it empty for version 1.

### macOS

On macOS, after installing the [Homebrew package manager](https://brew.sh),
add OSRF packages:
  ```sh
  brew tap osrf/simulation
  ```

Install sdformat:
  ```sh
  brew install sdformat<#>
  ```

Be sure to replace `<#>` with a number value, such as 14 or 15, depending on
which version you need.

### Windows

Install [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
Miniconda suffices.

Create if necessary, and activate a Conda environment:
```
conda create -n gz-ws
conda activate gz-ws
```

Install `sdformat`:
```
conda install libsdformat --channel conda-forge
```

You can view all the versions with
```
conda search libsdformat --channel conda-forge
```

and install a specific minor version with
```
conda install libsdformat=12.5.0 --channel conda-forge
```

## Source Installation


**Note:** the `main` branch is under development for `libsdformat15` and is
currently unstable. A release branch (`sdf12`, `sdf11`, `sdf10`, `sdf9`, etc.)
is recommended for most users.

## UNIX

### Prerequisites

Clone the repository
```sh
git clone https://github.com/gazebosim/sdformat -b sdf<#>
```
Be sure to replace `<#>` with a number value, such as 14 or 15, depending on
which version you need.

### Install dependencies

#### Ubuntu

```sh
cd sdformat
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | tr '\n' ' '))
```

#### macOS

```sh
brew install --only-dependencies sdformat<#>
```

Be sure to replace `<#>` with a number value, such as 14 or 15, depending on
which version you need.

### Build from Source

Standard installation can be performed in UNIX systems using the following
steps:

```sh
cd sdformat
mkdir build
cd build
cmake ..  # Consider specifying -DCMAKE_INSTALL_PREFIX=...
make install
```

sdformat supported cmake parameters at configuring time:

| Name                  | Type | Default  | Description                          |
|-----------------------|------|----------|--------------------------------------|
| `SKIP_PYBIND11`       | BOOL | False    | Skip generating Python bindings via pybind11 |
| `USE_INTERNAL_URDF`   | BOOL | False    | Use an internal copy of urdfdom 1.0.0 instead of looking for one installed in the system |
| `USE_UPSTREAM_CFLAGS` | BOOL | True     | Use the sdformat team compilation flags instead of the common set defined by cmake.      |

### Build python bindings separately from main library

If you want to build Python bindings separately from the main libsdformat library
(for example if you want to build Python bindings for multiple versions of Python),
you can invoke cmake on the `python` folder instead of the root folder.
Specify the path to the python executable with which you wish to build bindings
in the `Python3_EXECUTABLE` cmake variable.
Specify the install path for the bindings in the `CMAKE_INSTALL_PREFIX`
variable, and be sure to set your `PYTHONPATH` accordingly after install.

```bash
cd sdformat
mkdir build_python3
cd build_python3
cmake ../python \
    -DPython3_EXECUTABLE=/usr/local/bin/python3.12 \
    -DCMAKE_INSTALL_PREFIX=<prefix>
```

See the homebrew [sdformat15 formula](https://github.com/osrf/homebrew-simulation/blob/027d06f5be49da1e40d01180aedae7f76dc7ff47/Formula/sdformat15.rb#L12-L56)
for an example of building bindings for multiple versions of Python.

## Uninstallation

To uninstall the software installed with the previous steps:

```sh
cd build
make uninstall
```

## Windows

### Prerequisites

Install [Conda package management system](https://docs.conda.io/projects/conda/en/latest/user-guide/install/download.html).
Miniconda suffices.

Create if necessary, and activate a Conda environment:
```
conda create -n gz-ws
conda activate gz-ws
```

Install prerequisites:
```
conda install cmake git vcstool colcon-common-extensions ^
tinyxml2 urdfdom pybind11 -channel conda-forge
```

### Getting the sources and building from Source

Be sure to replace `<#>` with a number value, such as 14 or 15, depending on
which version you need.

1. Getting the sources

```
mkdir ws\src
cd ws
vcs import src --input https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/sdformat<#>.yaml
```

2. Build from source

Note: the Gazebo library dependencies are going to be compiled from source
with sdformat although it should be possible to install them from
conda-forge on stable Gazebo releases using the standard conda install
command.

Build the gazebo libraries needed as dependencies (skip testing to speed up the compilation)
using the [colcon](https://colcon.readthedocs.io/en/released/) tool:
```
  colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install --packages-skip sdformat<#>
```

Build sdformat with its test suite:
```
  colcon build --cmake-args -DBUILD_TESTING=ON --merge-install --packages-select sdformat<#>
```
