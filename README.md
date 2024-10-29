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
Test coverage | [![codecov](https://codecov.io/gh/gazebosim/sdformat/tree/sdf14/graph/badge.svg)](https://codecov.io/gh/gazebosim/sdformat/tree/sdf14)
Ubuntu Jammy  | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=sdformat-ci-sdf14-jammy-amd64)](https://build.osrfoundation.org/job/sdformat-ci-sdf14-jammy-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=sdformat-ci-sdf14-homebrew-amd64)](https://build.osrfoundation.org/job/sdformat-ci-sdf14-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=sdformat-sdf14-win)](https://build.osrfoundation.org/job/sdformat-sdf14-win)


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

Be sure to replace `<#>` with a number value, such as 2 or 3, depending on
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

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
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


**Note:** the `main` branch is under development for `libsdformat14` and is
currently unstable. A release branch (`sdf12`, `sdf11`, `sdf10`, `sdf9`, etc.)
is recommended for most users.

## UNIX

### Prerequisites

Clone the repository
```sh
git clone https://github.com/gazebosim/sdformat -b sdf<#>
```
Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
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
| `USE_INTERNAL_URDF`   | BOOL | False    | Use an internal copy of urdfdom 1.0.0 instead of looking for one installed in the system |
| `USE_UPSTREAM_CFLAGS` | BOOL | True     | Use the sdformat team compilation flags instead of the common set defined by cmake.      |

## Uninstallation

To uninstall the software installed with the previous steps:

```sh
cd build
make uninstall
```

<<<<<<< HEAD
## macOS

### Prerequisites

Clone the repository
```sh
git clone https://github.com/gazebosim/sdformat -b sdf<#>
```
Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

Install dependencies
```sh
brew install --only-dependencies sdformat<#>
```

### Build from Source

1. Configure and build
  ```sh
  cd sdformat
  mkdir build
  cd build
  cmake .. # Consider specifying -DCMAKE_INSTALL_PREFIX=...
  make
  ```

2. Optionally, install and uninstall
  ```sh
  sudo make install
  ```

  To uninstall the software installed with the previous steps:
  ```sh
  cd build/
  sudo make uninstall
  ```

=======
>>>>>>> 22684cbe (Improve installation instructions (#1490))
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
conda install tinyxml2 urdfdom --channel conda-forge
```

Install Gazebo dependencies:

You can view lists of dependencies:
```
conda search libsdformat --channel conda-forge --info
```

Install dependencies, replacing `<#>` with the desired versions:
```
conda install libgz-cmake<#> libgz-math<#> libgz-tools<#> libgz-utils<#> --channel conda-forge
```

### Build from Source

This assumes you have created and activated a Conda environment while installing the Prerequisites.

1. Configure and build
  ```
  mkdir build
  cd build
  cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
  cmake --build . --config Release
  ```

2. Install
  ```
  cmake --install . --config Release
  ```
