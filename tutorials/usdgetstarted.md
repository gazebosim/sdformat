\page usdgetstarted USD Converters Get Started

# Overview

This tutorial describes how to get started using the USD converters, for now we
can only use this code compiling from source.

## Tools
Install tools needed by this tutorial:

```bash
sudo apt install python3-pip wget lsb-release gnupg curl git libcgal-dev
```

We will use `colcon` to compile the workspace:

```bash
pip3 install -U colcon-common-extensions
```

To compile the code, we need to create a workspace:

```bash
mkdir -p ~/ignition/src
cd ~/ignition/src
git clone https://github.com/ignitionrobotics/sdformat -b ahcorde/usd/prototype_main
```

## Requirements

These are the requirements to compile this code from source:

  - [ignition-math6](https://github.com/ignitionrobotics/ign-math)
  - [ignition-common4](https://github.com/ignitionrobotics/ign-common)
  - [ignition-utils1](https://github.com/ignitionrobotics/ign-utils)
  - [ignition-cmake2](https://github.com/ignitionrobotics/ign-cmake)
  - [PXR 21.11](https://github.com/PixarAnimationStudios/USD)

Before compiling it is necessary to install all the dependencies of the different
packages.

Add `packages.osrfoundation.org` to the apt sources list:

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
```

### Install requirements (Binaries)

The binary install method will use pre-built packages which are typically
available through a package management utility such as [Apt](https://wiki.debian.org/Apt).

1. Configure package repositories.
  ```bash
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update
  ```

2. Install Ignition Gazebo
  ```bash
  sudo apt-get install libignition-math6-dev libignition-common4-dev libignition-utils1-dev libignition-cmake2-dev
  ```

### From source

Let's compile the requirement from source

#### Ignition dependencies

Get the ignition dependencies:

```bash
cd ~/ignition/src
git clone https://github.com/ignitionrobotics/ign-math -b ign-math6
git clone https://github.com/ignitionrobotics/ign-common -b ign-common4
git clone https://github.com/ignitionrobotics/ign-utils -b ign-utils1
git clone https://github.com/ignitionrobotics/ign-cmake -b ign-cmake2
```

The command below will install all dependencies in Ubuntu Bionic or Focal:

```bash
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
```

See [Source Install](https://ignitionrobotics.org/docs/fortress/install_ubuntu_src) for information on
installing Ignition Gazebo from source.

### PXR

To install the PXR library please follow the instruccions in the [Github repository](https://github.com/PixarAnimationStudios/USD)

## Compile

Now that we have the ignition dependencies and `pxr` installed we can compile the
code:

```bash
cd ~/ignition
colcon build --merge-install
```

To speed up the build process, you could also disable tests by using

```bash
cd ~/ignition
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

## Using the converter

The workspace needs to be sourced every time a new terminal is used.

Run the following command to source the workspace in bash:

```bash
. ~/ignition/install/setup.bash
```

Now we can run the converter:

```bash
sdfconverter -i <usd file> -o <sdf output file>
```
