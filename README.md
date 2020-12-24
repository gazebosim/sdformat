# sdformat #

SDFormat is an XML file format that describes environments, objects, and robots
in a manner suitable for robotic applications. SDFormat is capable of representing
and describing different physic engines, lighting properties, terrain, static
or dynamic objects, and articulated robots with various sensors, and acutators.
The format of SDFormat is also described by XML, which facilitates updates and
allows conversion from previous versions.

* SDFormat - The specification.
    * SDF - Synonym for SDFormat, though SDFormat should be preferred, as "SDF"
      is an acronym with other meanings.
* libsdformat - The C++ parsing code contained within this repository,
  which can be used to read SDFormat files and return a C++ interface.

Test coverage:

[![codecov](https://codecov.io/bb/osrf/sdformat/branch/default/graph/badge.svg)](https://codecov.io/bb/osrf/sdformat)


## Installation ##

### UNIX

Standard installation can be performed in UNIX systems using the following
steps:

```
mkdir build/
cd build/
cmake ..
sudo make install
```

sdformat supported cmake parameters at configuring time:
 - USE_INTERNAL_URDF (bool) [default False]
   Use an internal copy of urdfdom 1.0.0 instead of look for one
   installed in the system
 - USE_UPSTREAM_CFLAGS (bool) [default True]
   Use the sdformat team compilation flags instead of the common set defined
   by cmake.

### Windows

Create if necessary, and activate a Conda environment:

```
conda create -n ign-ws
conda activate ign-ws
```

Install prerequisites:

```
conda install urdfdom --channel conda-forge
```

Install Ignition dependencies:

```
conda install libignition-tools1 --channel conda-forge
```

Configure and build:

```
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF  # Optionally, -DCMAKE_INSTALL_PREFIX=path\to\install
cmake --build . --config Release
```

Install:

```
cmake --install . --config Release
```

## Uninstallation ##

### UNIX

To uninstall the software installed with the previous steps:

```
cd build/
sudo make uninstall
```
