# sdformat

SDFormat is an XML file format that describes environments, objects, and robots
in a manner suitable for robotic applications. SDFormat is capable of representing
and describing different physic engines, lighting properties, terrain, static
or dynamic objects, and articulated robots with various sensors, and acutators.
The format of SDFormat is also described by XML, which facilitates updates and
allows conversion from previous versions.

## Documentation

See the [SDFormat Website](http://sdformat.org/) for a more comprehensive
description of the specification, proposals for modifications, developer
information, etc.
This website is published using some information from the
[`sdf_tutorials`](https://github.com/osrf/sdf_tutorials) repository.

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

## Test coverage

[![codecov](https://codecov.io/gh/osrf/sdformat/branch/master/graph/badge.svg)](https://codecov.io/gh/osrf/sdformat)

## Installation

**Note:** the `master` branch is under development for `libsdformat11` and is
currently unstable. A release branch (`sdf10`, `sdf9`, etc.) is recommended
for most users.

Standard installation can be performed in UNIX systems using the following
steps:

```sh
mkdir build/
cd build/
cmake ..  # Consider specifying -DCMAKE_INSTALL_PREFIX=...
make install
```

sdformat supported cmake parameters at configuring time:

* `USE_INTERNAL_URDF (bool) [default False]` <br/>
  Use an internal copy of urdfdom 1.0.0 instead of look for one
  installed in the system
* `USE_UPSTREAM_CFLAGS (bool) [default True]` <br/>
  Use the sdformat team compilation flags instead of the common set defined
  by cmake.

## Uninstallation

To uninstall the software installed with the previous steps:

```sh
cd build/
make uninstall
```
