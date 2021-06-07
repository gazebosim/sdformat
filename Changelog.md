## SDFormat 6.0

### SDFormat 6.X.X (20XX-XX-XX)

1. Move recursiveSameTypeUniqueNames from ign.cc to parser.cc and make public.
    * [Pull request 580](https://github.com/osrf/sdformat/pull/580)

1. Parse rpyOffset as radians
    * [Pull request 497](https://github.com/osrf/sdformat/pull/497)

1. Parse urdf files to sdf 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Fix ign library path on macOS.
    * [BitBucket pull request 552](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/552)

1. Use `ign sdf --check` to check sibling elements of the same type for non-unique names.
    * [BitBucket pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [BitBucket pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### SDFormat 6.2.0 (2019-01-17)

1. Add geometry for sonar collision shape
    * [BitBucket pull request 495](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/495)

1. Add camera intrinsics (fx, fy, cx, cy, s)
    * [BitBucket pull request 496](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/496)

1. Add actor trajectory tension parameter
    * [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)


### SDFormat 6.1.0 (2018-10-04)

1. Add collision\_detector to dart physics config
    * [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix Windows support for SDFormat6
    * [BitBucket pull request 401](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/401)

1. root.sdf: default sdf version 1.6
    * [BitBucket pull request 425](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/425)

1. parser\_urdf: print value of highstop instead of pointer address
    * [BitBucket pull request 408](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/408)

1. Tweak error output so jenkins doesn't think it's a compiler warning
    * [BitBucket pull request 402](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/402)


### SDFormat 6.0.0 (2018-01-25)

1. SDF DOM: Added a document object model.
    * [BitBucket pull request 387](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/387)
    * [BitBucket pull request 389](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/389)

1. Add simplified ``readFile`` function.
    * [BitBucket pull request 347](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/347)

1. Remove boost::lexical cast instances
    * [BitBucket pull request 342](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/342)

1. Remove boost regex and iostreams as dependencies
    * [BitBucket pull request 302](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/302)

1. Change certain error checks from asserts to throwing
   sdf::AssertionInternalError, which is more appropriate for a library.
    * [BitBucket pull request 315](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/315)

1. Updated the internal copy of urdfdom to 1.0, removing more of boost.
    * [BitBucket pull request 324](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/324)

1. urdfdom 1.0 is now required on all platforms.
    * [BitBucket pull request 324](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/324)

1. Remove boost filesystem as a dependency
    * [BitBucket pull request 335](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/335)
    * [BitBucket pull request 338](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/338)
    * [BitBucket pull request 339](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/339)

1. Deprecated sdf::Color, and switch to use ignition::math::Color
    * [BitBucket pull request 330](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/330)

## SDFormat 5.x

### SDFormat 5.x.x (2017-xx-xx)

### SDFormat 5.3.0 (2017-11-13)

1. Added wrapper around root SDF for an SDF element
    * [BitBucket pull request 378](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/378)
    * [BitBucket pull request 372](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/372)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [BitBucket pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/343)

1. Add preserveFixedJoint option to the URDF parser
    * [BitBucket pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/352)

1. Add light as child of link
    * [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/373)

### SDFormat 5.2.0 (2017-08-03)

1. Added a block for DART-specific physics properties.
    * [BitBucket pull request 369](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/369)

1. Fix parser to read plugin child elements within an `<include>`
    * [BitBucket pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/350)

1. Choosing models with more recent sdf version with `<include>` tag
    * [BitBucket pull request 291](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/291)
    * [Issue 123](https://github.com/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/318)

1. Support light insertion in state
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/325)

1. Case insensitive boolean strings
    * [BitBucket pull request 322](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/322)

1. Enable coverage testing
    * [BitBucket pull request 317](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/317)

1. Add `friction_model` parameter to ode solver
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/294)
    * [Gazebo pull request 1522](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-request/1522)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [BitBucket pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/292)

### SDFormat 5.1.0 (2017-02-22)

1. Fixed `sdf::convertFile` and `sdf::convertString` always converting to latest version
    * [BitBucket pull request 320](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/320)
1. Added back the ability to set sdf version at runtime
    * [BitBucket pull request 307](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/307)

### SDFormat 5.0.0 (2017-01-25)

1. Removed SDFormat 4 deprecations
    * [BitBucket pull request 295](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/295)

1. Added an example
    * [BitBucket pull request 275](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/275)

1. Move functions that use TinyXML classes in private headers
   A contribution from Silvio Traversaro
    * [BitBucket pull request 262](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/262)

1. Fix issues found by the Coverity tool
   A contribution from Olivier Crave
    * [BitBucket pull request 259](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/259)

1. Add tag to allow for specification of initial joint position
    * [BitBucket pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/279)

1. Require ignition-math3 as dependency
    * [BitBucket pull request 299](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/299)

1. Simplifier way of retrieving a value from SDF using Get
    * [BitBucket pull request 285](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/285)

## SDFormat 4.0

### SDFormat 4.x.x (2017-xx-xx)

### SDFormat 4.4.0 (2017-10-26)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [BitBucket pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/343)

1. Add preserveFixedJoint option to the URDF parser
    * [BitBucket pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/352)

1. Add light as child of link
    * [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

### SDFormat 4.3.2 (2017-07-19)

1. Add documentation for `Element::GetFirstElement()` and `Element::GetNextElement()`
    * [BitBucket pull request 341](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/341)

1. Fix parser to read plugin child elements within an `<include>`
    * [BitBucket pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/350)

### SDFormat 4.3.1 (2017-03-24)

1. Fix segmentation Fault in `sdf::getBestSupportedModelVersion`
    * [BitBucket pull request 327](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/327)
    * [Issue 152](https://github.com/osrf/sdformat/issues/152)

### SDFormat 4.3.0 (2017-03-20)

1. Choosing models with more recent sdf version with `<include>` tag
    * [BitBucket pull request 291](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/291)
    * [Issue 123](https://github.com/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/318)

1. Support light insertion in state
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/325)

1. Case insensitive boolean strings
    * [BitBucket pull request 322](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/322)

1. Enable coverage testing
    * [BitBucket pull request 317](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/317)

1. Add `friction_model` parameter to ode solver
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)
    * [Gazebo pull request 1522](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1522)

1. Added `sampling` parameter to `<heightmap>` SDF element.
    * [BitBucket pull request 293](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/293)

1. Added Migration guide
    * [BitBucket pull request 290](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/290)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [BitBucket pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/292)

### SDFormat 4.2.0 (2016-10-10)

1. Added tag to specify ODE friction model.
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)

1. Fix URDF to SDF `self_collide` bug.
    * [BitBucket pull request 287](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/287)

1. Added IMU orientation specification to SDF.
    * [BitBucket pull request 284](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/284)

### SDFormat 4.1.1 (2016-07-08)

1. Added documentation and animation to `<actor>` element.
    * [BitBucket pull request 280](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/280)

1. Added tag to specify initial joint position
    * [BitBucket pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/279)

### SDFormat 4.1.0 (2016-04-01)

1. Added SDF conversion functions to parser including sdf::convertFile and sdf::convertString.
    * [BitBucket pull request 266](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/266)

1. Added an upload script
    * [BitBucket pull request 256](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/256)

### SDFormat 4.0.0 (2015-01-12)

1. Boost pointers and boost::function in the public API have been replaced
   by their std::equivalents (C++11 standard)
1. Move gravity and magnetic_field tags from physics to world
    * [BitBucket pull request 247](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/247)
1. Switch lump link prefix from lump:: to lump_
    * [BitBucket pull request 245](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/245)
1. New <wind> element.
   A contribution from Olivier Crave
    * [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)
1. Add scale to model state
    * [BitBucket pull request 246](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/246)
1. Use stof functions to parse hex strings as floating point params.
   A contribution from Rich Mattes
    * [BitBucket pull request 250](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/250)
1. Fix memory leaks.
   A contribution from Silvio Traversaro
    * [BitBucket pull request 249](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/249)
1. Update SDF to version 1.6: new style for representing the noise properties
   of an `imu`
    * [BitBucket pull request 243](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/243)
    * [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)

## SDFormat 3.0

### SDFormat 3.X.X (201X-XX-XX)

1. Improve precision of floating point parameters
    * [BitBucket pull request 273](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/273)
    * [BitBucket pull request 276](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/276)

### SDFormat 3.7.0 (2015-11-20)

1. Add spring pass through for sdf3
    * [Design document](https://github.com/osrf/gazebo_design/pull-requests/23)
    * [BitBucket pull request 242](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/242)

1. Support frame specification in SDF
    * [BitBucket pull request 237](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/237)

1. Remove boost from SDFExtension
    * [BitBucket pull request 229](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/229)

### SDFormat 3.6.0 (2015-10-27)

1. Add light state
    * [BitBucket pull request 227](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/227)
1. redo pull request #222 for sdf3 branch
    * [BitBucket pull request 232](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/232)
1. Fix links in API documentation
    * [BitBucket pull request 231](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/231)

### SDFormat 3.5.0 (2015-10-07)

1. Camera lens description (Replaces #213)
    * [BitBucket pull request 215](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/215)
1. Fix shared pointer reference loop in Element and memory leak (#104)
    * [BitBucket pull request 230](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/230)

### SDFormat 3.4.0 (2015-10-05)

1. Support nested model states
    * [BitBucket pull request 223](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/223)
1. Cleaner way to set SDF_PATH for tests
    * [BitBucket pull request 226](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/226)

### SDFormat 3.3.0 (2015-09-15)

1. Windows Boost linking errors
    * [BitBucket pull request 206](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/206)
1. Nested SDF -> sdf3
    * [BitBucket pull request 221](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/221)
1. Pointer types
    * [BitBucket pull request 218](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/218)
1. Torsional friction default surface radius not infinity
    * [BitBucket pull request 217](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/217)

### SDFormat 3.2.2 (2015-08-24)

1. Added battery element (contribution from Olivier Crave)
    * [BitBucket pull request #204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/204)
1. Torsional friction backport
    * [BitBucket pull request #211](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/211)
1. Allow Visual Studio 2015
    * [BitBucket pull request #208](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/208)

### SDFormat 3.1.1 (2015-08-03)

1. Fix tinyxml linking error
    * [BitBucket pull request #209](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/209)

### SDFormat 3.1.0 (2015-08-02)

1. Added logical camera sensor to SDF
    * [BitBucket pull request #207](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/207)

### SDFormat 3.0.0 (2015-07-24)

1. Added battery to SDF
    * [BitBucket pull request 204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/204)
1. Added altimeter sensor to SDF
    * [BitBucket pull request #197](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/197)
1. Added magnetometer sensor to SDF
    * [BitBucket pull request 198](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/198)
1. Fix detection of XML parsing errors
    * [BitBucket pull request 190](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/190)
1. Support for fixed joints
    * [BitBucket pull request 194](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/194)
1. Adding iterations to state
    * [BitBucket pull request 188](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/188)
1. Convert to use ignition-math
    * [BitBucket pull request 173](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/173)
1. Add world origin to scene
    * [BitBucket pull request 183](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/183)
1. Fix collide bitmask
    * [BitBucket pull request 182](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/182)
1. Adding meta information to visuals
    * [BitBucket pull request 180](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/180)
1. Add projection type to gui camera
    * [BitBucket pull request 178](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/178)
1. Fix print description to include attribute description
    * [BitBucket pull request 170](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/170)
1. Add -std=c++11 flag to sdf_config.cmake.in and sdformat.pc.in, needed by downstream code
    * [BitBucket pull request 172](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/172)
1. Added boost::any accessor for Param and Element
    * [BitBucket pull request 166](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/166)
1. Remove tinyxml from dependency list
    * [BitBucket pull request 152](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/152)
1. Added self_collide element for model
    * [BitBucket pull request 149](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/149)
1. Added a collision bitmask field to sdf-1.5 and c++11 support
    * [BitBucket pull request 145](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/145)
1. Fix problems with latin locales and decimal numbers (issue #60)
    * [BitBucket pull request 147](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/147)
    * [Issue 60](https://github.com/osrf/sdformat/issues/60)

## SDFormat 2.x

1. rename cfm_damping --> implicit_spring_damper
    * [BitBucket pull request 59](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/59)
1. add gear_ratio and reference_body for gearbox joint.
    * [BitBucket pull request 62](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/62)
1. Update joint stop stiffness and dissipation
    * [BitBucket pull request 61](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/61)
1. Support for GNUInstallDirs
    * [BitBucket pull request 64](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/64)
1. `<use_true_size>` element used by DEM heightmaps
    * [BitBucket pull request 67](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/67)
1. Do not export urdf symbols in sdformat 1.4
    * [BitBucket pull request 75](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/75)
1. adding deformable properties per issue #32
    * [BitBucket pull request 78](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/78)
    * [Issue 32](https://github.com/osrf/sdformat/issues/32)
1. Support to use external URDF
    * [BitBucket pull request 77](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/77)
1. Add lighting element to visual
    * [BitBucket pull request 79](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/79)
1. SDF 1.5: add flag to fix joint axis frame #43 (gazebo issue 494)
    * [BitBucket pull request 83](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/83)
    * [Issue 43](https://github.com/osrf/sdformat/issues/43)
    * [Gazebo issue 494](https://github.com/osrf/gazebo/issues/494)
1. Implement SDF_PROTOCOL_VERSION (issue #51)
    * [BitBucket pull request 90](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/90)
1. Port sdformat to compile on Windows (MSVC)
    * [BitBucket pull request 101](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/101)
1. Separate material properties in material.sdf
    * [BitBucket pull request 104](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/104)
1. Add road textures (repeat pull request #104 for sdf_2.0)
    * [BitBucket pull request 105](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/105)
1. Add Extruded Polylines as a model
    * [BitBucket pull request 93](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/93)
1. Added polyline for sdf_2.0
    * [BitBucket pull request 106](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/106)
1. Add spring_reference and spring_stiffness tags to joint axis dynamics
    * [BitBucket pull request 102](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/102)
1. Fix actor static
    * [BitBucket pull request 110](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/110)
1. New <Population> element
    * [BitBucket pull request 112](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/112)
1. Add camera distortion element
    * [BitBucket pull request 120](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/120)
1. Inclusion of magnetic field strength sensor
    * [BitBucket pull request 123](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/123)
1. Properly add URDF gazebo extensions blobs to SDF joint elements
    * [BitBucket pull request 125](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/125)
1. Allow gui plugins to be specified in SDF
    * [BitBucket pull request 127](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/127)
1. Backport magnetometer
    * [BitBucket pull request 128](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/128)
1. Add flag for MOI rescaling to sdf 1.4
    * [BitBucket pull request 121](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/121)
1. Parse urdf joint friction parameters, add corresponding test
    * [BitBucket pull request 129](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/129)
1. Allow reading of boolean values from plugin elements.
    * [BitBucket pull request 132](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/132)
1. Implement generation of XML Schema files (issue #2)
    * [BitBucket pull request 91](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/91)
1. Fix build for OS X 10.10
    * [BitBucket pull request 135](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/135)
1. Improve performance in loading <include> SDF elements
    * [BitBucket pull request 138](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/138)
1. Added urdf gazebo extension option to disable fixed joint lumping
    * [BitBucket pull request 133](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/133)
1. Support urdfdom 0.3 (alternative to pull request #122)
    * [BitBucket pull request 141](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/141)
1. Update list of supported joint types
    * [BitBucket pull request 142](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/142)
1. Ignore unknown elements
    * [BitBucket pull request 148](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/148)
1. Physics preset attributes
    * [BitBucket pull request 146](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/146)
1. Backport fix for latin locales (pull request #147)
    * [BitBucket pull request 150](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/150)

## SDFormat 1.4

### SDFormat 1.4.8 (2013-09-06)

1. Fix inertia transformations when reducing fixed joints in URDF
    * [BitBucket pull request 48](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/48/fix-for-issue-22-reducing-inertia-across/diff)
1. Add <use_terrain_paging> element to support terrain paging in gazebo
    * [BitBucket pull request 47](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/47/add-element-inside-heightmap/diff)
1. Further reduce console output when using URDF models
    * [BitBucket pull request 46](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/46/convert-a-few-more-sdfwarns-to-sdflog-fix/diff)
    * [Commit](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/commits/b15d5a1ecc57abee6691618d02d59bbc3d1b84dc)

### SDFormat 1.4.7 (2013-08-22)

1. Direct console messages to std_err
    * [BitBucket pull request 44](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/44/fix-19-direct-all-messages-to-std_err)

### SDFormat 1.4.6 (2013-08-20)

1. Add tags for GPS sensor and sensor noise
    * [BitBucket pull request 36](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/36/gps-sensor-sensor-noise-and-spherical)
1. Add tags for wireless transmitter/receiver models
    * [BitBucket pull request 34](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/34/transceiver-support)
    * [BitBucket pull request 43](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/43/updated-description-of-the-transceiver-sdf)
1. Add tags for playback of audio files in Gazebo
    * [BitBucket pull request 26](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/26/added-audio-tags)
    * [BitBucket pull request 35](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/35/move-audio-to-link-and-playback-on-contact)
1. Add tags for simbody physics parameters
    * [BitBucket pull request 32](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/32/merging-some-updates-from-simbody-branch)
1. Log messages to a file, reduce console output
    * [BitBucket pull request 33](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/33/log-messages-to-file-8)
1. Generalize ode's <provide_feedback> element
    * [BitBucket pull request 38](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/38/add-provide_feedback-for-bullet-joint)
1. Various bug, style and test fixes

### SDFormat 1.4.5 (2013-07-23)

1. Deprecated Gazebo's internal SDF code
1. Use templatized Get functions for retrieving values from SDF files
1. Removed dependency on ROS
