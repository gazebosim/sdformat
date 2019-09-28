## SDFormat 6.0

### SDFormat 6.X.X (20XX-XX-XX)

1. Parse urdf files to sdf 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [Pull request 575](https://bitbucket.org/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [Pull request 504](https://bitbucket.org/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [Pull request 565](https://bitbucket.org/osrf/sdformat/pull-requests/565)

1. Fix ign library path on macOS.
    * [Pull request 552](https://bitbucket.org/osrf/sdformat/pull-requests/552)

1. Use `ign sdf --check` to check sibling elements of the same type for non-unique names.
    * [Pull request 554](https://bitbucket.org/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [Pull request 551](https://bitbucket.org/osrf/sdformat/pull-requests/551)

### SDFormat 6.2.0 (2019-01-17)

1. Add geometry for sonar collision shape
    * [Pull request 495](https://bitbucket.org/osrf/sdformat/pull-requests/495)

1. Add camera intrinsics (fx, fy, cx, cy, s)
    * [Pull request 496](https://bitbucket.org/osrf/sdformat/pull-requests/496)

1. Add actor trajectory tension parameter
    * [Pull request 466](https://bitbucket.org/osrf/sdformat/pull-requests/466)


### SDFormat 6.1.0 (2018-10-04)

1. Add collision\_detector to dart physics config
    * [Pull request 440](https://bitbucket.org/osrf/sdformat/pull-requests/440)

1. Fix Windows support for SDFormat6
    * [Pull request 401](https://bitbucket.org/osrf/sdformat/pull-requests/401)

1. root.sdf: default sdf version 1.6
    * [Pull request 425](https://bitbucket.org/osrf/sdformat/pull-requests/425)

1. parser\_urdf: print value of highstop instead of pointer address
    * [Pull request 408](https://bitbucket.org/osrf/sdformat/pull-requests/408)

1. Tweak error output so jenkins doesn't think it's a compiler warning
    * [Pull request 402](https://bitbucket.org/osrf/sdformat/pull-requests/402)


### SDFormat 6.0.0 (2018-01-25)

1. SDF DOM: Added a document object model.
    * [Pull request 387](https://bitbucket.org/osrf/sdformat/pull-requests/387)
    * [Pull request 389](https://bitbucket.org/osrf/sdformat/pull-requests/389)

1. Add simplified ``readFile`` function.
    * [Pull request 347](https://bitbucket.org/osrf/sdformat/pull-requests/347)

1. Remove boost::lexical cast instances
    * [Pull request 342](https://bitbucket.org/osrf/sdformat/pull-requests/342)

1. Remove boost regex and iostreams as dependencies
    * [Pull request 302](https://bitbucket.org/osrf/sdformat/pull-requests/302)

1. Change certain error checks from asserts to throwing
   sdf::AssertionInternalError, which is more appropriate for a library.
    * [Pull request 315](https://bitbucket.org/osrf/sdformat/pull-requests/315)

1. Updated the internal copy of urdfdom to 1.0, removing more of boost.
    * [Pull request 324](https://bitbucket.org/osrf/sdformat/pull-requests/324)

1. urdfdom 1.0 is now required on all platforms.
    * [Pull request 324](https://bitbucket.org/osrf/sdformat/pull-requests/324)

1. Remove boost filesystem as a dependency
    * [Pull request 335](https://bitbucket.org/osrf/sdformat/pull-requests/335)
    * [Pull request 338](https://bitbucket.org/osrf/sdformat/pull-requests/338)
    * [Pull request 339](https://bitbucket.org/osrf/sdformat/pull-requests/339)

1. Deprecated sdf::Color, and switch to use ignition::math::Color
    * [Pull request 330](https://bitbucket.org/osrf/sdformat/pull-requests/330)

## SDFormat 5.x

### SDFormat 5.x.x (2017-xx-xx)

### SDFormat 5.3.0 (2017-11-13)

1. Added wrapper around root SDF for an SDF element
    * [Pull request 378](https://bitbucket.org/osrf/sdformat/pull-request/378)
    * [Pull request 372](https://bitbucket.org/osrf/sdformat/pull-request/372)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [Pull request 380](https://bitbucket.org/osrf/sdformat/pull-request/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [Pull request 343](https://bitbucket.org/osrf/sdformat/pull-request/343)

1. Add preserveFixedJoint option to the URDF parser
    * [Pull request 352](https://bitbucket.org/osrf/sdformat/pull-request/352)

1. Add light as child of link
    * [Pull request 373](https://bitbucket.org/osrf/sdformat/pull-request/373)

### SDFormat 5.2.0 (2017-08-03)

1. Added a block for DART-specific physics properties.
    * [Pull request 369](https://bitbucket.org/osrf/sdformat/pull-requests/369)

1. Fix parser to read plugin child elements within an `<include>`
    * [Pull request 350](https://bitbucket.org/osrf/sdformat/pull-request/350)

1. Choosing models with more recent sdf version with `<include>` tag
    * [Pull request 291](https://bitbucket.org/osrf/sdformat/pull-request/291)
    * [Issue 123](https://bitbucket.org/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [Pull request 318](https://bitbucket.org/osrf/sdformat/pull-request/318)

1. Support light insertion in state
    * [Pull request 325](https://bitbucket.org/osrf/sdformat/pull-request/325)

1. Case insensitive boolean strings
    * [Pull request 322](https://bitbucket.org/osrf/sdformat/pull-request/322)

1. Enable coverage testing
    * [Pull request 317](https://bitbucket.org/osrf/sdformat/pull-request/317)

1. Add `friction_model` parameter to ode solver
    * [Pull request 294](https://bitbucket.org/osrf/sdformat/pull-request/294)
    * [Gazebo pull request 1522](https://bitbucket.org/osrf/gazebo/pull-request/1522)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [Pull request 292](https://bitbucket.org/osrf/sdformat/pull-request/292)

### SDFormat 5.1.0 (2017-02-22)

1. Fixed `sdf::convertFile` and `sdf::convertString` always converting to latest version
    * [Pull request 320](https://bitbucket.org/osrf/sdformat/pull-requests/320)
1. Added back the ability to set sdf version at runtime
    * [Pull request 307](https://bitbucket.org/osrf/sdformat/pull-requests/307)

### SDFormat 5.0.0 (2017-01-25)

1. Removed SDFormat 4 deprecations
    * [Pull request 295](https://bitbucket.org/osrf/sdformat/pull-requests/295)

1. Added an example
    * [Pull request 275](https://bitbucket.org/osrf/sdformat/pull-requests/275)

1. Move functions that use TinyXML classes in private headers
   A contribution from Silvio Traversaro
    * [Pull request 262](https://bitbucket.org/osrf/sdformat/pull-requests/262)

1. Fix issues found by the Coverity tool
   A contribution from Olivier Crave
    * [Pull request 259](https://bitbucket.org/osrf/sdformat/pull-requests/259)

1. Add tag to allow for specification of initial joint position
    * [Pull request 279](https://bitbucket.org/osrf/sdformat/pull-requests/279)

1. Require ignition-math3 as dependency
    * [Pull request 299](https://bitbucket.org/osrf/sdformat/pull-requests/299)

1. Simplifier way of retrieving a value from SDF using Get
    * [Pull request 285](https://bitbucket.org/osrf/sdformat/pull-requests/285)

## SDFormat 4.0

### SDFormat 4.x.x (2017-xx-xx)

### SDFormat 4.4.0 (2017-10-26)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [Pull request 380](https://bitbucket.org/osrf/sdformat/pull-request/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [Pull request 343](https://bitbucket.org/osrf/sdformat/pull-request/343)

1. Add preserveFixedJoint option to the URDF parser
    * [Pull request 352](https://bitbucket.org/osrf/sdformat/pull-request/352)

1. Add light as child of link
    * [Pull request 373](https://bitbucket.org/osrf/sdformat/pull-request/373)

### SDFormat 4.3.2 (2017-07-19)

1. Add documentation for `Element::GetFirstElement()` and `Element::GetNextElement()`
    * [Pull request 341](https://bitbucket.org/osrf/sdformat/pull-request/341)

1. Fix parser to read plugin child elements within an `<include>`
    * [Pull request 350](https://bitbucket.org/osrf/sdformat/pull-request/350)

### SDFormat 4.3.1 (2017-03-24)

1. Fix segmentation Fault in `sdf::getBestSupportedModelVersion`
    * [Pull request 327](https://bitbucket.org/osrf/sdformat/pull-requests/327)
    * [Issue 152](https://bitbucket.org/osrf/sdformat/issues/152)

### SDFormat 4.3.0 (2017-03-20)

1. Choosing models with more recent sdf version with `<include>` tag
    * [Pull request 291](https://bitbucket.org/osrf/sdformat/pull-request/291)
    * [Issue 123](https://bitbucket.org/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [Pull request 318](https://bitbucket.org/osrf/sdformat/pull-request/318)

1. Support light insertion in state
    * [Pull request 325](https://bitbucket.org/osrf/sdformat/pull-request/325)

1. Case insensitive boolean strings
    * [Pull request 322](https://bitbucket.org/osrf/sdformat/pull-request/322)

1. Enable coverage testing
    * [Pull request 317](https://bitbucket.org/osrf/sdformat/pull-request/317)

1. Add `friction_model` parameter to ode solver
    * [Pull request 294](https://bitbucket.org/osrf/sdformat/pull-request/294)
    * [Gazebo pull request 1522](https://bitbucket.org/osrf/gazebo/pull-request/1522)

1. Added `sampling` parameter to `<heightmap>` SDF element.
    * [Pull request 293](https://bitbucket.org/osrf/sdformat/pull-request/293)

1. Added Migration guide
    * [Pull request 290](https://bitbucket.org/osrf/sdformat/pull-request/290)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [Pull request 292](https://bitbucket.org/osrf/sdformat/pull-request/292)

### SDFormat 4.2.0 (2016-10-10)

1. Added tag to specify ODE friction model.
    * [Pull request 294](https://bitbucket.org/osrf/sdformat/pull-request/294)

1. Fix URDF to SDF `self_collide` bug.
    * [Pull request 287](https://bitbucket.org/osrf/sdformat/pull-request/287)

1. Added IMU orientation specification to SDF.
    * [Pull request 284](https://bitbucket.org/osrf/sdformat/pull-request/284)

### SDFormat 4.1.1 (2016-07-08)

1. Added documentation and animation to `<actor>` element.
    * [Pull request 280](https://bitbucket.org/osrf/sdformat/pull-request/280)

1. Added tag to specify initial joint position
    * [Pull request 279](https://bitbucket.org/osrf/sdformat/pull-request/279)

### SDFormat 4.1.0 (2016-04-01)

1. Added SDF conversion functions to parser including sdf::convertFile and sdf::convertString.
    * [Pull request 266](https://bitbucket.org/osrf/sdformat/pull-request/266)

1. Added an upload script
    * [Pull request 256](https://bitbucket.org/osrf/sdformat/pull-request/256)

### SDFormat 4.0.0 (2015-01-12)

1. Boost pointers and boost::function in the public API have been replaced
   by their std::equivalents (C++11 standard)
1. Move gravity and magnetic_field tags from physics to world
    * [Pull request 247](https://bitbucket.org/osrf/sdformat/pull-request/247)
1. Switch lump link prefix from lump:: to lump_
    * [Pull request 245](https://bitbucket.org/osrf/sdformat/pull-request/245)
1. New <wind> element.
   A contribution from Olivier Crave
    * [Pull request 240](https://bitbucket.org/osrf/sdformat/pull-request/240)
1. Add scale to model state
    * [Pull request 246](https://bitbucket.org/osrf/sdformat/pull-request/246)
1. Use stof functions to parse hex strings as floating point params.
   A contribution from Rich Mattes
    * [Pull request 250](https://bitbucket.org/osrf/sdformat/pull-request/250)
1. Fix memory leaks.
   A contribution from Silvio Traversaro
    * [Pull request 249](https://bitbucket.org/osrf/sdformat/pull-request/249)
1. Update SDF to version 1.6: new style for representing the noise properties
   of an `imu`
    * [Pull request 243](https://bitbucket.org/osrf/sdformat/pull-request/243)
    * [Pull request 199](https://bitbucket.org/osrf/sdformat/pull-requests/199)

## SDFormat 3.0

### SDFormat 3.X.X (201X-XX-XX)

1. Improve precision of floating point parameters
     * [Pull request 273](https://bitbucket.org/osrf/sdformat/pull-requests/273)
     * [Pull request 276](https://bitbucket.org/osrf/sdformat/pull-requests/276)

### SDFormat 3.7.0 (2015-11-20)

1. Add spring pass through for sdf3
     * [Design document](https://bitbucket.org/osrf/gazebo_design/pull-requests/23)
     * [Pull request 242](https://bitbucket.org/osrf/sdformat/pull-request/242)

1. Support frame specification in SDF
     * [Pull request 237](https://bitbucket.org/osrf/sdformat/pull-request/237)

1. Remove boost from SDFExtension
     * [Pull request 229](https://bitbucket.org/osrf/sdformat/pull-request/229)

### SDFormat 3.6.0 (2015-10-27)

1. Add light state
    * [Pull request 227](https://bitbucket.org/osrf/sdformat/pull-request/227)
1. redo pull request #222 for sdf3 branch
    * [Pull request 232](https://bitbucket.org/osrf/sdformat/pull-request/232)
1. Fix links in API documentation
    * [Pull request 231](https://bitbucket.org/osrf/sdformat/pull-request/231)

### SDFormat 3.5.0 (2015-10-07)

1. Camera lens description (Replaces #213)
    * [Pull request 215](https://bitbucket.org/osrf/sdformat/pull-request/215)
1. Fix shared pointer reference loop in Element and memory leak (#104)
    * [Pull request 230](https://bitbucket.org/osrf/sdformat/pull-request/230)

### SDFormat 3.4.0 (2015-10-05)

1. Support nested model states
    * [Pull request 223](https://bitbucket.org/osrf/sdformat/pull-request/223)
1. Cleaner way to set SDF_PATH for tests
    * [Pull request 226](https://bitbucket.org/osrf/sdformat/pull-request/226)

### SDFormat 3.3.0 (2015-09-15)

1. Windows Boost linking errors
    * [Pull request 206](https://bitbucket.org/osrf/sdformat/pull-request/206)
1. Nested SDF -> sdf3
    * [Pull request 221](https://bitbucket.org/osrf/sdformat/pull-request/221)
1. Pointer types
    * [Pull request 218](https://bitbucket.org/osrf/sdformat/pull-request/218)
1. Torsional friction default surface radius not infinity
    * [Pull request 217](https://bitbucket.org/osrf/sdformat/pull-request/217)

### SDFormat 3.2.2 (2015-08-24)

1. Added battery element (contribution from Olivier Crave)
     * [Pull request #204](https://bitbucket.org/osrf/sdformat/pull-request/204)
1. Torsional friction backport
     * [Pull request #211](https://bitbucket.org/osrf/sdformat/pull-request/211)
1. Allow Visual Studio 2015
     * [Pull request #208](https://bitbucket.org/osrf/sdformat/pull-request/208)

### SDFormat 3.1.1 (2015-08-03)

1. Fix tinyxml linking error
     * [Pull request #209](https://bitbucket.org/osrf/sdformat/pull-request/209)

### SDFormat 3.1.0 (2015-08-02)

1. Added logical camera sensor to SDF
     * [Pull request #207](https://bitbucket.org/osrf/sdformat/pull-request/207)

### SDFormat 3.0.0 (2015-07-24)

1. Added battery to SDF
     * [Pull request 204](https://bitbucket.org/osrf/sdformat/pull-request/204)
1. Added altimeter sensor to SDF
     * [Pull request #197](https://bitbucket.org/osrf/sdformat/pull-request/197)
1. Added magnetometer sensor to SDF
     * [Pull request 198](https://bitbucket.org/osrf/sdformat/pull-request/198)
1. Fix detection of XML parsing errors
     * [Pull request 190](https://bitbucket.org/osrf/sdformat/pull-request/190)
1. Support for fixed joints
     * [Pull request 194](https://bitbucket.org/osrf/sdformat/pull-request/194)
1. Adding iterations to state
     * [Pull request 188](https://bitbucket.org/osrf/sdformat/pull-request/188)
1. Convert to use ignition-math
     * [Pull request 173](https://bitbucket.org/osrf/sdformat/pull-request/173)
1. Add world origin to scene
     * [Pull request 183](https://bitbucket.org/osrf/sdformat/pull-request/183)
1. Fix collide bitmask
     * [Pull request 182](https://bitbucket.org/osrf/sdformat/pull-request/182)
1. Adding meta information to visuals
     * [Pull request 180](https://bitbucket.org/osrf/sdformat/pull-request/180)
1. Add projection type to gui camera
     * [Pull request 178](https://bitbucket.org/osrf/sdformat/pull-request/178)
1. Fix print description to include attribute description
     * [Pull request 170](https://bitbucket.org/osrf/sdformat/pull-request/170)
1. Add -std=c++11 flag to sdf_config.cmake.in and sdformat.pc.in, needed by downstream code
     * [Pull request 172](https://bitbucket.org/osrf/sdformat/pull-request/172)
1. Added boost::any accessor for Param and Element
     * [Pull request 166](https://bitbucket.org/osrf/sdformat/pull-request/166)
1. Remove tinyxml from dependency list
     * [Pull request 152](https://bitbucket.org/osrf/sdformat/pull-request/152)
1. Added self_collide element for model
     * [Pull request 149](https://bitbucket.org/osrf/sdformat/pull-request/149)
1. Added a collision bitmask field to sdf-1.5 and c++11 support
     * [Pull request 145](https://bitbucket.org/osrf/sdformat/pull-request/145)
1. Fix problems with latin locales and decimal numbers (issue #60)
     * [Pull request 147](https://bitbucket.org/osrf/sdformat/pull-request/147)
     * [Issue 60](https://bitbucket.org/osrf/sdformat/issues/60)

## SDFormat 2.x

1. rename cfm_damping --> implicit_spring_damper
     * [Pull request 59](https://bitbucket.org/osrf/sdformat/pull-request/59)
1. add gear_ratio and reference_body for gearbox joint.
     * [Pull request 62](https://bitbucket.org/osrf/sdformat/pull-request/62)
1. Update joint stop stiffness and dissipation
     * [Pull request 61](https://bitbucket.org/osrf/sdformat/pull-request/61)
1. Support for GNUInstallDirs
     * [Pull request 64](https://bitbucket.org/osrf/sdformat/pull-request/64)
1. `<use_true_size>` element used by DEM heightmaps
     * [Pull request 67](https://bitbucket.org/osrf/sdformat/pull-request/67)
1. Do not export urdf symbols in sdformat 1.4
     * [Pull request 75](https://bitbucket.org/osrf/sdformat/pull-request/75)
1. adding deformable properties per issue #32
     * [Pull request 78](https://bitbucket.org/osrf/sdformat/pull-request/78)
     * [Issue 32](https://bitbucket.org/osrf/sdformat/issues/32)
1. Support to use external URDF
     * [Pull request 77](https://bitbucket.org/osrf/sdformat/pull-request/77)
1. Add lighting element to visual
     * [Pull request 79](https://bitbucket.org/osrf/sdformat/pull-request/79)
1. SDF 1.5: add flag to fix joint axis frame #43 (gazebo issue 494)
     * [Pull request 83](https://bitbucket.org/osrf/sdformat/pull-request/83)
     * [Issue 43](https://bitbucket.org/osrf/sdformat/issues/43)
     * [Gazebo issue 494](https://bitbucket.org/osrf/gazebo/issues/494)
1. Implement SDF_PROTOCOL_VERSION (issue #51)
     * [Pull request 90](https://bitbucket.org/osrf/sdformat/pull-request/90)
1. Port sdformat to compile on Windows (MSVC)
     * [Pull request 101](https://bitbucket.org/osrf/sdformat/pull-request/101)
1. Separate material properties in material.sdf
     * [Pull request 104](https://bitbucket.org/osrf/sdformat/pull-request/104)
1. Add road textures (repeat pull request #104 for sdf_2.0)
     * [Pull request 105](https://bitbucket.org/osrf/sdformat/pull-request/105)
1. Add Extruded Polylines as a model
     * [Pull request 93](https://bitbucket.org/osrf/sdformat/pull-request/93)
1. Added polyline for sdf_2.0
     * [Pull request 106](https://bitbucket.org/osrf/sdformat/pull-request/106)
1. Add spring_reference and spring_stiffness tags to joint axis dynamics
     * [Pull request 102](https://bitbucket.org/osrf/sdformat/pull-request/102)
1. Fix actor static
     * [Pull request 110](https://bitbucket.org/osrf/sdformat/pull-request/110)
1. New <Population> element
     * [Pull request 112](https://bitbucket.org/osrf/sdformat/pull-request/112)
1. Add camera distortion element
     * [Pull request 120](https://bitbucket.org/osrf/sdformat/pull-request/120)
1. Inclusion of magnetic field strength sensor
     * [Pull request 123](https://bitbucket.org/osrf/sdformat/pull-request/123)
1. Properly add URDF gazebo extensions blobs to SDF joint elements
     * [Pull request 125](https://bitbucket.org/osrf/sdformat/pull-request/125)
1. Allow gui plugins to be specified in SDF
     * [Pull request 127](https://bitbucket.org/osrf/sdformat/pull-request/127)
1. Backport magnetometer
     * [Pull request 128](https://bitbucket.org/osrf/sdformat/pull-request/128)
1. Add flag for MOI rescaling to sdf 1.4
     * [Pull request 121](https://bitbucket.org/osrf/sdformat/pull-request/121)
1. Parse urdf joint friction parameters, add corresponding test
     * [Pull request 129](https://bitbucket.org/osrf/sdformat/pull-request/129)
1. Allow reading of boolean values from plugin elements.
     * [Pull request 132](https://bitbucket.org/osrf/sdformat/pull-request/132)
1. Implement generation of XML Schema files (issue #2)
     * [Pull request 91](https://bitbucket.org/osrf/sdformat/pull-request/91)
1. Fix build for OS X 10.10
     * [Pull request 135](https://bitbucket.org/osrf/sdformat/pull-request/135)
1. Improve performance in loading <include> SDF elements
     * [Pull request 138](https://bitbucket.org/osrf/sdformat/pull-request/138)
1. Added urdf gazebo extension option to disable fixed joint lumping
     * [Pull request 133](https://bitbucket.org/osrf/sdformat/pull-request/133)
1. Support urdfdom 0.3 (alternative to pull request #122)
     * [Pull request 141](https://bitbucket.org/osrf/sdformat/pull-request/141)
1. Update list of supported joint types
     * [Pull request 142](https://bitbucket.org/osrf/sdformat/pull-request/142)
1. Ignore unknown elements
     * [Pull request 148](https://bitbucket.org/osrf/sdformat/pull-request/148)
1. Physics preset attributes
     * [Pull request 146](https://bitbucket.org/osrf/sdformat/pull-request/146)
1. Backport fix for latin locales (pull request #147)
     * [Pull request 150](https://bitbucket.org/osrf/sdformat/pull-request/150)

## SDFormat 1.4

### SDFormat 1.4.8 (2013-09-06)

1. Fix inertia transformations when reducing fixed joints in URDF
    * [Pull request 48](https://bitbucket.org/osrf/sdformat/pull-request/48/fix-for-issue-22-reducing-inertia-across/diff)
1. Add <use_terrain_paging> element to support terrain paging in gazebo
    * [Pull request 47](https://bitbucket.org/osrf/sdformat/pull-request/47/add-element-inside-heightmap/diff)
1. Further reduce console output when using URDF models
    * [Pull request 46](https://bitbucket.org/osrf/sdformat/pull-request/46/convert-a-few-more-sdfwarns-to-sdflog-fix/diff)
    * [Commit](https://bitbucket.org/osrf/sdformat/commits/b15d5a1ecc57abee6691618d02d59bbc3d1b84dc)

### SDFormat 1.4.7 (2013-08-22)

1. Direct console messages to std_err
    * [Pull request 44](https://bitbucket.org/osrf/sdformat/pull-request/44/fix-19-direct-all-messages-to-std_err)

### SDFormat 1.4.6 (2013-08-20)

1. Add tags for GPS sensor and sensor noise
    * [Pull request 36](https://bitbucket.org/osrf/sdformat/pull-request/36/gps-sensor-sensor-noise-and-spherical)
1. Add tags for wireless transmitter/receiver models
    * [Pull request 34](https://bitbucket.org/osrf/sdformat/pull-request/34/transceiver-support)
    * [Pull request 43](https://bitbucket.org/osrf/sdformat/pull-request/43/updated-description-of-the-transceiver-sdf)
1. Add tags for playback of audio files in Gazebo
    * [Pull request 26](https://bitbucket.org/osrf/sdformat/pull-request/26/added-audio-tags)
    * [Pull request 35](https://bitbucket.org/osrf/sdformat/pull-request/35/move-audio-to-link-and-playback-on-contact)
1. Add tags for simbody physics parameters
    * [Pull request 32](https://bitbucket.org/osrf/sdformat/pull-request/32/merging-some-updates-from-simbody-branch)
1. Log messages to a file, reduce console output
    * [Pull request 33](https://bitbucket.org/osrf/sdformat/pull-request/33/log-messages-to-file-8)
1. Generalize ode's <provide_feedback> element
    * [Pull request 38](https://bitbucket.org/osrf/sdformat/pull-request/38/add-provide_feedback-for-bullet-joint)
1. Various bug, style and test fixes

### SDFormat 1.4.5 (2013-07-23)

1. Deprecated Gazebo's internal SDF code
1. Use templatized Get functions for retrieving values from SDF files
1. Removed dependency on ROS
