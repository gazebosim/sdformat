## SDFormat 5.0

### SDFormat 5.0.0 (xxxx-xx-xx)

1. Added an example
    * [Pull request 275](https://bitbucket.org/osrf/sdformat/pull-requests/275)

1. Move functions that use TinyXML classes in private headers by creating 
   `parser_private.hh` from `parser.hh` and by not installing `Converter.hh`.
    * [Pull request 262](https://bitbucket.org/osrf/sdformat/pull-requests/262)

## SDFormat 4.0

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
