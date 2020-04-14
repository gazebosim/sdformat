## SDFormat 3.0

### SDFormat 3.X.X (201X-XX-XX)

1. Improve precision of floating point parameters
     * [BitBucket pull request 273](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/273)
     * [BitBucket pull request 276](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/276)

### SDFormat 3.7.0 (2015-11-20)

1. Add spring pass through for sdf3
     * [Design document](https://bitbucket.org/osrf/gazebo_design/pull-requests/23)
     * [BitBucket pull request 242](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/242)

1. Support frame specification in SDF
     * [BitBucket pull request 237](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/237)

1. Remove boost from SDFExtension
     * [BitBucket pull request 229](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/229)

### SDFormat 3.6.0 (2015-10-27)

1. Add light state
    * [BitBucket pull request 227](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/227)
1. redo pull request #222 for sdf3 branch
    * [BitBucket pull request 232](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/232)
1. Fix links in API documentation
    * [BitBucket pull request 231](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/231)

### SDFormat 3.5.0 (2015-10-07)

1. Camera lens description (Replaces #213)
    * [BitBucket pull request 215](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/215)
1. Fix shared pointer reference loop in Element and memory leak (#104)
    * [BitBucket pull request 230](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/230)

### SDFormat 3.4.0 (2015-10-05)

1. Support nested model states
    * [BitBucket pull request 223](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/223)
1. Cleaner way to set SDF_PATH for tests
    * [BitBucket pull request 226](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/226)

### SDFormat 3.3.0 (2015-09-15)

1. Windows Boost linking errors
    * [BitBucket pull request 206](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/206)
1. Nested SDF -> sdf3
    * [BitBucket pull request 221](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/221)
1. Pointer types
    * [BitBucket pull request 218](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/218)
1. Torsional friction default surface radius not infinity
    * [BitBucket pull request 217](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/217)

### SDFormat 3.2.2 (2015-08-24)

1. Added battery element (contribution from Olivier Crave)
     * [BitBucket pull request #204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/204)
1. Torsional friction backport
     * [BitBucket pull request #211](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/211)
1. Allow Visual Studio 2015
     * [BitBucket pull request #208](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/208)

### SDFormat 3.1.1 (2015-08-03)

1. Fix tinyxml linking error
     * [BitBucket pull request #209](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/209)

### SDFormat 3.1.0 (2015-08-02)

1. Added logical camera sensor to SDF
     * [BitBucket pull request #207](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/207)

### SDFormat 3.0.0 (2015-07-24)

1. Added battery to SDF
     * [BitBucket pull request 204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/204)
1. Added altimeter sensor to SDF
     * [BitBucket pull request #197](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/197)
1. Added magnetometer sensor to SDF
     * [BitBucket pull request 198](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/198)
1. Fix detection of XML parsing errors
     * [BitBucket pull request 190](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/190)
1. Support for fixed joints
     * [BitBucket pull request 194](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/194)
1. Adding iterations to state
     * [BitBucket pull request 188](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/188)
1. Convert to use ignition-math
     * [BitBucket pull request 173](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/173)
1. Add world origin to scene
     * [BitBucket pull request 183](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/183)
1. Fix collide bitmask
     * [BitBucket pull request 182](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/182)
1. Adding meta information to visuals
     * [BitBucket pull request 180](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/180)
1. Add projection type to gui camera
     * [BitBucket pull request 178](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/178)
1. Fix print description to include attribute description
     * [BitBucket pull request 170](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/170)
1. Add -std=c++11 flag to sdf_config.cmake.in and sdformat.pc.in, needed by downstream code
     * [BitBucket pull request 172](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/172)
1. Added boost::any accessor for Param and Element
     * [BitBucket pull request 166](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/166)
1. Remove tinyxml from dependency list
     * [BitBucket pull request 152](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/152)
1. Added self_collide element for model
     * [BitBucket pull request 149](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/149)
1. Added a collision bitmask field to sdf-1.5 and c++11 support
     * [BitBucket pull request 145](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/145)
1. Fix problems with latin locales and decimal numbers (issue #60)
     * [BitBucket pull request 147](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/147)
     * [Issue 60](https://github.com/osrf/sdformat/issues/60)

## SDFormat 2.x

1. rename cfm_damping --> implicit_spring_damper
     * [BitBucket pull request 59](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/59)
1. add gear_ratio and reference_body for gearbox joint.
     * [BitBucket pull request 62](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/62)
1. Update joint stop stiffness and dissipation
     * [BitBucket pull request 61](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/61)
1. Support for GNUInstallDirs
     * [BitBucket pull request 64](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/64)
1. `<use_true_size>` element used by DEM heightmaps
     * [BitBucket pull request 67](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/67)
1. Do not export urdf symbols in sdformat 1.4
     * [BitBucket pull request 75](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/75)
1. adding deformable properties per issue #32
     * [BitBucket pull request 78](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/78)
     * [Issue 32](https://github.com/osrf/sdformat/issues/32)
1. Support to use external URDF
     * [BitBucket pull request 77](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/77)
1. Add lighting element to visual
     * [BitBucket pull request 79](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/79)
1. SDF 1.5: add flag to fix joint axis frame #43 (gazebo issue 494)
     * [BitBucket pull request 83](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/83)
     * [Issue 43](https://github.com/osrf/sdformat/issues/43)
     * [Gazebo issue 494](https://bitbucket.org/osrf/gazebo/issues/494)
1. Implement SDF_PROTOCOL_VERSION (issue #51)
     * [BitBucket pull request 90](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/90)
1. Port sdformat to compile on Windows (MSVC)
     * [BitBucket pull request 101](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/101)
1. Separate material properties in material.sdf
     * [BitBucket pull request 104](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/104)
1. Add road textures (repeat pull request #104 for sdf_2.0)
     * [BitBucket pull request 105](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/105)
1. Add Extruded Polylines as a model
     * [BitBucket pull request 93](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/93)
1. Added polyline for sdf_2.0
     * [BitBucket pull request 106](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/106)
1. Add spring_reference and spring_stiffness tags to joint axis dynamics
     * [BitBucket pull request 102](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/102)
1. Fix actor static
     * [BitBucket pull request 110](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/110)
1. New <Population> element
     * [BitBucket pull request 112](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/112)
1. Add camera distortion element
     * [BitBucket pull request 120](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/120)
1. Inclusion of magnetic field strength sensor
     * [BitBucket pull request 123](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/123)
1. Properly add URDF gazebo extensions blobs to SDF joint elements
     * [BitBucket pull request 125](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/125)
1. Allow gui plugins to be specified in SDF
     * [BitBucket pull request 127](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/127)
1. Backport magnetometer
     * [BitBucket pull request 128](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/128)
1. Add flag for MOI rescaling to sdf 1.4
     * [BitBucket pull request 121](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/121)
1. Parse urdf joint friction parameters, add corresponding test
     * [BitBucket pull request 129](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/129)
1. Allow reading of boolean values from plugin elements.
     * [BitBucket pull request 132](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/132)
1. Implement generation of XML Schema files (issue #2)
     * [BitBucket pull request 91](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/91)
1. Fix build for OS X 10.10
     * [BitBucket pull request 135](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/135)
1. Improve performance in loading <include> SDF elements
     * [BitBucket pull request 138](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/138)
1. Added urdf gazebo extension option to disable fixed joint lumping
     * [BitBucket pull request 133](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/133)
1. Support urdfdom 0.3 (alternative to pull request #122)
     * [BitBucket pull request 141](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/141)
1. Update list of supported joint types
     * [BitBucket pull request 142](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/142)
1. Ignore unknown elements
     * [BitBucket pull request 148](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/148)
1. Physics preset attributes
     * [BitBucket pull request 146](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/146)
1. Backport fix for latin locales (pull request #147)
     * [BitBucket pull request 150](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/150)

## SDFormat 1.4

### SDFormat 1.4.8 (2013-09-06)

1. Fix inertia transformations when reducing fixed joints in URDF
    * [BitBucket pull request 48](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/48/fix-for-issue-22-reducing-inertia-across/diff)
1. Add <use_terrain_paging> element to support terrain paging in gazebo
    * [BitBucket pull request 47](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/47/add-element-inside-heightmap/diff)
1. Further reduce console output when using URDF models
    * [BitBucket pull request 46](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/46/convert-a-few-more-sdfwarns-to-sdflog-fix/diff)
    * [Commit](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/commits/b15d5a1ecc57abee6691618d02d59bbc3d1b84dc)

### SDFormat 1.4.7 (2013-08-22)

1. Direct console messages to std_err
    * [BitBucket pull request 44](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/44/fix-19-direct-all-messages-to-std_err)

### SDFormat 1.4.6 (2013-08-20)

1. Add tags for GPS sensor and sensor noise
    * [BitBucket pull request 36](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/36/gps-sensor-sensor-noise-and-spherical)
1. Add tags for wireless transmitter/receiver models
    * [BitBucket pull request 34](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/34/transceiver-support)
    * [BitBucket pull request 43](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/43/updated-description-of-the-transceiver-sdf)
1. Add tags for playback of audio files in Gazebo
    * [BitBucket pull request 26](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/26/added-audio-tags)
    * [BitBucket pull request 35](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/35/move-audio-to-link-and-playback-on-contact)
1. Add tags for simbody physics parameters
    * [BitBucket pull request 32](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/32/merging-some-updates-from-simbody-branch)
1. Log messages to a file, reduce console output
    * [BitBucket pull request 33](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/33/log-messages-to-file-8)
1. Generalize ode's <provide_feedback> element
    * [BitBucket pull request 38](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/38/add-provide_feedback-for-bullet-joint)
1. Various bug, style and test fixes

### SDFormat 1.4.5 (2013-07-23)

1. Deprecated Gazebo's internal SDF code
1. Use templatized Get functions for retrieving values from SDF files
1. Removed dependency on ROS
