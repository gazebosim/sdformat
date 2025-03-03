## libsdformat 15.X

### libsdformat 15.2.0 (2025-02-12)

1. Add entry to Migration guide about the updated auto-inertia behavior
    * [Pull request #1528](https://github.com/gazebosim/sdformat/pull/1528)

1. Add missed spec version 1.12 in the sdf_descriptions target
    * [Pull request #1529](https://github.com/gazebosim/sdformat/pull/1529)

1. Resolve auto inertia based on input mass
    * [Pull request #1513](https://github.com/gazebosim/sdformat/pull/1513)

1. ci.yml: run cppcheck, cpplint on noble
    * [Pull request #1521](https://github.com/gazebosim/sdformat/pull/1521)

1. Add non-const overload for Root::Model() getter
    * [Pull request #1524](https://github.com/gazebosim/sdformat/pull/1524)

1. Remove unncessary <iostream> includes
    * [Pull request #1523](https://github.com/gazebosim/sdformat/pull/1523)

1. Don't reparse parent elements when cloning.
    * [Pull request #1484](https://github.com/gazebosim/sdformat/pull/1484)

1. python bindings: get version from package.xml
    * [Pull request #1504](https://github.com/gazebosim/sdformat/pull/1504)

1. Permit to test when building bindings separately from main library
    * [Pull request #1509](https://github.com/gazebosim/sdformat/pull/1509)

### libsdformat 15.1.1 (2024-11-15)

1. **Baseline:** this includes all changes from 15.1.0 and earlier.

1. Fix bazel rules for layering_check and parse_headers with clang
    * [Pull request #1507](https://github.com/gazebosim/sdformat/pull/1507)
  
1. Enable header layering checks for bazel build
    * [Pull request #1505](https://github.com/gazebosim/sdformat/pull/1505)

### libsdformat 15.1.0 (2024-11-13)

1. Bazel CI
    * [Pull request #1500](https://github.com/gazebosim/sdformat/pull/1500)

1. Add bzlmod support to sdf15
    * [Pull request #1493](https://github.com/gazebosim/sdformat/pull/1493)

1. Support removing the actor, light, or model from the root
    * [Pull request #1492](https://github.com/gazebosim/sdformat/pull/1492)

1. Only look for psutil if testing is enabled
    * [Pull request #1495](https://github.com/gazebosim/sdformat/pull/1495)

1. Improve installation instructions
    * [Pull request #1496](https://github.com/gazebosim/sdformat/pull/1496)

1. Permit building python bindings separately from libsdformat library
    * [Pull request #1491](https://github.com/gazebosim/sdformat/pull/1491)

1. Change sdf\_config.h to sdf/config.hh everywhere
    * [Pull request #1494](https://github.com/gazebosim/sdformat/pull/1494)

1. Improve installation instructions
    * [Pull request #1490](https://github.com/gazebosim/sdformat/pull/1490)

1. Fix symbol checking test when compiled with debug symbols
    * [Pull request #1474](https://github.com/gazebosim/sdformat/pull/1474)

### libsdformat 15.0.0 (2024-09-25)

1. **Baseline:** this includes all changes from 14.5.0 and earlier.

1. Use colcon for Windows building compilation
    * [Pull request #1481](https://github.com/gazebosim/sdformat/pull/1481)

1. Spec 1.12: link_state, joint_state changes
    * [Pull request #1461](https://github.com/gazebosim/sdformat/pull/1461)

1. Fix symbol checking test when compiled with debug symbols
    * [Pull request #1474](https://github.com/gazebosim/sdformat/pull/1474)

1. README: update badges for sdf15
    * [Pull request #1472](https://github.com/gazebosim/sdformat/pull/1472)

1. Ionic Changelog
    * [Pull request #1471](https://github.com/gazebosim/sdformat/pull/1471)

1. Spec 1.12: add `_state` suffix to //state subelements
    * [Pull request #1455](https://github.com/gazebosim/sdformat/pull/1455)

1. Add optional binary relocatability
    * [Pull request #1414](https://github.com/gazebosim/sdformat/pull/1414)
    * [Pull request #1468](https://github.com/gazebosim/sdformat/pull/1468)
    * [Pull request #1469](https://github.com/gazebosim/sdformat/pull/1469)

1. FrameSemantics: fix NullVertex warnings
    * [Pull request #1460](https://github.com/gazebosim/sdformat/pull/1460)
    * [Pull request #1459](https://github.com/gazebosim/sdformat/pull/1459)
    * [Pull request #1458](https://github.com/gazebosim/sdformat/pull/1458)

1. Remove deprecated APIs
    * [Pull request #1456](https://github.com/gazebosim/sdformat/pull/1456)

1. Spec 1.12: add `//sensor/frame_id`
    * [Pull request #1454](https://github.com/gazebosim/sdformat/pull/1454)

1. Disable latex and class hierarchy generation
    * [Pull request #1447](https://github.com/gazebosim/sdformat/pull/1447)

1. Print auto inertial values with `gz sdf --print --expand-auto-inertials`
    * [Pull request #1422](https://github.com/gazebosim/sdformat/pull/1422)

1. Add cone shape to SDFormat spec
    * [Pull request #1418](https://github.com/gazebosim/sdformat/pull/1418)
    * [Pull request #1434](https://github.com/gazebosim/sdformat/pull/1434)

1. Enable 24.04 CI, remove distutils dependency
    * [Pull request #1408](https://github.com/gazebosim/sdformat/pull/1408)

1. Change behavior of `Param::Get<bool>`
    * [Pull request #1397](https://github.com/gazebosim/sdformat/pull/1397)

1. Parse kinematic property in Link, expand spec documentation of property
    * [Pull request #1390](https://github.com/gazebosim/sdformat/pull/1390)
    * [Pull request #1399](https://github.com/gazebosim/sdformat/pull/1399)
    * [Pull request #1462](https://github.com/gazebosim/sdformat/pull/1462)

1. Spec 1.11+: add `//mesh/@optimization`, `//mesh/convex_decomposition`
    * [Pull request #1382](https://github.com/gazebosim/sdformat/pull/1382)
    * [Pull request #1386](https://github.com/gazebosim/sdformat/pull/1386)
    * [Pull request #1403](https://github.com/gazebosim/sdformat/pull/1403)

1. Copy 1.11 spec to 1.12 for libsdformat15
    * [Pull request #1375](https://github.com/gazebosim/sdformat/pull/1375)

1. Fix find Python3 logic and macOS workflow
    * [Pull request #1367](https://github.com/gazebosim/sdformat/pull/1367)

1. Remove `HIDE_SYMBOLS_BY_DEFAULT`: replace by a default configuration in gz-cmake.
    * [Pull request #1355](https://github.com/gazebosim/sdformat/pull/1355)

1. Dependency version bumps in ionic: use gz-cmake4, gz-utils3, gz-math8
    * [Pull request #1340](https://github.com/gazebosim/sdformat/pull/1340)

1. Bump major version to 15
    * [Pull request #1338](https://github.com/gazebosim/sdformat/pull/1338)

## libsdformat 14.X

### libsdformat 14.5.0 (2024-08-05)

1. Adding Errors structure to XmlUtils
    * [Pull request #1296](https://github.com/gazebosim/sdformat/pull/1296)

1. Disable latex and class hierarchy generation
    * [Pull request #1447](https://github.com/gazebosim/sdformat/pull/1447)

1. Added SetHeightMap and Heighmap to Geometry Python binding
    * [Pull request #1440](https://github.com/gazebosim/sdformat/pull/1440)

1. workflows/ci.yml fix push branch regex
    * [Pull request #1445](https://github.com/gazebosim/sdformat/pull/1445)

1. SDF.cc update calls to use sdf::Errors output
    * [Pull request #1295](https://github.com/gazebosim/sdformat/pull/1295)

1. Added World::ActorByName
    * [Pull request #1436](https://github.com/gazebosim/sdformat/pull/1436)

### libsdformat 14.4.0 (2024-06-20)

1. Add Cone as a primitive parametric shape.
    * [Pull request #1415](https://github.com/gazebosim/sdformat/pull/1415)
    * Thanks to Benjamin Perseghetti

1. Add custom attribute to custom element in test
    * [Pull request #1406](https://github.com/gazebosim/sdformat/pull/1406)

### libsdformat 14.3.0 (2024-06-14)

1. Backport voxel_resolution sdf element
    * [Pull request #1429](https://github.com/gazebosim/sdformat/pull/1429)

1. Added Automatic Moment of Inertia Calculations for Basic Shapes Python wrappers
    * [Pull request #1424](https://github.com/gazebosim/sdformat/pull/1424)

1. Add support for no gravity link
    * [Pull request #1410](https://github.com/gazebosim/sdformat/pull/1410)
    * [Pull request #1419](https://github.com/gazebosim/sdformat/pull/1419)

1. Update default camera instrinsics skew to 0, which matches spec
    * [Pull request #1423](https://github.com/gazebosim/sdformat/pull/1423)
    * [Pull request #1425](https://github.com/gazebosim/sdformat/pull/1425)

1. Allow empty strings in plugin and custom attributes
    * [Pull request #1407](https://github.com/gazebosim/sdformat/pull/1407)

1. (Backport) Enable 24.04 CI, remove distutils dependency
    * [Pull request #1413](https://github.com/gazebosim/sdformat/pull/1413)

1. Fix macOS workflow and backport windows fix
    * [Pull request #1409](https://github.com/gazebosim/sdformat/pull/1409)

1. Fix warning with pybind11 2.12
    * [Pull request #1389](https://github.com/gazebosim/sdformat/pull/1389)

1. Add bullet and torsional friction DOM
    * [Pull request #1351](https://github.com/gazebosim/sdformat/pull/1351)
    * [Pull request #1427](https://github.com/gazebosim/sdformat/pull/1427)

### libsdformat 14.2.0 (2024-04-23)

1. Fix trivial warning on 24.04 for JointAxis_TEST.cc
    * [Pull request #1402](https://github.com/gazebosim/sdformat/pull/1402)

1. Add package.xml, fix `gz sdf` tests on Windows
    * [Pull request #1374](https://github.com/gazebosim/sdformat/pull/1374)

1. Backport mesh optimization feature
    * [Pull request #1398](https://github.com/gazebosim/sdformat/pull/1398)
    * [Pull request #1386](https://github.com/gazebosim/sdformat/pull/1386)
    * [Pull request #1382](https://github.com/gazebosim/sdformat/pull/1382)

1. Param_TEST: Check return values of Param::Get/Set
    * [Pull request #1394](https://github.com/gazebosim/sdformat/pull/1394)

### libsdformat 14.1.1 (2024-03-28)

1. Fix warning with pybind11 2.12
    * [Pull request #1389](https://github.com/gazebosim/sdformat/pull/1389)

1. Use relative install paths in CMake
    * [Pull request #1387](https://github.com/gazebosim/sdformat/pull/1387)

### libsdformat 14.1.0 (2024-03-14)

1. Resolve URIs relative to file path
    * [Pull request #1373](https://github.com/gazebosim/sdformat/pull/1373)

1. Use `//link/inertial/density` for auto-inertials
    * [Pull request #1335](https://github.com/gazebosim/sdformat/pull/1335)

1. Fix a little typo in the README.md
    * [Pull request #1365](https://github.com/gazebosim/sdformat/pull/1365)

1. Fix version variable in reference manual PDF filename
    * [Pull request #1363](https://github.com/gazebosim/sdformat/pull/1363)

1. In parser config test, use a filename less likely to exist
    * [Pull request #1362](https://github.com/gazebosim/sdformat/pull/1362)

1. Update CI badges in README
    * [Pull request #1352](https://github.com/gazebosim/sdformat/pull/1352)

1. Bazel updates for Garden build
    * [Pull request #1239](https://github.com/gazebosim/sdformat/pull/1239)

1. Fix static builds and optimize test compilation
    * [Pull request #1343](https://github.com/gazebosim/sdformat/pull/1343)
    * [Pull request #1347](https://github.com/gazebosim/sdformat/pull/1347)

1. Install ruby commands on Windows
    * [Pull request #1339](https://github.com/gazebosim/sdformat/pull/1339)

1. Infrastructure
    * [Pull request #1336](https://github.com/gazebosim/sdformat/pull/1336)
    * [Pull request #1345](https://github.com/gazebosim/sdformat/pull/1345)
    * [Pull request #1367](https://github.com/gazebosim/sdformat/pull/1367)

1. URDF parser: use SDFormat 1.11, parse joint mimic
    * [Pull request #1333](https://github.com/gazebosim/sdformat/pull/1333)

1. URDF->SDF handle links with no inertia or small mass
    * [Pull request #1238](https://github.com/gazebosim/sdformat/pull/1238)

### libsdformat 14.0.0 (2023-09-29)

1. Add missing conda dependencies
    * [Pull request #1324](https://github.com/gazebosim/sdformat/pull/1324)

1. Correct spelling of Python variable in CMake
    * [Pull request #1319](https://github.com/gazebosim/sdformat/pull/1319)

1. Remove hard dependence on ruby
    * [Pull request #1323](https://github.com/gazebosim/sdformat/pull/1323)

1. Documentation fixes
    * [Pull request #1322](https://github.com/gazebosim/sdformat/pull/1322)

1. Automatic Moment of Inertia Calculations for Basic Shapes
    * [Pull request #1299](https://github.com/gazebosim/sdformat/pull/1299)
    * [Pull request #1304](https://github.com/gazebosim/sdformat/pull/1304)
    * [Pull request #1317](https://github.com/gazebosim/sdformat/pull/1317)
    * [Pull request #1325](https://github.com/gazebosim/sdformat/pull/1325)

1. Joint axis mimic constraints: add `sdf` element
    * [Pull request #1166](https://github.com/gazebosim/sdformat/pull/1166)

1. Add python bindings for `sdf::Element` and `sdf::Param`
    * [Pull request #1303](https://github.com/gazebosim/sdformat/pull/1303)

1. World: handle name collisions like Model
    * [Pull request #1311](https://github.com/gazebosim/sdformat/pull/1311)

1. ign -> gz
    * [Pull request #1301](https://github.com/gazebosim/sdformat/pull/1301)

1. Copy 1.10 spec to 1.11 for sdformat14
    * [Pull request #1298](https://github.com/gazebosim/sdformat/pull/1298)

1. Make sure the deprecated kSdfScopeDelimiter still works
    * [Pull request #1305](https://github.com/gazebosim/sdformat/pull/1305)

1. Python script to replace ruby xmlschema generator
    * [Pull request #1232](https://github.com/gazebosim/sdformat/pull/1232)

1. Make dataPtr private in sdf::Plugin
    * [Pull request #1268](https://github.com/gazebosim/sdformat/pull/1268)

1. Fix GSG violations on non-trivially destructible types
    * [Pull request #1264](https://github.com/gazebosim/sdformat/pull/1264)

1. Port embedSdf script from Ruby to Python3 and provide unittests
    * [Pull request #884](https://github.com/gazebosim/sdformat/pull/884)
    * [Pull request #1297](https://github.com/gazebosim/sdformat/pull/1297)

1. World: sdfwarns to sdf::Errors when warnings policy set to sdf::EnforcementPolicy::ERR
    * [Pull request #1131](https://github.com/gazebosim/sdformat/pull/1131)

1. Add sdf::Errors output to API methods
    * [Pull request #1098](https://github.com/gazebosim/sdformat/pull/1098)
    * [Pull request #1095](https://github.com/gazebosim/sdformat/pull/1095)

1. ⬆️  Bump main to 14.0.0~pre1
    * [Pull request #1104](https://github.com/gazebosim/sdformat/pull/1104)

## libsdformat 13.X

### libsdformat 13.8.0 (2024-06-25)

1. Added `World::ActorByName`
    * [Pull request #1436](https://github.com/gazebosim/sdformat/pull/1436)

1. Backport #1367 to Garden: Fix find Python3 logic.
    * [Pull request #1370](https://github.com/gazebosim/sdformat/pull/1370)

### libsdformat 13.7.0 (2024-06-13)

1. Add support for no gravity link
    * [Pull request #1410](https://github.com/gazebosim/sdformat/pull/1410)
    * [Pull request #1419](https://github.com/gazebosim/sdformat/pull/1419)

1. Fix macOS workflow and backport windows fix
    * [Pull request #1409](https://github.com/gazebosim/sdformat/pull/1409)

1. Fix warning with pybind11 2.12
    * [Pull request #1389](https://github.com/gazebosim/sdformat/pull/1389)

1. Add bullet and torsional friction DOM
    * [Pull request #1351](https://github.com/gazebosim/sdformat/pull/1351)
    * [Pull request #1427](https://github.com/gazebosim/sdformat/pull/1427)

1. Resolve URIs relative to file path
    * [Pull request #1373](https://github.com/gazebosim/sdformat/pull/1373)

1. Bazel updates for Garden build
    * [Pull request #1239](https://github.com/gazebosim/sdformat/pull/1239)

1. Fix static builds and optimize test compilation
    * [Pull request #1343](https://github.com/gazebosim/sdformat/pull/1343)
    * [Pull request #1347](https://github.com/gazebosim/sdformat/pull/1347)

1. Install ruby commands on Windows
    * [Pull request #1339](https://github.com/gazebosim/sdformat/pull/1339)
    * [Pull request #1341](https://github.com/gazebosim/sdformat/pull/1341)

1. Update github action workflows
    * [Pull request #1345](https://github.com/gazebosim/sdformat/pull/1345)

1. URDF->SDF handle links with no inertia or small mass
    * [Pull request #1238](https://github.com/gazebosim/sdformat/pull/1238)

### libsdformat 13.6.0 (2023-08-30)

1. Use relative path in an urdf include to avoid confusion between internal and system headers
    * [Pull request #1259](https://github.com/gazebosim/sdformat/pull/1259)

1. parser.cc update calls to use sdf::Errors output
    * [Pull request #1294](https://github.com/gazebosim/sdformat/pull/1294)

1. Fix deeply nested merge-include for custom parsed files
    * [Pull request #1293](https://github.com/gazebosim/sdformat/pull/1293)

1. Updated findfile() to search localpath first
    * [Pull request #1292](https://github.com/gazebosim/sdformat/pull/1292)

1. World requires a scene and atmosphere
    * [Pull request #1308](https://github.com/gazebosim/sdformat/pull/1308)

1. Infrastructure
    * [Pull request #1307](https://github.com/gazebosim/sdformat/pull/1307)
    * [Pull request #1306](https://github.com/gazebosim/sdformat/pull/1306)

1. Remove robot not found error when parsing fails
    * [Pull request #1290](https://github.com/gazebosim/sdformat/pull/1290)

1. Make some sdfdbg messages sdfmsgs
    * [Pull request #1288](https://github.com/gazebosim/sdformat/pull/1288)

1. Minor clean up of tests
    * [Pull request #1289](https://github.com/gazebosim/sdformat/pull/1289)

### libsdformat 13.5.0 (2023-05-18)

1. Added projector Python wrapper
    * [Pull request #1279](https://github.com/gazebosim/sdformat/pull/1279)

1. Added new error codes in Python
    * [Pull request #1280](https://github.com/gazebosim/sdformat/pull/1280)

1. Fixed 1.9/light.sdf
    * [Pull request #1281](https://github.com/gazebosim/sdformat/pull/1281)

1. Add Projector DOM
    * [Pull request #1277](https://github.com/gazebosim/sdformat/pull/1277)

1. Disable pybind11 tests on windows
    * [Pull request #1278](https://github.com/gazebosim/sdformat/pull/1278)

1. Geometry and others: update calls to use sdf::Errors parameters
    * [Pull request #1153](https://github.com/gazebosim/sdformat/pull/1153)

1. Fix GitHub Actions on macOS
    * [Pull request #1271](https://github.com/gazebosim/sdformat/pull/1271)

1. JointAxis: improve code coverage in Load()
    * [Pull request #1267](https://github.com/gazebosim/sdformat/pull/1267)

1. Scene: update calls to use sdf::Errors parameters
    * [Pull request #1164](https://github.com/gazebosim/sdformat/pull/1164)

1. ForceTorque: update calls to use sdf::Errors output
    * [Pull request #1163](https://github.com/gazebosim/sdformat/pull/1163)

1. Fix Element::Set method return value
    * [Pull request #1256](https://github.com/gazebosim/sdformat/pull/1256)

1. Sky: update calls to use sdf::Errors output
    * [Pull request #1162](https://github.com/gazebosim/sdformat/pull/1162)

1. Add missing values in Surface ToElement method
    * [Pull request #1263](https://github.com/gazebosim/sdformat/pull/1263)

1. Atmosphere: update calls to use sdf::Errors output
    * [Pull request #1161](https://github.com/gazebosim/sdformat/pull/1161)

1. Altimeter: update calls to use sdf::Errors output
    * [Pull request #1160](https://github.com/gazebosim/sdformat/pull/1160)

1. AirPressure: update calls to use sdf::Errors output
    * [Pull request #1159](https://github.com/gazebosim/sdformat/pull/1159)

1. ParticleEmitter: update calls to use sdf::Errors output
    * [Pull request #1158](https://github.com/gazebosim/sdformat/pull/1158)

1. Physics: update calls to use sdf::Errors output
    * [Pull request #1157](https://github.com/gazebosim/sdformat/pull/1157)

1. Frame: update calls to use sdf::Errors output
    * [Pull request #1156](https://github.com/gazebosim/sdformat/pull/1156)

1. Material: update calls to use sdf::Errors output
    * [Pull request #1155](https://github.com/gazebosim/sdformat/pull/1155)

1. Light: update calls to use sdf::Errors parameters
    * [Pull request #1154](https://github.com/gazebosim/sdformat/pull/1154)

1. Imu: update calls to use sdf::Errors output
    * [Pull request #1152](https://github.com/gazebosim/sdformat/pull/1152)

1. JointAxis: update calls to use sdf::Errors output
    * [Pull request #1145](https://github.com/gazebosim/sdformat/pull/1145)

1. Noise: update calls to use sdf::Errors parameters
    * [Pull request #1151](https://github.com/gazebosim/sdformat/pull/1151)

1. Plugin: update calls to use sdf::Errors output
    * [Pull request #1144](https://github.com/gazebosim/sdformat/pull/1144)

1. URDF->SDF handle links with no inertia or small mass
    * [Pull request #1238](https://github.com/gazebosim/sdformat/pull/1238)

1. Element: update calls to use sdf::Errors output
    * [Pull request #1141](https://github.com/gazebosim/sdformat/pull/1141)

1. Add: new error enums to Python
    * [Pull request #1249](https://github.com/gazebosim/sdformat/pull/1249)

1. Rename COPYING to LICENSE
    * [Pull request #1252](https://github.com/gazebosim/sdformat/pull/1252)

1. Allowing relative paths in URDF
    * [Pull request #1213](https://github.com/gazebosim/sdformat/pull/1213)

### libsdformat 13.4.1 (2023-03-08)

1. Fix camera_info_topic default value
    * [Pull request #1247](https://github.com/gazebosim/sdformat/pull/1247)

1. CI workflow: use checkout v3
    * [Pull request #1245](https://github.com/gazebosim/sdformat/pull/1245)

### libsdformat 13.4.0 (2023-03-03)

1. Fix camera info topic default value
    * [Pull request #1241](https://github.com/gazebosim/sdformat/pull/1241)

1. Add support for merge-includes in worlds
    * [Pull request #1233](https://github.com/gazebosim/sdformat/pull/1233)

1. Backport the python3 embedSdf script variant
    * [Pull request #1240](https://github.com/gazebosim/sdformat/pull/1240)

1. Go back to SDF_ASSERT instead of FATAL_ERROR
    * [Pull request #1235](https://github.com/gazebosim/sdformat/pull/1235)

1. Add missing sdf files from xsd generation
    * [Pull request #1231](https://github.com/gazebosim/sdformat/pull/1231)

1. CI workflow: use checkout v3
    * [Pull request #1225](https://github.com/gazebosim/sdformat/pull/1225)

1. Use `File.exist?` for Ruby 3.2 compatibility
    * [Pull request #1216](https://github.com/gazebosim/sdformat/pull/1216)

### libsdformat 13.3.0 (2023-02-07)

1. Add airspeed sensor
    * [Pull request #1215](https://github.com/gazebosim/sdformat/pull/1215)

1. Use `File.exist?` for Ruby 3.2 compatibility
    * [Pull request #1216](https://github.com/gazebosim/sdformat/pull/1216)

1. Make ThrowOrPrintError a free internal function
    * [Pull request #1221](https://github.com/gazebosim/sdformat/pull/1221)
    * [Pull request #1220](https://github.com/gazebosim/sdformat/pull/1220)

1. macos workflow: don't upgrade existing packages
    * [Pull request #1217](https://github.com/gazebosim/sdformat/pull/1217)

1. update Param calls to use error vectors parameters
    * [Pull request #1140](https://github.com/gazebosim/sdformat/pull/1140)

1. ign -> gz Migrate Ignition Headers : sdformat
    * [Pull request #1118](https://github.com/gazebosim/sdformat/pull/1118)

1. Sensor: add sdf::Errors output to API methods
    * [Pull request #1138](https://github.com/gazebosim/sdformat/pull/1138)

1. Warn child joint that resolves to world
    * [Pull request #1211](https://github.com/gazebosim/sdformat/pull/1211)

1. Converter: add sdf::Errors output to API methods
    * [Pull request #1123](https://github.com/gazebosim/sdformat/pull/1123)

1. ParamPassing: sdfwarns to sdf::Errors when warnings policy set to sdf::EnforcementPolicy::ERR
    * [Pull request #1135](https://github.com/gazebosim/sdformat/pull/1135)

1. Camera: added HasLensProjection
    * [Pull request #1203](https://github.com/gazebosim/sdformat/pull/1203)

1. Change default `camera_info_topic` value to `__default__`
    * [Pull request #1201](https://github.com/gazebosim/sdformat/pull/1201)

1. Check JointAxis expressed-in values during Load
    * [Pull request #1195](https://github.com/gazebosim/sdformat/pull/1195)

1. Added camera info topic to Camera
    * [Pull request #1198](https://github.com/gazebosim/sdformat/pull/1198)
    * [Pull request #1200](https://github.com/gazebosim/sdformat/pull/1200)

1. Fix static URDF models with fixed joints
    * [Pull request #1193](https://github.com/gazebosim/sdformat/pull/1193)

### libsdformat 13.2.0 (2022-10-20)

1. sdf/1.10/joint.sdf: add `screw_thread_pitch`
    * [Pull request #1168](https://github.com/gazebosim/sdformat/pull/1168)

1. sdf/1.10: support //world/joint specification
    * [Pull request #1117](https://github.com/gazebosim/sdformat/pull/1117)
    * [Pull request #1189](https://github.com/gazebosim/sdformat/pull/1189)

1. Model: add sdf::Errors output to API methods
    * [Pull request #1122](https://github.com/gazebosim/sdformat/pull/1122)

1. Added Root::WorldByName
    * [Pull request #1121](https://github.com/gazebosim/sdformat/pull/1121)

1. Python: add OpticalFrameID APIs to pyCamera
    * [Pull request #1184](https://github.com/gazebosim/sdformat/pull/1184)

1. Fix `GZ_PYTHON_INSTALL_PATH` value
    * [Pull request #1183](https://github.com/gazebosim/sdformat/pull/1183)

1. Rename python bindings import library for Windows
    * [Pull request #1165](https://github.com/gazebosim/sdformat/pull/1165)

### libsdformat 13.1.0 (2022-10-12)

1. Add test helper python package for encapsulating versioned python packages
    * [Pull request #1180](https://github.com/gazebosim/sdformat/pull/1180)

1. Add a configuration option to resolve URIs
    * [Pull request #1147](https://github.com/gazebosim/sdformat/pull/1147)

1. World: sdfwarns to sdf::Errors when warnings policy set to sdf::EnforcementPolicy::ERR
    * [Pull request #1131](https://github.com/gazebosim/sdformat/pull/1131)

1. PrintConfig: add sdf::Errors output to API methods
    * [Pull request #1098](https://github.com/gazebosim/sdformat/pull/1098)

1. Element: add sdf::Errors output to API methods
    * [Pull request #1095](https://github.com/gazebosim/sdformat/pull/1095)

1.  python: Import gz.math at startup to fix #1129
    * [Pull request #1130](https://github.com/gazebosim/sdformat/pull/#1130)
    * [Issue 1129](https://github.com/osrf/sdformat/issues/1129)

1. parser_urdf: add //frame for reduced links/joints
    * [Pull request #1148](https://github.com/gazebosim/sdformat/pull/1148)
    * [Pull request #1182](https://github.com/gazebosim/sdformat/pull/1182)

1. Param::Set: fix truncation of floating-point values
    * [Pull request #1137](https://github.com/gazebosim/sdformat/pull/1137)

1. Reduce the number of include dirs for sdformat
    * [Pull request #1170](https://github.com/gazebosim/sdformat/pull/1170)

1. urdf: fix test and clean up internals
    * [Pull request #1126](https://github.com/gazebosim/sdformat/pull/1126)

1. sdf/camera.sdf: fields for projection matrix
    * [Pull request #1088](https://github.com/gazebosim/sdformat/pull/1088)
    * [Pull request #1133](https://github.com/gazebosim/sdformat/pull/1133)
    * [Pull request #1177](https://github.com/gazebosim/sdformat/pull/1177)

1. Add camera optical_frame_id element
    * [Pull request #1109](https://github.com/gazebosim/sdformat/pull/1109)
    * [Pull request #1133](https://github.com/gazebosim/sdformat/pull/1133)
    * [Pull request #1177](https://github.com/gazebosim/sdformat/pull/1177)

### libsdformat 13.0.1 (2022-09-27)

1. Fix arm tests
    * [Pull request #1173](https://github.com/gazebosim/sdformat/pull/1173)

### libsdformat 13.0.0 (2022-09-23)

1. Add camera `optical_frame_id` element
    * [Pull request #1109](https://github.com/gazebosim/sdformat/pull/1109)

1. sdf/1.10: remove unused spec files
    * [Pull request #1113](https://github.com/gazebosim/sdformat/pull/1113)

1. 1.10/joint.sdf: better default limit values
    * [Pull request #1112](https://github.com/gazebosim/sdformat/pull/1112)

1. Remove unused macros from config.hh
    * [Pull request #1108](https://github.com/gazebosim/sdformat/pull/1108)

1. Make //plugin/@name optional
    * [Pull request #1101](https://github.com/gazebosim/sdformat/pull/1101)

1. Add Error enums and update Migration guide
    * [Pull request #1099](https://github.com/gazebosim/sdformat/pull/1099)

1. Warn by default on unrecognized elements
    * [Pull request #1096](https://github.com/gazebosim/sdformat/pull/1096)

1. InterfaceElements: remove deprecated data members
    * [Pull request #1097](https://github.com/gazebosim/sdformat/pull/1097)

1. Add fluid added mass to inertial
    * [Pull request #1077](https://github.com/gazebosim/sdformat/pull/1077)

1. Change Root to non-unique impl pointer
    * [Pull request #844](https://github.com/gazebosim/sdformat/pull/844)

1. Add python interfaces
    * [Pull request #932](https://github.com/gazebosim/sdformat/pull/932)
    * [Pull request #933](https://github.com/gazebosim/sdformat/pull/933)
    * [Pull request #934](https://github.com/gazebosim/sdformat/pull/934)
    * [Pull request #937](https://github.com/gazebosim/sdformat/pull/937)
    * [Pull request #938](https://github.com/gazebosim/sdformat/pull/938)
    * [Pull request #940](https://github.com/gazebosim/sdformat/pull/940)
    * [Pull request #941](https://github.com/gazebosim/sdformat/pull/941)
    * [Pull request #942](https://github.com/gazebosim/sdformat/pull/942)
    * [Pull request #944](https://github.com/gazebosim/sdformat/pull/944)
    * [Pull request #945](https://github.com/gazebosim/sdformat/pull/945)
    * [Pull request #946](https://github.com/gazebosim/sdformat/pull/946)
    * [Pull request #947](https://github.com/gazebosim/sdformat/pull/947)
    * [Pull request #948](https://github.com/gazebosim/sdformat/pull/948)
    * [Pull request #949](https://github.com/gazebosim/sdformat/pull/949)
    * [Pull request #950](https://github.com/gazebosim/sdformat/pull/950)
    * [Pull request #951](https://github.com/gazebosim/sdformat/pull/951)
    * [Pull request #952](https://github.com/gazebosim/sdformat/pull/952)
    * [Pull request #953](https://github.com/gazebosim/sdformat/pull/953)
    * [Pull request #957](https://github.com/gazebosim/sdformat/pull/957)
    * [Pull request #960](https://github.com/gazebosim/sdformat/pull/960)
    * [Pull request #961](https://github.com/gazebosim/sdformat/pull/961)
    * [Pull request #962](https://github.com/gazebosim/sdformat/pull/962)
    * [Pull request #963](https://github.com/gazebosim/sdformat/pull/963)
    * [Pull request #964](https://github.com/gazebosim/sdformat/pull/964)
    * [Pull request #967](https://github.com/gazebosim/sdformat/pull/967)
    * [Pull request #968](https://github.com/gazebosim/sdformat/pull/968)
    * [Pull request #969](https://github.com/gazebosim/sdformat/pull/969)
    * [Pull request #970](https://github.com/gazebosim/sdformat/pull/970)
    * [Pull request #971](https://github.com/gazebosim/sdformat/pull/971)
    * [Pull request #972](https://github.com/gazebosim/sdformat/pull/972)
    * [Pull request #973](https://github.com/gazebosim/sdformat/pull/973)
    * [Pull request #981](https://github.com/gazebosim/sdformat/pull/981)
    * [Pull request #982](https://github.com/gazebosim/sdformat/pull/982)
    * [Pull request #983](https://github.com/gazebosim/sdformat/pull/983)
    * [Pull request #984](https://github.com/gazebosim/sdformat/pull/984)
    * [Pull request #988](https://github.com/gazebosim/sdformat/pull/988)
    * [Pull request #989](https://github.com/gazebosim/sdformat/pull/989)
    * [Pull request #992](https://github.com/gazebosim/sdformat/pull/992)
    * [Pull request #993](https://github.com/gazebosim/sdformat/pull/993)
    * [Pull request #994](https://github.com/gazebosim/sdformat/pull/994)
    * [Pull request #995](https://github.com/gazebosim/sdformat/pull/995)
    * [Pull request #996](https://github.com/gazebosim/sdformat/pull/996)
    * [Pull request #997](https://github.com/gazebosim/sdformat/pull/997)
    * [Pull request #998](https://github.com/gazebosim/sdformat/pull/998)
    * [Pull request #999](https://github.com/gazebosim/sdformat/pull/999)
    * [Pull request #1001](https://github.com/gazebosim/sdformat/pull/1001)
    * [Pull request #1020](https://github.com/gazebosim/sdformat/pull/1020)
    * [Pull request #1028](https://github.com/gazebosim/sdformat/pull/1028)
    * [Pull request #1029](https://github.com/gazebosim/sdformat/pull/1029)
    * [Pull request #1036](https://github.com/gazebosim/sdformat/pull/1036)
    * [Pull request #1060](https://github.com/gazebosim/sdformat/pull/1060)
    * [Pull request #1061](https://github.com/gazebosim/sdformat/pull/1061)
    * [Pull request #1063](https://github.com/gazebosim/sdformat/pull/1063)
    * [Pull request #1078](https://github.com/gazebosim/sdformat/pull/1078)
    * [Pull request #1083](https://github.com/gazebosim/sdformat/pull/1083)
    * [Pull request #1084](https://github.com/gazebosim/sdformat/pull/1084)
    * [Pull request #1085](https://github.com/gazebosim/sdformat/pull/1085)
    * [Pull request #1106](https://github.com/gazebosim/sdformat/pull/1106)
    * [Pull request #1127](https://github.com/gazebosim/sdformat/pull/1127)
    * [Pull request #1143](https://github.com/gazebosim/sdformat/pull/1143)

1. Copy skybox uri field to sdf/1.10/scene.sdf
    * [Pull request #1082](https://github.com/gazebosim/sdformat/pull/1082)

1. Accept moon and custom surfaces in world spherical coordinates
    * [Pull request #1050](https://github.com/gazebosim/sdformat/pull/1050)

1. Migrate ign -> gz
    * [Pull request #1008](https://github.com/gazebosim/sdformat/pull/1008)
    * [Pull request #1022](https://github.com/gazebosim/sdformat/pull/1022)
    * [Pull request #1040](https://github.com/gazebosim/sdformat/pull/1040)
    * [Pull request #1045](https://github.com/gazebosim/sdformat/pull/1045)
    * [Pull request #1047](https://github.com/gazebosim/sdformat/pull/1047)
    * [Pull request #1048](https://github.com/gazebosim/sdformat/pull/1048)
    * [Pull request #1052](https://github.com/gazebosim/sdformat/pull/1052)
    * [Pull request #1057](https://github.com/gazebosim/sdformat/pull/1057)
    * [Pull request #1058](https://github.com/gazebosim/sdformat/pull/1058)
    * [Pull request #1067](https://github.com/gazebosim/sdformat/pull/1067)
    * [Pull request #1074](https://github.com/gazebosim/sdformat/pull/1074)
    * [Pull request #1078](https://github.com/gazebosim/sdformat/pull/1078)
    * [Pull request #1092](https://github.com/gazebosim/sdformat/pull/1092)

1. Copy 1.9 spec to 1.10
    * [Pull request #1073](https://github.com/gazebosim/sdformat/pull/1073)
    * [Pull request #1076](https://github.com/gazebosim/sdformat/pull/1076)

1. Root: get the world name
    * [Pull request #1027](https://github.com/gazebosim/sdformat/pull/1027)

1. Add SDF::SetRoot and deprecate non-const SDF::Root
    * [Pull request #1070](https://github.com/gazebosim/sdformat/pull/1070)

1. Update GoogleTest to latest version
    * [Pull request #1059](https://github.com/gazebosim/sdformat/pull/1059)
    * [Pull request #1072](https://github.com/gazebosim/sdformat/pull/1072)

1. Update return types for Plugin's Name and Filename
    * [Pull request #1055](https://github.com/gazebosim/sdformat/pull/1055)

1. Surface::ToElement: add //friction/ode/mu
    * [Pull request #1049](https://github.com/gazebosim/sdformat/pull/1049)

1. Joint: rename parent/child `*LinkName` APIs
    * [Pull request #1053](https://github.com/gazebosim/sdformat/pull/1053)
    * [Pull request #1103](https://github.com/gazebosim/sdformat/pull/1103)

1. Deprecate sdf::Inertia class
    * [Pull request #1019](https://github.com/gazebosim/sdformat/pull/1019)

1. Don't include the gz/math.hh header from library code
    * [Pull request #1043](https://github.com/gazebosim/sdformat/pull/1043)

1. Use pose multiplication instead of subtraction
    * [Pull request #1039](https://github.com/gazebosim/sdformat/pull/1039)

1. Remove deprecation warnings
    * [Pull request #1014](https://github.com/gazebosim/sdformat/pull/1014)

1. Added light methods to Link, Root and World
    * [Pull request #1013](https://github.com/gazebosim/sdformat/pull/1013)

1. Add sdf::Error logging in sdf::Param
    * [Pull request #939](https://github.com/gazebosim/sdformat/pull/939)

1. Changes for replacing PythonInterp with Python3
    * [Pull request #907](https://github.com/gazebosim/sdformat/pull/907)

1. Combine find_package(ignition-utils) calls
    * [Pull request #966](https://github.com/gazebosim/sdformat/pull/966)

1. Change default floating point precision to max
    * [Pull request #872](https://github.com/gazebosim/sdformat/pull/872)

1. Clean up compiler warnings
    * [Pull request #882](https://github.com/gazebosim/sdformat/pull/882)

1. Switch to utils version of env functions
    * [Pull request #854](https://github.com/gazebosim/sdformat/pull/854)

1. Updated tests for ign-math's ostream fix
    * [Pull request #847](https://github.com/gazebosim/sdformat/pull/847)

1. Infrastructure
    * [Pull request #803](https://github.com/gazebosim/sdformat/pull/803)
    * [Pull request #805](https://github.com/gazebosim/sdformat/pull/805)
    * [Pull request #878](https://github.com/gazebosim/sdformat/pull/878)
    * [Pull request #980](https://github.com/gazebosim/sdformat/pull/980)
    * [Pull request #974](https://github.com/gazebosim/sdformat/pull/974)

1. Remove completely unused define
    * [Pull request #758](https://github.com/gazebosim/sdformat/pull/758)

## libsdformat 12.X

### libsdformat 12.8.0 (2024-06-06)

1. Add support for no gravity link
    * [Pull request #1410](https://github.com/gazebosim/sdformat/pull/1410)

1. Add bullet and torsional friction DOM
    * [Pull request #1351](https://github.com/gazebosim/sdformat/pull/1351)

1. Fix static builds and optimize test compilation
    * [Pull request #1343](https://github.com/gazebosim/sdformat/pull/1343)
    * [Pull request #1347](https://github.com/gazebosim/sdformat/pull/1347)

1. Update github action workflows
    * [Pull request #1345](https://github.com/gazebosim/sdformat/pull/1345)

### libsdformat 12.7.2 (2023-09-01)

1. Fixed 1.9/light.sdf
    * [Pull request #1281](https://github.com/gazebosim/sdformat/pull/1281)

1. URDF->SDF handle links with no inertia or small mass
    * [Pull request #1238](https://github.com/gazebosim/sdformat/pull/1238)

1. Fix Element::Set method return value
    * [Pull request #1256](https://github.com/gazebosim/sdformat/pull/1256)

1. Add missing values in Surace ToElement method
    * [Pull request #1263](https://github.com/gazebosim/sdformat/pull/1263)

1. Rename COPYING to LICENSE
    * [Pull request #1252](https://github.com/gazebosim/sdformat/pull/1252)

1. Infrastructure
    * [Pull request #1245](https://github.com/gazebosim/sdformat/pull/1245)
    * [Pull request #1271](https://github.com/gazebosim/sdformat/pull/1271)

1. Allow relative paths in URDF
    * [Pull request #1213](https://github.com/gazebosim/sdformat/pull/1213)

### libsdformat 12.7.1 (2023-02-28)

1. Fix camera info topic default value
    * [Pull request #1241](https://github.com/gazebosim/sdformat/pull/1241)

### libsdformat 12.7.0 (2023-02-03)

1. Forward port libsdformat9.10.0. This includes the ign to gz headers.

1. Use File.exist? for Ruby 3.2 compatibility.
    * [Pull request #1216](https://github.com/gazebosim/sdformat/pull/1216)

1. Infrastructure
  1. CI workflow: use checkout v3.
      * [Pull request #1225](https://github.com/gazebosim/sdformat/pull/1225)

  1. macos workflow: don't upgrade existing packages.
      * [Pull request #1217](https://github.com/gazebosim/sdformat/pull/1217)

### libsdformat 12.6.0 (2022-09-07)

1. Reduce the number of include dirs for sdformat.
    * [Pull request #1170](https://github.com/gazebosim/sdformat/pull/1170)

1. Add camera `optical_frame_id` element
    * [Pull request #1109](https://github.com/gazebosim/sdformat/pull/1109)

1. urdf: fix sensor/light pose for links lumped by fixed joints
    * [Pull request #1114](https://github.com/gazebosim/sdformat/pull/1114)

1. Removed USD component from SDFormat and move to gz-usd
    * [Pull request #1094](https://github.com/gazebosim/sdformat/pull/1094)

1. Fix URDF fixed joint reduction of SDF joints
    * [Pull request #1089](https://github.com/gazebosim/sdformat/pull/1089)

1. Test model name as `placement_frame`
    * [Pull request #1079](https://github.com/gazebosim/sdformat/pull/1079)

1. Test using `__model__`, `world` in `@attached_to`, `@relative_to`
    * [Pull request #1066](https://github.com/gazebosim/sdformat/pull/1066)

1. Remove unused sdf.hh.in template
    * [Pull request #1081](https://github.com/gazebosim/sdformat/pull/1081)

1. Readme: Ignition -> Gazebo
    * [Pull request #1080](https://github.com/gazebosim/sdformat/pull/1080)

1. Document major and minor SDFormat version numbers
    * [Pull request #1065](https://github.com/gazebosim/sdformat/pull/1065)

1. Add skybox URI
    * [Pull request #1037](https://github.com/gazebosim/sdformat/pull/1037)

1. Bash completion for flags
    * [Pull request #1042](https://github.com/gazebosim/sdformat/pull/1042)

1. Fix bug with resolving poses for joint sensors.
    * [Pull request #1033](https://github.com/gazebosim/sdformat/pull/1033)

1. sdf::Joint: Mutable versions of SensorByName and SensorByIndex
    * [Pull request #1031](https://github.com/gazebosim/sdformat/pull/1031)

1. Add Link::ResolveInertial API
    * [Pull request #1012](https://github.com/gazebosim/sdformat/pull/1012)

### libsdformat 12.5.0 (2022-05-12)

1. Add visibility mask to Lidar / Ray sensor
    * [Pull request #1015](https://github.com/gazebosim/sdformat/pull/1015)

1. Camera: fix default trigger topic
    * [Pull request #1006](https://github.com/gazebosim/sdformat/pull/1006)

1. Polyline geometry DOM
    * [Pull request #1003](https://github.com/gazebosim/sdformat/pull/1003)

1. Added `<shininess>` to `<material>`
    * [Pull request #985](https://github.com/gazebosim/sdformat/pull/985)
    * [Pull request #1016](https://github.com/gazebosim/sdformat/pull/1016)

1. `inertial.sdf`: fix ambiguities in documentation
    * [Pull request #990](https://github.com/gazebosim/sdformat/pull/990)

1. Added equality operators to Plugin
    * [Pull request #912](https://github.com/gazebosim/sdformat/pull/912)

1. Added convenience constructor to plugin
    * [Pull request #911](https://github.com/gazebosim/sdformat/pull/911)

1. Use gz-utils instead of gz-cmake utilities
    * [Pull request #978](https://github.com/gazebosim/sdformat/pull/978)

1. Added `Friction` and `ODE` classes
    * [Pull request #955](https://github.com/gazebosim/sdformat/pull/955)

1. Add `L16` pixel format to Camera pixel format conversion function
    * [Pull request #487](https://github.com/gazebosim/sdformat/pull/487)

1. Added ``--inertial-stats`` option to ``gz sdf``
    * [Pull request #936](https://github.com/gazebosim/sdformat/pull/936)

1. Added `anti_aliasing` element to camera's SDF
    * [Pull request #922](https://github.com/gazebosim/sdformat/pull/922)
    * [Pull request #909](https://github.com/gazebosim/sdformat/pull/909)

1. DOC 1.9: clarify behavior of //model/model/static
    * [Pull request #921](https://github.com/gazebosim/sdformat/pull/921)

1. SDF to USD
    * [Pull request #975](https://github.com/gazebosim/sdformat/pull/975)
    * [Pull request #976](https://github.com/gazebosim/sdformat/pull/976)
    * [Pull request #914](https://github.com/gazebosim/sdformat/pull/914)
    * [Pull request #917](https://github.com/gazebosim/sdformat/pull/917)
    * [Pull request #925](https://github.com/gazebosim/sdformat/pull/925)

1. USD to SDF
    * [Pull request #991](https://github.com/gazebosim/sdformat/pull/991)
    * [Pull request #904](https://github.com/gazebosim/sdformat/pull/904)
    * [Pull request #903](https://github.com/gazebosim/sdformat/pull/903)
    * [Pull request #900](https://github.com/gazebosim/sdformat/pull/900)
    * [Pull request #899](https://github.com/gazebosim/sdformat/pull/899)
    * [Pull request #898](https://github.com/gazebosim/sdformat/pull/898)
    * [Pull request #897](https://github.com/gazebosim/sdformat/pull/897)
    * [Pull request #902](https://github.com/gazebosim/sdformat/pull/902)
    * [Pull request #876](https://github.com/gazebosim/sdformat/pull/876)
    * [Pull request #875](https://github.com/gazebosim/sdformat/pull/875)
    * [Pull request #871](https://github.com/gazebosim/sdformat/pull/871)

### libsdformat 12.4.0 (2022-03-29)

1. Use ParserConfig more in parser.cc
    * [Pull request #883](https://github.com/gazebosim/sdformat/pull/883)
    * [Pull request #885](https://github.com/gazebosim/sdformat/pull/885)
    * [Pull request #910](https://github.com/gazebosim/sdformat/pull/910)

1. Added option to visualize light in GUI
    * [Pull request #877](https://github.com/gazebosim/sdformat/pull/877)

1. Make `computeMergedModelProxyFrameName` public
    * [Pull request #868](https://github.com/gazebosim/sdformat/pull/868)

1. SDFormat to USD conversion
    * [Pull request #818](https://github.com/gazebosim/sdformat/pull/818)
    * [Pull request #827](https://github.com/gazebosim/sdformat/pull/827)
    * [Pull request #828](https://github.com/gazebosim/sdformat/pull/828)
    * [Pull request #829](https://github.com/gazebosim/sdformat/pull/829)
    * [Pull request #830](https://github.com/gazebosim/sdformat/pull/830)
    * [Pull request #831](https://github.com/gazebosim/sdformat/pull/831)
    * [Pull request #837](https://github.com/gazebosim/sdformat/pull/837)
    * [Pull request #862](https://github.com/gazebosim/sdformat/pull/862)
    * [Pull request #863](https://github.com/gazebosim/sdformat/pull/863)
    * [Pull request #870](https://github.com/gazebosim/sdformat/pull/870)
    * [Pull request #888](https://github.com/gazebosim/sdformat/pull/888)
    * [Pull request #889](https://github.com/gazebosim/sdformat/pull/889)
    * [Pull request #895](https://github.com/gazebosim/sdformat/pull/895)
    * [Pull request #896](https://github.com/gazebosim/sdformat/pull/896)
    * [Pull request #901](https://github.com/gazebosim/sdformat/pull/901)
    * [Pull request #906](https://github.com/gazebosim/sdformat/pull/906)
    * [Pull request #908](https://github.com/gazebosim/sdformat/pull/908)
    * [Pull request #913](https://github.com/gazebosim/sdformat/pull/913)
    * [Pull request #915](https://github.com/gazebosim/sdformat/pull/915)

1. Add ToElement conversions for various classes
    * [Pull request #771](https://github.com/gazebosim/sdformat/pull/771)
    * [Pull request #772](https://github.com/gazebosim/sdformat/pull/772)
    * [Pull request #775](https://github.com/gazebosim/sdformat/pull/775)
    * [Pull request #776](https://github.com/gazebosim/sdformat/pull/776)
    * [Pull request #777](https://github.com/gazebosim/sdformat/pull/777)
    * [Pull request #781](https://github.com/gazebosim/sdformat/pull/781)
    * [Pull request #782](https://github.com/gazebosim/sdformat/pull/782)
    * [Pull request #783](https://github.com/gazebosim/sdformat/pull/783)
    * [Pull request #842](https://github.com/gazebosim/sdformat/pull/842)
    * [Pull request #887](https://github.com/gazebosim/sdformat/pull/887)
    * [Pull request #918](https://github.com/gazebosim/sdformat/pull/918)

1. Fix compiler warnings
    * [Pull request #808](https://github.com/gazebosim/sdformat/pull/808)
    * [Pull request #810](https://github.com/gazebosim/sdformat/pull/810)

1. Infrastructure and Documentation
    * [Pull request #861](https://github.com/gazebosim/sdformat/pull/861)
    * [Pull request #800](https://github.com/gazebosim/sdformat/pull/800)
    * [Pull request #686](https://github.com/gazebosim/sdformat/pull/686)
    * [Pull request #713](https://github.com/gazebosim/sdformat/pull/713)
    * [Pull request #864](https://github.com/gazebosim/sdformat/pull/864)
    * [Pull request #880](https://github.com/gazebosim/sdformat/pull/880)
    * [Pull request #890](https://github.com/gazebosim/sdformat/pull/890)
    * [Pull request #891](https://github.com/gazebosim/sdformat/pull/891)

1. Use the Plugin DOM in other DOM objects
    * [Pull request #858](https://github.com/gazebosim/sdformat/pull/858)

1. Add SDFormat tags for Triggered Cameras
    * [Pull request #846](https://github.com/gazebosim/sdformat/pull/846)

1. Fix bug where //include/pose was ignored when using the Interface API
    * [Pull request #853](https://github.com/gazebosim/sdformat/pull/853)

1. Fix joint parent/child frame existence checks to include interface elements
    * [Pull request #855](https://github.com/gazebosim/sdformat/pull/855)

1. Remove USD visibility macro from internal APIs
    * [Pull request #857](https://github.com/gazebosim/sdformat/pull/857)

1. Added non-const mutable accessors for world child objects
    * [Pull request #840](https://github.com/gazebosim/sdformat/pull/840)

1. Added non-const accessors for Model child objects
    * [Pull request #839](https://github.com/gazebosim/sdformat/pull/839)

1. Added to light if the light is on or off
    * [Pull request #851](https://github.com/gazebosim/sdformat/pull/851)

1. Added Root mutable accessors, and Root::Clone function
    * [Pull request #841](https://github.com/gazebosim/sdformat/pull/841)

1. Hide USDUtils.hh file from public API
    * [Pull request #850](https://github.com/gazebosim/sdformat/pull/850)

1. Added non-const accessors for Link child objects
    * [Pull request #838](https://github.com/gazebosim/sdformat/pull/838)

1. Add USDError class
    * [Pull request #836](https://github.com/gazebosim/sdformat/pull/836)

1. Use USD component visibility macro
    * [Pull request #849](https://github.com/gazebosim/sdformat/pull/849)

1. Add support for merge-include in the Interface API
    * [Pull request #768](https://github.com/gazebosim/sdformat/pull/768)

1. Handle `__model__` in joint parent or child when using merge-include
    * [Pull request #835](https://github.com/gazebosim/sdformat/pull/835)

1. Allow model frames (__model__) to be used as joint parent or child
    * [Pull request #833](https://github.com/gazebosim/sdformat/pull/833)

1. Fix bug where a sdf::ParserConfig object was not passed to all sdf::readFile calls
    * [Pull request #824](https://github.com/gazebosim/sdformat/pull/824)

1. Make SDF to USD a separate component of sdformat
    * [Pull request #817](https://github.com/gazebosim/sdformat/pull/817)

1. Add ParserConfig flag for preserveFixedJoint
    * [Pull request #815](https://github.com/gazebosim/sdformat/pull/815)

1. Fix parsing 'type' attibutes in plugins
    * [Pull request #809](https://github.com/gazebosim/sdformat/pull/809)

1. sdf_custom: fix nested model expectations
    * [Pull request #807](https://github.com/gazebosim/sdformat/pull/807)

1. Replace custom cmake code with gz-cmake2
    * [Pull request #780](https://github.com/gazebosim/sdformat/pull/780)

1. Support printing sdf poses in degrees and allow snapping to commonly used angles
    * [Pull request #689](https://github.com/gazebosim/sdformat/pull/689)

1. Refactor FrameSemantics.cc
    * [Pull request #764](https://github.com/gazebosim/sdformat/pull/764)

1. Fix loading nested include with custom attributes
    * [Pull request #789](https://github.com/gazebosim/sdformat/pull/789)

1. Added plugin to SDF DOM
    * [Pull request #788](https://github.com/gazebosim/sdformat/pull/788)

1. Support URI in the Model DOM
    * [Pull request #786](https://github.com/gazebosim/sdformat/pull/786)

1. Support adding and clearing sensors from a joint
    * [Pull request #785](https://github.com/gazebosim/sdformat/pull/785)

1. PrintConfig option to preserve includes when converting to string
    * [Pull request #749](https://github.com/gazebosim/sdformat/pull/749)

### libsdformat 12.3.0 (2021-12-01)

1. Fix empty pose parsing fail for rotation_format='quat_xyzw'
    * [Pull request #729](https://github.com/gazebosim/sdformat/pull/729)

1. Added Add & Clear function to World, Model, and Link.
    * [Pull request #765](https://github.com/gazebosim/sdformat/pull/765)

### libsdformat 12.2.0 (2021-11-23)

1. Convert Joint DOM to Element.
    * [Pull request #759](https://github.com/gazebosim/sdformat/pull/759)

1. Populate light sdf::ElementPtr from Light DOM
    * [Pull request #755](https://github.com/gazebosim/sdformat/pull/755)

1. Add function to convert Sensor DOM to sdf::ElementPtr
    * [Pull request #753](https://github.com/gazebosim/sdformat/pull/753)
    * [Pull request #757](https://github.com/gazebosim/sdformat/pull/757)

1. Support wide angle camera.
    * [Pull request #744](https://github.com/gazebosim/sdformat/pull/744)

1. Forward ports
    * [Pull request #756](https://github.com/gazebosim/sdformat/pull/756)
    * [Pull request #734](https://github.com/gazebosim/sdformat/pull/734)
    * [Pull request #738](https://github.com/gazebosim/sdformat/pull/738)
    * [Pull request #750](https://github.com/gazebosim/sdformat/pull/750)
    * [Pull request #752](https://github.com/gazebosim/sdformat/pull/752)

1. Changelog links to BitBucket backup.
    * [Pull request #237](https://github.com/gazebosim/sdformat/pull/237)

1. Update BitBucket links.
    * [Pull request #248](https://github.com/gazebosim/sdformat/pull/248)

1. Cherry-pick [sdf4] Update BitBucket links -> sdf6
    * [Pull request #258](https://github.com/gazebosim/sdformat/pull/258)

1.  Patch popen/pclose method for Windows.
    * [Pull request #297](https://github.com/gazebosim/sdformat/pull/297)

1. Parse rpyOffset as radians
    * [Pull request #497](https://github.com/gazebosim/sdformat/pull/497)

1. Fix flattening logic for nested model names (sdf6)
    * [Pull request #597](https://github.com/gazebosim/sdformat/pull/597)

1. Translate poses of nested models inside other nested models (sdf6).
    * [Pull request #596](https://github.com/gazebosim/sdformat/pull/596)

1. Use Ubuntu bionic in CI
    * [Pull request #626](https://github.com/gazebosim/sdformat/pull/626)

1. Create CODEOWNERS with azeey and scpeters.
    * [Pull request #650](https://github.com/gazebosim/sdformat/pull/650)

1. Remove bitbucket-pipelines.
    * [Pull request #674](https://github.com/gazebosim/sdformat/pull/674)

1. Check joint parent/child names in Root::Load.
    * [Pull request #727](https://github.com/gazebosim/sdformat/pull/727)

1. Check joint parent link names in Model::Load.
    * [Pull request #726](https://github.com/gazebosim/sdformat/pull/726)

1. Add Joint DOM API to access joint sensors
    * [Pull request #517](https://github.com/gazebosim/sdformat/pull/517)

1. Remove outdated deprecation note from parser_urdf.hh
    * [Pull request #740](https://github.com/gazebosim/sdformat/pull/740)

1. DOC: only allow one canonical_link attribute
    * [Pull request #716](https://github.com/gazebosim/sdformat/pull/716)

1. Fix URDF fixed joint reduction of plugins
    * [Pull request #500](https://github.com/gazebosim/sdformat/pull/500)
    * [Pull request #745](https://github.com/gazebosim/sdformat/pull/745)

### libsdformat 12.1.0 (2021-11-09)

1. Support accessing mutable sensor types.
    * [Pull request #737](https://github.com/gazebosim/sdformat/pull/737)

### libsdformat 12.0.0 (2021-09-30)

1. Make exception for plugins when checking for name uniqueness
    * [Pull request #721](https://github.com/gazebosim/sdformat/pull/721)

1. Remove empty //inertial/pose/@relative_to during 1_7->1.8 conversion
    * [Pull request #720](https://github.com/gazebosim/sdformat/pull/720)

1. Added macos install instructions to README.md
    * [Pull request #714](https://github.com/gazebosim/sdformat/pull/714)

1. Do not automatically remove //axis/initial_position
    * [Pull request #717](https://github.com/gazebosim/sdformat/pull/717)

1. DOC: don't mention elements that can't be included
    * [Pull request #715](https://github.com/gazebosim/sdformat/pull/715)

1. Prefix merged frames with an underscore
    * [Pull request #711](https://github.com/gazebosim/sdformat/pull/711)

1. Add API changes for PrintConfig
    * [Pull request #708](https://github.com/gazebosim/sdformat/pull/708)

1. Add Force Torque Noise functions + Unit tests
    * [Pull request #669](https://github.com/gazebosim/sdformat/pull/669)

1. Support quaternions representation for poses
    * [Pull request #690](https://github.com/gazebosim/sdformat/pull/690)

1. Support merge-include of nested models
    * [Pull request #659](https://github.com/gazebosim/sdformat/pull/659)

1. Fix documentation on Euler angle convention in pose.sdf
    * [Pull request #698](https://github.com/gazebosim/sdformat/pull/698)
    * [Pull request #702](https://github.com/gazebosim/sdformat/pull/702)

1. Clarify documentation on //pose/@relative_to in the spec
    * [Pull request #666](https://github.com/gazebosim/sdformat/pull/666)

1. 🌐 Parse spherical coordinates
    * [Pull request #685](https://github.com/gazebosim/sdformat/pull/685)

1. Fix bug when using degrees in //include/pose
    * [Pull request #697](https://github.com/gazebosim/sdformat/pull/697)

1. Support rotation in degrees (#589)
    * [Pull request #589](https://github.com/gazebosim/sdformat/pull/589)

1. Add segmentation and bounding box sensor types
    * [Pull request #592](https://github.com/gazebosim/sdformat/pull/592)

1. Add support for custom sensors
    * [Pull request #652](https://github.com/gazebosim/sdformat/pull/652)

1. Remove deprecated functions and classes
    * [Pull request #622](https://github.com/gazebosim/sdformat/pull/622)

1. Emit an error instead of a warning when a file has multiple root level
    * [Pull request #619](https://github.com/gazebosim/sdformat/pull/619)

1. Copy spec 1.8 to 1.9
    * [Pull request #568](https://github.com/gazebosim/sdformat/pull/568)

1. Use encapsulated string constants for non-file sources
    * [Pull request #551](https://github.com/gazebosim/sdformat/pull/551)

1. Forward ports
    * [Pull request #559](https://github.com/gazebosim/sdformat/pull/559)
    * [Pull request #576](https://github.com/gazebosim/sdformat/pull/576)
    * [Pull request #594](https://github.com/gazebosim/sdformat/pull/594)
    * [Pull request #613](https://github.com/gazebosim/sdformat/pull/613)
    * [Pull request #627](https://github.com/gazebosim/sdformat/pull/627)
    * [Pull request #660](https://github.com/gazebosim/sdformat/pull/660)
    * [Pull request #693](https://github.com/gazebosim/sdformat/pull/693)
    * [Pull request #706](https://github.com/gazebosim/sdformat/pull/706)
    * [Pull request #723](https://github.com/gazebosim/sdformat/pull/723)

1. Infrastructure
    * [Pull request #532](https://github.com/gazebosim/sdformat/pull/532)
    * [Pull request #564](https://github.com/gazebosim/sdformat/pull/564)

## libsdformat 11.X

### libsdformat 11.4.1 (2022-03-21)

1. Install sdf/1.8 to versioned path
    * [Pull request #898](https://github.com/gazebosim/sdformat/pull/898)

### libsdformat 11.4.0 (2022-03-14)

1. Added option to visualize light on the GUI
    * [Pull request #877](https://github.com/gazebosim/sdformat/pull/877)

1. Fix joint parent/child frame existence checks to include interface elements
    * [Pull request #855](https://github.com/gazebosim/sdformat/pull/855)

1. Added to light whether it is on or off
    * [Pull request #851](https://github.com/gazebosim/sdformat/pull/851)

1. Allow model frames (__model__) to be used as joint parent or child
    * [Pull request #833](https://github.com/gazebosim/sdformat/pull/833)

1. Add ParserConfig flag for preserveFixedJoint
    * [Pull request #815](https://github.com/gazebosim/sdformat/pull/815)

1. Fix compiler warnings
    * [Pull request #808](https://github.com/gazebosim/sdformat/pull/808)

1. `sdf_custom`: fix nested model expectations
    * [Pull request #807](https://github.com/gazebosim/sdformat/pull/807)

1. Fix test compilation with `USE_INTERNAL_URDF`
    * [Pull request #800](https://github.com/gazebosim/sdformat/pull/800)

1. Replace custom CMake code with `gz-cmake2`
    * [Pull request #780](https://github.com/gazebosim/sdformat/pull/780)

1. Fix loading nested include with custom attributes
    * [Pull request #789](https://github.com/gazebosim/sdformat/pull/789)

1. Documentation
    1. Clarify behavior of `//model/model/static`
        * [Pull request #713](https://github.com/gazebosim/sdformat/pull/713)
    1. Only allow one `canonical_link` attribute for model
        * [Pull request #716](https://github.com/gazebosim/sdformat/pull/716)
    1. Don't mention elements that can't be included
        * [Pull request #715](https://github.com/gazebosim/sdformat/pull/715)
    1. Clarify documentation on `//pose/@relative_to` in the spec
        * [Pull request #666](https://github.com/gazebosim/sdformat/pull/666)
    1. Remove duplicate link documentation
        * [Pull request #702](https://github.com/gazebosim/sdformat/pull/702)

1. Fix URDF fixed joint reduction of plugins
    * [Pull request #745](https://github.com/gazebosim/sdformat/pull/745)

1. Add `enable_orientation` to 1.6 spec
    * [Pull request #686](https://github.com/gazebosim/sdformat/pull/686)

1. Remove outdated deprecation note from `parser_urdf.hh`
    * [Pull request #740](https://github.com/gazebosim/sdformat/pull/740)

1. Add Joint DOM API to access joint sensors
    * [Pull request #517](https://github.com/gazebosim/sdformat/pull/517)

1. Add force torque sensor
    * [Pull request #393](https://github.com/gazebosim/sdformat/pull/393)
    * [Pull request #669](https://github.com/gazebosim/sdformat/pull/669)

1. Check joint parent link names in `Model::Load`
    * [Pull request #726](https://github.com/gazebosim/sdformat/pull/726)

1. Check joint parent/child names in `Root::Load`
    * [Pull request #727](https://github.com/gazebosim/sdformat/pull/727)

1. Remove empty `//inertial/pose/@relative_to` during 1_7->1.8 conversion
    * [Pull request #720](https://github.com/gazebosim/sdformat/pull/720)

1. Fix `xyz` and `rpy` offsets in fixed joint reduction
    * [Pull request #500](https://github.com/gazebosim/sdformat/pull/500)

1. Infrastructure updates
    * [Pull request #674](https://github.com/gazebosim/sdformat/pull/674)
    * [Pull request #650](https://github.com/gazebosim/sdformat/pull/650)
    * [Pull request #626](https://github.com/gazebosim/sdformat/pull/626)
    * [Pull request #258](https://github.com/gazebosim/sdformat/pull/258)
    * [Pull request #237](https://github.com/gazebosim/sdformat/pull/237)
    * [Pull request #730](https://github.com/gazebosim/sdformat/pull/730)

1. Translate poses of nested models inside other nested models
    * [Pull request #596](https://github.com/gazebosim/sdformat/pull/596)

1. Fix flattening logic for nested model names
    * [Pull request #597](https://github.com/gazebosim/sdformat/pull/597)

1. Parse `rpyOffset` as radians
    * [Pull request #497](https://github.com/gazebosim/sdformat/pull/497)

### libsdformat 11.3.0 (2021-09-10)

1. Fix world-complete.sdf and add particle_scatter_ratio to v1.8
    * [Pull request #695](https://github.com/gazebosim/sdformat/pull/695)

1. Parse URDF continuous joint effort/velocity limits
    * [Pull request #684](https://github.com/gazebosim/sdformat/pull/684)

1. Add `enable_orientation` SDF element to imu
    * [Pull request #651](https://github.com/gazebosim/sdformat/pull/651)

1. Add a codecheck make target
    * [Pull request #682](https://github.com/gazebosim/sdformat/pull/682)

1. Refactor sdf::readXml
    * [Pull request #681](https://github.com/gazebosim/sdformat/pull/681)

1. Upgrade cpplint and fix new errors
    * [Pull request #680](https://github.com/gazebosim/sdformat/pull/680)

1. Infrastructure and documentation
    * [Pull request #679](https://github.com/gazebosim/sdformat/pull/679)
    * [Pull request #678](https://github.com/gazebosim/sdformat/pull/678)
    * [Pull request #676](https://github.com/gazebosim/sdformat/pull/676)
    * [Pull request #673](https://github.com/gazebosim/sdformat/pull/673)
    * [Pull request #630](https://github.com/gazebosim/sdformat/pull/630)

1. Added comment reminder to update functions
    * [Pull request #677](https://github.com/gazebosim/sdformat/pull/677)

1. BUG: add missing plugin element to include
    * [Pull request #668](https://github.com/gazebosim/sdformat/pull/668)
    * [Pull request #675](https://github.com/gazebosim/sdformat/pull/675)

1. Adds `enable_metrics` flag to Sensor.
    * [Pull request #665](https://github.com/gazebosim/sdformat/pull/665)

1. BUG: make time type string [s ns]
    * [Pull request #662](https://github.com/gazebosim/sdformat/pull/662)

1. Add GPS sensor to sdf9
    * [Pull request #453](https://github.com/gazebosim/sdformat/pull/453)

1. Spec change booleans from 0/1 to false/true
    * [Pull request #663](https://github.com/gazebosim/sdformat/pull/663)

1. Support parsing elements that are not part of the schema
    * [Pull request #638](https://github.com/gazebosim/sdformat/pull/638)

1. Add lightmap to 1.7 spec and PBR material DOM
    * [Pull request #429](https://github.com/gazebosim/sdformat/pull/429)

1. Fix urdf link extension tags
    * [Pull request #628](https://github.com/gazebosim/sdformat/pull/628)

1. Fix unreported invalid model when reference frame is unavailable
    * [Pull request #636](https://github.com/gazebosim/sdformat/pull/636)

1. Updated material spec
    * [Pull request #644](https://github.com/gazebosim/sdformat/pull/644)

1. BUG: add missing sdf files to CMakeLists
    * [Pull request #631](https://github.com/gazebosim/sdformat/pull/631)

1. Update build system to allow overriding CXX flags and using clang on Linux
    * [Pull request #621](https://github.com/gazebosim/sdformat/pull/621)

1. Error: move << operator from .hh to .cc file
    * [Pull request #623](https://github.com/gazebosim/sdformat/pull/623)
    * [Pull request #625](https://github.com/gazebosim/sdformat/pull/625)

1. Add Element::FindElement as an alternative to Element::GetElement
    * [Pull request #620](https://github.com/gazebosim/sdformat/pull/620)

1. Parameter passing prototype
    * [Pull request #413](https://github.com/gazebosim/sdformat/pull/413)

1. Port particle scatter ratio param to sdf 1.6
    * [Pull request #595](https://github.com/gazebosim/sdformat/pull/595)

### libsdformat 11.2.2 (2021-07-01)

1. Fix segfault when checking for required elements in joint
    * [Pull request #610](https://github.com/gazebosim/sdformat/pull/610)

1. Add ValidateGraphs methods to Model/World
    * [Pull request #601](https://github.com/gazebosim/sdformat/pull/601)

1. Making `PrintValues()` and `ToString()` able to not print default elements
    * [Pull request #575](https://github.com/gazebosim/sdformat/pull/575)

### libsdformat 11.2.1 (2021-06-28)

1. Fix ABI break on sdf11
    * [Pull request #606](https://github.com/gazebosim/sdformat/pull/606)

1. Add triage, remove ticket templates
    * [Pull request #608](https://github.com/gazebosim/sdformat/pull/608)

### libsdformat 11.2.0 (2021-06-23)

1. Revert behavior of FilePath() for elements loaded from strings
    * [Pull request #582](https://github.com/gazebosim/sdformat/pull/582)

1. Adding comment and modifications to element_tracing test regarding path sanitization
    * [Pull request #578](https://github.com/gazebosim/sdformat/pull/578)

1. Allow to convert URDF color to SDF material tag
    * [Pull request #526](https://github.com/gazebosim/sdformat/pull/526)

1. Encode XML path and line number in Element and add XML path setter Error.
    * [Pull request #548](https://github.com/gazebosim/sdformat/pull/548)

1. Merge sdf 1.7 changes forward to 1.8
    * [Pull request #570](https://github.com/gazebosim/sdformat/pull/570)

1. Add API for determining if an element was set by the user
    * [Pull request #542](https://github.com/gazebosim/sdformat/pull/542)

1. Add scatter ratio parameter to Particle Emitter DOM
    * [Pull request #547](https://github.com/gazebosim/sdformat/pull/547)

1. Fix bug where included URDFs are not parsed when there are no custom parsers
    * [Pull request #562](https://github.com/gazebosim/sdformat/pull/562)

### libsdformat 11.1.0 (2021-04-30)

1. Clean up use of PROJECT_SOURCE_PATH in tests
    * [Pull request #549](https://github.com/osrf/sdformat/pull/549)

1. Methods for removing attributes from an element
    * [Pull request #555](https://github.com/osrf/sdformat/pull/555)

1. Add an EnforcementPolicy for deprecated elements
    * [Pull request #543](https://github.com/osrf/sdformat/pull/543)

1. Fixed application of <sensor><pose> tags in lumped linkes during URDF conversion
    * [Pull request #525](https://github.com/osrf/sdformat/pull/525)

1. Particle emitter
    * [Pull request #528](https://github.com/osrf/sdformat/pull/528)

1. Improve docs of collision_bitmask.
    * [Pull request #521](https://github.com/osrf/sdformat/pull/521)


### libsdformat 11.0.0 (2021-03-30)

1. Add ParserConfig class to encapsulate file path settings.
    * [Pull request 439](https://github.com/osrf/sdformat/pull/439)

1. Add EnforcementPolicy in ParserConfig to configure parsing strictness.
    * [Pull request 481](https://github.com/osrf/sdformat/pull/481)

1. Use `ImplPtr` / `UniqueImplPtr` from gz-utils
    * [Pull request 472](https://github.com/osrf/sdformat/pull/472)
    * [Pull request 474](https://github.com/osrf/sdformat/pull/474)

1. Allow files paths for include URIs
    * [Pull request 448](https://github.com/osrf/sdformat/pull/448)

1. Add split/join for separating the link name from fully-qualified name.
    * [Pull request 457](https://github.com/osrf/sdformat/pull/457)

1. SDFormat 1.8: Add ellipsoid geometry type.
    * [Pull request 434](https://github.com/osrf/sdformat/pull/434)

1. SDFormat 1.8: Add capsule geometry type.
    * [Pull request 389](https://github.com/osrf/sdformat/pull/389)

1. SDFormat 1.8: Add light intensity field.
    * [Pull request 484](https://github.com/osrf/sdformat/pull/484)

1. SDFormat 1.8: reduce default heightmap sampling from 2 to 1.
    * [Pull request 459](https://github.com/osrf/sdformat/pull/459)

1. SDFormat 1.8: remove `//inertial/pose/@relative_to` from spec.
    * [Pull request 480](https://github.com/osrf/sdformat/pull/480)

1. Allow only one of actor/light/model for <include> tags.
    * [Pull request 433](https://github.com/osrf/sdformat/pull/433)
    * [Pull request 444](https://github.com/osrf/sdformat/pull/444)

1. Add support for building static library.
    * [Pull request 394](https://github.com/osrf/sdformat/pull/394)

1. Add force torque sensor.
    * [Pull request 393](https://github.com/osrf/sdformat/pull/393)

1. Simplify data embedding.
    * [Pull request 270](https://github.com/osrf/sdformat/pull/270)

1. Properly handle the requirement of C++17 at the CMake exported target level.
    * [Pull request 251](https://github.com/osrf/sdformat/pull/251)

1. Update documentation for Cylinder length.
    * [Pull request 318](https://github.com/osrf/sdformat/pull/318)

1. Root: fix grammar in error message.
    * [Pull request 478](https://github.com/osrf/sdformat/pull/478)

1. Ensure relocatable config files.
    * [Pull request 419](https://github.com/osrf/sdformat/pull/419)

1. Update CI.
    * [Pull request 452](https://github.com/osrf/sdformat/pull/452)
    * [Pull request 255](https://github.com/osrf/sdformat/pull/255)

1. Update README, Changelog, Contributing Guide, and Code of Conduct.
    * [Pull request 431](https://github.com/osrf/sdformat/pull/431)
    * [Pull request 275](https://github.com/osrf/sdformat/pull/275)
    * [Pull request 250](https://github.com/osrf/sdformat/pull/250)
    * [Pull request 243](https://github.com/osrf/sdformat/pull/243)
    * [Pull request 524](https://github.com/osrf/sdformat/pull/524)

1. Implement SDFormat 1.8 Model Composition.
    * [Pull request 426](https://github.com/osrf/sdformat/pull/426)
    * [Pull request 381](https://github.com/osrf/sdformat/pull/381)
    * [Pull request 355](https://github.com/osrf/sdformat/pull/355)
    * [Pull request 324](https://github.com/osrf/sdformat/pull/324)
    * [Pull request 304](https://github.com/osrf/sdformat/pull/304)
    * [Pull request #518](https://github.com/osrf/sdformat/pull/518)
    * [Pull request #504](https://github.com/osrf/sdformat/pull/504)

1. Fix precision loss when adding nested models.
    * [Pull request 314](https://github.com/osrf/sdformat/pull/314)

1. Initial version of SDFormat 1.8 specification.
    * [BitBucket pull request 682](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/682)

1. SDFormat 1.8: Deprecate //joint/axis/initial_position.
    * [BitBucket pull request 683](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/683)

1. Implement SDFormat 1.8 Interface API
    * [Pull request #475](https://github.com/osrf/sdformat/pull/475)

1. Add Joint DOM API to access joint sensors
    * [Pull request #517](https://github.com/osrf/sdformat/pull/517)

1. Store include info in Element
    * [Pull request #509](https://github.com/osrf/sdformat/pull/509)

1. Updated loading //material colors
    * [Pull request #519](https://github.com/osrf/sdformat/pull/519)

1. Include file path and line number in parsing errors
    * [Pull request #512](https://github.com/osrf/sdformat/pull/512)

1. Error when delimiter "::" found in element name in SDFormat 1.8
    * [Pull request #515](https://github.com/osrf/sdformat/pull/515)

1. Add camera type aliases to docs.
    * [Pull request #514](https://github.com/osrf/sdformat/pull/514)

1. Fix lidar resolution description about interpolation
    * [Pull request #506](https://github.com/osrf/sdformat/pull/506)

1. Added issue & PR templates
    * [Pull request #486](https://github.com/osrf/sdformat/pull/486)

1. Add L16 pixel format to Camera pixel format conversion function
    * [Pull request #487](https://github.com/osrf/sdformat/pull/487)

1. Quietly search for urdfdom
    * [Pull request #493](https://github.com/osrf/sdformat/pull/493)

1. tests: Ensure removed SDFormat elements raise errors in newer versions
    * [Pull request #490](https://github.com/osrf/sdformat/pull/490)

1. sdformat.pc.in: requires ignition-utils1
    * [Pull request #489](https://github.com/osrf/sdformat/pull/489)
    * [Pull request #503](https://github.com/osrf/sdformat/pull/503)

1. Wrap description tags in CDATA
    * [Pull request #483](https://github.com/osrf/sdformat/pull/483)

1. Fix temperature gradient default value in Atmosphere DOM
    * [Pull request #482](https://github.com/osrf/sdformat/pull/482)

1. Add laser_retro in Visual
    * [Pull request #454](https://github.com/osrf/sdformat/pull/454)

1. Add Windows installation
    * [Pull request #463](https://github.com/osrf/sdformat/pull/463)

1. Replace PROJECT_SOURCE_DIR in tests
    * [Pull request #460](https://github.com/osrf/sdformat/pull/460)

## libsdformat 10.X

### libsdformat 10.7.0 (2021-12-27)

1. Fix flattening logic for nested model names (merged forward from sdf6)
    * [Pull request #597](https://github.com/gazebosim/sdformat/pull/597)

1. Create CODEOWNERS with azeey and scpeters
    * [Pull request #650](https://github.com/gazebosim/sdformat/pull/650)

1. Fix xyz and rpy offsets in fixed joint reduction
    * [Pull request #500](https://github.com/gazebosim/sdformat/pull/500)

1. Check joint parent link names in Model::Load
    * [Pull request #726](https://github.com/osrf/sdformat/pull/726)

1. Make exception for plugins when checking for name uniqueness
    * [Pull request #733](https://github.com/gazebosim/sdformat/pull/733)

1. Added Force Torque Noise functions + Unit tests
    * [Pull request #669](https://github.com/gazebosim/sdformat/pull/669)

1. Add Joint DOM API to access joint sensors
    * [Pull request #517](https://github.com/gazebosim/sdformat/pull/517)

1. Add force torque sensor
    * [Pull request #393](https://github.com/gazebosim/sdformat/pull/393)
    * A contribution from Nick Lamprianidis <nlamprian@gmail.com>

1. Remove outdated deprecation note from parser_urdf.hh
    * [Pull request #740](https://github.com/osrf/sdformat/pull/740)

1. Fix URDF fixed joint reduction of plugins
    * [Pull request #745](https://github.com/osrf/sdformat/pull/745)

1. Fix loading nested include with custom attributes
    * [Pull request #789](https://github.com/gazebosim/sdformat/pull/789)

1. Replace custom cmake code with gz-cmake2
    * [Pull request #780](https://github.com/osrf/sdformat/pull/780)

1. Fix test compilation with USE_INTERNAL_URDF
    * [Pull request #800](https://github.com/gazebosim/sdformat/pull/800)

### libsdformat 10.6.0 (2021-09-08)

1. Parse URDF continuous joint effort/velocity limits
    * [Pull request #684](https://github.com/gazebosim/sdformat/pull/684)

1. Add enable_orientation SDF element to imu
    * [Pull request #651](https://github.com/gazebosim/sdformat/pull/651)

1. Add a codecheck make target
    * [Pull request #682](https://github.com/gazebosim/sdformat/pull/682)

1. Refactor sdf::readXml
    * [Pull request #681](https://github.com/gazebosim/sdformat/pull/681)

1. Upgrade cpplint and fix new errors
    * [Pull request #680](https://github.com/gazebosim/sdformat/pull/680)

1. BUG: add missing plugin element to include
    * [Pull request #675](https://github.com/gazebosim/sdformat/pull/675)

1. Added comment reminder to update functions
    * [Pull request #677](https://github.com/gazebosim/sdformat/pull/677)

1. Adds enable_metrics flag to Sensor.
    * [Pull request #665](https://github.com/gazebosim/sdformat/pull/665)

1. Add GPS / NavSat sensor to sdf9
    * [Pull request #453](https://github.com/gazebosim/sdformat/pull/453)

1. Support parsing elements that are not part of the schema
    * [Pull request #638](https://github.com/gazebosim/sdformat/pull/638)

1. Add lightmap to 1.7 spec and PBR material DOM
    * [Pull request #429](https://github.com/gazebosim/sdformat/pull/429)

1. Fix urdf link extension tags
    * [Pull request #628](https://github.com/gazebosim/sdformat/pull/628)

1. Updated material spec
    * [Pull request #644](https://github.com/gazebosim/sdformat/pull/644)

1. Minor fix to Migration guide
    * [Pull request #630](https://github.com/gazebosim/sdformat/pull/630)

1. Error: move << operator from .hh to .cc file
    * [Pull request #625](https://github.com/gazebosim/sdformat/pull/625)

1. Update build system to allow overriding CXX flags and using clang on Linux
    * [Pull request #621](https://github.com/gazebosim/sdformat/pull/621)

1. Add Element::FindElement as an alternative to Element::GetElement
    * [Pull request #620](https://github.com/gazebosim/sdformat/pull/620)

1. Add ValidateGraphs methods to Model/World (sdf9)
    * [Pull request #602](https://github.com/gazebosim/sdformat/pull/602)

1. Fix ABI break
    * [Pull request #605](https://github.com/gazebosim/sdformat/pull/605)

1. Parameter passing prototype
    * [Pull request #413](https://github.com/gazebosim/sdformat/pull/413)

1. Port particle scatter ratio param to sdf 1.6
    * [Pull request #595](https://github.com/gazebosim/sdformat/pull/595)

1. Making PrintValues() and ToString() able to not print default elements
    * [Pull request #575](https://github.com/gazebosim/sdformat/pull/575)

1. Add API for determining if an element was set by the user
    * [Pull request #542](https://github.com/gazebosim/sdformat/pull/542)

### libsdformat 10.5.0 (2021-05-17)

1. Add scatter ratio parameter to Particle Emitter DOM.
    + [Pull request 547](https://github.com/osrf/sdformat/pull/547)

1. Methods for removing attributes from an element.
    + [Pull request 555](https://github.com/osrf/sdformat/pull/555)

1. Improve docs of collision bitmask.
    + [Pull request 521](https://github.com/osrf/sdformat/pull/521)

### libsdformat 10.4.0 (2021-04-06)

1. Added particle emitter.
    + [Pull request 528](https://github.com/osrf/sdformat/pull/528)

1. Fixed application of <sensor><pose> tags in lumped linkes during URDF
   conversion.
    + [Pull request 525](https://github.com/osrf/sdformat/pull/525)

1. Add camera type aliases to docs.
    + [Pull request 514](https://github.com/osrf/sdformat/pull/514)

### libsdformat 10.3.0 (2021-02-18)

1. Replace PROJECT_SOURCE_DIR in tests
    + [Pull request 460](https://github.com/osrf/sdformat/pull/460)

1. Add Windows installation
    + [Pull request 463](https://github.com/osrf/sdformat/pull/463)

1. Add laser_retro in Visual
    + [Pull request 454](https://github.com/osrf/sdformat/pull/454)

1. Fix temperature gradient default value in Atmosphere DOM
    + [Pull request 482](https://github.com/osrf/sdformat/pull/482)

1. Wrap description tags in CDATA
    + [Pull request 483](https://github.com/osrf/sdformat/pull/483)

1. Add L16 pixel format to Camera pixel format conversion function
    + [Pull request 487](https://github.com/osrf/sdformat/pull/487)

### libsdformat 10.2.0 (2021-01-12)

1. Disable gz test on Windows
    + [Pull request 456](https://github.com/osrf/sdformat/pull/456)

1. Add Heightmap class
    + [Pull request 388](https://github.com/osrf/sdformat/pull/388)

1. Added `render_order` to material
    + [Pull request 446](https://github.com/osrf/sdformat/pull/446)

### libsdformat 10.1.0 (2020-12-15)

1. Fix supported shader types (`normal_map_X_space`)
    * [Pull request 383](https://github.com/osrf/sdformat/pull/383)

1. Prefix nested model names when flattening
    * [Pull request 399](https://github.com/osrf/sdformat/pull/399)

1. Move list of debian dependencies to packages.apt
    * [Pull request 392](https://github.com/osrf/sdformat/pull/392)

1. Remove custom element warning/error.
    * [Pull request 402](https://github.com/osrf/sdformat/pull/402)

1. Add Sky DOM.
    * [Pull request 397](https://github.com/osrf/sdformat/pull/397)

1. Add `<double_sided>` to material spec.
    * [Pull request 410](https://github.com/osrf/sdformat/pull/410)

1. Decrease far clip lower bound.
    * [Pull request 437](https://github.com/osrf/sdformat/pull/437)

1. Enable/disable tests for issue #202, add macOS workflow.
    * [Pull request 414](https://github.com/osrf/sdformat/pull/414)
    * [Pull request 438](https://github.com/osrf/sdformat/pull/438)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

1. Make labeler work with PRs from forks.
    * [Pull request 390](https://github.com/osrf/sdformat/pull/390)

1. Test included model folder missing model.config
    * [Pull request 422](https://github.com/osrf/sdformat/pull/422)

1. Add lightmap to 1.7 spec and PBR material DOM
    * [Pull request 429](https://github.com/osrf/sdformat/pull/429)

### libsdformat 10.0.0 (2020-09-28)

1. Return positive `INF` instead of `-1` in DOM API for unbounded symmetric joint limits.
    * [Pull request 357](https://github.com/osrf/sdformat/pull/357)

1. Add cmake option to disable console logfile.
    * [Pull request 348](https://github.com/osrf/sdformat/pull/348)

1. CMake fixes: include CMakePackageConfigHelpers and use modern cmake target for  gz math.
    * [Pull request 358](https://github.com/osrf/sdformat/pull/358)

1. Cmake: add tinyxml2 to Config names.
    * [Pull request 360](https://github.com/osrf/sdformat/pull/360)

1. Define `PATH_MAX` for Debian Hurd system.
    * [Pull request 369](https://github.com/osrf/sdformat/pull/369)

1. Normalize joint axis xyz vector when parsing from SDFormat.
    * [Pull request 312](https://github.com/osrf/sdformat/pull/312)

1. Migrate to using TinyXML2.
    * [Pull request 264](https://github.com/osrf/sdformat/pull/264)
    * [Pull request 321](https://github.com/osrf/sdformat/pull/321)
    * [Pull request 359](https://github.com/osrf/sdformat/pull/359)

1. Enforce minimum/maximum values specified in SDFormat description files.
    * [Pull request 303](https://github.com/osrf/sdformat/pull/303)

1. Make parsing of values syntactically more strict with bad values generating an error.
    * [Pull request 244](https://github.com/osrf/sdformat/pull/244)

1. Don't install deprecated parser\_urdf.hh header file, fix cmake warning about newline file, fix cmake warning about newlines.
    * [Pull request 276](https://github.com/osrf/sdformat/pull/276)

1. Remove deprecated Pose(), PoseFrame() functions from DOM objects.
    * [Pull request 308](https://github.com/osrf/sdformat/pull/308)

1. Remove deprecated UseParentModelFrame methods from JointAxis DOM.
    * [Pull request 379](https://github.com/osrf/sdformat/pull/379)

1. Changed the default radius of a Cylinder from 1.0 to 0.5 meters.
    * [BitBucket pull request 643](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/643)

## libsdformat 9.X

### libsdformat 9.10.1 (2024-01-05)

1. URDF->SDF handle links with no inertia or small mass
    * [Pull request #1238](https://github.com/gazebosim/sdformat/pull/1238)

1. Fix Element::Set method return value
    * [Pull request #1256](https://github.com/gazebosim/sdformat/pull/1256)

1. Allowing relative paths in URDF
    * [Pull request #1213](https://github.com/gazebosim/sdformat/pull/1213)

1. Use `File.exist?` for Ruby 3.2 compatibility
    * [Pull request #1216](https://github.com/gazebosim/sdformat/pull/1216)

1. Infrastructure
    * [Pull request #1217](https://github.com/gazebosim/sdformat/pull/1217)
    * [Pull request #1225](https://github.com/gazebosim/sdformat/pull/1225)
    * [Pull request #1271](https://github.com/gazebosim/sdformat/pull/1271)
    * [Pull request #1345](https://github.com/gazebosim/sdformat/pull/1345)
    * [Pull request #1252](https://github.com/gazebosim/sdformat/pull/1252)

### libsdformat 9.10.0 (2022-11-30)

1. Ign to gz header migration.
    * [Pull request #1118](https://github.com/gazebosim/sdformat/pull/1118)

1. Added HasLensProjection.
    * [Pull request #1203](https://github.com/gazebosim/sdformat/pull/1203)

1. Added camera info topic to Camera
    * [Pull request #1198](https://github.com/gazebosim/sdformat/pull/1198)
    * [Pull request #1201](https://github.com/gazebosim/sdformat/pull/1201)

### libsdformat 9.9.1 (2022-11-08)

1. Fix static URDF models with fixed joints
    * [Pull request #1193](https://github.com/gazebosim/sdformat/pull/1193)

1. Don't assume `CMAKE_INSTALL_*DIR` variables are relative
    * [Pull request #1190](https://github.com/gazebosim/sdformat/pull/1190)

### libsdformat 9.9.0 (2022-09-07)

1. sdf/camera.sdf: fields for projection matrix
    * [Pull request #1088](https://github.com/gazebosim/sdformat/pull/1088)

1. urdf: add //frame for reduced links/joints
    * [Pull request #1148](https://github.com/gazebosim/sdformat/pull/1148)

1. urdf: fix sensor/light pose for links lumped by fixed joints
    * [Pull request #1114](https://github.com/gazebosim/sdformat/pull/1114)

1. urdf: fix test and clean up internals
    * [Pull request #1126](https://github.com/gazebosim/sdformat/pull/1126)

1. Ensure relocatable config files
    * [Pull request #419](https://github.com/gazebosim/sdformat/pull/419)
    * [Pull request #1093](https://github.com/gazebosim/sdformat/pull/1093)

1. Test using `__model__`, `world` in @attached_to, @relative_to
    * [Pull request #1066](https://github.com/gazebosim/sdformat/pull/1066)

1. Readme: Ignition -> Gazebo
    * [Pull request #1080](https://github.com/gazebosim/sdformat/pull/1080)

1. Document major and minor SDFormat version numbers
    * [Pull request #1065](https://github.com/gazebosim/sdformat/pull/1065)

1. Bash completion for flags
    * [Pull request #1042](https://github.com/gazebosim/sdformat/pull/1042)

1. Add Link::ResolveInertial API
    * [Pull request #1012](https://github.com/gazebosim/sdformat/pull/1012)

### libsdformat 9.8.0 (2022-04-26)

1. Polyline geometry DOM
    * [Pull request #1000](https://github.com/gazebosim/sdformat/pull/1000)

1. Added `<shininess>` to `<material>`
    * [Pull request #985](https://github.com/gazebosim/sdformat/pull/985)

1. Backport ``gz sdf --inertial-stats``
    * [Pull request #958](https://github.com/gazebosim/sdformat/pull/958)

1. Add L16 pixel format to Camera pixel format conversion function
    * [Pull request #487](https://github.com/gazebosim/sdformat/pull/487)

1. Anti-aliasing element for `<camera><image>`
    * [Pull request #909](https://github.com/gazebosim/sdformat/pull/909)

1. Fix loading nested include with custom attributes
    * [Pull request #789](https://github.com/gazebosim/sdformat/pull/789)

1. add enable_orientation to 1.6 spec
    * [Pull request #686](https://github.com/gazebosim/sdformat/pull/686)

1. Fix xyz and rpy offsets in fixed joint reduction
    * [Pull request #500](https://github.com/gazebosim/sdformat/pull/500)

1. 👩‍🌾 Remove bitbucket-pipelines and backport labeler / triage
    * [Pull request #674](https://github.com/gazebosim/sdformat/pull/674)

1. Create CODEOWNERS with azeey and scpeters
    * [Pull request #650](https://github.com/gazebosim/sdformat/pull/650)

1. Use Ubuntu bionic in CI
    * [Pull request #626](https://github.com/gazebosim/sdformat/pull/626)

1. Translate poses of nested models inside other nested models (sdf6)
    * [Pull request #596](https://github.com/gazebosim/sdformat/pull/596)

1. Fix flattening logic for nested model names (sdf6)
    * [Pull request #597](https://github.com/gazebosim/sdformat/pull/597)

1. Parse rpyOffset as radians
    * [Pull request #497](https://github.com/gazebosim/sdformat/pull/497)

1. BitBucket
    * [Pull request #258](https://github.com/gazebosim/sdformat/pull/258)
    * [Pull request #237](https://github.com/gazebosim/sdformat/pull/237)

### libsdformat 9.7.0 (2021-11-03)

1. Make exception for plugins when checking for name uniqueness
    * [Pull request #733](https://github.com/gazebosim/sdformat/pull/733)

1. Backport test utilities from sdf10
    * [Pull request #731](https://github.com/gazebosim/sdformat/pull/731)

1. Added Force Torque Noise functions + Unit tests
    * [Pull request #669](https://github.com/gazebosim/sdformat/pull/669)

1. Add Joint DOM API to access joint sensors
    * [Pull request #517](https://github.com/gazebosim/sdformat/pull/517)

1. Add force torque sensor
    * [Pull request #393](https://github.com/gazebosim/sdformat/pull/393)

### libsdformat 9.6.1 (2021-09-07)

1. Parse URDF continuous joint effort/velocity limits
    * [Pull request #684](https://github.com/gazebosim/sdformat/pull/684)

1. Add a codecheck make target
    * [Pull request #682](https://github.com/gazebosim/sdformat/pull/682)

1. Refactor sdf::readXml
    * [Pull request #681](https://github.com/gazebosim/sdformat/pull/681)

1. Upgrade cpplint and fix new errors
    * [Pull request #680](https://github.com/gazebosim/sdformat/pull/680)

1. BUG: add missing plugin element to include
    * [Pull request #675](https://github.com/gazebosim/sdformat/pull/675)

1. Added comment reminder to update functions
    * [Pull request #677](https://github.com/gazebosim/sdformat/pull/677)

### libsdformat 9.6.0 (2021-08-18)

1. Adds `enable_metrics` flag to Sensor.
    * [Pull request #665](https://github.com/gazebosim/sdformat/pull/665)

1. Add GPS / NavSat sensor DOM to sdf9
    * [Pull request #453](https://github.com/gazebosim/sdformat/pull/453)

1. Support parsing elements that are not part of the schema
    * [Pull request #638](https://github.com/gazebosim/sdformat/pull/638)

1. Add lightmap to 1.7 spec and PBR material DOM
    * [Pull request #429](https://github.com/gazebosim/sdformat/pull/429)

1. Fix urdf link extension tags
    * [Pull request #628](https://github.com/gazebosim/sdformat/pull/628)

1. Updated material spec
    * [Pull request #644](https://github.com/gazebosim/sdformat/pull/644)

1. Minor fix to Migration guide
    * [Pull request #630](https://github.com/gazebosim/sdformat/pull/630)

1. Error: move << operator from .hh to .cc file
    * [Pull request #625](https://github.com/gazebosim/sdformat/pull/625)

1. Update build system to allow overriding CXX flags and using clang on Linux
    * [Pull request #621](https://github.com/gazebosim/sdformat/pull/621)

1. Add Element::FindElement as an alternative to Element::GetElement
    * [Pull request #620](https://github.com/gazebosim/sdformat/pull/620)

1. Add ValidateGraphs methods to Model/World (sdf9)
    * [Pull request #602](https://github.com/gazebosim/sdformat/pull/602)

1. Fix ABI break
    * [Pull request #605](https://github.com/gazebosim/sdformat/pull/605)

1. Making PrintValues() and ToString() able to not print default elements
    * [Pull request #575](https://github.com/gazebosim/sdformat/pull/575)

1. Add API for determining if an element was set by the user
    * [Pull request #542](https://github.com/gazebosim/sdformat/pull/542)

1. Methods for removing attributes from an element
    * [Pull request #555](https://github.com/gazebosim/sdformat/pull/555)

1. Improve docs of collision_bitmask.
    * [Pull request #521](https://github.com/gazebosim/sdformat/pull/521)

1. Add camera type aliases to docs.
    * [Pull request #514](https://github.com/gazebosim/sdformat/pull/514)

1. Add action-gz-ci
    * [Pull request #501](https://github.com/gazebosim/sdformat/pull/452)

### libsdformat 9.5.0 (2021-02-11)

1. Add Windows installation
    * [Pull request 463](https://github.com/osrf/sdformat/pull/463)

1. Add laser_retro in Visual
    * [Pull request 454](https://github.com/osrf/sdformat/pull/454)

1. Wrap description tags in CDATA
    * [Pull request 483](https://github.com/osrf/sdformat/pull/483)

### libsdformat 9.4.0 (2021-01-08)

1. Define `PATH_MAX` for Debian Hurd system
    * [Pull request 369](https://github.com/osrf/sdformat/pull/369)

1. Fix supported shader types (`normal_map_X_space`)
    * [Pull request 383](https://github.com/osrf/sdformat/pull/383)

1. Move list of debian dependencies to packages.apt
    * [Pull request 392](https://github.com/osrf/sdformat/pull/392)

1. Enable/disable tests for issue #202, add macOS workflow
    * [Pull request 414](https://github.com/osrf/sdformat/pull/414)
    * [Pull request 438](https://github.com/osrf/sdformat/pull/438)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

1. Test included model folder missing model.config
    * [Pull request 422](https://github.com/osrf/sdformat/pull/422)

1. Prefix nested model names when flattening
    * [Pull request 399](https://github.com/osrf/sdformat/pull/399)

1. Add Sky DOM
    * [Pull request 417](https://github.com/osrf/sdformat/pull/417)
    * [Pull request 397](https://github.com/osrf/sdformat/pull/397)

1. camera.sdf: decrease far clip lower bound
    * [Pull request 435](https://github.com/osrf/sdformat/pull/435)

1. material.sdf: add `double_sided` parameter
    * [Pull request 410](https://github.com/osrf/sdformat/pull/410)

1. Migration to GitHub: CI, links...
    * [Pull request 239](https://github.com/osrf/sdformat/pull/239)
    * [Pull request 310](https://github.com/osrf/sdformat/pull/310)
    * [Pull request 390](https://github.com/osrf/sdformat/pull/390)

1. Fix Actor copy operators and increase test coverage.
    * [Pull request 301](https://github.com/osrf/sdformat/pull/301)

1. Add Heightmap class
    * [Pull request 388](https://github.com/osrf/sdformat/pull/388)

### libsdformat 9.3.0 (2020-XX-XX)

1. Store material file path information.
    + [Pull request 349](https://github.com/osrf/sdformat/pull/349)

1. Support nested models in DOM and frame semantics.
    * [Pull request 316](https://github.com/osrf/sdformat/pull/316)
    + [Pull request 341](https://github.com/osrf/sdformat/pull/341)

1. Find python3 in cmake, fix cmake warning.
    * [Pull request 328](https://github.com/osrf/sdformat/pull/328)

1. Fix Actor copy operators and increase test coverage.
    * [Pull request 301](https://github.com/osrf/sdformat/pull/301)

1. GitHub Actions CI, pull request labels.
    * [Pull request 311](https://github.com/osrf/sdformat/pull/311)
    * [Pull request 363](https://github.com/osrf/sdformat/pull/363)

1. Change bitbucket links to GitHub.
    * [Pull request 240](https://github.com/osrf/sdformat/pull/240)

1. Param\_TEST: test parsing +Inf and -Inf.
    * [Pull request 277](https://github.com/osrf/sdformat/pull/277)

1. SearchForStuff: add logic to find urdfdom without pkg-config.
    * [Pull request 245](https://github.com/osrf/sdformat/pull/245)

1. Observe the CMake variable `BUILD_TESTING` if it is defined.
    * [Pull request 269](https://github.com/osrf/sdformat/pull/269)

1. Collision: don't load Surface without `<surface>`.
    * [Pull request 268](https://github.com/osrf/sdformat/pull/268)

1. Properly handle the requirement of C++17 at the CMake exported target level.
    * [Pull request 251](https://github.com/osrf/sdformat/pull/251)

1. Fix homebrew build with external urdfdom.
    * [BitBucket pull request 677](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/677)
    * [BitBucket pull request 686](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/686)

### libsdformat 9.2.0 (2020-04-02)

1. Remove URI scheme, if present, when finding files.
    * [BitBucket pull request 650](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/650)
    * [BitBucket pull request 652](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/652)

1. Build `Utils_TEST` with Utils.cc explicitly passed since its symbols are not visible.
    * [BitBucket pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

1. Keep the URDF style of specifying kinematics when converting URDF to SDF by using frame semantics.
    * [BitBucket pull request 676](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/676)

1. Increase output precision of URDF to SDF conversion, output -0 as 0.
    * [BitBucket pull request 675](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/675)

1. Add test of URDF frame semantics.
    * [BitBucket pull request 680](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/680)

1. Support frame semantics for models nested with <include>
    * [BitBucket pull request 668](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/668)

1. Add surface DOM
    * [BitBucket pull request 660](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/660)

1. Add Transparency to visual DOM
    * [BitBucket pull request 671](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/671)

1. Add camera visibility mask and visual visibility flags
    * [BitBucket pull request 673](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/673)

1. Include overrides for actor and light
    * [BitBucket pull request 669](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/669)

1. Add functionality to generate aggregated SDFormat descriptions via CMake.
    * [BitBucket pull request 667](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/667)
    * [BitBucket pull request 665](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/665)

1. parser addNestedModel: check `//axis/xyz/@expressed_in` before rotating joint axis.
    * [BitBucket pull request 657](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/657)
    * [Issue 219](https://github.com/osrf/sdformat/issues/219)

1. Remove TinyXML symbols from public API: Deprecate URDF2SDF
    * [BitBucket pull request 658](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/658)

1. Remove TinyXML symbols from public API: Move uninstalled headers
    * [BitBucket pull request 662](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/662)

1. Install the Windows `.dll` shared libraries to bin folder.
    * [BitBucket pull request 659](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/659)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Fix cmake type for `tinyxml_INCLUDE_DIRS`.
    * [BitBucket pull request 661](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/661)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Rename SDF to SDFormat / libsdformat on documentation
    * [BitBucket pull request 666](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/666)

### libsdformat 9.1.0 (2020-01-29)

1. Remove URI scheme, if present, when finding files.
    * [BitBucket pull request 653](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/653)

1. Fix parsing of pose elements under `<include>`
    * [BitBucket pull request 649](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/649)

1. Parser: add readFileWithoutConversion and readStringWithoutConversion.
    * [BitBucket pull request 647](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/647)

1. Added accessors to `gz::math::[Boxd, Cylinderd, Planed, Sphered]`
   in the matching `sdf::[Box, Cylinder, Plane, Sphere]` classes.
    * [BitBucket pull request 639](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/639)

1. Forward port of adjustments for memory leaks:
    * [BitBucket pull request 641](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/641) and
    * [BitBucket pull request 644](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/644)
    * [BitBucket pull request 645](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/645)

1. SDFormat 1.7: remove `//world/joint` element since it has never been used.
    * [BitBucket pull request 637](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/637)

1. Add clipping for depth camera on rgbd camera sensor
    * [BitBucket pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

1. Add tests to confirm that world is not allowed as child link of a joint.
    * [BitBucket pull request 634](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/634)

1. Fix link pose multiplication for URDF.
    * [BitBucket pull request 630](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/630)

1. Enable linter for URDF parser and fix style.
    * [BitBucket pull request 631](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/631)

1. Converter: fix memory leak pointed out by ASan.
    * [BitBucket pull request 638](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/638)

1. Access the original parsed version of an SDF document with `Element::OriginalVersion`.
    * [BitBucket pull request 640](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/640)

1. Model::Load: fail fast if an SDFormat 1.7 file has name collisions.
    * [BitBucket pull request 648](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/648)

1. Keep DOM objects even if they were loaded with errors.
    * [BitBucket pull request 655](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/655)

### libsdformat 9.0.0 (2019-12-10)

1. Move recursiveSameTypeUniqueNames from gz.cc to parser.cc and make public.
    * [BitBucket pull request 606](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/606)

1. Check that joints have valid parent and child names in `gz sdf --check`.
    * [BitBucket pull request 609](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/609)

1. Model DOM: error when trying to load nested models, which aren't yet supported.
    * [BitBucket pull request 610](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/610)

1. Use consistent namespaces in Filesystem.
    * [BitBucket pull request 567](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/567)

1. Enforce rules about reserved names and unique names among sibling elements.
    * [BitBucket pull request 600](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/600)
    * This also implements changes necessary for parsing custom elements and
    attributes per the following proposal:
    [Custom elements and attributes](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal)

1. Relax name checking, so name collisions generate warnings and names are automatically changed.
    * [BitBucket pull request 621](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/621)

1. Unversioned library name for gz tool commands.
    * [BitBucket pull request 612](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/612)

1. Initial version of SDFormat 1.7 specification.
    * [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. Converter: add `<map>` element for converting fixed values.
    * [BitBucket pull request 580](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/580)

1. Converter: add `descendant_name` attribute to recursively search for elements to convert.
    * [BitBucket pull request 596](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/596)

1. SDFormat 1.7: replace `use_parent_model_frame` element with `//axis/xyz/@expressed_in` attribute.
    * [BitBucket pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. SDFormat 1.7: replace `//pose/@frame` attribute with `//pose/@relative_to` attribute.
    * [BitBucket pull request 597](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/597)

1. SDFormat 1.7: add `//model/@canonical_link` attribute and require models to have at least one link.
    * [BitBucket pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)

1. Static models: allow them to have no links and skip building FrameAttachedToGraph.
    * [BitBucket pull request 626](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/626)

1. SDFormat 1.7: add `//frame/attached_to`, only allow frames in model and world, add Frame DOM.
    * [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. FrameSemantics API: add FrameAttachedToGraph and functions for building graph and resolving attached-to body.
    * [BitBucket pull request 613](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/613)

1. FrameSemantics API: add PoseRelativeToGraph and functions for building graph and resolving poses.
    * [BitBucket pull request 614](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/614)

1. Build and validate graphs during Model::Load and World::Load.
    * [BitBucket pull request 615](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/615)

1. Add SemanticPose class with implementation for Link.
    * [BitBucket pull request 616](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/616)

1. Add JointAxis::ResolveXyz and Joint::SemanticPose.
    * [BitBucket pull request 617](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/617)

1. Implement SemanticPose() for Collision, Frame, Light, Model, Sensor, Visual.
    * [BitBucket pull request 618](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/618)

1. Add Frame::ResolveAttachedToBody API for resolving the attached-to body of a frame.
    * [BitBucket pull request 619](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/619)

1. DOM API: deprecate `(Set)?PoseFrame` API and replace with `(Set)?PoseRelativeTo`
    * [BitBucket pull request 598](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/598)

1. DOM API: deprecate `(Set)?Pose` API and replace with `(Set)?RawPose`
    * [BitBucket pull request 599](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/599)

1. Hide FrameSemantics implementation.
    * [BitBucket pull request 622](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/622)
    * [BitBucket pull request 623](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/623)

## libsdformat 8.0

### libsdformat 8.X.X (202X-XX-XX)

### libsdformat 8.9.0 (2020-09-04)

1. Find python3 in cmake, fix warning
    * [Pull request 328](https://github.com/osrf/sdformat/pull/328)

1. Store material file path information
    * [Pull request 349](https://github.com/osrf/sdformat/pull/349)

1. Fix Actor copy operators and increase test coverage.
    * [Pull request 301](https://github.com/osrf/sdformat/pull/301)

1. Migration to GitHub: CI, links...
    * [Pull request 239](https://github.com/osrf/sdformat/pull/239)
    * [Pull request 310](https://github.com/osrf/sdformat/pull/310)

1. Increase output precision of URDF to SDF conversion, output -0 as 0.
    * [BitBucket pull request 675](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/675)

1. Fix homebrew build with external urdfdom.
    * [BitBucket pull request 677](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/677)
    * [BitBucket pull request 686](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/686)

### libsdformat 8.8.0 (2020-03-18)

1. Add Transparency to visual DOM
    * [BitBucket pull request 671](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/671)

1. Install the Windows `.dll` shared libraries to bin folder.
    * [BitBucket pull request 659](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/659)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Fix cmake type for `tinyxml_INCLUDE_DIRS`.
    * [BitBucket pull request 661](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/661)
    * [BitBucket pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Add functionality to generate aggregated SDFormat descriptions via CMake.
    * [BitBucket pull request 665](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/665)

1. Remove URI scheme, if present, when finding files.
    * [BitBucket pull request 650](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/650)
    * [BitBucket pull request 652](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/652)

1. Rename SDF to SDFormat / libsdformat on documentation
    * [BitBucket pull request 666](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/666)

### libsdformat 8.7.1 (2020-01-13)

1. Fix memory leaks in move assignment operator.
    * [BitBucket pull request 641](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/641)

1. Refactoring based on rule-of-five guidance to address memory leaks
    * [BitBucket pull request 644](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/644)

### libsdformat 8.7.0 (2019-12-13)

1. Remove some URDF error messages
    * [BitBucket pull request 605](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/605)

1. Fix parsing URDF without <material> inside <gazebo>
    * [BitBucket pull request 608](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/608)

1. Backport URDF multiplication and linter
    * [BitBucket pull request 632](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/632)

1. Add clipping for depth camera on rgbd camera sensor
    * [BitBucket pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

### libsdformat 8.6.1 (2019-12-05)

1. Unversioned lib name for cmds
    * [BitBucket pull request 612](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/612)

### libsdformat 8.6.0 (2019-11-20)

1. configure.bat: use gz-math6, not gz11
    * [BitBucket pull request 595](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/595)

1. Set `sdformat8_PKGCONFIG_*` variables in cmake config instead of `SDFormat_PKGCONFIG*`.
    * [BitBucket pull request 594](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/594)

1. Relax cmake check to allow compiling with gcc-7.
    * [BitBucket pull request 592](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/592)

1. Use custom callbacks when reading file (support Fuel URIs).
    * [BitBucket pull request 591](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/591)

1. Update visual DOM to parse `cast_shadows` property of a visual.
    * [BitBucket pull request 590](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/590)

1. Build `Utils_TEST` with Utils.cc explicitly passed since its symbols are not visible.
    * [BitBucket pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

### libsdformat 8.5.0 (2019-11-06)

1. Add `thermal_camera` sensor type
    * [BitBucket pull request 586](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/586)

1. Use inline namespaces in Utils.cc
    * [BitBucket pull request 574](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/574)

1. Convert `gz sdf` file inputs to absolute paths before processing them
    * [BitBucket pull request 583](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/583)

1. Add `emissive_map` to material sdf
    * [BitBucket pull request 585](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/585)

1. Converter: fix bug when converting across multiple versions.
    * [BitBucket pull request 584](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/584)
    * [BitBucket pull request 573](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/573)

### libsdformat 8.4.0 (2019-10-22)

1. Accept relative path in `<uri>`.
    * [BitBucket pull request 558](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/558)

1. Element: don't print unset attributes.
    * [BitBucket pull request 571](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/571)
    * [BitBucket pull request 576](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/576)

1. Lidar.hh: remove 'using namespace ignition'.
    * [BitBucket pull request 577](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/577)

1. Parse urdf files to sdf 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. SDF Root DOM: add ActorCount, ActorByIndex, and ActorNameExists.
    * [BitBucket pull request 566](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/566)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Move private headers from include/sdf to src folder.
    * [BitBucket pull request 553](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/553)

1. Fix gz library path on macOS.
    * [BitBucket pull request 542](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/542)
    * [BitBucket pull request 564](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/564)

1. Use `gz sdf --check` to check sibling elements of the same type for non-unique names.
    * [BitBucket pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [BitBucket pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### libsdformat 8.3.0 (2019-08-17)

1. Added Actor DOM
    * [BitBucket pull request 547](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/547)

1. Print cmake build warnings and errors to std_err
    * [BitBucket pull request 549](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/549)

### libsdformat 8.2.0 (2019-06-18)

1. Added RGBD Camera Sensor type.
    * [BitBucket pull request 540](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/540)

### libsdformat 8.1.0 (2019-05-20)

1.  Change installation path of SDF description files to allow side-by-side installation.
    * [BitBucket pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)

1. Added Lidar Sensor DOM. Also added `lidar` and `gpu_lidar` as sensor
   types. These two types are equivalent to `ray` and `gpu_ray`.
    * [BitBucket pull request 536](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/536)

1. SDF Sensor DOM: copy update rate in copy constructor.
    * [BitBucket pull request 534](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/534)

1. Added IMU Sensor DOM.
    * [BitBucket pull request 532](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/532)

1. Added Camera Sensor DOM.
    * [BitBucket pull request 531](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/531)

1. Added wind to link dom.
    * [BitBucket pull request 530](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/530)

1. Added Sensor DOM `==` operator.
    * [BitBucket pull request 529](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/529)

1. Added AirPressure SDF DOM
    * [BitBucket pull request 528](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/528)

1. Update SDFormat noise elements
    * [BitBucket pull request 525](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/525)
    * [BitBucket pull request 522](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/522)

1. Apply rule of five for various DOM classes
    * [BitBucket pull request 524](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/524)

1. Support setting sensor types from a string.
    * [BitBucket pull request 523](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/523)

1. Added Altimeter SDF DOM
    * [BitBucket pull request 527](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/527)

1. Added Magnetometer SDF DOM
    * [BitBucket pull request 518](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/518)
    * [BitBucket pull request 519](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/519)

1. Add Scene SDF DOM
    * [BitBucket pull request 517](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/517)

1. Add PBR material SDF element
    * [BitBucket pull request 512](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/512)
    * [BitBucket pull request 520](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/520)
    * [BitBucket pull request 535](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/535)

1. Set geometry shapes
    * [BitBucket pull request 515](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/515)

1. Clarify names of libsdformat parser and SDF specification in Readme.
    * [BitBucket pull request 514](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/514)

1. Disable macOS tests failing due to issue 202.
    * [BitBucket pull request 511](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/511)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

### libsdformat 8.0.0 (2019-03-01)

1. Rename depth camera from 'depth' to 'depth_camera'
    * [BitBucket pull request 507](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/507)

1. Rename enum Ray to Lidar
    * [BitBucket pull request 502](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/502)

1. Add support for files that have light tags in the root
    * [BitBucket pull request 499](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/499)

1. Fix locale problems of std::stringstream and of Param::ValueFromString
    * [BitBucket pull request 492](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/492)
    * Contribution by Silvio Traversaro

1. Add functions to set visual dom's geometry and material
    * [BitBucket pull request 490](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/490)

1. Change cmake project name to sdformat8, export cmake targets
    * [BitBucket pull request 475](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/475)
    * [BitBucket pull request 476](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/476)

1. SDF DOM: Add copy constructor and assignment operator to Light. Add lights to Link
    * [BitBucket pull request 469](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/469)

1. Make `<limit>` a required element for `<axis2>`
    * [BitBucket pull request #472](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/472)

1. SDF DOM: Add DOM methods for setting axis and thread pitch in `sdf::Joint`
    * [BitBucket pull request #471](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/471)
    * [BitBucket pull request #474](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/474)

1. SDF DOM: Add copy constructors and assignment operator to JointAxis
    * [BitBucket pull request #470](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/470)

1. Removed boost
    * [BitBucket pull request #438](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/438)

1. Versioned namespace
    * [BitBucket pull request 464](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/464)

1. Versioned library install
    * [BitBucket pull request 463](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/463)

1. Add SetGeom to Collision
    * [BitBucket pull request 465](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/465)

1. SDF DOM: Add copy/move constructors and assignment operator to Geometry
    * [BitBucket pull request 460](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/460)

1. SDF DOM: Add copy/move constructors and assignment operator to Material
    * [BitBucket pull request 461](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/461)

1. Add collision_detector to dart physics config
    * [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix cpack now that project name has version number
    * [BitBucket pull request 478](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/478)

1. Animation tension
    * [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)

1. Add "geometry" for sonar collision shape
    * [BitBucket pull request 479](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/479)

1. Fix Gui copy constructor
    * [BitBucket pull request 486](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/486)

1. Sensor DOM
    * [BitBucket pull request 488](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/488)
    * [BitBucket pull request 481](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/481)

## libsdformat 7.0

### libsdformat 7.0.0 (xxxx-xx-xx)

1. Build Utils_TEST with Utils.cc explicitly passed since its symbols are not visible.
    * [BitBucket pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Prevent duplicate `use_parent_model_frame` tags during file conversion.
    * [BitBucket pull request 573](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/573)

1. Backport inline versioned namespace from version 8.
    * [BitBucket pull request 557](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/557)
    * [BitBucket pull request 464](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/464)

1. Backport cmake and SDFormat spec changes from version 8.
    * [BitBucket pull request 550](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/550)
    * [BitBucket pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)
    * [BitBucket pull request 525](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/525)
    * [BitBucket pull request 475](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/475)
    * [BitBucket pull request 476](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/476)
    * [BitBucket pull request 463](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/463)

1. Fix gz library path on macOS.
    * [BitBucket pull request 542](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/542)

1. Preserve XML elements that are not part of the SDF specification.
    * [BitBucket pull request 449](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/449)

1. Embed SDF specification files directly in libsdformat.so.
    * [BitBucket pull request 434](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/434)

1. Removed support for SDF spec versions 1.0 and 1.2
    * [BitBucket pull request #432](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/432)

1. SDF DOM: Additions to the document object model.
    * [BitBucket pull request 433](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/433)
    * [BitBucket pull request 441](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/441)
    * [BitBucket pull request 442](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/442)
    * [BitBucket pull request 445](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/445)
    * [BitBucket pull request 451](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/451)
    * [BitBucket pull request 455](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/455)
    * [BitBucket pull request 481](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/481)

1. SDF DOM: Add Element() accessor to Gui, JointAxis and World classes.
    * [BitBucket pull request 450](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/450)

1. Adds the equalivent of gz sdf -d to libsdformat. The command line option
   will print the full description of the SDF spec.
    * [BitBucket pull request 424](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/424)

1. Adds the equalivent of gz sdf -p to libsdformat. The command line option
   will convert and print the specified SDFormat file.
    * [BitBucket pull request 494](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/494)

1. SDF DOM: Additions to the document object model.
    * [BitBucket pull request 393](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/393)
    * [BitBucket pull request 394](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/394)
    * [BitBucket pull request 395](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/395)
    * [BitBucket pull request 396](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/396)
    * [BitBucket pull request 397](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/397)
    * [BitBucket pull request 406](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/406)
    * [BitBucket pull request 407](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/407)
    * [BitBucket pull request 410](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/410)
    * [BitBucket pull request 415](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/415)
    * [BitBucket pull request 420](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/420)


## libsdformat 6.0

### libsdformat 6.3.1 (2021-07-06)

1. Fix flattening logic for nested model names
    * [Pull request 597](https://github.com/osrf/sdformat/pull/597)

1. Translate poses of nested models inside other nested models
    * [Pull request 596](https://github.com/osrf/sdformat/pull/596)

### libsdformat 6.3.0 (2021-06-21)

1. Move recursiveSameTypeUniqueNames from gz.cc to parser.cc and make public.
    * [Pull request 580](https://github.com/osrf/sdformat/pull/580)

1. Parse rpyOffset as radians
    * [Pull request 497](https://github.com/osrf/sdformat/pull/497)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [BitBucket pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [BitBucket pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [BitBucket pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Fix gz library path on macOS.
    * [BitBucket pull request 552](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/552)

1. Use `gz sdf --check` to check sibling elements of the same type for non-unique names.
    * [BitBucket pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [BitBucket pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### libsdformat 6.2.0 (2019-01-17)

1. Add geometry for sonar collision shape
    * [BitBucket pull request 495](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/495)

1. Add camera intrinsics (fx, fy, cx, cy, s)
    * [BitBucket pull request 496](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/496)

1. Add actor trajectory tension parameter
    * [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)


### libsdformat 6.1.0 (2018-10-04)

1. Add collision\_detector to dart physics config
    * [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix Windows support for libsdformat6
    * [BitBucket pull request 401](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/401)

1. root.sdf: default SDFormat version 1.6
    * [BitBucket pull request 425](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/425)

1. parser\_urdf: print value of highstop instead of pointer address
    * [BitBucket pull request 408](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/408)

1. Tweak error output so jenkins doesn't think it's a compiler warning
    * [BitBucket pull request 402](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/402)


### libsdformat 6.0.0 (2018-01-25)

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

1. Deprecated sdf::Color, and switch to use gz::math::Color
    * [BitBucket pull request 330](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/330)

## libsdformat 5.x

### libsdformat 5.x.x (2017-xx-xx)

### libsdformat 5.3.0 (2017-11-13)

1. Added wrapper around root SDF for an SDF element
    * [BitBucket pull request 378](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/378)
    * [BitBucket pull request 372](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/372)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [BitBucket pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/343)

1. Add preserveFixedJoint option to the URDF parser
    * [BitBucket pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/352)

1. Add light as child of link
    * [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

### libsdformat 5.2.0 (2017-08-03)

1. Added a block for DART-specific physics properties.
    * [BitBucket pull request 369](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/369)

1. Fix parser to read plugin child elements within an `<include>`
    * [BitBucket pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/350)

1. Choosing models with more recent SDFormat version with `<include>` tag
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

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [BitBucket pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/292)

### libsdformat 5.1.0 (2017-02-22)

1. Fixed `sdf::convertFile` and `sdf::convertString` always converting to latest version
    * [BitBucket pull request 320](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/320)
1. Added back the ability to set SDFormat version at runtime
    * [BitBucket pull request 307](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/307)

### libsdformat 5.0.0 (2017-01-25)

1. Removed libsdformat 4 deprecations
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

## libsdformat 4.0

### libsdformat 4.x.x (2017-xx-xx)

### libsdformat 4.4.0 (2017-10-26)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [BitBucket pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/343)

1. Add preserveFixedJoint option to the URDF parser
    * [BitBucket pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/352)

1. Add light as child of link
    * [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

### libsdformat 4.3.2 (2017-07-19)

1. Add documentation for `Element::GetFirstElement()` and `Element::GetNextElement()`
    * [BitBucket pull request 341](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/341)

1. Fix parser to read plugin child elements within an `<include>`
    * [BitBucket pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/350)

### libsdformat 4.3.1 (2017-03-24)

1. Fix segmentation Fault in `sdf::getBestSupportedModelVersion`
    * [BitBucket pull request 327](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/327)
    * [Issue 152](https://github.com/osrf/sdformat/issues/152)

### libsdformat 4.3.0 (2017-03-20)

1. Choosing models with more recent SDFormat version with `<include>` tag
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

### libsdformat 4.2.0 (2016-10-10)

1. Added tag to specify ODE friction model.
    * [BitBucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)

1. Fix URDF to SDF `self_collide` bug.
    * [BitBucket pull request 287](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/287)

1. Added IMU orientation specification to SDF.
    * [BitBucket pull request 284](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/284)

### libsdformat 4.1.1 (2016-07-08)

1. Added documentation and animation to `<actor>` element.
    * [BitBucket pull request 280](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/280)

1. Added tag to specify initial joint position
    * [BitBucket pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/279)

### libsdformat 4.1.0 (2016-04-01)

1. Added SDF conversion functions to parser including sdf::convertFile and sdf::convertString.
    * [BitBucket pull request 266](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/266)

1. Added an upload script
    * [BitBucket pull request 256](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/256)

### libsdformat 4.0.0 (2015-01-12)

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

## libsdformat 3.0

### libsdformat 3.X.X (201X-XX-XX)

1. Improve precision of floating point parameters
    * [BitBucket pull request 273](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/273)
    * [BitBucket pull request 276](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/276)

### libsdformat 3.7.0 (2015-11-20)

1. Add spring pass through for sdf3
    * [Design document](https://osrf-migration.github.io/osrf-others-gh-pages/#!/osrf/gazebo_design/pull-requests/23)
    * [BitBucket pull request 242](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/242)

1. Support frame specification in SDF
    * [BitBucket pull request 237](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/237)

1. Remove boost from SDFExtension
    * [BitBucket pull request 229](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/229)

### libsdformat 3.6.0 (2015-10-27)

1. Add light state
    * [BitBucket pull request 227](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/227)
1. redo pull request #222 for sdf3 branch
    * [BitBucket pull request 232](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/232)
1. Fix links in API documentation
    * [BitBucket pull request 231](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/231)

### libsdformat 3.5.0 (2015-10-07)

1. Camera lens description (Replaces #213)
    * [BitBucket pull request 215](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/215)
1. Fix shared pointer reference loop in Element and memory leak (#104)
    * [BitBucket pull request 230](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/230)

### libsdformat 3.4.0 (2015-10-05)

1. Support nested model states
    * [BitBucket pull request 223](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/223)
1. Cleaner way to set SDF_PATH for tests
    * [BitBucket pull request 226](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/226)

### libsdformat 3.3.0 (2015-09-15)

1. Windows Boost linking errors
    * [BitBucket pull request 206](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/206)
1. Nested SDF -> sdf3
    * [BitBucket pull request 221](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/221)
1. Pointer types
    * [BitBucket pull request 218](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/218)
1. Torsional friction default surface radius not infinity
    * [BitBucket pull request 217](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/217)

### libsdformat 3.2.2 (2015-08-24)

1. Added battery element (contribution from Olivier Crave)
    * [BitBucket pull request #204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/204)
1. Torsional friction backport
    * [BitBucket pull request #211](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/211)
1. Allow Visual Studio 2015
    * [BitBucket pull request #208](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/208)

### libsdformat 3.1.1 (2015-08-03)

1. Fix tinyxml linking error
    * [BitBucket pull request #209](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/209)

### libsdformat 3.1.0 (2015-08-02)

1. Added logical camera sensor to SDF
    * [BitBucket pull request #207](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/207)

### libsdformat 3.0.0 (2015-07-24)

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
1. Convert to use gz-math
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

## libsdformat 2.x

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
1. Do not export urdf symbols in SDFormat 1.4
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
1. Port libsdformat to compile on Windows (MSVC)
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
1. Add flag for MOI rescaling to SDFormat 1.4
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

## libsdformat 1.4

### libsdformat 1.4.8 (2013-09-06)

1. Fix inertia transformations when reducing fixed joints in URDF
    * [BitBucket pull request 48](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/48/fix-for-issue-22-reducing-inertia-across/diff)
1. Add <use_terrain_paging> element to support terrain paging in gazebo
    * [BitBucket pull request 47](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/47/add-element-inside-heightmap/diff)
1. Further reduce console output when using URDF models
    * [BitBucket pull request 46](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/46/convert-a-few-more-sdfwarns-to-sdflog-fix/diff)
    * [Commit](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/commits/b15d5a1ecc57abee6691618d02d59bbc3d1b84dc)

### libsdformat 1.4.7 (2013-08-22)

1. Direct console messages to std_err
    * [BitBucket pull request 44](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/44/fix-19-direct-all-messages-to-std_err)

### libsdformat 1.4.6 (2013-08-20)

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

### libsdformat 1.4.5 (2013-07-23)

1. Deprecated Gazebo's internal SDF code
1. Use templatized Get functions for retrieving values from SDF files
1. Removed dependency on ROS
