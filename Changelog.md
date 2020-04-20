## libsdformat 10.X

### libsdformat 10.X.X (202X-XX-XX)

### libsdformat 10.0.0 (202X-XX-XX)

1. Changed the default radius of a Cylinder from 1.0 to 0.5 meters.
    * [Pull request 643](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/643)

1. Initial version of SDFormat 1.8 specification.
    * [Pull request 682](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/682)

1. SDFormat 1.8: Deprecate //joint/axis/initial_position.
    * [Pull request 683](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/683)

## libsdformat 9.X

### libsdformat 9.X.X (202X-XX-XX)

1. Fix homebrew build with external urdfdom.
    * [Pull request 677](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/677)
    * [Pull request 686](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/686)

### libsdformat 9.2.0 (2020-04-02)

1. Remove URI scheme, if present, when finding files.
    * [Pull request 650](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/650)
    * [Pull request 652](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/652)

1. Build `Utils_TEST` with Utils.cc explicitly passed since its symbols are not visible.
    * [Pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

1. Keep the URDF style of specifying kinematics when converting URDF to SDF by using frame semantics.
    * [Pull request 676](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/676)

1. Increase output precision of URDF to SDF conversion, output -0 as 0.
    * [Pull request 675](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/675)

1. Add test of URDF frame semantics.
    * [Pull request 680](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/680)

1. Support frame semantics for models nested with <include>
    * [Pull request 668](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/668)

1. Add surface DOM
    * [pull request 660](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/660)

1. Add Transparency to visual DOM
    * [Pull request 671](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/671)

1. Add camera visibility mask and visual visibility flags
    * [Pull request 673](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/673)

1. Include overrides for actor and light
    * [Pull request 669](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/669)

1. Add functionality to generate aggregated SDFormat descriptions via CMake.
    * [Pull request 667](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/667)
    * [Pull request 665](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/665)

1. parser addNestedModel: check `//axis/xyz/@expressed_in` before rotating joint axis.
    * [Pull request 657](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/657)
    * [Issue 219](https://github.com/osrf/sdformat/issues/219)

1. Remove TinyXML symbols from public API: Deprecate URDF2SDF
    * [Pull request 658](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/658)

1. Remove TinyXML symbols from public API: Move uninstalled headers
    * [Pull request 662](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/662)

1. Install the Windows `.dll` shared libraries to bin folder.
    * [Pull request 659](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/659)
    * [Pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Fix cmake type for `tinyxml_INCLUDE_DIRS`.
    * [Pull request 661](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/661)
    * [Pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Rename SDF to SDFormat / libsdformat on documentation
    * [Pull request 666](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/666)

### libsdformat 9.1.0 (2020-01-29)

1. Remove URI scheme, if present, when finding files.
    * [Pull request 653](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/653)

1. Fix parsing of pose elements under `<include>`
    * [Pull request 649](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/649)

1. Parser: add readFileWithoutConversion and readStringWithoutConversion.
    * [Pull request 647](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/647)

1. Added accessors to `ignition::math::[Boxd, Cylinderd, Planed, Sphered]`
   in the matching `sdf::[Box, Cylinder, Plane, Sphere]` classes.
    * [Pull request 639](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/639)

1. Forward port of adjustments for memory leaks:
   [Pull Request 641](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/641) and
   [Pull Request 644](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/644)
    * [Pull Request 645](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/645)

1. SDFormat 1.7: remove `//world/joint` element since it has never been used.
    * [Pull request 637](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/637)

1. Add clipping for depth camera on rgbd camera sensor
    * [Pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

1. Add tests to confirm that world is not allowed as child link of a joint.
    * [Pull request 634](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/634)

1. Fix link pose multiplication for URDF.
    * [Pull request 630](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/630)

1. Enable linter for URDF parser and fix style.
    * [Pull request 631](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/631)

1. Converter: fix memory leak pointed out by ASan.
    * [Pull request 638](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/638)

1. Access the original parsed version of an SDF document with `Element::OriginalVersion`.
    * [Pull request 640](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/640)

1. Model::Load: fail fast if an SDFormat 1.7 file has name collisions.
    * [Pull request 648](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/648)

1. Keep DOM objects even if they were loaded with errors.
    * [Pull request 655](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/655)

### libsdformat 9.0.0 (2019-12-10)

1. Move recursiveSameTypeUniqueNames from ign.cc to parser.cc and make public.
    * [Pull request 606](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/606)

1. Check that joints have valid parent and child names in `ign sdf --check`.
    * [Pull request 609](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/609)

1. Model DOM: error when trying to load nested models, which aren't yet supported.
    * [Pull request 610](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/610)

1. Use consistent namespaces in Filesystem.
    * [Pull request 567](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/567)

1. Enforce rules about reserved names and unique names among sibling elements.
    * [Pull request 600](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/600)

1. Relax name checking, so name collisions generate warnings and names are automatically changed.
    * [Pull request 621](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/621)

1. Unversioned library name for ign tool commands.
    * [Pull request 612](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/612)

1. Initial version of SDFormat 1.7 specification.
    * [Pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. Converter: add `<map>` element for converting fixed values.
    * [Pull request 580](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/580)

1. Converter: add `descendant_name` attribute to recursively search for elements to convert.
    * [Pull request 596](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/596)

1. SDFormat 1.7: replace `use_parent_model_frame` element with `//axis/xyz/@expressed_in` attribute.
    * [Pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. SDFormat 1.7: replace `//pose/@frame` attribute with `//pose/@relative_to` attribute.
    * [Pull request 597](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/597)

1. SDFormat 1.7: add `//model/@canonical_link` attribute and require models to have at least one link.
    * [Pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)

1. Static models: allow them to have no links and skip building FrameAttachedToGraph.
    * [Pull request 626](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/626)

1. SDFormat 1.7: add `//frame/attached_to`, only allow frames in model and world, add Frame DOM.
    * [pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. FrameSemantics API: add FrameAttachedToGraph and functions for building graph and resolving attached-to body.
    * [Pull request 613](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/613)

1. FrameSemantics API: add PoseRelativeToGraph and functions for building graph and resolving poses.
    * [Pull request 614](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/614)

1. Build and validate graphs during Model::Load and World::Load.
    * [Pull request 615](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/615)

1. Add SemanticPose class with implementation for Link.
    * [Pull request 616](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/616)

1. Add JointAxis::ResolveXyz and Joint::SemanticPose.
    * [Pull request 617](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/617)

1. Implement SemanticPose() for Collision, Frame, Light, Model, Sensor, Visual.
    * [Pull request 618](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/618)

1. Add Frame::ResolveAttachedToBody API for resolving the attached-to body of a frame.
    * [Pull request 619](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/619)

1. DOM API: deprecate `(Set)?PoseFrame` API and replace with `(Set)?PoseRelativeTo`
    * [Pull request 598](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/598)

1. DOM API: deprecate `(Set)?Pose` API and replace with `(Set)?RawPose`
    * [Pull request 599](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/599)

1. Hide FrameSemantics implementation.
    * [Pull request 622](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/622)
    * [Pull request 623](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/623)

## libsdformat 8.0

### libsdformat 8.X.X (202X-XX-XX)

1. Increase output precision of URDF to SDF conversion, output -0 as 0.
    * [Pull request 675](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/675)

1. Fix homebrew build with external urdfdom.
    * [Pull request 677](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/677)
    * [Pull request 686](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/686)

### libsdformat 8.8.0 (2020-03-18)

1. Add Transparency to visual DOM
    * [Pull request 671](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/671)

1. Install the Windows `.dll` shared libraries to bin folder.
    * [Pull request 659](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/659)
    * [Pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Fix cmake type for `tinyxml_INCLUDE_DIRS`.
    * [Pull request 661](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/661)
    * [Pull request 663](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/663)

1. Add functionality to generate aggregated SDFormat descriptions via CMake.
    * [Pull request 665](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/665)

1. Remove URI scheme, if present, when finding files.
    * [Pull request 650](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/650)
    * [Pull request 652](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/652)

1. Rename SDF to SDFormat / libsdformat on documentation
    * [Pull request 666](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/666)

### libsdformat 8.7.1 (2020-01-13)

1. Fix memory leaks in move assignment operator.
    * [Pull request 641](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/641)

1. Refactoring based on rule-of-five guidance to address memory leaks
   * [Pull request 644](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/644)

### libsdformat 8.7.0 (2019-12-13)

1. Remove some URDF error messages
    * [Pull request 605](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/605)

1. Fix parsing URDF without <material> inside <gazebo>
    * [Pull request 608](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/608)

1. Backport URDF multiplication and linter
    * [Pull request 632](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/632)

1. Add clipping for depth camera on rgbd camera sensor
    * [Pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

### libsdformat 8.6.1 (2019-12-05)

1. Unversioned lib name for cmds
    * [Pull request 612](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/612)

### libsdformat 8.6.0 (2019-11-20)

1. configure.bat: use ign-math6, not gz11
    * [Pull request 595](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/595)

1. Set `sdformat8_PKGCONFIG_*` variables in cmake config instead of `SDFormat_PKGCONFIG*`.
    * [Pull request 594](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/594)

1. Relax cmake check to allow compiling with gcc-7.
    * [Pull request 592](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/592)

1. Use custom callbacks when reading file (support Fuel URIs).
    * [Pull request 591](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/591)

1. Update visual DOM to parse `cast_shadows` property of a visual.
    * [Pull request 590](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/590)

1. Build `Utils_TEST` with Utils.cc explicitly passed since its symbols are not visible.
    * [Pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

### libsdformat 8.5.0 (2019-11-06)

1. Add `thermal_camera` sensor type
    * [Pull request 586](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/586)

1. Use inline namespaces in Utils.cc
    * [Pull request 574](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/574)

1. Convert `ign sdf` file inputs to absolute paths before processing them
    * [Pull request 583](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/583)

1. Add `emissive_map` to material sdf
    * [Pull request 585](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/585)

1. Converter: fix bug when converting across multiple versions.
    * [Pull request 584](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/584)
    * [Pull request 573](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/573)

### libsdformat 8.4.0 (2019-10-22)

1. Accept relative path in `<uri>`.
    * [Pull request 558](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/558)

1. Element: don't print unset attributes.
    * [Pull request 571](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/571)
    * [Pull request 576](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/576)

1. Lidar.hh: remove 'using namespace ignition'.
    * [Pull request 577](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/577)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [Pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [Pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. SDF Root DOM: add ActorCount, ActorByIndex, and ActorNameExists.
    * [Pull request 566](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/566)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [Pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Move private headers from include/sdf to src folder.
    * [Pull request 553](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/553)

1. Fix ign library path on macOS.
    * [Pull request 542](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/542)
    * [Pull request 564](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/564)

1. Use `ign sdf --check` to check sibling elements of the same type for non-unique names.
    * [Pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [Pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### libsdformat 8.3.0 (2019-08-17)

1. Added Actor DOM
    * [Pull request 547](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/547)

1. Print cmake build warnings and errors to std_err
    * [Pull request 549](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/549)

### libsdformat 8.2.0 (2019-06-18)

1. Added RGBD Camera Sensor type.
    * [Pull request 540](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/540)

### libsdformat 8.1.0 (2019-05-20)

1.  Change installation path of SDF description files to allow side-by-side installation.
    * [pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)

1. Added Lidar Sensor DOM. Also added `lidar` and `gpu_lidar` as sensor
   types. These two types are equivalent to `ray` and `gpu_ray`.
    * [Pull request 536](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/536)

1. SDF Sensor DOM: copy update rate in copy constructor.
    * [Pull request 534](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/534)

1. Added IMU Sensor DOM.
    * [Pull request 532](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/532)

1. Added Camera Sensor DOM.
    * [Pull request 531](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/531)

1. Added wind to link dom.
    * [Pull request 530](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/530)

1. Added Sensor DOM `==` operator.
    * [Pull request 529](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/529)

1. Added AirPressure SDF DOM
    * [Pull request 528](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/528)

1. Update SDFormat noise elements
    * [Pull request 525](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/525)
    * [Pull request 522](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/522)

1. Apply rule of five for various DOM classes
    * [Pull request 524](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/524)

1. Support setting sensor types from a string.
    * [Pull request 523](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/523)

1. Added Altimeter SDF DOM
    * [Pull request 527](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/527)

1. Added Magnetometer SDF DOM
    * [Pull request 518](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/518)
    * [Pull request 519](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/519)

1. Add Scene SDF DOM
    * [Pull request 517](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/517)

1. Add PBR material SDF element
    * [Pull request 512](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/512)
    * [Pull request 520](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/520)
    * [Pull request 535](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/535)

1. Set geometry shapes
    * [Pull request 515](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/515)

1. Clarify names of libsdformat parser and SDF specification in Readme.
    * [Pull request 514](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/514)

1. Disable macOS tests failing due to issue 202.
    * [Pull request 511](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/511)
    * [Issue 202](https://github.com/osrf/sdformat/issues/202)

### libsdformat 8.0.0 (2019-03-01)

1. Rename depth camera from 'depth' to 'depth_camera'
    * [Pull request 507](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/507)

1. Rename enum Ray to Lidar
    * [Pull request 502](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/502)

1. Add support for files that have light tags in the root
    * [Pull request 499](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/499)

1. Fix locale problems of std::stringstream and of Param::ValueFromString
    * [Pull request 492](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/492)
    * Contribution by Silvio Traversaro

1. Add functions to set visual dom's geometry and material
    * [Pull request 490](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/490)

1. Change cmake project name to sdformat8, export cmake targets
    * [Pull request 475](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/475)
    * [Pull request 476](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/476)

1. SDF DOM: Add copy constructor and assignment operator to Light. Add lights to Link
    * [Pull request 469](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/469)

1. Make `<limit>` a required element for `<axis2>`
    * [Pull request #472](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/472)

1. SDF DOM: Add DOM methods for setting axis and thread pitch in `sdf::Joint`
    * [Pull request #471](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/471)
    * [Pull request #474](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/474)

1. SDF DOM: Add copy constructors and assignment operator to JointAxis
    * [Pull request #470](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/470)

1. Removed boost
    * [Pull request #438](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/438)

1. Versioned namespace
    * [Pull request 464](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/464)

1. Versioned library install
    * [Pull request 463](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/463)

1. Add SetGeom to Collision
    * [Pull request 465](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/465)

1. SDF DOM: Add copy/move constructors and assignment operator to Geometry
    * [Pull request 460](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/460)

1. SDF DOM: Add copy/move constructors and assignment operator to Material
    * [Pull request 461](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/461)

1. Add collision_detector to dart physics config
    * [Pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix cpack now that project name has version number
    * [Pull request 478](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/478)

1. Animation tension
    * [Pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)

1. Add "geometry" for sonar collision shape
    * [Pull request 479](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/479)

1. Fix Gui copy constructor
    * [Pull request 486](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/486)

1. Sensor DOM
    * [Pull request 488](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/488)
    * [Pull request 481](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/481)

## libsdformat 7.0

### libsdformat 7.0.0 (xxxx-xx-xx)

1. Build Utils_TEST with Utils.cc explicitly passed since its symbols are not visible.
    * [Pull request 572](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/572)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [Pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [Pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [Pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Prevent duplicate `use_parent_model_frame` tags during file conversion.
    * [Pull request 573](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/573)

1. Backport inline versioned namespace from version 8.
    * [Pull request 557](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/557)
    * [pull request 464](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/464)

1. Backport cmake and SDFormat spec changes from version 8.
    * [Pull request 550](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/550)
    * [pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)
    * [Pull request 525](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/525)
    * [Pull request 475](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/475)
    * [Pull request 476](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/476)
    * [Pull request 463](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/463)

1. Fix ign library path on macOS.
    * [Pull request 542](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/542)

1. Preserve XML elements that are not part of the SDF specification.
    * [Pull request 449](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/449)

1. Embed SDF specification files directly in libsdformat.so.
    * [Pull request 434](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/434)

1. Removed support for SDF spec versions 1.0 and 1.2
    * [Pull request #432](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/432)

1. SDF DOM: Additions to the document object model.
    * [Pull request 433](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/433)
    * [Pull request 441](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/441)
    * [Pull request 442](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/442)
    * [Pull request 445](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/445)
    * [Pull request 451](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/451)
    * [Pull request 455](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/455)
    * [Pull request 481](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/481)

1. SDF DOM: Add Element() accessor to Gui, JointAxis and World classes.
    * [Pull request 450](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/450)

1. Adds the equalivent of gz sdf -d to libsdformat. The command line option
   will print the full description of the SDF spec.
    * [Pull request 424](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/424)

1. Adds the equalivent of gz sdf -p to libsdformat. The command line option
   will convert and print the specified SDFormat file.
    * [Pull request 494](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/494)

1. SDF DOM: Additions to the document object model.
    * [Pull request 393](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/393)
    * [Pull request 394](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/394)
    * [Pull request 395](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/395)
    * [Pull request 396](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/396)
    * [Pull request 397](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/397)
    * [Pull request 406](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/406)
    * [Pull request 407](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/407)
    * [Pull request 410](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/410)
    * [Pull request 415](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/415)
    * [Pull request 420](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/420)


## libsdformat 6.0

### libsdformat 6.X.X (20XX-XX-XX)

1. Parse urdf files to SDFormat 1.5 instead of 1.4 to avoid `use_parent_model_frame`.
    * [Pull request 575](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/575)

1. Set camera intrinsics axis skew (s) default value to 0
    * [Pull request 504](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/504)

1. Avoid hardcoding /machine:x64 flag on 64-bit on MSVC with CMake >= 3.5.
    * [Pull request 565](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/565)

1. Fix ign library path on macOS.
    * [Pull request 552](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/552)

1. Use `ign sdf --check` to check sibling elements of the same type for non-unique names.
    * [Pull request 554](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/554)

1. Converter: remove all matching elements specified by `<remove>` tag.
    * [Pull request 551](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/551)

### libsdformat 6.2.0 (2019-01-17)

1. Add geometry for sonar collision shape
    * [Pull request 495](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/495)

1. Add camera intrinsics (fx, fy, cx, cy, s)
    * [Pull request 496](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/496)

1. Add actor trajectory tension parameter
    * [Pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)


### libsdformat 6.1.0 (2018-10-04)

1. Add collision\_detector to dart physics config
    * [Pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. Fix Windows support for libsdformat6
    * [Pull request 401](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/401)

1. root.sdf: default SDFormat version 1.6
    * [Pull request 425](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/425)

1. parser\_urdf: print value of highstop instead of pointer address
    * [Pull request 408](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/408)

1. Tweak error output so jenkins doesn't think it's a compiler warning
    * [Pull request 402](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/402)


### libsdformat 6.0.0 (2018-01-25)

1. SDF DOM: Added a document object model.
    * [Pull request 387](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/387)
    * [Pull request 389](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/389)

1. Add simplified ``readFile`` function.
    * [Pull request 347](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/347)

1. Remove boost::lexical cast instances
    * [Pull request 342](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/342)

1. Remove boost regex and iostreams as dependencies
    * [Pull request 302](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/302)

1. Change certain error checks from asserts to throwing
   sdf::AssertionInternalError, which is more appropriate for a library.
    * [Pull request 315](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/315)

1. Updated the internal copy of urdfdom to 1.0, removing more of boost.
    * [Pull request 324](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/324)

1. urdfdom 1.0 is now required on all platforms.
    * [Pull request 324](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/324)

1. Remove boost filesystem as a dependency
    * [Pull request 335](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/335)
    * [Pull request 338](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/338)
    * [Pull request 339](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/339)

1. Deprecated sdf::Color, and switch to use ignition::math::Color
    * [Pull request 330](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/330)

## libsdformat 5.x

### libsdformat 5.x.x (2017-xx-xx)

### libsdformat 5.3.0 (2017-11-13)

1. Added wrapper around root SDF for an SDF element
    * [Pull request 378](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/378)
    * [Pull request 372](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/372)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [Pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [Pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/343)

1. Add preserveFixedJoint option to the URDF parser
    * [Pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/352)

1. Add light as child of link
    * [Pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/373)

### libsdformat 5.2.0 (2017-08-03)

1. Added a block for DART-specific physics properties.
    * [Pull request 369](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/369)

1. Fix parser to read plugin child elements within an `<include>`
    * [Pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/350)

1. Choosing models with more recent SDFormat version with `<include>` tag
    * [Pull request 291](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/291)
    * [Issue 123](https://github.com/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [Pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/318)

1. Support light insertion in state
    * [Pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/325)

1. Case insensitive boolean strings
    * [Pull request 322](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/322)

1. Enable coverage testing
    * [Pull request 317](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/317)

1. Add `friction_model` parameter to ode solver
    * [Pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/294)
    * [Gazebo pull request 1522](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/gazebo/pull-request/1522)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [Pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/292)

### libsdformat 5.1.0 (2017-02-22)

1. Fixed `sdf::convertFile` and `sdf::convertString` always converting to latest version
    * [Pull request 320](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/320)
1. Added back the ability to set SDFormat version at runtime
    * [Pull request 307](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/307)

### libsdformat 5.0.0 (2017-01-25)

1. Removed libsdformat 4 deprecations
    * [Pull request 295](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/295)

1. Added an example
    * [Pull request 275](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/275)

1. Move functions that use TinyXML classes in private headers
   A contribution from Silvio Traversaro
    * [Pull request 262](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/262)

1. Fix issues found by the Coverity tool
   A contribution from Olivier Crave
    * [Pull request 259](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/259)

1. Add tag to allow for specification of initial joint position
    * [Pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/279)

1. Require ignition-math3 as dependency
    * [Pull request 299](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/299)

1. Simplifier way of retrieving a value from SDF using Get
    * [Pull request 285](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/285)

## libsdformat 4.0

### libsdformat 4.x.x (2017-xx-xx)

### libsdformat 4.4.0 (2017-10-26)

1. Add ODE parallelization parameters: threaded islands and position correction
    * [Pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/380)

1. surface.sdf: expand documentation of friction and slip coefficients
    * [Pull request 343](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/343)

1. Add preserveFixedJoint option to the URDF parser
    * [Pull request 352](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/352)

1. Add light as child of link
    * [Pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/373)

### libsdformat 4.3.2 (2017-07-19)

1. Add documentation for `Element::GetFirstElement()` and `Element::GetNextElement()`
    * [Pull request 341](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/341)

1. Fix parser to read plugin child elements within an `<include>`
    * [Pull request 350](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/350)

### libsdformat 4.3.1 (2017-03-24)

1. Fix segmentation Fault in `sdf::getBestSupportedModelVersion`
    * [Pull request 327](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/327)
    * [Issue 152](https://github.com/osrf/sdformat/issues/152)

### libsdformat 4.3.0 (2017-03-20)

1. Choosing models with more recent SDFormat version with `<include>` tag
    * [Pull request 291](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/291)
    * [Issue 123](https://github.com/osrf/sdformat/issues/123)

1. Added `<category_bitmask>` to 1.6 surface contact parameters
    * [Pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/318)

1. Support light insertion in state
    * [Pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/325)

1. Case insensitive boolean strings
    * [Pull request 322](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/322)

1. Enable coverage testing
    * [Pull request 317](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/317)

1. Add `friction_model` parameter to ode solver
    * [Pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/294)
    * [Gazebo pull request 1522](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/gazebo/pull-request/1522)

1. Added `sampling` parameter to `<heightmap>` SDF element.
    * [Pull request 293](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/293)

1. Added Migration guide
    * [Pull request 290](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/290)

1. Add cmake `@PKG_NAME@_LIBRARY_DIRS` variable to cmake config file
    * [Pull request 292](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/292)

### libsdformat 4.2.0 (2016-10-10)

1. Added tag to specify ODE friction model.
    * [Pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/294)

1. Fix URDF to SDF `self_collide` bug.
    * [Pull request 287](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/287)

1. Added IMU orientation specification to SDF.
    * [Pull request 284](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/284)

### libsdformat 4.1.1 (2016-07-08)

1. Added documentation and animation to `<actor>` element.
    * [Pull request 280](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/280)

1. Added tag to specify initial joint position
    * [Pull request 279](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/279)

### libsdformat 4.1.0 (2016-04-01)

1. Added SDF conversion functions to parser including sdf::convertFile and sdf::convertString.
    * [Pull request 266](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/266)

1. Added an upload script
    * [Pull request 256](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/256)

### libsdformat 4.0.0 (2015-01-12)

1. Boost pointers and boost::function in the public API have been replaced
   by their std::equivalents (C++11 standard)
1. Move gravity and magnetic_field tags from physics to world
    * [Pull request 247](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/247)
1. Switch lump link prefix from lump:: to lump_
    * [Pull request 245](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/245)
1. New <wind> element.
   A contribution from Olivier Crave
    * [Pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/240)
1. Add scale to model state
    * [Pull request 246](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/246)
1. Use stof functions to parse hex strings as floating point params.
   A contribution from Rich Mattes
    * [Pull request 250](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/250)
1. Fix memory leaks.
   A contribution from Silvio Traversaro
    * [Pull request 249](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/249)
1. Update SDF to version 1.6: new style for representing the noise properties
   of an `imu`
    * [Pull request 243](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/243)
    * [Pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)

## libsdformat 3.0

### libsdformat 3.X.X (201X-XX-XX)

1. Improve precision of floating point parameters
     * [Pull request 273](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/273)
     * [Pull request 276](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/276)

### libsdformat 3.7.0 (2015-11-20)

1. Add spring pass through for sdf3
     * [Design document](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/gazebo_design/pull-requests/23)
     * [Pull request 242](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/242)

1. Support frame specification in SDF
     * [Pull request 237](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/237)

1. Remove boost from SDFExtension
     * [Pull request 229](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/229)

### libsdformat 3.6.0 (2015-10-27)

1. Add light state
    * [Pull request 227](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/227)
1. redo pull request #222 for sdf3 branch
    * [Pull request 232](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/232)
1. Fix links in API documentation
    * [Pull request 231](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/231)

### libsdformat 3.5.0 (2015-10-07)

1. Camera lens description (Replaces #213)
    * [Pull request 215](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/215)
1. Fix shared pointer reference loop in Element and memory leak (#104)
    * [Pull request 230](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/230)

### libsdformat 3.4.0 (2015-10-05)

1. Support nested model states
    * [Pull request 223](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/223)
1. Cleaner way to set SDF_PATH for tests
    * [Pull request 226](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/226)

### libsdformat 3.3.0 (2015-09-15)

1. Windows Boost linking errors
    * [Pull request 206](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/206)
1. Nested SDF -> sdf3
    * [Pull request 221](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/221)
1. Pointer types
    * [Pull request 218](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/218)
1. Torsional friction default surface radius not infinity
    * [Pull request 217](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/217)

### libsdformat 3.2.2 (2015-08-24)

1. Added battery element (contribution from Olivier Crave)
     * [Pull request #204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/204)
1. Torsional friction backport
     * [Pull request #211](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/211)
1. Allow Visual Studio 2015
     * [Pull request #208](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/208)

### libsdformat 3.1.1 (2015-08-03)

1. Fix tinyxml linking error
     * [Pull request #209](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/209)

### libsdformat 3.1.0 (2015-08-02)

1. Added logical camera sensor to SDF
     * [Pull request #207](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/207)

### libsdformat 3.0.0 (2015-07-24)

1. Added battery to SDF
     * [Pull request 204](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/204)
1. Added altimeter sensor to SDF
     * [Pull request #197](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/197)
1. Added magnetometer sensor to SDF
     * [Pull request 198](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/198)
1. Fix detection of XML parsing errors
     * [Pull request 190](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/190)
1. Support for fixed joints
     * [Pull request 194](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/194)
1. Adding iterations to state
     * [Pull request 188](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/188)
1. Convert to use ignition-math
     * [Pull request 173](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/173)
1. Add world origin to scene
     * [Pull request 183](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/183)
1. Fix collide bitmask
     * [Pull request 182](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/182)
1. Adding meta information to visuals
     * [Pull request 180](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/180)
1. Add projection type to gui camera
     * [Pull request 178](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/178)
1. Fix print description to include attribute description
     * [Pull request 170](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/170)
1. Add -std=c++11 flag to sdf_config.cmake.in and sdformat.pc.in, needed by downstream code
     * [Pull request 172](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/172)
1. Added boost::any accessor for Param and Element
     * [Pull request 166](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/166)
1. Remove tinyxml from dependency list
     * [Pull request 152](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/152)
1. Added self_collide element for model
     * [Pull request 149](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/149)
1. Added a collision bitmask field to sdf-1.5 and c++11 support
     * [Pull request 145](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/145)
1. Fix problems with latin locales and decimal numbers (issue #60)
     * [Pull request 147](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/147)
     * [Issue 60](https://github.com/osrf/sdformat/issues/60)

## libsdformat 2.x

1. rename cfm_damping --> implicit_spring_damper
     * [Pull request 59](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/59)
1. add gear_ratio and reference_body for gearbox joint.
     * [Pull request 62](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/62)
1. Update joint stop stiffness and dissipation
     * [Pull request 61](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/61)
1. Support for GNUInstallDirs
     * [Pull request 64](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/64)
1. `<use_true_size>` element used by DEM heightmaps
     * [Pull request 67](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/67)
1. Do not export urdf symbols in SDFormat 1.4
     * [Pull request 75](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/75)
1. adding deformable properties per issue #32
     * [Pull request 78](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/78)
     * [Issue 32](https://github.com/osrf/sdformat/issues/32)
1. Support to use external URDF
     * [Pull request 77](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/77)
1. Add lighting element to visual
     * [Pull request 79](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/79)
1. SDF 1.5: add flag to fix joint axis frame #43 (gazebo issue 494)
     * [Pull request 83](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/83)
     * [Issue 43](https://github.com/osrf/sdformat/issues/43)
     * [Gazebo issue 494](https://bitbucket.org/osrf/gazebo/issues/494)
1. Implement SDF_PROTOCOL_VERSION (issue #51)
     * [Pull request 90](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/90)
1. Port libsdformat to compile on Windows (MSVC)
     * [Pull request 101](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/101)
1. Separate material properties in material.sdf
     * [Pull request 104](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/104)
1. Add road textures (repeat pull request #104 for sdf_2.0)
     * [Pull request 105](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/105)
1. Add Extruded Polylines as a model
     * [Pull request 93](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/93)
1. Added polyline for sdf_2.0
     * [Pull request 106](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/106)
1. Add spring_reference and spring_stiffness tags to joint axis dynamics
     * [Pull request 102](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/102)
1. Fix actor static
     * [Pull request 110](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/110)
1. New <Population> element
     * [Pull request 112](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/112)
1. Add camera distortion element
     * [Pull request 120](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/120)
1. Inclusion of magnetic field strength sensor
     * [Pull request 123](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/123)
1. Properly add URDF gazebo extensions blobs to SDF joint elements
     * [Pull request 125](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/125)
1. Allow gui plugins to be specified in SDF
     * [Pull request 127](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/127)
1. Backport magnetometer
     * [Pull request 128](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/128)
1. Add flag for MOI rescaling to SDFormat 1.4
     * [Pull request 121](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/121)
1. Parse urdf joint friction parameters, add corresponding test
     * [Pull request 129](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/129)
1. Allow reading of boolean values from plugin elements.
     * [Pull request 132](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/132)
1. Implement generation of XML Schema files (issue #2)
     * [Pull request 91](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/91)
1. Fix build for OS X 10.10
     * [Pull request 135](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/135)
1. Improve performance in loading <include> SDF elements
     * [Pull request 138](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/138)
1. Added urdf gazebo extension option to disable fixed joint lumping
     * [Pull request 133](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/133)
1. Support urdfdom 0.3 (alternative to pull request #122)
     * [Pull request 141](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/141)
1. Update list of supported joint types
     * [Pull request 142](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/142)
1. Ignore unknown elements
     * [Pull request 148](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/148)
1. Physics preset attributes
     * [Pull request 146](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/146)
1. Backport fix for latin locales (pull request #147)
     * [Pull request 150](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/150)

## libsdformat 1.4

### libsdformat 1.4.8 (2013-09-06)

1. Fix inertia transformations when reducing fixed joints in URDF
    * [Pull request 48](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/48/fix-for-issue-22-reducing-inertia-across/diff)
1. Add <use_terrain_paging> element to support terrain paging in gazebo
    * [Pull request 47](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/47/add-element-inside-heightmap/diff)
1. Further reduce console output when using URDF models
    * [Pull request 46](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/46/convert-a-few-more-sdfwarns-to-sdflog-fix/diff)
    * [Commit](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/commits/b15d5a1ecc57abee6691618d02d59bbc3d1b84dc)

### libsdformat 1.4.7 (2013-08-22)

1. Direct console messages to std_err
    * [Pull request 44](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/44/fix-19-direct-all-messages-to-std_err)

### libsdformat 1.4.6 (2013-08-20)

1. Add tags for GPS sensor and sensor noise
    * [Pull request 36](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/36/gps-sensor-sensor-noise-and-spherical)
1. Add tags for wireless transmitter/receiver models
    * [Pull request 34](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/34/transceiver-support)
    * [Pull request 43](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/43/updated-description-of-the-transceiver-sdf)
1. Add tags for playback of audio files in Gazebo
    * [Pull request 26](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/26/added-audio-tags)
    * [Pull request 35](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/35/move-audio-to-link-and-playback-on-contact)
1. Add tags for simbody physics parameters
    * [Pull request 32](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/32/merging-some-updates-from-simbody-branch)
1. Log messages to a file, reduce console output
    * [Pull request 33](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/33/log-messages-to-file-8)
1. Generalize ode's <provide_feedback> element
    * [Pull request 38](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-request/38/add-provide_feedback-for-bullet-joint)
1. Various bug, style and test fixes

### libsdformat 1.4.5 (2013-07-23)

1. Deprecated Gazebo's internal SDF code
1. Use templatized Get functions for retrieving values from SDF files
1. Removed dependency on ROS
