# Migration Guide for SDFormat Specification
This document contains information about migrating
between different versions of the SDFormat specification.
The SDFormat specification version number is specified in the `version`
attribute of the `sdf` element (1.4, 1.5, 1.6, etc.)
and is distinct from sdformat library version
(2.3, 3.0, 4.0, etc.).

# Note on backward compatibility
There are `*.convert` files that allow old sdf files to be migrated
forward programmatically.
This document aims to contain similar information to those files
but with improved human-readability..

## libsdformat 14.x to 15.x

### Additions

1. **New SDFormat specification version 1.12**
    + Details about the 1.11 to 1.12 transition are explained below in this same
      document

### Modifications

1. **Behavior change of `Param::Get<bool>`**
    + Previously when a Param was set from a string, the `Get<bool>` method
      would always return `true`, and the value would be `true` if the lowercase
      string value matched `"1"` or `"true"` and `false` otherwise. Now,
      the `Get<bool>` method returns `true` only if the lowercase value string
      matches `"0"`, `"1"`, `"true"`, or `"false"` and returns `false`
      otherwise.

### Deprecations

- **sdf/Camera.hh**:
   + The `//sensor/camera/optical_frame_id` SDF element and corresponding functions
     in the Camera DOM class are deprecated. Please specify camera frame using
     the `//sensor/frame_id` SDF element instead.
   + ***Deprecation:*** std::string OpticalFrameId() const
   + ***Replacement:*** std::string Sensor::FrameId() const
   + ***Deprecation:*** void SetOpticalFrameId(const std::string &)
   + ***Replacement:*** void Sensor::SetFrameId(const std::string &)

### Removals

- **sdf/Joint.hh**:
   + `const std::string &ChildLinkName() const` (use `ChildName()` instead)
   + `const std::string &ParentLinkName() const` (use `ParentName()` instead)
   + `void SetChildLinkName(const std::string &)` (use `SetChildName()` instead)
   + `void SetParentLinkName(const std::string &)` (use `SetParentName()` instead)

- **sdf/SDFImpl.hh**:
   + `void Root(const ElementPtr)` (use `SetRoot(const ElementPtr)` instead)

- **sdf/Types.hh**:
   + `const std::string &kSdfScopeDelimiter` (use `kScopeDelimiter` instead)
   + `const std::string &SdfScopeDelimiter()` (use `kScopeDelimiter` instead)

- **sdf/parser.hh**:
   + `bool checkJointParentChildLinkNames(const sdf::Root *)` (use `checkJointParentChildNames(const sdf::Root *)` instead)

## libsdformat 13.x to 14.x

### Additions

1. **New SDFormat specification version 1.11**
    + Details about the 1.10 to 1.11 transition are explained below in this same
      document

### Modifications

1. The default camera lens intrinsics skew value in the Camera DOM class changed
   from `1` to `0` to match the SDF specification.

1. World class only renames frames with name collisions if original file version
   is 1.6 or earlier. Name collisions in newer files will cause `DUPLICATE_NAME`
   errors, which now matches the behavior of the Model class.

1. Python is now required to build libsdformat instead of Ruby.

1. **sdf/ParserConfig.hh** The following new functions were added to support automatic calculation of moments of inertia.
    + ConfigureResolveAutoInertials CalculateInertialConfiguration() const
    + void SetCalculateInertialConfiguration(ConfigureResolveAutoInertials)
    + void RegisterCustomInertiaCalc(CustomInertiaCalculator)
    + const CustomInertiaCalculator &CustomInertiaCalc() const

## libsdformat 12.x to 13.x

### Additions

1. **New SDFormat specification version 1.10**
    + Details about the 1.9 to 1.10 transition are explained below in this same
      document

### Modifications

1. ParserConfig defaults to WARN instead of LOG when parsing unrecognized
   elements.
2. Updated search order for `sdf::findFile()` making local path (current directory) the first to be searched.

### Deprecations

- The `ignition` namespace is deprecated and will be removed in future versions.
  Use `gz` instead.

- Header files under `ignition/...` are deprecated and will be removed in future versions.
  Use `gz/...` instead.

- **sdf/Types.hh**: The `Inertia` class has been deprecated. Please use the
￼   `Inertial` class in the `gz-math` library.

- **sdf/Joint.hh**:
   + ***Deprecation:*** const std::string &ChildLinkName() const
   + ***Replacement:*** const std::string &ChildName() const
   + ***Deprecation:*** const std::string &ParentLinkName() const
   + ***Replacement:*** const std::string &ParentName() const
   + ***Deprecation:*** void SetChildLinkName(const std::string &)
   + ***Replacement:*** void SetChildName(const std::string &)
   + ***Deprecation:*** void SetParentLinkName(const std::string &)
   + ***Replacement:*** void SetParentName(const std::string &)

- **sdf/parser.hh**:
   + ***Deprecation:*** bool checkJointParentChildLinkNames(const sdf::Root \*)
   + ***Replacement:*** bool checkJointParentChildNames(const sdf::Root \*)

- **sdf/SDFImpl.hh**:
   + ***Deprecation:*** void Root(sdf::ElementPtr)
   + ***Replacement:*** void SetRoot(sdf::ElementPtr)

### Removals

- **sdf/InterfaceElements.hh**: The deprecated data members from the
   `NestedInclude` class have been removed. Instead use the corresponding
   member functions.

- **sdf/Types.hh**: The `SDF_DEPRECATED` and `SDF_SUPPRESS_*` macros have been
  removed in favor of `GZ_DEPRECATED` and `GZ_UTILS_WARN_*`.

## libsdformat 12.5.0 to 12.6.0

### Modifications

1. USD component now is living in https://github.com/gazebosim/gz-usd as an
   independent package.

1. URDF parser now properly transforms poses for lights, projectors and sensors
   from gazebo extension tags that are moved to a new link during fixed joint
   reduction.
    + [pull request 1114](https://github.com/osrf/sdformat/pull/1114)

## libsdformat 11.x to 12.0

An error is now emitted instead of a warning for a file containing more than
one root level model, actor or light.

### Additions

1. **New SDFormat specification version 1.9**
    + Details about the 1.8 to 1.9 transition are explained below in this same
      document

1. **sdf/Camera.hh** The following new functions were added for segmentation cameras
    + void SetHasSegmentationType(bool)
    + bool HasSegmentationType() const
    + const std::string &SegmentationType() const
    + void SetSegmentationType(const std::string &)
    + void SetHasBoundingBoxType(bool)
    + bool HasBoundingBoxType() const
    + const std::string &BoundingBoxType() const
    + void SetBoundingBoxType(const std::string &)

1. **sdf/Elementh.hh**
    + const Param_V &GetAttributes() const

1. **sdf/Error.hh**
    + void SetFilePath(const std::string &)
    + void SetLineNumber(int)

1. **sdf/ForceTorque.hh**
    + const Noise &ForceXNoise() const
    + void SetForceXNoise(const Noise &)
    + const Noise &ForceYNoise() const
    + void SetForceYNoise(const Noise &)
    + const Noise &ForceZNoise() const
    + void SetForceZNoise(const Noise &)
    + const Noise &TorqueXNoise() const
    + void SetTorqueXNoise(const Noise &)
    + const Noise &TorqueYNoise() const
    + void SetTorqueYNoise(const Noise &)
    + const Noise &TorqueZNoise() const
    + void SetTorqueZNoise(const Noise &)

1. **sdf/InterfaceElements.hh** `sdf::NestedInclude` has the following new methods
    + const std::string &Uri() const
    + void SetUri(const std::string &)
    + const std::string &ResolvedFileName() const
    + void SetResolvedFileName(const std::string &)
    + const std::string &AbsoluteParentName() const
    + void SetAbsoluteParentName(const std::string &)
    + const std::optional<std::string> &LocalModelName() const
    + void SetLocalModelName(const std::string &)
    + const std::optional<bool> &IsStatic() const
    + void SetIsStatic(bool)
    + const std::optional<gz::math::Pose3d> &IncludeRawPose() const
    + void SetIncludeRawPose(const gz::math::Pose3d &includeRawPose)
    + const std::optional<std::string> &IncludePoseRelativeTo() const
    + void SetIncludePoseRelativeTo(const std::string &)
    + const std::optional<std::string> &PlacementFrame() const
    + void SetPlacementFrame(const std::string &)
    + sdf::ElementPtr IncludeElement() const
    + void SetIncludeElement(sdf::ElementPtr)

1. **sdf/Param.hh**:
    + ElementPtr GetParentElement() const;
    + bool SetParentElement(ElementPtr);
    + bool Reparse();
    + bool IgnoresParentElementAttribute() const;

1. **sdf/World.hh**:
    + const gz::math::SphericalCoordinates * SphericalCoordinates() const;
    + void SetSphericalCoordinates(const gz::math::SphericalCoordinates &);

### Modifications

1. **sdf/Element.hh**: The following methods now have an additional parameter of
   type `PrintConfig` with a default value
    + void PrintValues(std::string, const PrintConfig &\_config = PrintConfig()) const
    + void PrintValues(const std::string, bool, bool, const PrintConfig &\_config = PrintConfig()) const
    + std::string ToString(const std::string &, const PrintConfig &\_config = PrintConfig()) const
    + std::string ToString(const std::string &, bool, bool ,const PrintConfig &\_config = PrintConfig()) const

1. **sdf/Param.hh**: The following methods now have an additional parameter of
   type `PrintConfig` with a default value
    + std::string GetAsString(const PrintConfig &\_config = PrintConfig()) const
    + std::string GetDefaultAsString(const PrintConfig &\_config = PrintConfig()) const
    + std::optional<std::string> GetMinValueAsString(const PrintConfig &\_config = PrintConfig()) const
    + std::optional<std::string> GetMaxValueAsString(const PrintConfig &\_config = PrintConfig()) const

    The following now has an additional bool parameter
    + bool SetFromString(const std::string &, bool \_ignoreParentAttributes);

1. **sdf/SDFImpl.hh**: The following methods now have an additional parameter of
   type `PrintConfig` with a default value
    + void PrintValues(const PrintConfig &\_config = PrintConfig())
    + std::string ToString(const PrintConfig &\_config = PrintConfig()) const

1. The string literals used to indicate non-file sources have been changed to
   `<data-string>` and `<urdf-string>` for SDFormat and URDF source
   respectively. Users are encouraged to use the constants `kSdfStringSource`
   and `kUrdfStringSource` instead of hard-coding the string literals.

### Removals

The following deprecated methods and classes have been removed.

1. **sdf/Element.hh**
    + void SetInclude(const std::string)
    + std::string GetInclude() const

1. **sdf/Types.hh**
    + sdf::Color class

1. **sdf/JointAxis.hh**
    + double InitialPosition() const
    + void SetInitialPosition(const double)

1. **sdf/Root.hh**:
    + const sdf::Model \*ModelByIndex()
    + uint64_t ModelCount()
    + bool ModelNameExists(const std::string &\_name) const
    + const sdf::Light \*LightByIndex()
    + uint64_t LightCount()
    + bool LightNameExists(const std::string &\_name) const
    + const sdf::Actor \*ActorByIndex()
    + uint64_t ActorCount()
    + bool ActorNameExists(const std::string &\_name) const

### Deprecations

1. **sdf/InterfaceElements.hh**: The struct `NestedInclude` has been converted
   to a class. Accessing data members directly is deprecated. Instead use the
   corresponding member functions.

## libsdformat 11.1.0 to 11.2.0

ABI was broken for `sdf::Element`, and restored on version 11.2.1.

## libsdformat 11.0.0 to 11.x.x

### Additions

1. **sdf/Console.hh** Add functions to retrieve message stream objects. These
   can be useful for redirecting the console output to other streams.
    + ConsoleStream &GetMsgStream()
    + ConsoleStream &GetLogStream()

1. **sdf/ParserConfig.hh**:
    + void SetDeprecatedElementsPolicy(EnforcementPolicy _policy)
    + void ResetDeprecatedElementsPolicy()
    + EnforcementPolicy DeprecatedElementsPolicy() const

1. **test/test_utils.hh**:
    + ScopeExit: A utility struct that calls a function when going out of scope.
    + RedirectConsoleStream: A utility class used for redirecting the output of
      sdferr, sdfwarn, etc to a more convenient stream object like a
      std::stringstream for testing purposes.

## libsdformat 10.x to 11.0

### Additions

1. **sdf/ParserConfig.hh** A class that contains configuration options for the
   libsdformat parser.

1. + Depend on ignition-utils1 for the ImplPtr and UniqueImplPtr.
   + [Pull request 474](https://github.com/osrf/sdformat/pull/474)

1. **sdf/Joint.hh**
    + Errors ResolveChildLink(std::string&) const
    + Errors ResolveParentLink(std::string&) const

1. **sdf/Model.hh**:
    + std::pair<const Link *, std::string> CanonicalLinkAndRelativeName() const;

1. **sdf/Root.hh** sdf::Root elements can now only contain one of either Model,
      Light or Actor since multiple items would conflict with overrides
      specified in an <include> tag.
    + const sdf::Model \*Model();
    + const sdf::Light \*Light();
    + const sdf::Actor \*Actor();

### Modifications

1. **sdf/Model.hh**: the following methods now accept nested names relative to
      the model's scope that can begin with a sequence of nested model names
      separated by `::` and may end with the name of an object of the specified
      type.
    + const Frame \*FrameByName(const std::string &) const
    + const Joint \*JointByName(const std::string &) const
    + const Link \*LinkByName(const std::string &) const
    + bool FrameNameExists(const std::string &) const
    + bool JointNameExists(const std::string &) const
    + bool LinkNameExists(const std::string &) const

1. **sdf/Heightmap.hh**: sampling now defaults to 1 instead of 2.

### Deprecations

1. **src/Root.hh**: The following methods have been deprecated in favor of the
      new methods. For now the behavior is unchanged, but Root elements should
      only contain one or none of Model/Light/Actor.
    + const sdf::Model \*ModelByIndex();
    + uint64_t ModelCount();
    + bool ModelNameExists(const std::string &\_name) const;
    + const sdf::Light \*LightByIndex();
    + uint64_t LightCount();
    + bool LightNameExists(const std::string &\_name) const;
    + const sdf::Actor \*ActorByIndex();
    + uint64_t ActorCount();
    + bool ActorNameExists(const std::string &\_name) const;

## libsdformat 10.2.0 to 10.x.x

### Modifications

1. Fixed Atmosphere DOM class's temperature default value. Changed from -0.065 to -0.0065.
    * [Pull request 482](https://github.com/osrf/sdformat/pull/482)

1. Fixed parsing of `<sensor><pose>` tags on lumped links when converting from URDF.
    * [Pull request 525](https://github.com/osrf/sdformat/pull/525)

## libsdformat 9.x to 10.0

### Modifications

1. Axis vectors specified in <joint><axis><xyz> are normalized if their norm is
   greater than 0. A vector with 0 norm generates an error
    * [Pull request 312](https://github.com/osrf/sdformat/pull/312)

1. + Depend on tinyxml2 instead of tinyxml for XML parsing.
   + [Pull request 264](https://github.com/osrf/sdformat/pull/264)

1. + Minimum/maximum values specified in SDFormat description files are now enforced
   + [Pull request 303](https://github.com/osrf/sdformat/pull/303)

1. + Parsing of bad values generates an error
   + [Pull request 244](https://github.com/osrf/sdformat/pull/244)

### Deletions

1. + Removed the `parser_urdf.hh` header file and its `URDF2SDF` class
   + [Pull request 276](https://github.com/osrf/sdformat/pull/276)

1. + Removed the deprecated `Pose()`, `SetPose()`, and `*PoseFrame()` API's in all DOM classes:
   + const gz::math::Pose3d &Pose()
   + void SetPose(const gz::math::Pose3d &)
   + const std::string &PoseFrame()
   + void SetPoseFrame(const std::string &)

1. + Removed deprecated functions from **sdf/JointAxis.hh**:
   + bool UseParentModelFrame()
   + void SetUseParentModelFrame(bool)

### Additions

1. **sdf/Element.hh**
    + void AddValue(const std::string &, const std::string &, bool, const std::string &, const std::string &, const std::string &)

1. **sdf/Param.hh**
    + Param(const std::string &, const std::string &e, const std::string &, bool, const std::string &, const std::string &, const std::string &)
    + std::optional<std::string> GetMinValueAsString() const;
    + std::optional<std::string> GetMaxValueAsString() const;
    + bool ValidateValue() const;

## libsdformat 9.8.0 to 9.9.0

### Additions

1. **sdf/Camera.hh**
    + Get/Set functions for Camera projection matrix parameters.

### Modifications

1. URDF parser now properly transforms poses for lights, projectors and sensors
   from gazebo extension tags that are moved to a new link during fixed joint
   reduction.
    + [pull request 1114](https://github.com/osrf/sdformat/pull/1114)

## libsdformat 9.4 to 9.5

### Additions

1. **sdf/Element.hh**
    + sdf::ElementPtr FindElement() const

## libsdformat 9.3 to 9.4

### Modifications

1. **camera.sdf**
    + Reduce minimum value of `//camera/clip/far`

### Deprecations

1. Fix spelling of supported shader types in `//material/shader/@type`
    + `normal_map_objectspace`  replaced by `normal_map_object_space`
    + `normal_map_tangentspace` replaced by `normal_map_tangent_space`
    + [pull request 383](https://github.com/osrf/sdformat/pull/383)

## libsdformat 9.0 to 9.3

### Additions

1. **sdf/Model.hh**
    + uint64\_t ModelCount() const
    + const Model \*ModelByIndex(const uint64\_t) const
    + const Model \*ModelByName(const std::string &) const
    + bool ModelNameExists(const std::string &) const

### Modifications

1. Permit models without links if they contain a nested canonical link.
    + [pull request 341](https://github.com/osrf/sdformat/pull/341)

## SDFormat 8.x to 9.0

### Additions

1. **sdf/Collision.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Element.hh**
    + void Clear()
    + const std::string &OriginalVersion() const
    + void SetOriginalVersion(const std::string &)

1. **sdf/Frame.hh**: DOM class for frames in the model or world.
    + Errors ResolveAttachedToBody(std::string&) const
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Joint.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/JointAxis.hh**
    + Errors ResolveXyz(gz::math::Vector3d &, const std::string &) const

1. **sdf/Light.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Link.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Model.hh**
    + uint64\_t FrameCount() const
    + const Frame \*FrameByIndex(const uint64\_t) const
    + const Frame \*FrameByName(const std::string &) const
    + bool FrameNameExists(const std::string &) const
    + sdf::SemanticPose SemanticPose() const

1. **sdf/SDFImpl.hh**
    + void Clear()
    + const std::string &OriginalVersion() const
    + void SetOriginalVersion(const std::string &)

1. **sdf/SemanticPose.hh**: Helper class for resolving poses of DOM objects.

1. **sdf/Sensor.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/Visual.hh**
    + sdf::SemanticPose SemanticPose() const

1. **sdf/World.hh**
    + uint64\_t FrameCount() const
    + const Frame \*FrameByIndex(const uint64\_t) const
    + const Frame \*FrameByName(const std::string &) const
    + bool FrameNameExists(const std::string &) const
    + const Model \*ModelByName(const std::string &) const

1. **sdf/parser.hh**
   + bool checkCanonicalLinkNames(sdf::Root\*)
   + bool checkFrameAttachedToGraph(sdf::Root\*)
   + bool checkFrameAttachedToNames(sdf::Root\*)
   + bool checkJointParentChildLinkNames(sdf::Root\*)
   + bool checkPoseRelativeToGraph(sdf::Root\*)
   + bool recursiveSameTypeUniqueNames(sdf::ElementPtr)
   + bool recursiveSiblingUniqueNames(sdf::ElementPtr)
   + bool shouldValidateElement(sdf::ElementPtr)

### Deprecations

1. **sdf/parser_urdf.hh**
   + ***Deprecation:*** URDF2SDF
   + ***Replacement:*** None. Use the functions sdf::readFile or sdf::readString, which automatically convert URDF to SDFormat.

1. All DOM classes with `Pose()` and `PoseFrame()` API's:
   + ***Deprecation:*** const gz::math::Pose3d &Pose()
   + ***Replacement:*** const gz::math::Pose3d &RawPose()
   + ***Deprecation:*** const std::string &PoseFrame()
   + ***Replacement:*** const std::string &PoseRelativeTo()
   + ***Deprecation:*** void SetPose(const gz::math::Pose3d &)
   + ***Replacement:*** void SetRawPose(const gz::math::Pose3d &)
   + ***Deprecation:*** void SetPoseFrame(const std::string &)
   + ***Replacement:*** void SetPoseRelativeTo(const std::string &)

1. **sdf/JointAxis.hh**
   + ***Deprecation:*** bool UseParentModelFrame()
   + ***Replacement:*** const std::string &XyzExpressedIn()
   + ***Deprecation:*** void SetUseParentModelFrame(bool)
   + ***Replacement:*** void SetXyzExpressedIn(const std::string &)

## SDFormat 8.0 to 8.1

### Modifications

1.  + Change installation path of SDF description files to allow side-by-side installation.
    + `{prefix}/share/sdformat/1.*/*.sdf` -> `{prefix}/share/sdformat8/1.*/*.sdf`
    + [BitBucket pull request 538](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/538)

## SDFormat 5.x to 6.x

### Additions

1. **sdf/parser.hh**
   + bool recursiveSameTypeUniqueNames(sdf::ElementPtr)

### Deprecations

1. **sdf/Types.hh**
   + ***Deprecated:*** sdf::Color class
   + ***Replacement:*** gz::math::Color class

## SDFormat 4.x to 5.x

### Deletions

1. **Removed the following functions from `parser.hh`**
    + bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf);
    + bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf);
    + bool initXml(TiXmlElement *_xml, ElementPtr _sdf);
    + bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf, const std::string &_source);
    + bool readDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf, const std::string &_source);
    + bool readXml(TiXmlElement *_xml, ElementPtr _sdf);
    + void copyChildren(ElementPtr _sdf, TiXmlElement *_xml);
    + std::string getBestSupportedModelVersion(TiXmlElement *_modelXML, std::string &_modelFileName);

### Deprecations

1. **sdf/Param.hh**
    + ***Deprecation:*** const std::type_info &GetType() const
    + ***Replacement:*** template<typename Type> bool IsType() const

1. **sdf/SDFImpl.hh**
    + ***Deprecation:*** ElementPtr root
    + ***Replacement:*** ElementPtr Root() const / void Root(const ElementPtr \_root)
    + ***Deprecation:*** static std::string version
    + ***Replacement:*** static std::string Version()

1. **sdf/Types.hh**
    + ***Deprecation:*** sdf::Vector2i
    + ***Replacement:*** gz::math::Vector2i
    + ***Deprecation:*** sdf::Vector2d
    + ***Replacement:*** gz::math::Vector2d
    + ***Deprecation:*** sdf::Vector3
    + ***Replacement:*** gz::math::Vector3d
    + ***Deprecation:*** sdf::Quaternion
    + ***Replacement:*** gz::math::Quaterniond
    + ***Deprecation:*** sdf::Pose
    + ***Replacement:*** gz::math::Pose3d

## SDFormat 3.x to 4.x

### Additions

1. **New SDFormat specification version 1.6**
    + Details about the 1.5 to 1.6 transition are explained below in this same
      document

### Modifications

1. **Boost pointers and boost::function**
    + All boost pointers, boost::function in the public API have been replaced
      by their std:: equivalents (C++11 standard)

1. **Lump:: prefix in link names**
    + Changed to `_fixed_joint_lump__` to avoid confusion with scoped names
    + [BitBucket pull request 245](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/245)

## SDFormat specification 1.11 to 1.12

### Additions

1. **joint_state.sdf**:
    + `//joint_state/axis_state/position`
    + `//joint_state/axis_state/velocity`
    + `//joint_state/axis_state/acceleration`
    + `//joint_state/axis_state/effort`
    + `//joint_state/axis2_state/position`
    + `//joint_state/axis2_state/velocity`
    + `//joint_state/axis2_state/acceleration`
    + `//joint_state/axis2_state/effort`

1. **link_state.sdf**:
    + `//link_state/linear_velocity`
    + `//link_state/angular_velocity`
    + `//link_state/linear_acceleration`
    + `//link_state/angular_acceleration`
    + `//link_state/linear_wrench`
    + `//link_state/angular_wrench`

### Modifications

1. **state.sdf**, **model_state.sdf**, **joint_state.sdf**, **link_state.sdf**,
   **light_state.sdf**: A `_state` suffix has been added to state element names
   to match the `.sdf` file names and for consistency.
    + `//state/light` renamed to `//state/light_state`
    + `//state/model` renamed to `//state/model_state`
    + `//state/model/joint` renamed to `//state/model_state/joint_state`
    + `//state/model/light` renamed to `//state/model_state/light_state`
    + `//state/model/link`  renamed to `//state/model_state/link_state`
    + `//state/model/model` renamed to `//state/model_state/model_state`
    + `//state/model/link/collision` renamed to `//state/model_state/link_state/collision_state`

1. **state.sdf**: `//state/joint_state` has been added to represent the state of a
    `//world/joint` and `//state/insertions/joint` can represent inserted
    `//world/joint` elements.

### Deprecations

1. **joint_state.sdf**:
    + `//joint_state/angle` is deprecated in favor of `//axis_state/position`
      and  `//axis2_state/position`.

1. **link_state.sdf**:
    + `//link_state/velocity` is deprecated in favor of `//link_state/angular_velocity`
      and  `//link_state/linear_velocity`.
    + `//link_state/acceleration` is deprecated in favor of `//link_state/angular_acceleration`
      and  `//link_state/linear_acceleration`.
    + `//link_state/wrench` is deprecated in favor of `//link_state/angular_wrench`
      and  `//link_state/linear_wrench`.

## SDFormat specification 1.10 to 1.11

### Additions

1. **mimic.sdf**, **joint.sdf**: a mimic tag can be added to `//joint/axis` and
    `//joint/axis2` to specify a linear relationship between the position of two
    joint axes according to the following equation:
    `follower_position = multiplier * (leader_position - reference) + offset`.
    The joint axis containing the mimic tag is the follwer and the leader is
    specified using the `@joint` and `@axis` attributes.
    + `//mimic/@joint`: name of joint containing the leader axis.
    + `//mimic/@axis`: name of the leader axis. Only valid values are "axis" or "axis2".
    + `//mimic/multiplier`: parameter representing ratio between changes in follower axis position relative to changes in leader axis position.
    + `//mimic/offset`: parameter representing offset to follower position.
    + `//mimic/reference`: parameter representing reference for leader position before applying the multiplier.

1. **inertial.sdf**: A new attribute `//inertial/@auto` has been added
   to specify whether the moment of inertia of a link should be calculated
   automatically. In addition, the following elements have been added:
   + `//inertial/density`: Common mass density of all collision geometries.
   + `//inertial/auto_inertia_params`: Container for custom parameters passed
     to the moment of inertia calculator.

1. **collision.sdf**: A new element `//collision/density` has been added
   to specify the density of a material for automatic calculation of the moment
   of inertia of the parent link. This parameter overrides the value in
   `//inertial/density` of the parent link if specified. In addition,
   `//collision/auto_inertia_params` can be used to override
   `//inertial/auto_inertia_params` of the parent link.

## SDFormat specification 1.9 to 1.10

### Additions

1. **world.sdf**: A joint can be specified directly in a world.
1. **world.sdf**: Merge-includes are now allowed in worlds. The included models must not contain top-level links or grippers.

### Modifications

1. **joint.sdf**: axis limits default values have changed
    + `//limit/lower`: `-inf` (formerly `-1e16`)
    + `//limit/upper`: `inf` (formerly `1e16`)
    + `//limit/velocity`: `inf` (formerly `-1`)
    + `//limit/effort`: `inf` (formerly `-1`)

1. **joint.sdf**: thread_pitch is deprecated in favor of screw_thread_pitch.

1. **plugin.sdf**: name attribute is now optional with empty default value.

### Removals

1. **collision_engine.sdf**: unused specification file is removed.

1. **urdf.sdf**: unused specification file is removed.

## SDFormat specification 1.8 to 1.9

### Additions

1. **camera.sdf**: New elements to configure segmentation and boundingbox cameras
    + `//sensor/camera/segmentation_type`
    + `//sensor/camera/box_type`
    + [Pull request #592](https://github.com/gazebosim/sdformat/pull/592)

1. **forcetorque.sdf**: New elements to specify the noise characteristics of the force-torque sensor
    + `//sensor/force_torque/force`
    + `//sensor/force_torque/torque`
    + [Pull request #669](https://github.com/gazebosim/sdformat/pull/669)

1. **model.sdf**: `//model/include/@merge` for merging included nested models into the containing model
    + [Pull request #659](https://github.com/gazebosim/sdformat/pull/659)

1. **pose.sdf**: New attributes to support specifying angles in degrees and specifying rotations in quaternions
    + `//pose/@rotation_format`
    + `//pose/@degrees`
    + [Pull request #690](https://github.com/gazebosim/sdformat/pull/690)
    + [Pull request #589](https://github.com/gazebosim/sdformat/pull/589)

1. **sensor.sdf**: New sensor types `boundingbox_camera`, `segmentation_camera`, and `custom`.
    + [Pull request #592](https://github.com/gazebosim/sdformat/pull/592)

### Removals

1. **joint.sdf**
    + Deprecated elements `//joint/axis/initial_position` and `//joint/axis2/initial_position` have been removed
    * [Pull request #622](https://github.com/gazebosim/sdformat/pull/622)

1. **spherical_coordinates**: Unsupported options `NED` and `NWU` have been removed from `//spherical_coordinates/world_frame_orientation`
    * [Pull request #685](https://github.com/gazebosim/sdformat/pull/685)

## SDFormat specification 1.7 to 1.8

### Additions

1. **capsule.sdf and ellipsoid.sdf** new shape types included in `//geometry`
    + `capsule.sdf`: A shape consisting of a cylinder capped by hemispheres
      with parameters for the `radius` and `length` of cylindrical section.
    + `ellipsoid.sdf`: A convex shape with up to three radii defining its
      shape in of the form (x^2/a^2 + y^2/b^2 + z^2/c^2 = 1).
    * [Pull request 389](https://github.com/osrf/sdformat/pull/389)
    * [Pull request 434](https://github.com/osrf/sdformat/pull/434)

1. **light.sdf** `//light/intensity` element
    + description: Scale factor to set the relative power of a light.
    + type: double
    + default: 1
    + required: 0
    + [pull request 484](https://github.com/osrf/sdformat/pull/484)

### Modifications

1. **joint.sdf** `child` and `parent` elements accept frame names instead of only link names
    * [Issue 204](https://github.com/osrf/sdformat/issues/204)

1. **heightmap.sdf**: sampling now defaults to 1 instead of 2.

1. Element attribute names containing delimiter "::" no longer accepted
    * [Issue 420](https://github.com/osrf/sdformat/issues/420)

### Deprecations

1. **joint.sdf** `initial_position` element in `<joint><axis>` and `<joint><axis2>` is deprecated

### Removals

1. **inerial.sdf** `//inertial/pose/@relative_to` attribute is removed
    + [Pull request 480](https://github.com/osrf/sdformat/pull/480)

## SDFormat specification 1.6 to 1.7

### Additions

1. **frame.sdf** `//frame/@attached_to` attribute
    + description: Name of the link or frame to which this frame is attached.
      If a frame is specified, recursively following the attached\_to attributes
      of the specified frames must lead to the name of a link, a model, or the world frame.
    + type: string
    + default: ""
    + required: *
    + [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. **joint.sdf** `//axis/xyz/@expressed_in` and `//axis2/xyz/@expressed_in` attributes
    + description: The name of the frame in which the `//axis/xyz` value is
      expressed. When migrating from sdf 1.6, a `use_parent_model_frame` value
      of `true` will be mapped to a value of `__model__` for the `expressed_in`
      attribute.
    + type: string
    + default: ""
    + required: 0
    + [BitBucket pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. **material.sdf** `//material/double_sided` element
    + description: Flag to indicate whether the mesh that this material is applied to
      will be rendered as double sided.
    + type: bool
    + default: false
    + required: 0
    + [pull request 418](https://github.com/osrf/sdformat/pull/418)

1. **model.sdf** `//model/@canonical_link` attribute
    + description: The name of the canonical link in this model to which the
      model's implicit frame is attached. This implies that a model must have
      at least one link (unless it is static), which is also stated in the
      Modifications section.
    + type: string
    + default: ""
    + required: 0
    + [BitBucket pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)

1. **world.sdf** `//world/frame` element is now allowed.
    + [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

### Modifications

1.  A non-static model must have at least one link, as specified in the
    [proposal](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&#2-model-frame-and-canonical-link).
    + [BitBucket pull request 601](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/601)
    + [BitBucket pull request 626](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/626)

1. Unique names for all sibling elements:
    + As described in the [proposal](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&#3-2-unique-names-for-all-sibling-elements),
      all named sibling elements must have unique names.
      Uniqueness is forced so that referencing implicit frames is not ambiguous,
      e.g. you cannot have a link and joint share an implicit frame name.
      Some existing SDFormat models may not comply with this requirement.
      The `gz sdf --check` command can be used to identify models that violate
      this requirement.
    + [BitBucket pull request 600](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/600)

1. Reserved names:
    + As described in the [proposal](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&#3-3-reserved-names),
      entities in a simulation must not use world as a name.
      It has a special interpretation when specified as a parent or child link
      of a joint.
      Names starting and ending with double underscores (eg. `__wheel__`) must
      be reserved for use by library implementors and the specification.
      For example, such names might be useful during parsing for setting
      sentinel or default names for elements with missing names.
      If explicitly stated, they can be referred to (e.g. `__model__` / `world`
      for implicit model / world frames, respectively).

1. **joint.sdf** `//joint/child` may no longer be specified as `world`.
    + [BitBucket pull request 634](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/634)

1. **pose.sdf** `//pose/@frame` attribute is renamed to `//pose/@relative_to`.
    + [BitBucket pull request 597](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/597)

### Removals

1. `<frame>` element is now only allowed in `<model>` and `<world>`.
    It is no longer allowed in the following elements:
    + actor
    + audio\_source
    + camera
    + collision
    + frame
    + gui
    + inertial
    + joint
    + light
    + light\_state
    + link
    + link\_state
    + population
    + projector
    + sensor
    + visual
    + [BitBucket pull request 603](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/603)

1. **actor.sdf** `static` element was deprecated in
    [BitBucket pull request 280](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/280)
    and is now removed.
    + [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. **imu.sdf** `topic` element was deprecated in
    [BitBucket pull request 532](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/532)
    and is now removed.
    + [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. **joint.sdf** `//axis/use_parent_model_frame` and `//axis2/use_parent_model_frame` elements
    are removed in favor of the `//axis/xyz/@expressed_in` and
    `//axis2/xyz/@expressed_in` attributes.
    When migrating from sdf 1.6, a `use_parent_model_frame` value
    of `true` will be mapped to a value of `__model__` for the `expressed_in`
    attribute.
    + [BitBucket pull request 589](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/589)

1. **joint.sdf** `//physics/ode/provide_feedback` was deprecated in
    [BitBucket pull request 38](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/38)
    and is now removed.
    + [BitBucket pull request 588](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/588)

1. **world.sdf** `//world/joint` was removed as it has never been used.
    + [BitBucket pull request 637](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/637)

## SDFormat specification 1.5 to 1.6

### Additions

1. **actor.sdf** `tension` element
    + description: The tension of the trajectory spline. The default value of
      zero equates to a Catmull-Rom spline, which may also cause the animation
      to overshoot keyframes. A value of one will cause the animation to stick
      to the keyframes.
    + type: double
    + default: 0.0
    + min: 0.0
    + max: 1.0
    + required: 0
    + [BitBucket pull request 466](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/466)

1. **camera.sdf** `depth_camera/clip` sub-elements: `near`, `far`
    + description: Clipping parameters for depth camera on rgbd camera sensor.
    + [BitBucket pull request 628](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/628)

1. **camera.sdf** `intrinsics` sub-elements: `fx`, `fy`, `cx`, `cy`, `s`
    + description: Camera intrinsic parameters for setting a custom perspective projection matrix.
    + [BitBucket pull request 496](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/496)

1. **heightmap_shape.sdf** `sampling` element
    + description: Samples per heightmap datum.
      For rasterized heightmaps, this indicates the number of samples to take per pixel.
      Using a lower value, e.g. 1, will generally improve the performance
      of the heightmap but lower the heightmap quality.
    + type: unsigned int
    + default: 2
    + required: 0
    + [Bitbucket pull request 293](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/293)

1. **link.sdf** `enable_wind` element
    + description: If true, the link is affected by the wind
    + type: bool
    + default: false
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **link.sdf** `light` element
    + included from `light.sdf` with `required="*"`,
      so a link can have any number of attached lights.
    + [BitBucket pull request 373](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/373)

1. **model.sdf** `enable_wind` element
    + description: If set to true, all links in the model will be affected by
      the wind.  Can be overriden by the link wind property.
    + type: bool
    + default: false
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **model_state.sdf** `scale` element
    + description: Scale for the 3 dimensions of the model.
    + type: vector3
    + default: "1 1 1"
    + required: 0
    + [BitBucket pull request 246](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/246)

1. **physics.sdf** `dart::collision_detector` element
    + description: The collision detector for DART to use.
      Can be dart, fcl, bullet or ode.
    + type: string
    + default: fcl
    + required: 0
    + [BitBucket pull request 440](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/440)

1. **physics.sdf** `dart::solver::solver_type` element
    + description: The DART LCP/constraint solver to use.
      Either dantzig or pgs (projected Gauss-Seidel)
    + type: string
    + default: dantzig
    + required: 0
    + [BitBucket pull request 369](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/369)

1. **physics.sdf** `friction_model` element
    + description: Name of ODE friction model to use. Valid values include:
        + pyramid_model: (default) friction forces limited in two directions
          in proportion to normal force.
        + box_model: friction forces limited to constant in two directions.
        + cone_model: friction force magnitude limited in proportion to normal force.
          See [gazebo pull request 1522](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-request/1522)
          (merged in [gazebo 8c05ad64967c](https://github.com/osrf/gazebo/commit/968dccafdfbfca09c9b3326f855612076fed7e6f))
          for the implementation of this feature.
    + type: string
    + default: "pyramid_model"
    + required: 0
    + [Bitbucket pull request 294](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/294)

1. **physics.sdf** `island_threads` element under `ode::solver`
    + description: Number of threads to use for "islands" of disconnected models.
    + type: int
    + default: 0
    + required: 0
    + [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. **physics.sdf** `thread_position_correction` element under `ode::solver`
    + description: Flag to use threading to speed up position correction computation.
    + type: bool
    + default: 0
    + required: 0
    + [BitBucket pull request 380](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/380)

1. **sonar.sdf** `geometry` element
    + description: The sonar collision shape. Currently supported geometries are: "cone" and "sphere".
    + type: string
    + default: "cone"
    + required: 0
    + [BitBucket pull request 495](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/495)

1. **state.sdf** allow `light` tags within `insertions` element
    * [BitBucket pull request 325](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/325)

1. **surface.sdf** `category_bitmask` element
    + description: Bitmask for category of collision filtering.
      Collision happens if `((category1 & collision2) | (category2 & collision1))` is not zero.
      If not specified, the category_bitmask should be interpreted as being the same as collide_bitmask.
    + type: unsigned int
    + default: 65535
    + required: 0
    + [BitBucket pull request 318](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/318)

1. **world.sdf** `wind` element
    + description: The wind tag specifies the type and properties of the wind.
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

1. **world.sdf** `wind::linear_velocity` element
    + description: Linear velocity of the wind.
    + type: vector3
    + default: "0 0 0"
    + required: 0
    + [BitBucket pull request 240](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/240)

### Modifications

1. **`gravity` and `magnetic_field` elements are moved  from `physics` to `world`**
    + In physics element: gravity and `magnetic_field` tags have been moved
      from Physics to World element.
    + [BitBucket pull request 247](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/247)
    + [BitBucket gazebo pull request 2090](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2090)

1. **New noise for IMU**
    + A new style for representing the noise properties of an `imu` was implemented
      in [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)
      for sdf 1.5 and the old style was declared as deprecated.
      The old style has been removed from sdf 1.6 with the conversion script
      updating to the new style.
    + [BitBucket pull request 199](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/199)
    + [BitBucket pull request 243](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/243)
    + [BitBucket pull request 244](https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/244)
