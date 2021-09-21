/*
 * Copyright 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef SDF_INTERFACE_ELEMENTS_HH_
#define SDF_INTERFACE_ELEMENTS_HH_

#include <string>
#include <memory>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/Element.hh"
#include "sdf/InterfaceModel.hh"
#include "sdf/Types.hh"

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
#ifdef _WIN32
// Disable warning C4251 which is triggered by std::string
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
/// \brief Contains the necessary information about an included model file
/// for custom model parsers to be able to find the file and parse it.
class SDFORMAT_VISIBLE NestedInclude
{
  /// \brief Constructor
  public: NestedInclude();
  // Defaulted copy, move constructors and destructors are needed to avoid
  // deprecation warnings on memeber variables when simply instantiating this
  // class.
  // TODO(anyone) Remove the constructor and destructor once the deprecated
  // members are removed.
  SDF_SUPPRESS_DEPRECATED_BEGIN
  public: NestedInclude(const NestedInclude&) = default;
  public: NestedInclude(NestedInclude&&) = default;
  public: NestedInclude& operator=(const NestedInclude&) = default;
  public: NestedInclude& operator=(NestedInclude&&) = default;
  public: ~NestedInclude() = default;
  SDF_SUPPRESS_DEPRECATED_END

  /// \brief Provides the URI as specified in `//include/uri`. This may or may
  /// not end with a file extension (it will not end with an extension if it
  /// refers to a model package).
  /// \return URI of the included model
  public: const std::string &Uri() const;

  /// \brief Set the URI of the included model
  /// \param[in] _uri URI of the included model
  public: void SetUri(const std::string &_uri);

  /// \brief Provides the *resolved* absolute file path from the URI.
  /// It is recommended to use this in `CustomModelParser` when checking
  /// predicates on filenames -- however, the predicates should generally only
  /// check the file extension.
  /// \return The resolved absolute file path from the URI.
  public: const std::string &ResolvedFileName() const;

  /// \brief Set the resolved absolute file path.
  /// \param[in] _resolvedFileName The resolved absolute file path
  public: void SetResolvedFileName(const std::string &_resolvedFileName);

  /// \brief Name of the parent entity in absolute hierarchy.
  /// Example: if the interface model's name is
  /// `top_model::middle_model::my_new_model`, the absoluteParentName would be
  /// `top_model::middle_model`. If the parent entity is the world, this would
  /// be an empty string.
  /// \return Absolute name of parent entity
  public: const std::string &AbsoluteParentName() const;

  /// \brief Set the absolute name of parent entity
  /// \param[in] _absoluteparentname Absolute name of parent entity
  public: void SetAbsoluteParentName(const std::string &_absoluteparentname);

  /// \brief Name relative to immediate parent as specified in
  /// `//include/name`. This is nullopt if `//include/name` is not set. Then the
  /// name of the model must be determined by the custom model parser from the
  /// included model file.
  /// Example: `my_new_model`
  /// \return The local name. nullopt if `//include/name` is not set
  public: const std::optional<std::string> &LocalModelName() const;

  /// \brief Set the name relative to immediate parent as specified in
  /// `//include/name`
  /// \param[in] _localModelName The local name
  public: void SetLocalModelName(const std::string &_localModelName);

  /// \brief Whether the model is static as defined by `//include/static`. This
  /// is nullopt if `//include/static` is not set.
  /// \return Whether the model is static. nullopt if `//include/static` is not
  /// set.
  public: const std::optional<bool> &IsStatic() const;

  /// \brief Set whether the model is static.
  /// \param[in] _isStatic True if the model is static.
  public: void SetIsStatic(bool _isStatic);

  /// \brief The raw pose as specified in `//include/pose`. This is nullopt if
  /// `//include/pose` is not set.
  /// \return The raw pose.  nullopt if `//include/pose` is not set.
  public: const std::optional<ignition::math::Pose3d> &IncludeRawPose() const;

  /// \brief Set the raw pose as specified in `//include/pose`.
  /// \param[in] _includeRawPose The raw pose
  public: void SetIncludeRawPose(const ignition::math::Pose3d &_includeRawPose);

  /// \brief The relative-to frame of the pose as specified in
  /// `//include/pose/@relative_to`. This is nullopt if
  /// `//include/pose/@relative_to` is not set.
  /// \return The relative-to frame of the pose. nullopt if
  /// `//include/pose/@relative_to` is not set.
  public: const std::optional<std::string> &IncludePoseRelativeTo() const;

  /// \brief Set the relative-to frame of the pose.
  /// \param[in] _includePoseRelativeTo The relative-to frame.
  public: void SetIncludePoseRelativeTo(
              const std::string &_includePoseRelativeTo);

  /// \brief The placement frame as specified in `//include/placement_frame`.
  /// This is nullopt if `//include/placement_frame` is is not set.
  /// \return The placement frame. nullopt if `//include/placement_frame` is is
  /// not set.
  public: const std::optional<std::string> &PlacementFrame() const;

  /// \brief Set the placement frame.
  /// \param[in] _placementFrame The placement frame.
  public: void SetPlacementFrame(const std::string &_placementFrame);

  /// This is the `//include` element. This can be used to pass custom elements
  /// and attributes to the custom model parser.
  /// \return The `//include` element
  public: sdf::ElementPtr IncludeElement() const;

  /// Set the `//include` element.
  /// \param[in] _includeElement The include element
  public: void SetIncludeElement(sdf::ElementPtr _includeElement);

  /// \brief Provides the URI as specified in `//include/uri`. This may or may
  /// not end with a file extension (it will not end with an extension if it
  /// refers to a model package).
  /// \deprecated Use NestedInclude::Uri() instead
  public: std::string uri SDF_DEPRECATED(12);

  /// \brief Provides the *resolved* absolute file path from the URI.
  /// It is recommended to use this in `CustomModelParser` when checking
  /// predicates on filenames -- however, the predicates should generally only
  /// check the file extension.
  /// \deprecated Use NestedInclude::ResolvedFileName() instead
  public: std::string resolvedFileName SDF_DEPRECATED(12);

  /// \brief Name of the parent entity in absolute hierarchy.
  /// Example: if the interface model's name is
  /// `top_model::middle_model::my_new_model`, the absoluteParentName would be
  /// `top_model::middle_model`. If the parent entity is the world, this would
  /// be an empty string.
  /// \deprecated Use NestedInclude::AbsoluteParentName() instead
  public: std::string absoluteParentName SDF_DEPRECATED(12);

  /// \brief Name relative to immediate parent as specified in
  /// `//include/name`. This is nullopt if `//include/name` is not set. Then the
  /// name of the model must be determined by the custom model parser from the
  /// included model file.
  /// Example: `my_new_model`
  /// \deprecated Use NestedInclude::LocalModelName() instead
  public: std::optional<std::string> localModelName SDF_DEPRECATED(12);

  /// \brief Whether the model is static as defined by `//include/static`. This
  /// is nullopt if `//include/static` is not set.
  /// \deprecated Use NestedInclude::IsStatic() instead
  public: std::optional<bool> isStatic SDF_DEPRECATED(12);

  /// \brief The raw pose as specified in //include/pose. This is nullopt if
  /// `//include/pose` is not set.
  /// \deprecated Use NestedInclude::IncludeRawPose() instead
  public: std::optional<ignition::math::Pose3d> includeRawPose
              SDF_DEPRECATED(12);

  /// \brief The relative-to frame of the pose as specified in
  /// `//include/pose/@relative_to`. This is nullopt if
  /// `//include/pose/@relative_to` is not set.
  /// \deprecated Use NestedInclude::IncludePoseRelativeTo() instead
  public: std::optional<std::string> includePoseRelativeTo SDF_DEPRECATED(12);

  /// \brief The placement frame as specified in `//include/placement_frame`.
  /// This is nullopt if `//include/placement_frame` is is not set.
  /// \deprecated Use NestedInclude::PlacementFrame() instead
  public: std::optional<std::string> placementFrame SDF_DEPRECATED(12);

  /// This is the `//include` element. This can be used to pass custom elements
  /// and attributes to the custom model parser.
  /// \deprecated Use NestedInclude::IncludeElement() instead
  public: sdf::ElementPtr includeElement SDF_DEPRECATED(12);

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
#ifdef _MSC_VER
#pragma warning(pop)
#endif

/// Defines a custom model parser.
///
/// Every custom model parser should define it's own way of (quickly)
/// determining if it should parse a model. This should generally be done by
/// looking at the file extension of `include.resolvedFileName`, and
/// returning nullptr if it doesn't match a given criteria.
///
/// Custom model parsers are visited in the *reverse* of how they are defined.
/// The latest parser gains precedence.
///
/// Custom model parsers are *never* checked if resolved file extension ends
/// with `*.sdf` or `*.world`.
/// If libsdformat encounters a `*.urdf` file, it will first check custom
/// parser. If no custom parser is found, it will then convert the URDF XML to
/// SDFormat XML, and parse it as an SDFormat file.
///
/// \param[in] include The parsed //include information from which this model
///   should be parsed.
/// \param[out] errors Errors encountered during custom parsing.
///   If any errors are reported, this must return nullptr.
/// \return An optional ModelInterface.
///   * If not nullptr, the returned model interface is incorporated into the
///     existing model and its frames are exposed through the frame graph.
///   * If nullptr and no errors are reported, then libsdformat should
///     continue testing out other custom parsers registered under the same
///     extension (e.g. a parser for `.yaml`, which may cover many different
///     schemas).
///
/// If an exception is raised by this callback, libsdformat will *not* try to
/// intercept the exception.
///
/// To see an example implementation, please refer to
/// test/integration/interface_api.cc
using CustomModelParser =
    std::function<sdf::InterfaceModelPtr(const sdf::NestedInclude &, Errors &)>;
}
}

#endif
