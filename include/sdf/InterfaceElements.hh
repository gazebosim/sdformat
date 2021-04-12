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
struct SDFORMAT_VISIBLE NestedInclude
{
  /// \brief Provides the URI as specified in `//include/uri`. This may or may
  /// not end with a file extension (it will not end with an extension if it
  /// refers to a model package).
  std::string uri;

  /// \brief Provides the *resolved* absolute file path from the URI.
  /// It is recommended to use this in `CustomModelParser` when checking
  /// predicates on filenames -- however, the predicates should generally only
  /// check the file extension.
  std::string resolvedFileName;

  /// \brief Name of the parent entity in absolute hierarchy.
  /// Example: if the interface model's name is
  /// `top_model::middle_model::my_new_model`, the absoluteParentName would be
  /// `top_model::middle_model`. If the parent entity is the world, this would
  /// be an empty string.
  std::string absoluteParentName;

  /// \brief Name relative to immediate parent as specified in
  /// `//include/name`. This is nullopt if `//include/name` is not set. Then the
  /// name of the model must be determined by the custom model parser from the
  /// included model file.
  /// Example: `my_new_model`
  std::optional<std::string> localModelName;

  /// \brief Whether the model is static as defined by `//include/static`. This
  /// is nullopt if `//include/static` is not set.
  std::optional<bool> isStatic;

  /// \brief The raw pose as specified in //include/pose. This is nullopt if
  /// `//include/pose` is not set.
  std::optional<ignition::math::Pose3d> includeRawPose;

  /// \brief The relative-to frame of the pose as specified in
  /// `//include/pose/@relative_to`. This is nullopt if
  /// `//include/pose/@relative_to` is not set.
  std::optional<std::string> includePoseRelativeTo;

  /// \brief The placement frame as specified in `//include/placement_frame`.
  /// This is nullopt if `//include/placement_frame` is is not set.
  std::optional<std::string> placementFrame;

  /// This is the `//include` element. This can be used to pass custom elements
  /// and attributes to the custom model parser.
  sdf::ElementPtr includeElement;
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
