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
// TODO (addisu) docs
// This can be used in both //model elements as well as /world.
struct NestedInclude {
  /// Provides the URI as specified in `//include/uri`. This may or may not end
  /// with a file extension (it will not end an extension if it refers to a
  /// model package).
  std::string uri;

  /// Provides the *resolved* absolute file path from the URI.
  /// It is recommended to use this in `CustomModelParser` when checking
  /// predicates on filenames -- however, the predicates should generally only
  /// check the file extension.
  std::string resolvedFileName;

  /// Name of the model in absolute hierarchy.
  /// Example: `top_model::middle_model::my_new_model`
  std::string absoluteModelName;

  /// Name relative to immediate parent as specified in `//include/@name`.
  /// Example: `my_new_model`
  std::string localModelName;

  /// As defined by `//include/static`.
  std::optional<bool> isStatic;

  // TODO(addisu) Don't we need to pass //include/pose?

  /// This is a "virtual" XML element that will contain all custom (*unparsed*)
  /// elements and attributes within `//include`.
  sdf::ElementPtr virtualCustomElements;
};

/// Defines a custom model parser.
///
/// Every custom model parser should define it's own way of (quickly)
/// determining if it should parse a model. This should generally be done by
/// looking at the file extension of `include.resolved_file_name`, and
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
/// \returns An optional ModelInterface.
///   * If not nullptr, the returned model interface is incorporated into the
///     existing model and its frames are exposed through the frame graph.
///   * If nullptr and no errors are reported, then libsdformat should
///     continue testing out other custom parsers registered under the same
///     extension (e.g. a parser for `.yaml`, which may cover many different
///     schemas).
///
/// If an exception is raised by this callback, libsdformat will *not* try to
/// intercept the exception.
using CustomModelParser =
    std::function<sdf::InterfaceModelPtr(const sdf::NestedInclude &, Errors &)>;
}
}

#endif
