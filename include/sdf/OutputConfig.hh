/*
 * Copyright 2022 Open Source Robotics Foundation
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

#ifndef SDF_OUTPUT_CONFIG_HH_
#define SDF_OUTPUT_CONFIG_HH_

#include <gz/utils/ImplPtr.hh>

#include "sdf/InterfaceElements.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"


namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
// Forward declare private data class.
class OutputConfigPrivate;

/// This class contains configuration options for SDF output. Output
/// configuration can be used to specify how SDF is generated from in-memory
/// represenations, such as the DOM classes.
///
/// Example:
/// The default behavior of the `ToElement` functions is to use `<include>`
/// tags if the tags were present during load. You can choose not to use
/// the `<include>` tags using the following snippet.
///
/// \code{.cpp}
///   // Create an output config;
///   sdf::OutputConfig config;
///
///   // Set the config so that include tags are not used.
///   config.SetToElementUseIncludeTag(false);
///
///   // Use the new config when generating the SDFormat element.
///   sdf::Root root;
///   sdf::ElementPtr elem = root.ToElement(config);
///
///   // You can now output the element as a string.
///   std::cout << elem->ToString("") << std::endl;
/// \endcode
class SDFORMAT_VISIBLE OutputConfig
{
  /// \brief Default constructor
  public: OutputConfig();

  /// Mutable access to a singleton OutputConfig that serves as the global
  /// OutputConfig object for all parsing operations that do not specify their
  /// own OutputConfig.
  /// \return A mutable reference to the singleton OutputConfig object
  public: static OutputConfig &GlobalConfig();

  /// \brief Several DOM classes have ToElement() methods that return an
  /// XML Element populated from the contents of the DOM object. When
  /// populating the details of a model that was included using the
  /// <include> tag, one may wish to retain the exact include tag and
  /// URI instead of copying the full details of the included model.
  /// This method lets you set this behavior.
  /// \param[in] _useIncludeTag When true, the model's URI is used to create
  /// an SDF `<include>` rather than a `<model>`. The model's URI must be
  /// first set using the `Model::SetUri` function. If the model's URI is
  /// empty, then a `<model>` element will be generated. The default is true
  /// so that URI values are used when ToElement is called from a
  /// World object. Make sure to use `Model::SetUri` even when the model
  /// is loaded from an `<include>` tag since the parser will
  /// automatically expand an `<include>` element to a `<model>` element.
  public: void SetToElementUseIncludeTag(bool _useIncludeTag);

  /// \brief Get the policy value about whether <include> tags are
  /// reconstituted in ToElement() invocations.
  /// \return True if include tags are reconstituted, or false
  /// if the fully populated model is returned instead.
  public: [[nodiscard]] bool ToElementUseIncludeTag() const;

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
