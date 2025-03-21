/*
 * Copyright 2020 Open Source Robotics Foundation
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

#ifndef SDF_PARSER_CONFIG_HH_
#define SDF_PARSER_CONFIG_HH_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <gz/utils/ImplPtr.hh>

#include "sdf/Error.hh"
#include "sdf/InterfaceElements.hh"
#include "sdf/CustomInertiaCalcProperties.hh"
#include "sdf/config.hh"
#include "sdf/system_util.hh"


namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
/// \brief Policy to describe how to treat certain conditions when parsing
enum class EnforcementPolicy
{
  /// \brief Policy is to treat condition as an error and fail parsing
  ERR,

  /// \brief Treat condition as a warning and issue to user
  WARN,

  /// \brief Ignore condition in favor of best effort parsing
  LOG,
};

/// \enum ConfigureResolveAutoInertials
/// \brief Configuration options of how CalculateInertial() function
/// would be used
enum class ConfigureResolveAutoInertials
{
  /// \brief If this value is used, CalculateInertial() won't be
  /// called from inside the Root::Load() function
  SKIP_CALCULATION_IN_LOAD,

  /// \brief If this values is used, CalculateInertial() would be
  /// called and the computed inertial values would be saved
  SAVE_CALCULATION,

  /// \brief If this values is used, CalculateInertial() would be
  /// called and the computed inertial values would be saved and
  /// written to the XML Element, allowing the calculated values
  /// to be printed with `gz sdf --print`.
  SAVE_CALCULATION_IN_ELEMENT,
};

/// \enum CalculateInertialFailurePolicyType
/// \brief Configuration options of how CalculateInertial() failures should
/// be handled.
enum class CalculateInertialFailurePolicyType
{
  /// \brief If this value is used, failures of Geometry::CalculateInertial()
  /// will result in a LINK_INERTIA_INVALID error with no inertial values
  /// written.
  ERR,

  /// \brief If this value is used, failures of Geometry::CalculateInertial()
  /// will result in default inertial values used and a WARNING.
  WARN_AND_USE_DEFAULT_INERTIAL,
};

// Forward declare private data class.
class ParserConfigPrivate;

/// This class contains configuration options for the libsdformat parser.
///
/// The configuration options include:
///   * A callback function used to locate a file from a given URI
///   * A map from URI schemes to search directories
///
/// For backward compatibility, the functions sdf::setFindCallback() and
/// sdf::addURIPath() update a singleton ParserConfig object, which can be
/// retrieved by ParserConfig::GlobalConfig().
///
/// The functions sdf::readFile(), sdf::readString(), and \ref Root::Load
/// "sdf::Root::Load()" have overloads that take a ParserConfig object. If
/// the ParserConfig object is omitted, these functions will use the singleton
/// ParserConfig object.
///
/// Example:
/// To set an additional URI scheme search directory without affecting the
/// global config,
///
/// \code{.cpp}
///   // Copy the global config
///   sdf::ParserConfig config = ParserConfig::GlobalConfig();
///
///   // Add the new scheme to the config
///   config.AddURIPath("newScheme://", "path/to/directory");
///
///   // Use the new config when loading a new SDFormat file
///   sdf::Root root;
///   root.Load("path/to/file.sdf", config);
/// \endcode
class SDFORMAT_VISIBLE ParserConfig
{
  /// type alias for the map from URI scheme to search directories
  public: using SchemeToPathMap =
          std::map<std::string, std::vector<std::string> >;

  /// \brief Default constructor
  public: ParserConfig();

  /// Mutable access to a singleton ParserConfig that serves as the global
  /// ParserConfig object for all parsing operations that do not specify their
  /// own ParserConfig.
  /// \return A mutable reference to the singleton ParserConfig object
  public: static ParserConfig &GlobalConfig();

  /// \brief Get the find file callback function
  /// \return Immutable reference to the find file callback function
  public: const std::function<std::string(const std::string &)> &
          FindFileCallback() const;

  /// \brief Set the callback to use when libsdformat can't find a file.
  /// The callback should return a complete path to the requested file, or
  /// an empty string if the file was not found in the callback. Generally, the
  /// input argument is a URI or a file path (absolute or relative) obtained
  /// from a `//include/uri` element. For example, if the value
  /// `custom://model_name` is given in a `//include/uri`, sdf::findFile() may
  /// invoke the callback with the argument `custom://model_name` if it is
  /// unable to find a file using the steps listed in sdf::findfile().
  ///
  /// Note, however, the input is not limited to URIs and file paths, and it is
  /// left up to the callback to interpret the contents of the input string.
  /// \param[in] _cb The callback function.
  /// \sa sdf::findFile() for the order of search operations
  public: void SetFindCallback(
              std::function<std::string(const std::string &)> _cb);

  /// \brief Get the URI scheme to search directories map
  /// \return Immutable reference to the URI scheme to search directories map
  public: const SchemeToPathMap &URIPathMap() const;

  /// \brief Associate paths to a URI.
  /// Example parameters: "model://", "/usr/share/models:~/.gazebo/models"
  /// \param[in] _uri URI that will be mapped to _path
  /// \param[in] _path Colon separated set of paths.
  /// \sa sdf::findFile() for the order of search operations
  public: void AddURIPath(const std::string &_uri, const std::string &_path);

  /// \brief Set the warning enforcement policy.
  /// \param[in] _policy policy enum value to set
  public: void SetWarningsPolicy(EnforcementPolicy _policy);

  /// \brief Get the current warning enforcement policy
  /// \return The warning enforcement policy enum value
  public: EnforcementPolicy WarningsPolicy() const;

  /// \brief Set the policy for unrecognized elements without an xmlns
  /// \param[in] _policy The unrecognized elements enforcement policy
  public: void SetUnrecognizedElementsPolicy(EnforcementPolicy _policy);

  /// \brief Get the current unrecognized elements policy
  /// \return The unrecognized elements policy enum value
  public: EnforcementPolicy UnrecognizedElementsPolicy() const;

  /// \brief Set the policy for deprecated elements.
  /// \param[in] _policy The deprecated elements enforcement policy
  public: void SetDeprecatedElementsPolicy(EnforcementPolicy _policy);

  /// \brief Resets the policy for deprecated elements so that it follows
  /// WarningsPolicy.
  public: void ResetDeprecatedElementsPolicy();

  /// \brief Get the current deprecated elements policy. By default, the policy
  /// is the same as the overall WarningsPolicy, but it can be overridden by
  /// SetDeprecatedElementsPolicy. Once it is overridden, changing
  /// SetWarningsPolicy will not change the value of DeprecatedElementsPolicy
  /// unless ResetDeprecatedElementsPolicy is called.
  /// \return The deprecated elements policy enum value
  public: EnforcementPolicy DeprecatedElementsPolicy() const;

  /// \brief Get the current configuration for the CalculateInertial()
  /// function
  /// \return Current set value of the ConfigureResolveAutoInertials enum
  public: ConfigureResolveAutoInertials CalculateInertialConfiguration() const;

  /// \brief Set the configuration for the CalculateInertial() function
  /// \param[in] _configuration The configuration to set for the
  /// CalculateInertial() function
  public: void SetCalculateInertialConfiguration(
    ConfigureResolveAutoInertials _configuration);

  /// \brief Get the current policy for handling failures of the
  /// CalculateInertial() function. By default an error is reported.
  /// \return Current set value of the CalculateInertialFailurePolicyType enum
  public: CalculateInertialFailurePolicyType
    CalculateInertialFailurePolicy() const;

  /// \brief Set the policy for handling failures of the CalculateInertial()
  /// function
  /// \param[in] _policy The policy to set for handling failures of the
  /// CalculateInertial() function
  public: void SetCalculateInertialFailurePolicy(
    CalculateInertialFailurePolicyType _policy);

  /// \brief Registers a custom model parser.
  /// \param[in] _modelParser Callback as described in
  /// sdf/InterfaceElements.hh.
  public: void RegisterCustomModelParser(CustomModelParser _modelParser);

  /// \brief Get the registered custom model parsers
  /// \return Vector of registered model parser callbacks.
  public: const std::vector<CustomModelParser> &CustomModelParsers() const;

  /// \brief Registers a custom Moment of Inertia Calculator for Meshes
  /// \param[in] _inertiaCalculator Callback with signature as described in
  /// sdf/CustomInertiaCalcProperties.hh.
  public: void RegisterCustomInertiaCalc(
      CustomInertiaCalculator _inertiaCalculator);

  /// \brief Get the registered custom mesh MOI Calculator
  /// \return registered mesh MOI Calculator.
  public: const CustomInertiaCalculator &CustomInertiaCalc() const;

  /// \brief Set the preserveFixedJoint flag.
  /// \param[in] _preserveFixedJoint True to preserve fixed joints, false to
  /// reduce the fixed joints and merge the child link into the parent.
  public: void URDFSetPreserveFixedJoint(bool _preserveFixedJoint);

  /// \brief Get the preserveFixedJoint flag value.
  /// \return True to preserve fixed joints, false to reduce the fixed joints
  /// and merge the child link into the parent.
  public: bool URDFPreserveFixedJoint() const;

  /// \brief Set the storeResolvedURIs flag value.
  /// \sa SetStoreResolvedURIs
  public: GZ_DEPRECATED(15) void SetStoreResovledURIs(bool _resolveURI);

  /// \brief Set the storeResolvedURIs flag value.
  /// \param[in] _resolveURI True to make the parser attempt to resolve any
  /// URIs found and store them.  False to preserve original URIs
  ///
  /// The Parser will use the FindFileCallback provided to attempt to resolve
  /// URIs in the Mesh, Material, Heightmap, and Skybox DOM objects
  /// If the FindFileCallback provides a non-empty string, the URI will be
  /// stored in the DOM object, and the original (unresolved) URI will be
  /// stored in the underlying Element.
  public: void SetStoreResolvedURIs(bool _resolveURI);

  /// \brief Get the storeResolvedURIs flag value.
  /// \return True if the parser will attempt to resolve any URIs found and
  /// store them.  False to preserve original URIs
  public: bool StoreResolvedURIs() const;

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
