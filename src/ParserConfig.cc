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

#include <optional>

#include "sdf/ParserConfig.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Types.hh"
#include "sdf/CustomInertiaCalcProperties.hh"

using namespace sdf;

class sdf::ParserConfig::Implementation
{
  public: ParserConfig::SchemeToPathMap uriPathMap;
  public: std::function<std::string(const std::string &)> findFileCB;

  /// \brief Indicates how warnings and errors are tolerated.
  /// Default is for warnings to be streamed via sdfwarn
  public: EnforcementPolicy warningsPolicy = EnforcementPolicy::WARN;

  /// \brief Policy indicating how unrecognized elements without an xmlns are
  /// treated.
  /// Default is to ignore them for compatibility with legacy behavior
  public: EnforcementPolicy unrecognizedElementsPolicy =
    EnforcementPolicy::WARN;

  /// \brief Policy indicating how deprecated elements are treated.
  /// Defaults to the value of `warningsPolicy`. It can be overriden by the user
  /// to behave behave differently than the `warningsPolicy`.
  public: std::optional<EnforcementPolicy> deprecatedElementsPolicy;

  /// \brief Configuration that is set for the CalculateInertial() function
  /// By default it is set to SAVE_CALCULATION to preserve the behavior of
  /// Root::Load() generating complete inertial information.
  public: ConfigureResolveAutoInertials resolveAutoInertialsConfig =
    ConfigureResolveAutoInertials::SAVE_CALCULATION;

  /// \brief Collection of custom model parsers.
  public: std::vector<CustomModelParser> customParsers;

  /// \brief Collection of custom model parsers.
  public: CustomInertiaCalculator customInertiaCalculator;

  /// \brief Flag to explicitly preserve fixed joints when
  /// reading the SDF/URDF file.
  public: bool preserveFixedJoint = false;

  /// \brief Flag to use <include> tags within ToElement methods instead of
  /// the fully included model.
  public: bool toElementUseIncludeTag = true;

  /// \brief Flag to expand URIs where possible store the resolved paths
  public: bool storeResolvedURIs = false;
};


/////////////////////////////////////////////////
ParserConfig::ParserConfig()
    : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
ParserConfig &ParserConfig::GlobalConfig()
{
  static auto *defaultConfig = new ParserConfig;
  return *defaultConfig;
}

/////////////////////////////////////////////////
const std::function<std::string(const std::string &)> &
ParserConfig::FindFileCallback() const
{
  return this->dataPtr->findFileCB;
}


/////////////////////////////////////////////////
void ParserConfig::SetFindCallback(
    std::function<std::string(const std::string &)> _cb)
{
  this->dataPtr->findFileCB = _cb;
}

/////////////////////////////////////////////////
const ParserConfig::SchemeToPathMap &ParserConfig::URIPathMap() const
{
  return this->dataPtr->uriPathMap;
}

/////////////////////////////////////////////////
void ParserConfig::AddURIPath(const std::string &_uri, const std::string &_path)
{
  // Split _path on colons.
  // Add each part of the colon separated path to the global URI map.
#ifdef _WIN32
  constexpr char multiplePathSeparator[] = ";";
#else
  constexpr char multiplePathSeparator[] = ":";
#endif

  for (const auto &part : sdf::split(_path, multiplePathSeparator))
  {
    // Only add valid paths
    if (!part.empty() && sdf::filesystem::is_directory(part))
    {
      this->dataPtr->uriPathMap[_uri].push_back(part);
    }
  }
}

/////////////////////////////////////////////////
void ParserConfig::SetWarningsPolicy(EnforcementPolicy policy)
{
  this->dataPtr->warningsPolicy = policy;
}

/////////////////////////////////////////////////
EnforcementPolicy ParserConfig::WarningsPolicy() const
{
  return this->dataPtr->warningsPolicy;
}

/////////////////////////////////////////////////
void ParserConfig::SetUnrecognizedElementsPolicy(EnforcementPolicy _policy)
{
  this->dataPtr->unrecognizedElementsPolicy = _policy;
}

/////////////////////////////////////////////////
EnforcementPolicy ParserConfig::UnrecognizedElementsPolicy() const
{
  return this->dataPtr->unrecognizedElementsPolicy;
}

/////////////////////////////////////////////////
void ParserConfig::SetDeprecatedElementsPolicy(EnforcementPolicy _policy)
{
  this->dataPtr->deprecatedElementsPolicy = _policy;
}

/////////////////////////////////////////////////
void ParserConfig::ResetDeprecatedElementsPolicy()
{
  this->dataPtr->deprecatedElementsPolicy.reset();
}

/////////////////////////////////////////////////
EnforcementPolicy ParserConfig::DeprecatedElementsPolicy() const
{
  return this->dataPtr->deprecatedElementsPolicy.value_or(
      this->dataPtr->warningsPolicy);
}

/////////////////////////////////////////////////
ConfigureResolveAutoInertials
  ParserConfig::CalculateInertialConfiguration() const
{
  return this->dataPtr->resolveAutoInertialsConfig;
}

/////////////////////////////////////////////////
void ParserConfig::SetCalculateInertialConfiguration(
  ConfigureResolveAutoInertials _configuration)
{
  this->dataPtr->resolveAutoInertialsConfig = _configuration;
}

/////////////////////////////////////////////////
void ParserConfig::RegisterCustomModelParser(CustomModelParser _modelParser)
{
  this->dataPtr->customParsers.push_back(_modelParser);
}

/////////////////////////////////////////////////
const std::vector<CustomModelParser> &ParserConfig::CustomModelParsers() const
{
  return this->dataPtr->customParsers;
}

/////////////////////////////////////////////////
void ParserConfig::RegisterCustomInertiaCalc(
    CustomInertiaCalculator _inertiaCalculator)
{
  this->dataPtr->customInertiaCalculator = _inertiaCalculator;
}

/////////////////////////////////////////////////
const CustomInertiaCalculator &ParserConfig::CustomInertiaCalc() const
{
  return this->dataPtr->customInertiaCalculator;
}

/////////////////////////////////////////////////
void ParserConfig::URDFSetPreserveFixedJoint(bool _preserveFixedJoint)
{
  this->dataPtr->preserveFixedJoint = _preserveFixedJoint;
}

/////////////////////////////////////////////////
bool ParserConfig::URDFPreserveFixedJoint() const
{
  return this->dataPtr->preserveFixedJoint;
}

/////////////////////////////////////////////////
void ParserConfig::SetStoreResovledURIs(bool _resolveURI)
{
  this->SetStoreResolvedURIs(_resolveURI);
}

/////////////////////////////////////////////////
void ParserConfig::SetStoreResolvedURIs(bool _resolveURI)
{
  this->dataPtr->storeResolvedURIs = _resolveURI;
}

/////////////////////////////////////////////////
bool ParserConfig::StoreResolvedURIs() const
{
  return this->dataPtr->storeResolvedURIs;
}
