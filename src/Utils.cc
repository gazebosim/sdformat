/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <string>
#include <utility>
#include "sdf/SDFImpl.hh"
#include "Utils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
bool isReservedName(const std::string &_name)
{
  const std::size_t size = _name.size();
  return _name == "world" ||
      (size >= 4 &&
       _name.compare(0, 2, "__") == 0 &&
       _name.compare(size-2, 2, "__") == 0);
}

/////////////////////////////////////////////////
bool loadName(sdf::ElementPtr _sdf, std::string &_name)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");

  _name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
bool loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
              std::string &_frame)
{
  sdf::ElementPtr sdf = _sdf;
  if (_sdf->GetName() != "pose")
  {
    if (_sdf->HasElement("pose"))
      sdf = _sdf->GetElement("pose");
    else
      return false;
  }

  // Read the frame. An empty frame implies the parent frame.
  std::pair<std::string, bool> framePair =
      sdf->Get<std::string>("relative_to", "");

  // Read the pose value.
  std::pair<ignition::math::Pose3d, bool> posePair =
    sdf->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero);

  // Set output, but only if the return value is true.
  if (posePair.second)
  {
    _pose = posePair.first;
    _frame = framePair.first;
  }

  // The frame attribute is optional, so only return true or false based
  // on the pose element value.
  return posePair.second;
}

/////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
double infiniteIfNegative(const double _value)
{
  if (_value < 0.0)
    return std::numeric_limits<double>::infinity();

  return _value;
}

/////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
bool isValidFrameReference(const std::string &_name)
{
  return "__root__" != _name;
}

/////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
void enforceConfigurablePolicyCondition(
  const sdf::EnforcementPolicy _policy,
  const std::string &_message,
  const ErrorCode _error,
  sdf::Errors &_errors)
{
  switch (_policy)
  {
    case EnforcementPolicy::ERR:
      _errors.push_back({_error, _message});
      break;
    case EnforcementPolicy::WARN:
      sdfwarn << _message;
      break;
    case EnforcementPolicy::LOG:
      sdfdbg << _message;
      break;
    default:
      throw std::runtime_error("Unhandled warning policy enum value");
  }
}

/////////////////////////////////////////////////
/// \brief Compute the absolute name of an entity by walking up the element
/// tree.
/// \param[in] _sdf sdf::ElementPtr of the entity with the name attribute
/// \param[out] _errors Will contain errors encountered in the function.
/// \return Absolute name of the entity of no errors occured. nullopt otherwise.
static std::optional<std::string> computeAbsoluteName(
    const sdf::ElementPtr &_sdf, sdf::Errors &_errors)
{
  std::vector<std::string> names;
  for (auto parent = _sdf;
       parent->GetName() != "world" && parent->GetName() != "sdf";
       parent = parent->GetParent())
  {
    if (parent->HasAttribute("name"))
    {
      names.push_back(parent->GetAttribute("name")->GetAsString());
    }
    else
    {
      _errors.emplace_back(sdf::ErrorCode::ATTRIBUTE_MISSING,
          "Name attribute missing from " + parent->GetName() + ".");
      return std::nullopt;
    }
  }
  if (names.size() > 0)
  {
    std::string absoluteParentName = names.back();
    auto it = names.rbegin();
    std::advance(it, 1);
    for (; it != names.rend(); ++it)
    {
      absoluteParentName.append(kSdfScopeDelimiter);
      absoluteParentName.append(*it);
    }

    return absoluteParentName;
  }

  return std::nullopt;
}

/////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
sdf::Errors loadIncludedInterfaceModels(sdf::ElementPtr _sdf,
    const sdf::ParserConfig &_config,
    std::vector<InterfaceModelWrapper> &_models)
{
  sdf::Errors allErrors;
  for (auto includeElem = _sdf->GetElementImpl("include"); includeElem;
       includeElem = includeElem->GetNextElement("include"))
  {
    sdf::NestedInclude include;
    include.uri = includeElem->Get<std::string>("uri");
    auto absoluteParentName = computeAbsoluteName(_sdf, allErrors);

    if (absoluteParentName.has_value())
    {
      include.absoluteParentName = *absoluteParentName;
    }

    include.localModelName = includeElem->Get<std::string>("name", "").first;
    if (includeElem->HasElement("static"))
    {
      include.isStatic = includeElem->Get<bool>("static");
    }
    include.resolvedFileName = sdf::findFile(include.uri, true, true, _config);

    sdf::ElementPtr virtualCustomElements = includeElem->Clone();
    for (unsigned int i = 0; i < includeElem->GetElementDescriptionCount(); ++i)
    {
      auto elemDescr = includeElem->GetElementDescription(i);
      while (virtualCustomElements->HasElement(elemDescr->GetName()))
      {
        auto elemToRemove =
          virtualCustomElements->GetElement(elemDescr->GetName());
        virtualCustomElements->RemoveChild(elemToRemove);
      }
    }

    include.virtualCustomElements = virtualCustomElements;
    if (includeElem->HasElement("pose"))
    {
      auto poseElem = includeElem->GetElement("pose");
      include.includeRawPose = poseElem->Get<ignition::math::Pose3d>();
      include.includePoseRelativeTo = poseElem->Get<std::string>("relative_to");
    }

    if (includeElem->HasElement("placement_frame"))
    {
      include.placementFrame = includeElem->Get<std::string>("placement_frame");
    }

    const auto &customParsers =  _config.CustomModelParsers();
    for (auto parserIt = customParsers.rbegin();
         parserIt != customParsers.rend(); ++parserIt)
    {
      sdf::Errors errors;
      auto model = (*parserIt)(include, errors);
      if (!errors.empty())
      {
        // If there are any errors, stop iterating through the custom parsers
        // and report the error
        allErrors.insert(allErrors.end(), errors.begin(), errors.end());
        break;
      }
      else if (nullptr != model)
      {
        if (model->Name() == "")
        {
          allErrors.emplace_back(sdf::ErrorCode::ATTRIBUTE_INVALID,
              "Missing name of custom model with URI [" + include.uri + "]");
        }
        else
        {
          InterfaceModelWrapper modelWrapper;
          modelWrapper.nestedInclude = include;
          modelWrapper.interfaceModel = model;
          // TODO: (addisu) Check for model name uniqueness
          _models.push_back(std::move(modelWrapper));
        }
        break;
      }
      // If there are no errors and model == nullptr, continue iterating through
      // the custom parsers.
    }
  }

  return allErrors;
}
}
}
