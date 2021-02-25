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
#include <optional>

#include "InterfaceElementsImpl.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Types.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
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
sdf::Errors loadInterfaceElements(sdf::ElementPtr _sdf,
    const sdf::ParserConfig &_config, std::vector<InterfaceModelPtr> &_models)
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

    for (const auto &parser : _config.CustomModelParsers())
    {
      sdf::Errors errors;
      auto model = parser(include, errors);
      if (!errors.empty())
      {
        // If there are any errors, stop iterating through the custom parsers
        // and report the error
        allErrors.insert(allErrors.end(), errors.begin(), errors.end());
        break;
      }
      else if (nullptr != model)
      {
        // TODO: (addisu) Check for model name uniqueness
        _models.push_back(std::move(model));
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
