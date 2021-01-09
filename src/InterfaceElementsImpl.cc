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

#include "InterfaceElementsImpl.hh"
#include "sdf/SDFImpl.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
/////////////////////////////////////////////////
sdf::Errors loadInterfaceElements(sdf::ElementPtr _sdf,
    const sdf::ParserConfig &_config, std::vector<InterfaceModelPtr> &_models)
{
  sdf::Errors allErrors;
  for (auto includeElem = _sdf->GetElementImpl("include"); includeElem;
       includeElem = includeElem->GetNextElement("include"))
  {
    const auto uri = includeElem->Get<std::string>("uri");
    const auto localName = includeElem->Get<std::string>("name", "").first;
    const auto isStatic = includeElem->Get<bool>("static", false).first;
    const std::string modelPath = sdf::findFile(uri, true, true, _config);

    sdf::ElementPtr virtualCustomElements = includeElem->Clone();
    for (std::size_t i = 0; i < includeElem->GetElementDescriptionCount(); ++i)
    {
      auto elemDescr = includeElem->GetElementDescription(i);
      while (virtualCustomElements->HasElement(elemDescr->GetName()))
      {
        auto elemToRemove =
          virtualCustomElements->GetElement(elemDescr->GetName());
        virtualCustomElements->RemoveChild(elemToRemove);
      }
    }

    sdf::NestedInclude include;
    include.uri = uri;
    include.resolvedFileName = modelPath;
    include.localModelName = localName;
    include.isStatic = isStatic;
    include.virtualCustomElements = virtualCustomElements;
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
