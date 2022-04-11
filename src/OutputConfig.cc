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

#include "sdf/OutputConfig.hh"
#include "sdf/Types.hh"

using namespace sdf;

class sdf::OutputConfig::Implementation
{
  /// \brief Flag to use <include> tags within ToElement methods instead of
  /// the fully included model.
  public: bool toElementUseIncludeTag = true;
};

/////////////////////////////////////////////////
OutputConfig::OutputConfig()
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
OutputConfig &OutputConfig::GlobalConfig()
{
  static auto *defaultConfig = new OutputConfig;
  return *defaultConfig;
}

/////////////////////////////////////////////////
void OutputConfig::SetToElementUseIncludeTag(bool _useIncludeTag)
{
  this->dataPtr->toElementUseIncludeTag = _useIncludeTag;
}

/////////////////////////////////////////////////
bool OutputConfig::ToElementUseIncludeTag() const
{
  return this->dataPtr->toElementUseIncludeTag;
}
