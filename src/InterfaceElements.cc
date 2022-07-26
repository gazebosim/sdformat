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

#include <gz/utils/SuppressWarning.hh>
#include "sdf/InterfaceElements.hh"

using namespace sdf;

class sdf::NestedInclude::Implementation
{
  /// \brief Whether the included model should be merged as specified in
  /// //include/[@merge]
  /// This is nullopt if `//include/[@merge]` is is not set.
  public: std::optional<bool> isMerge;
};

GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
/////////////////////////////////////////////////
NestedInclude::NestedInclude()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
const std::string &NestedInclude::Uri() const
{
  return this->uri;
}

/////////////////////////////////////////////////
void NestedInclude::SetUri(const std::string &_uri)
{
  this->uri = _uri;
}

/////////////////////////////////////////////////
const std::string &NestedInclude::ResolvedFileName() const
{
  return this->resolvedFileName;
}

/////////////////////////////////////////////////
void NestedInclude::SetResolvedFileName(const std::string &_resolvedFileName)
{
  this->resolvedFileName = _resolvedFileName;
}

/////////////////////////////////////////////////
const std::string &NestedInclude::AbsoluteParentName() const
{
  return this->absoluteParentName;
}

/////////////////////////////////////////////////
void NestedInclude::SetAbsoluteParentName(
    const std::string &_absoluteParentName)
{
  this->absoluteParentName = _absoluteParentName;
}

/////////////////////////////////////////////////
const std::optional<std::string> &NestedInclude::LocalModelName() const
{
  return this->localModelName;
}

/////////////////////////////////////////////////
void NestedInclude::SetLocalModelName(const std::string &_localModelName)
{
  this->localModelName = _localModelName;
}

/////////////////////////////////////////////////
const std::optional<bool> &NestedInclude::IsStatic() const
{
  return this->isStatic;

}

/////////////////////////////////////////////////
void NestedInclude::SetIsStatic(bool _isStatic)
{
  this->isStatic = _isStatic;
}

/////////////////////////////////////////////////
const std::optional<gz::math::Pose3d> &NestedInclude::IncludeRawPose()
    const
{
  return this->includeRawPose;
}

/////////////////////////////////////////////////
void NestedInclude::SetIncludeRawPose(
    const gz::math::Pose3d &_includeRawPose)
{
  this->includeRawPose = _includeRawPose;
}

/////////////////////////////////////////////////
const std::optional<std::string> &NestedInclude::IncludePoseRelativeTo() const
{
  return this->includePoseRelativeTo;
}

/////////////////////////////////////////////////
void NestedInclude::SetIncludePoseRelativeTo(
    const std::string &_includePoseRelativeTo)
{
  this->includePoseRelativeTo = _includePoseRelativeTo;
}

/////////////////////////////////////////////////
const std::optional<std::string> &NestedInclude::PlacementFrame() const
{
  return this->placementFrame;
}

/////////////////////////////////////////////////
void NestedInclude::SetPlacementFrame(const std::string &_placementFrame)
{
  this->placementFrame = _placementFrame;
}

/////////////////////////////////////////////////
sdf::ElementPtr NestedInclude::IncludeElement() const
{
  return this->includeElement;
}

/////////////////////////////////////////////////
void NestedInclude::SetIncludeElement(sdf::ElementPtr _includeElement)
{
  this->includeElement = _includeElement;
}
GZ_UTILS_WARN_RESUME__DEPRECATED_DECLARATION

/////////////////////////////////////////////////
void NestedInclude::SetIsMerge(bool _isMerge)
{
  this->dataPtr->isMerge = _isMerge;
}

/////////////////////////////////////////////////
const std::optional<bool> &NestedInclude::IsMerge() const
{
  return this->dataPtr->isMerge;
}
