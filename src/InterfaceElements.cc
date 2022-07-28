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

  /// \brief Provides the URI as specified in `//include/uri`. This may or may
  /// not end with a file extension (it will not end with an extension if it
  /// refers to a model package).
  public: std::string uri;

  /// \brief Provides the *resolved* absolute file path from the URI.
  /// It is recommended to use this in `CustomModelParser` when checking
  /// predicates on filenames -- however, the predicates should generally only
  /// check the file extension.
  public: std::string resolvedFileName;

  /// \brief Name of the parent entity in absolute hierarchy.
  /// Example: if the interface model's name is
  /// `top_model::middle_model::my_new_model`, the absoluteParentName would be
  /// `top_model::middle_model`. If the parent entity is the world, this would
  /// be an empty string.
  public: std::string absoluteParentName;

  /// \brief Name relative to immediate parent as specified in
  /// `//include/name`. This is nullopt if `//include/name` is not set. Then the
  /// name of the model must be determined by the custom model parser from the
  /// included model file.
  /// Example: `my_new_model`
  public: std::optional<std::string> localModelName;

  /// \brief Whether the model is static as defined by `//include/static`. This
  /// is nullopt if `//include/static` is not set.
  public: std::optional<bool> isStatic;

  /// \brief The raw pose as specified in //include/pose. This is nullopt if
  /// `//include/pose` is not set.
  public: std::optional<gz::math::Pose3d> includeRawPose;

  /// \brief The relative-to frame of the pose as specified in
  /// `//include/pose/@relative_to`. This is nullopt if
  /// `//include/pose/@relative_to` is not set.
  public: std::optional<std::string> includePoseRelativeTo;

  /// \brief The placement frame as specified in `//include/placement_frame`.
  /// This is nullopt if `//include/placement_frame` is is not set.
  public: std::optional<std::string> placementFrame;

  /// This is the `//include` element. This can be used to pass custom elements
  /// and attributes to the custom model parser.
  public: sdf::ElementPtr includeElement;
};

/////////////////////////////////////////////////
NestedInclude::NestedInclude()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
const std::string &NestedInclude::Uri() const
{
  return this->dataPtr->uri;
}

/////////////////////////////////////////////////
void NestedInclude::SetUri(const std::string &_uri)
{
  this->dataPtr->uri = _uri;
}

/////////////////////////////////////////////////
const std::string &NestedInclude::ResolvedFileName() const
{
  return this->dataPtr->resolvedFileName;
}

/////////////////////////////////////////////////
void NestedInclude::SetResolvedFileName(const std::string &_resolvedFileName)
{
  this->dataPtr->resolvedFileName = _resolvedFileName;
}

/////////////////////////////////////////////////
const std::string &NestedInclude::AbsoluteParentName() const
{
  return this->dataPtr->absoluteParentName;
}

/////////////////////////////////////////////////
void NestedInclude::SetAbsoluteParentName(
    const std::string &_absoluteParentName)
{
  this->dataPtr->absoluteParentName = _absoluteParentName;
}

/////////////////////////////////////////////////
const std::optional<std::string> &NestedInclude::LocalModelName() const
{
  return this->dataPtr->localModelName;
}

/////////////////////////////////////////////////
void NestedInclude::SetLocalModelName(const std::string &_localModelName)
{
  this->dataPtr->localModelName = _localModelName;
}

/////////////////////////////////////////////////
const std::optional<bool> &NestedInclude::IsStatic() const
{
  return this->dataPtr->isStatic;

}

/////////////////////////////////////////////////
void NestedInclude::SetIsStatic(bool _isStatic)
{
  this->dataPtr->isStatic = _isStatic;
}

/////////////////////////////////////////////////
const std::optional<gz::math::Pose3d> &NestedInclude::IncludeRawPose()
    const
{
  return this->dataPtr->includeRawPose;
}

/////////////////////////////////////////////////
void NestedInclude::SetIncludeRawPose(
    const gz::math::Pose3d &_includeRawPose)
{
  this->dataPtr->includeRawPose = _includeRawPose;
}

/////////////////////////////////////////////////
const std::optional<std::string> &NestedInclude::IncludePoseRelativeTo() const
{
  return this->dataPtr->includePoseRelativeTo;
}

/////////////////////////////////////////////////
void NestedInclude::SetIncludePoseRelativeTo(
    const std::string &_includePoseRelativeTo)
{
  this->dataPtr->includePoseRelativeTo = _includePoseRelativeTo;
}

/////////////////////////////////////////////////
const std::optional<std::string> &NestedInclude::PlacementFrame() const
{
  return this->dataPtr->placementFrame;
}

/////////////////////////////////////////////////
void NestedInclude::SetPlacementFrame(const std::string &_placementFrame)
{
  this->dataPtr->placementFrame = _placementFrame;
}

/////////////////////////////////////////////////
sdf::ElementPtr NestedInclude::IncludeElement() const
{
  return this->dataPtr->includeElement;
}

/////////////////////////////////////////////////
void NestedInclude::SetIncludeElement(sdf::ElementPtr _includeElement)
{
  this->dataPtr->includeElement = _includeElement;
}

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
