/*
 * Copyright 2023 Open Source Robotics Foundation
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
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sdf/Error.hh"
#include "sdf/parser.hh"
#include "sdf/Projector.hh"
#include "sdf/Types.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Projector::Implementation
{
  /// \brief Name of the projector
  public: std::string name;

  /// \brief Near clip plane
  public: double nearClip{0.1};

  /// \brief far clip plane
  public: double farClip{10.0};

  /// \brief Visibility flags
  public: uint32_t visibilityFlags{UINT32_MAX};

  /// \brief Horizontal field of view
  public: gz::math::Angle hfov{0.785};

  /// \brief Texture used by the projector
  public: std::string texture;

  /// \brief Pose of the projector
  public: gz::math::Pose3d pose = gz::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo;

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName = "";

  /// \brief The path to the file where this projector was defined.
  public: std::string filePath = "";

  /// \brief Sensor plugins.
  public: std::vector<Plugin> plugins;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Projector::Projector()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Projector::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  this->dataPtr->filePath = _sdf->FilePath();

  // Check that the provided SDF element is a <projector>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "projector")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a projector, but the provided SDF element "
        "is not a <projector>."});
    return errors;
  }

  // Read the projector's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A projector name is required, but the name is not set."});
  }

  // Check that the projector's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
        "The supplied projector name [" + this->dataPtr->name +
        "] is reserved."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  this->dataPtr->nearClip = _sdf->Get<double>("near_clip",
      this->dataPtr->nearClip).first;

  this->dataPtr->farClip = _sdf->Get<double>("far_clip",
      this->dataPtr->farClip).first;

   double fov = _sdf->Get<double>("fov",
      this->dataPtr->hfov.Radian()).first;
  this->dataPtr->hfov = gz::math::Angle(fov);

  this->dataPtr->visibilityFlags = _sdf->Get<uint32_t>("visibility_flags",
      this->dataPtr->visibilityFlags).first;

  this->dataPtr->texture = _sdf->Get<std::string>("texture",
      this->dataPtr->texture).first;
  if (this->dataPtr->texture == "__default__")
    this->dataPtr->texture = "";

  // Load the projector plugins
  Errors pluginErrors = loadRepeated<Plugin>(_sdf, "plugin",
    this->dataPtr->plugins);
  errors.insert(errors.end(), pluginErrors.begin(), pluginErrors.end());

  return errors;
}

/////////////////////////////////////////////////
std::string Projector::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Projector::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const gz::math::Pose3d &Projector::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Projector::SetRawPose(const gz::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
const std::string &Projector::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Projector::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
sdf::SemanticPose Projector::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
sdf::ElementPtr Projector::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
const std::string &Projector::FilePath() const
{
  return this->dataPtr->filePath;
}

//////////////////////////////////////////////////
void Projector::SetFilePath(const std::string &_filePath)
{
  this->dataPtr->filePath = _filePath;
}

/////////////////////////////////////////////////
void Projector::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void Projector::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}


//////////////////////////////////////////////////
double Projector::NearClip() const
{
  return this->dataPtr->nearClip;
}

//////////////////////////////////////////////////
void Projector::SetNearClip(double _near)
{
  this->dataPtr->nearClip = _near;
}

//////////////////////////////////////////////////
double Projector::FarClip() const
{
  return this->dataPtr->farClip;
}

//////////////////////////////////////////////////
void Projector::SetFarClip(double _far)
{
  this->dataPtr->farClip = _far;
}

/////////////////////////////////////////////////
gz::math::Angle Projector::HorizontalFov() const
{
  return this->dataPtr->hfov;
}

/////////////////////////////////////////////////
void Projector::SetHorizontalFov(const gz::math::Angle &_hfov)
{
  this->dataPtr->hfov = _hfov;
}

/////////////////////////////////////////////////
uint32_t Projector::VisibilityFlags() const
{
  return this->dataPtr->visibilityFlags;
}

/////////////////////////////////////////////////
void Projector::SetVisibilityFlags(uint32_t _flags)
{
  this->dataPtr->visibilityFlags = _flags;
}

//////////////////////////////////////////////////
std::string Projector::Texture() const
{
  return this->dataPtr->texture;
}

//////////////////////////////////////////////////
void Projector::SetTexture(const std::string &_texture)
{
  this->dataPtr->texture = _texture;
}

/////////////////////////////////////////////////
const sdf::Plugins &Projector::Plugins() const
{
  return this->dataPtr->plugins;
}

/////////////////////////////////////////////////
sdf::Plugins &Projector::Plugins()
{
  return this->dataPtr->plugins;
}

/////////////////////////////////////////////////
void Projector::ClearPlugins()
{
  this->dataPtr->plugins.clear();
}

/////////////////////////////////////////////////
void Projector::AddPlugin(const Plugin &_plugin)
{
  this->dataPtr->plugins.push_back(_plugin);
}

/////////////////////////////////////////////////
sdf::ElementPtr Projector::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("projector.sdf", elem);

  // Set pose
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<gz::math::Pose3d>(this->RawPose());

  elem->GetAttribute("name")->Set(this->Name());
  elem->GetElement("near_clip")->Set(this->NearClip());
  elem->GetElement("far_clip")->Set(this->FarClip());
  elem->GetElement("fov")->Set(this->HorizontalFov());
  elem->GetElement("texture")->Set(this->Texture());
  elem->GetElement("visibility_flags")->Set(this->VisibilityFlags());

  // Add in the plugins
  for (const Plugin &plugin : this->dataPtr->plugins)
    elem->InsertElement(plugin.ToElement(), true);

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view Projector::SchemaFile() 
{
    static char kSchemaFile[] = "projector.sdf";
    return kSchemaFile;
}

