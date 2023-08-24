/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <string>
#include <gz/math/Pose3.hh>
#include <gz/math/Inertial.hh>
#include "sdf/Collision.hh"
#include "sdf/Error.hh"
#include "sdf/Geometry.hh"
#include "sdf/parser.hh"
#include "sdf/Surface.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Collision::Implementation
{
  /// \brief Name of the collision.
  public: std::string name = "";

  /// \brief Pose of the collision object
  public: gz::math::Pose3d pose = gz::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The collision's geometry.
  public: Geometry geom;

  /// \brief The collision's surface parameters.
  public: sdf::Surface surface;

  /// \brief Density of the collision. Default is 1000.0
  public: double density{1000.0};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Scoped Pose Relative-To graph at the parent model scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;
};

/////////////////////////////////////////////////
Collision::Collision()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Collision::Load(ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Collision::Load(ElementPtr _sdf, const ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <collision>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "collision")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Collision, but the provided SDF element is not a "
        "<collision>."});
    return errors;
  }

  // Read the collisions's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A collision name is required, but the name is not set."});
  }

  // Check that the collision's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied collision name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  // Load the geometry
  Errors geomErr = this->dataPtr->geom.Load(
      _sdf->GetElement("geometry", errors), _config);
  errors.insert(errors.end(), geomErr.begin(), geomErr.end());

  // Load the surface parameters if they are given
  if (_sdf->HasElement("surface"))
  {
    this->dataPtr->surface.Load(_sdf->GetElement("surface", errors));
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Collision::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Collision::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
double Collision::Density() const
{
  return this->dataPtr->density;
}

/////////////////////////////////////////////////
void Collision::SetDensity(double _density)
{
  this->dataPtr->density = _density;
}

/////////////////////////////////////////////////
const Geometry *Collision::Geom() const
{
  return &this->dataPtr->geom;
}

/////////////////////////////////////////////////
void Collision::SetGeom(const Geometry &_geom)
{
  this->dataPtr->geom = _geom;
}

/////////////////////////////////////////////////
const sdf::Surface *Collision::Surface() const
{
  return &this->dataPtr->surface;
}

/////////////////////////////////////////////////
void Collision::SetSurface(const sdf::Surface &_surface)
{
  this->dataPtr->surface = _surface;
}

/////////////////////////////////////////////////
const gz::math::Pose3d &Collision::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Collision::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Collision::SetRawPose(const gz::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Collision::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Collision::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void Collision::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
sdf::SemanticPose Collision::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
Errors Collision::CalculateInertial(gz::math::Inertiald &_inertial,
                                    const ParserConfig &_config)
{
  Errors errors;

  // Set the density value for the collision material
  if (this->dataPtr->sdf->HasElement("density"))
  {
    this->dataPtr->density = this->dataPtr->sdf->Get<double>("density");
  }
  else
  {
    // If the density element is missing, let the user know that a default
    // value would be used according to the policy
    Error densityMissingErr(
      ErrorCode::ELEMENT_MISSING,
      "Collision is missing a <density> child element. "
      "Using a default density value of 1000.0 kg/m^3. "
    );
    enforceConfigurablePolicyCondition(
      _config.WarningsPolicy(), densityMissingErr, errors
    );
  }

  auto geomInertial =
    this->dataPtr->geom.CalculateInertial(this->dataPtr->density, _config, errors);

  if (!geomInertial)
  {
    errors.push_back({ErrorCode::LINK_INERTIA_INVALID,
        "Inertia Calculated for collision: " +
        this->dataPtr->name + " seems invalid."});
  }
  else
  {
    _inertial = geomInertial.value();

    // If geometry type is not mesh than calculate inertial pose in Link frame
    // considering collision frame to be same as inertial frame
    // In case of mesh the custom inertia calculator should return
    // the inertial object with the pose already set
    if (this->dataPtr->geom.Type() != GeometryType::MESH)
    {
      // If collision pose is in Link Frame then set that as inertial pose
      // Else resolve collision pose in Link Frame and then set as inertial pose
      if (this->dataPtr->poseRelativeTo.empty())
      {
        _inertial.SetPose(this->dataPtr->pose);
      }
      else
      {
        gz::math::Pose3d collisionPoseLinkFrame;
        Errors poseConvErrors =
          this->SemanticPose().Resolve(collisionPoseLinkFrame);
        errors.insert(errors.end(),
                      poseConvErrors.begin(),
                      poseConvErrors.end());
        _inertial.SetPose(collisionPoseLinkFrame);
      }
    }
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Collision::Element() const
{
  return this->dataPtr->sdf;
}

sdf::ElementPtr Collision::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Collision::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("collision.sdf", elem);

  elem->GetAttribute("name")->Set(this->Name(), _errors);

  // Set pose
  sdf::ElementPtr poseElem = elem->GetElement("pose", _errors);
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo, _errors);
  }
  poseElem->Set<gz::math::Pose3d>(_errors, this->RawPose());

  // Set the geometry
  elem->InsertElement(this->dataPtr->geom.ToElement(_errors), true);

  // Set the surface
  elem->InsertElement(this->dataPtr->surface.ToElement(_errors), true);

  return elem;
}
