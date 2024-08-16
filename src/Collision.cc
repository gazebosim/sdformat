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
#include "sdf/ParserConfig.hh"
#include "sdf/Element.hh"
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

  /// \brief Density of the collision if it has been set.
  public: std::optional<double> density;

  /// \brief SDF element pointer to <auto_inertia_params> tag
  public: sdf::ElementPtr autoInertiaParams{nullptr};

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

  // Load the density value if given
  if (_sdf->HasElement("density"))
  {
    this->dataPtr->density = _sdf->Get<double>("density");
  }

  // Load the auto_inertia_params element
  if (this->dataPtr->sdf->HasElement("auto_inertia_params"))
  {
    this->dataPtr->autoInertiaParams =
      this->dataPtr->sdf->GetElement("auto_inertia_params");
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
double Collision::DensityDefault()
{
  return 1000.0;
}

/////////////////////////////////////////////////
double Collision::Density() const
{
  if (this->dataPtr->density)
  {
    return *this->dataPtr->density;
  }
  return DensityDefault();
}

/////////////////////////////////////////////////
void Collision::SetDensity(double _density)
{
  this->dataPtr->density = _density;
}

/////////////////////////////////////////////////
sdf::ElementPtr Collision::AutoInertiaParams() const
{
  return this->dataPtr->autoInertiaParams;
}

/////////////////////////////////////////////////
void Collision::SetAutoInertiaParams(const sdf::ElementPtr _autoInertiaParams)
{
  this->dataPtr->autoInertiaParams = _autoInertiaParams;
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
void Collision::CalculateInertial(
  sdf::Errors &_errors,
  gz::math::Inertiald &_inertial,
  const ParserConfig &_config)
{
  this->CalculateInertial(
    _errors, _inertial, _config, std::nullopt, ElementPtr());
}

/////////////////////////////////////////////////
void Collision::CalculateInertial(
  sdf::Errors &_errors,
  gz::math::Inertiald &_inertial,
  const ParserConfig &_config,
  const std::optional<double> &_density,
  sdf::ElementPtr _autoInertiaParams)
{
  // Order of precedence for density:
  double density;
  // 1. Density explicitly set in this collision, either from the
  // `//collision/density` element or from Collision::SetDensity.
  if (this->dataPtr->density)
  {
    density = *this->dataPtr->density;
  }
  // 2. Density passed into this function, which likely comes from the
  // `//link/inertial/density` element or from Link::SetDensity.
  else if (_density)
  {
    density = *_density;
  }
  // 3. DensityDefault value.
  else
  {
    // If density was not explicitly set, send a warning
    // about the default value being used
    density = DensityDefault();
    Error densityMissingErr(
      ErrorCode::ELEMENT_MISSING,
      "Collision is missing a <density> child element. "
      "Using a default density value of " +
      std::to_string(DensityDefault()) + " kg/m^3. "
    );
    enforceConfigurablePolicyCondition(
      _config.WarningsPolicy(), densityMissingErr, _errors
    );
  }

  // If this Collision's auto inertia params have not been set, then use the
  // params passed into this function.
  sdf::ElementPtr autoInertiaParams = this->dataPtr->autoInertiaParams;
  if (!autoInertiaParams)
  {
    autoInertiaParams = _autoInertiaParams;
  }
  auto geomInertial =
    this->dataPtr->geom.CalculateInertial(_errors, _config,
      density, autoInertiaParams);

  if (!geomInertial)
  {
    _errors.push_back({ErrorCode::LINK_INERTIA_INVALID,
        "Inertia Calculated for collision: " +
        this->dataPtr->name + " is invalid."});
  }
  else
  {
    _inertial = geomInertial.value();

    // If collision pose is in Link Frame then set that as inertial pose
    // Else resolve collision pose in Link Frame and then set as inertial pose
    if (this->dataPtr->poseRelativeTo.empty())
    {
      _inertial.SetPose(this->dataPtr->pose * _inertial.Pose());
    }
    else
    {
      gz::math::Pose3d collisionPoseLinkFrame;
      Errors poseConvErrors =
        this->SemanticPose().Resolve(collisionPoseLinkFrame);
      _errors.insert(_errors.end(),
                    poseConvErrors.begin(),
                    poseConvErrors.end());
      _inertial.SetPose(collisionPoseLinkFrame * _inertial.Pose());
    }
  }
}

/////////////////////////////////////////////////
sdf::ElementPtr Collision::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
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
  sdf::initFile(std::string(this->SchemaFile()), elem);

  elem->GetAttribute("name")->Set(this->Name(), _errors);

  // Set pose
  sdf::ElementPtr poseElem = elem->GetElement("pose", _errors);
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo, _errors);
  }
  poseElem->Set<gz::math::Pose3d>(_errors, this->RawPose());

  // Set the density
  if (this->dataPtr->density.has_value())
  {
    sdf::ElementPtr densityElem = elem->GetElement("density", _errors);
    densityElem->Set<double>(this->Density());
  }

  // Set the geometry
  elem->InsertElement(this->dataPtr->geom.ToElement(_errors), true);

  // Set the surface
  elem->InsertElement(this->dataPtr->surface.ToElement(_errors), true);

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view Collision::SchemaFile()
{
    static const char kSchemaFile[] = "collision.sdf";
    return kSchemaFile;
}

