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
#include <algorithm>
#include <iterator>
#include <limits>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "sdf/Assert.hh"
#include "sdf/Error.hh"
#include "sdf/JointAxis.hh"
#include "sdf/parser.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::JointAxis::Implementation
{
  /// \brief Represents the x,y,z components of the axis unit vector.
  /// The axis is expressed in the joint frame unless the
  /// use_parent_model_frame flag is set to true. The vector should be
  /// normalized.
  public: gz::math::Vector3d xyz = gz::math::Vector3d::UnitZ;

  /// \brief Frame in which xyz is expressed in.
  public: std::string xyzExpressedIn = "";

  /// \brief The physical velocity dependent viscous damping coefficient
  /// of the joint.
  public: double damping = 0.0;

  /// \brief The physical static friction value of the joint.
  public: double friction = 0.0;

  /// \brief The spring reference position for this joint axis.
  public: double springReference = 0.0;

  /// \brief The spring stiffness for this joint axis.
  public: double springStiffness = 0.0;

  /// \brief Specifies the lower joint limit (radians for revolute joints,
  /// meters for prismatic joints). Omit if joint is continuous.
  public: double lower = -std::numeric_limits<double>::infinity();

  /// \brief Specifies the upper joint limit (radians for revolute joints,
  /// meters for prismatic joints). Omit if joint is continuous.
  public: double upper = std::numeric_limits<double>::infinity();

  /// \brief A value for enforcing the maximum joint effort applied.
  /// Limit is not enforced if value is negative.
  public: double effort = std::numeric_limits<double>::infinity();

  /// \brief A value for enforcing the maximum joint velocity.
  public: double maxVelocity = std::numeric_limits<double>::infinity();

  /// \brief Joint stop stiffness.
  public: double stiffness = 1e8;

  /// \brief Joint stop dissipation.
  public: double dissipation = 1.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Scoped Pose Relative-To graph at the parent model scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief Joint to be mimicked.
  public: std::optional<mimicJoint> mimic = std::nullopt;
};

/////////////////////////////////////////////////
JointAxis::JointAxis()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors JointAxis::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Read the xyz values.
  if (_sdf->HasElement("xyz"))
  {
    using gz::math::Vector3d;
    auto errs = this->SetXyz(_sdf->Get<Vector3d>("xyz",
        this->dataPtr->xyz).first);
    std::copy(errs.begin(), errs.end(), std::back_inserter(errors));
    auto e = _sdf->GetElement("xyz");
    if (e->HasAttribute("expressed_in"))
    {
      this->dataPtr->xyzExpressedIn = e->Get<std::string>("expressed_in");
    }
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The xyz element in joint axis is required"});
  }

  // Load dynamic values, if present
  if (_sdf->HasElement("dynamics"))
  {
    sdf::ElementPtr dynElement = _sdf->GetElement("dynamics");

    this->dataPtr->damping = dynElement->Get<double>("damping",
        this->dataPtr->damping).first;
    this->dataPtr->friction = dynElement->Get<double>("friction",
        this->dataPtr->friction).first;
    this->dataPtr->springReference = dynElement->Get<double>("spring_reference",
        this->dataPtr->springReference).first;
    this->dataPtr->springStiffness = dynElement->Get<double>("spring_stiffness",
        this->dataPtr->springStiffness).first;
  }

  // Load limit values
  if (_sdf->HasElement("limit"))
  {
    sdf::ElementPtr limitElement = _sdf->GetElement("limit");

    this->dataPtr->lower = limitElement->Get<double>("lower",
        this->dataPtr->lower).first;
    this->dataPtr->upper = limitElement->Get<double>("upper",
        this->dataPtr->upper).first;
    this->dataPtr->effort = limitElement->Get<double>("effort",
        this->dataPtr->effort).first;
    this->dataPtr->maxVelocity = limitElement->Get<double>("velocity",
        this->dataPtr->maxVelocity).first;
    this->dataPtr->stiffness = limitElement->Get<double>("stiffness",
        this->dataPtr->stiffness).first;
    this->dataPtr->dissipation = limitElement->Get<double>("dissipation",
        this->dataPtr->dissipation).first;
  }

  // Load mimic joint details if provided.
  auto mimicElement = _sdf->FindElement("mimic");
  if (mimicElement)
  {
    auto newMimicJoint = mimicJoint();
    newMimicJoint.joint = mimicElement->Get<std::string>("joint",
        "").first;
    newMimicJoint.multiplier = mimicElement->Get<double>("multiplier",
        0).first;
    newMimicJoint.offset = mimicElement->Get<double>("offset",
        0).first;
    newMimicJoint.reference = mimicElement->Get<double>("reference",
        0).first;

    this->dataPtr->mimic = std::make_optional(newMimicJoint);
  }

  return errors;
}

/////////////////////////////////////////////////
gz::math::Vector3d JointAxis::Xyz() const
{
  return this->dataPtr->xyz;
}

/////////////////////////////////////////////////
sdf::Errors JointAxis::SetXyz(const gz::math::Vector3d &_xyz)
{
  if (sdf::equal(_xyz.Length(), 0.0))
  {
    return {Error(ErrorCode::ELEMENT_INVALID,
                  "The norm of the xyz vector cannot be zero")};
  }
  this->dataPtr->xyz = _xyz;
  this->dataPtr->xyz.Normalize();
  return sdf::Errors();
}

/////////////////////////////////////////////////
void JointAxis::SetMimicJoint(const mimicJoint
    _mimicJoint)
{
  this->dataPtr->mimic = std::make_optional(_mimicJoint);
}

/////////////////////////////////////////////////
std::optional<mimicJoint> JointAxis::MimicJoint() const
{
  return this->dataPtr->mimic;
}

/////////////////////////////////////////////////
double JointAxis::Damping() const
{
  return this->dataPtr->damping;
}
/////////////////////////////////////////////////
void JointAxis::SetDamping(const double _damping)
{
  this->dataPtr->damping = _damping;
}

/////////////////////////////////////////////////
double JointAxis::Friction() const
{
  return this->dataPtr->friction;
}

/////////////////////////////////////////////////
void JointAxis::SetFriction(const double _friction)
{
  this->dataPtr->friction = _friction;
}

/////////////////////////////////////////////////
double JointAxis::SpringReference() const
{
  return this->dataPtr->springReference;
}

/////////////////////////////////////////////////
void JointAxis::SetSpringReference(const double _spring)
{
  this->dataPtr->springReference = _spring;
}

/////////////////////////////////////////////////
double JointAxis::SpringStiffness() const
{
  return this->dataPtr->springStiffness;
}

/////////////////////////////////////////////////
void JointAxis::SetSpringStiffness(const double _spring)
{
  this->dataPtr->springStiffness = _spring;
}

/////////////////////////////////////////////////
double JointAxis::Lower() const
{
  return this->dataPtr->lower;
}

/////////////////////////////////////////////////
void JointAxis::SetLower(const double _lower)
{
  this->dataPtr->lower = _lower;
}

/////////////////////////////////////////////////
double JointAxis::Upper() const
{
  return this->dataPtr->upper;
}

/////////////////////////////////////////////////
void JointAxis::SetUpper(const double _upper)
{
  this->dataPtr->upper = _upper;
}

/////////////////////////////////////////////////
double JointAxis::Effort() const
{
  return infiniteIfNegative(this->dataPtr->effort);
}

/////////////////////////////////////////////////
void JointAxis::SetEffort(double _effort)
{
  this->dataPtr->effort = _effort;
}

/////////////////////////////////////////////////
double JointAxis::MaxVelocity() const
{
  return infiniteIfNegative(this->dataPtr->maxVelocity);
}

/////////////////////////////////////////////////
void JointAxis::SetMaxVelocity(const double _velocity)
{
  this->dataPtr->maxVelocity = _velocity;
}

/////////////////////////////////////////////////
double JointAxis::Stiffness() const
{
  return this->dataPtr->stiffness;
}

/////////////////////////////////////////////////
void JointAxis::SetStiffness(const double _stiffness)
{
  this->dataPtr->stiffness = _stiffness;
}

/////////////////////////////////////////////////
double JointAxis::Dissipation() const
{
  return this->dataPtr->dissipation;
}

/////////////////////////////////////////////////
void JointAxis::SetDissipation(const double _dissipation)
{
  this->dataPtr->dissipation = _dissipation;
}

/////////////////////////////////////////////////
const std::string &JointAxis::XyzExpressedIn() const
{
  return this->dataPtr->xyzExpressedIn;
}

/////////////////////////////////////////////////
void JointAxis::SetXyzExpressedIn(const std::string &_frame)
{
  this->dataPtr->xyzExpressedIn = _frame;
}

/////////////////////////////////////////////////
void JointAxis::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void JointAxis::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
Errors JointAxis::ResolveXyz(
    gz::math::Vector3d &_xyz,
    const std::string &_resolveTo) const
{
  Errors errors;
  auto graph = this->dataPtr->poseRelativeToGraph;
  if (!graph)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "JointAxis has invalid pointer to PoseRelativeToGraph."});
    return errors;
  }
  if (this->dataPtr->xmlParentName.empty())
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "JointAxis has invalid name of xml parent object."});
    return errors;
  }

  // JointAxis is not in the graph, but its XyzExpressedIn() name should be.
  // If XyzExpressedIn() is empty, use the name of the xml parent object.
  std::string axisExpressedIn = this->XyzExpressedIn();
  if (axisExpressedIn.empty())
  {
    axisExpressedIn = this->dataPtr->xmlParentName;
  }

  std::string resolveTo = _resolveTo;
  if (resolveTo.empty())
  {
    resolveTo = this->dataPtr->xmlParentName;
  }

  gz::math::Pose3d pose;
  errors = resolvePose(pose, graph, axisExpressedIn, resolveTo);

  if (errors.empty())
  {
    _xyz = pose.Rot() * this->Xyz();
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr JointAxis::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
sdf::ElementPtr JointAxis::ToElement(unsigned int _index) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("joint.sdf", elem);

  std::string axisElemName = "axis";
  if (_index > 0u)
    axisElemName += std::to_string(_index + 1);
  sdf::ElementPtr axisElem = elem->GetElement(axisElemName);
  sdf::ElementPtr xyzElem = axisElem->GetElement("xyz");
  xyzElem->Set<gz::math::Vector3d>(this->Xyz());
  if (!this->XyzExpressedIn().empty())
  {
    xyzElem->GetAttribute("expressed_in")->Set<std::string>(
        this->XyzExpressedIn());
  }
  sdf::ElementPtr dynElem = axisElem->GetElement("dynamics");
  dynElem->GetElement("damping")->Set<double>(this->Damping());
  dynElem->GetElement("friction")->Set<double>(this->Friction());
  dynElem->GetElement("spring_reference")->Set<double>(
      this->SpringReference());
  dynElem->GetElement("spring_stiffness")->Set<double>(
      this->SpringStiffness());

  sdf::ElementPtr limitElem = axisElem->GetElement("limit");
  limitElem->GetElement("lower")->Set<double>(this->Lower());
  limitElem->GetElement("upper")->Set<double>(this->Upper());
  limitElem->GetElement("effort")->Set<double>(this->Effort());
  limitElem->GetElement("velocity")->Set<double>(this->MaxVelocity());
  limitElem->GetElement("stiffness")->Set<double>(this->Stiffness());
  limitElem->GetElement("dissipation")->Set<double>(this->Dissipation());

  if (this->dataPtr->mimic)
  {
    sdf::ElementPtr mimicElement = axisElem->GetElement("mimic");
    mimicElement->GetAttribute("joint")->SetFromString(
      this->dataPtr->mimic->joint);
    mimicElement->GetElement("multiplier")->Set<double>(
        this->dataPtr->mimic->multiplier);
    mimicElement->GetElement("offset")->Set<double>(
        this->dataPtr->mimic->offset);
    mimicElement->GetElement("reference")->Set<double>(
        this->dataPtr->mimic->reference);
  }

  return axisElem;
}
