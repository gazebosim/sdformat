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
#include <ignition/math/Vector3.hh>
#include "sdf/Error.hh"
#include "sdf/JointAxis.hh"

using namespace sdf;

class sdf::JointAxisPrivate
{
  /// \brief Default joint position for this joint axis.
  public: double initialPosition = 0.0;

  /// \brief Represents the x,y,z components of the axis unit vector.
  /// The axis is expressed in the joint frame unless the
  /// use_parent_model_frame flag is set to true. The vector should be
  /// normalized.
  public: ignition::math::Vector3d xyz = ignition::math::Vector3d::UnitZ;

  /// \brief Flag to interpret the axis xyz element in the parent model
  /// frame instead of joint frame.
  public: bool useParentModelFrame = false;

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
  public: double lower = -1e16;

  /// \brief Specifies the upper joint limit (radians for revolute joints,
  /// meters for prismatic joints). Omit if joint is continuous.
  public: double upper = 1e16;

  /// \brief A value for enforcing the maximum joint effort applied.
  /// Limit is not enforced if value is negative.
  public: double effort = -1;

  /// \brief A value for enforcing the maximum joint velocity.
  public: double maxVelocity = -1;

  /// \brief Joint stop stiffness.
  public: double stiffness = 1e8;

  /// \brief Joint stop dissipation.
  public: double dissipation = 1.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
JointAxis::JointAxis()
  : dataPtr(new JointAxisPrivate)
{
}

/////////////////////////////////////////////////
JointAxis::JointAxis(const JointAxis &_jointAxis)
  : dataPtr(new JointAxisPrivate(*_jointAxis.dataPtr))
{
}

/////////////////////////////////////////////////
JointAxis &JointAxis::operator=(const JointAxis &_jointAxis)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new JointAxisPrivate;
  }
  *this->dataPtr = (*_jointAxis.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
JointAxis::JointAxis(JointAxis &&_jointAxis) noexcept
{
  this->dataPtr = _jointAxis.dataPtr;
  _jointAxis.dataPtr = nullptr;
}

/////////////////////////////////////////////////
JointAxis &JointAxis::operator=(JointAxis &&_jointAxis)
{
  this->dataPtr = _jointAxis.dataPtr;
  _jointAxis.dataPtr = nullptr;
  return *this;
}

/////////////////////////////////////////////////
JointAxis::~JointAxis()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors JointAxis::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Read the initial position. This is optional, with a default value of 0.
  this->dataPtr->initialPosition = _sdf->Get<double>(
      "initial_position", 0.0).first;

  // Read the xyz values.
  if (_sdf->HasElement("xyz"))
  {
    this->dataPtr->xyz = _sdf->Get<ignition::math::Vector3d>("xyz",
        ignition::math::Vector3d::UnitZ).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The xyz element in joint axis is required"});
  }

  // Get whether to use the parent model frame.
  this->dataPtr->useParentModelFrame = _sdf->Get<bool>(
      "use_parent_model_frame", false).first;

  // Load dynamic values, if present
  if (_sdf->HasElement("dynamics"))
  {
    sdf::ElementPtr dynElement = _sdf->GetElement("dynamics");

    this->dataPtr->damping = dynElement->Get<double>("damping", 0.0).first;
    this->dataPtr->friction = dynElement->Get<double>("friction", 0.0).first;
    this->dataPtr->springReference =
      dynElement->Get<double>("spring_reference", 0.0).first;
    this->dataPtr->springStiffness =
      dynElement->Get<double>("spring_stiffness", 0.0).first;
  }

  // Load limit values
  if (_sdf->HasElement("limit"))
  {
    sdf::ElementPtr limitElement = _sdf->GetElement("limit");

    this->dataPtr->lower = limitElement->Get<double>("lower", -1e16).first;
    this->dataPtr->upper = limitElement->Get<double>("upper", 1e16).first;
    this->dataPtr->effort = limitElement->Get<double>("effort", -1).first;
    this->dataPtr->maxVelocity = limitElement->Get<double>(
        "velocity", -1).first;
    this->dataPtr->stiffness = limitElement->Get<double>(
        "stiffness", 1e8).first;
    this->dataPtr->dissipation = limitElement->Get<double>(
        "dissipation", 1.0).first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "A limit element is a required child of a joint axis"});
  }

  return errors;
}

/////////////////////////////////////////////////
double JointAxis::InitialPosition() const
{
  return this->dataPtr->initialPosition;
}
/////////////////////////////////////////////////
void JointAxis::SetInitialPosition(const double _pos)
{
  this->dataPtr->initialPosition = _pos;
}

/////////////////////////////////////////////////
ignition::math::Vector3d JointAxis::Xyz() const
{
  return this->dataPtr->xyz;
}

/////////////////////////////////////////////////
void JointAxis::SetXyz(const ignition::math::Vector3d &_xyz)
{
  this->dataPtr->xyz = _xyz;
}

/////////////////////////////////////////////////
bool JointAxis::UseParentModelFrame() const
{
  return this->dataPtr->useParentModelFrame;
}
/////////////////////////////////////////////////
void JointAxis::SetUseParentModelFrame(const bool _parentModelFrame)
{
  this->dataPtr->useParentModelFrame = _parentModelFrame;
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
void JointAxis::SetUpper(const double _upper) const
{
  this->dataPtr->upper = _upper;
}

/////////////////////////////////////////////////
double JointAxis::Effort() const
{
  return this->dataPtr->effort;
}

/////////////////////////////////////////////////
void JointAxis::SetEffort(double _effort)
{
  this->dataPtr->effort = _effort;
}

/////////////////////////////////////////////////
double JointAxis::MaxVelocity() const
{
  return this->dataPtr->maxVelocity;
}

/////////////////////////////////////////////////
void JointAxis::SetMaxVelocity(const double _velocity) const
{
  this->dataPtr->maxVelocity = _velocity;
}

/////////////////////////////////////////////////
double JointAxis::Stiffness() const
{
  return this->dataPtr->stiffness;
}

/////////////////////////////////////////////////
void JointAxis::SetStiffness(const double _stiffness) const
{
  this->dataPtr->stiffness = _stiffness;
}

/////////////////////////////////////////////////
double JointAxis::Dissipation() const
{
  return this->dataPtr->dissipation;
}

/////////////////////////////////////////////////
void JointAxis::SetDissipation(const double _dissipation) const
{
  this->dataPtr->dissipation = _dissipation;
}

/////////////////////////////////////////////////
sdf::ElementPtr JointAxis::Element() const
{
  return this->dataPtr->sdf;
}
