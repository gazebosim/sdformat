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

#include "sdf/Element.hh"
#include "sdf/parser.hh"
#include "sdf/Surface.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Contact::Implementation
{
  // \brief The bitmask used to filter collisions.
  public: uint16_t collideBitmask = 0xff;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

class sdf::ODE::Implementation
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};

  /// \brief Coefficient of friction in first friction pyramid direction,
  /// the unitless maximum ratio of force in first friction pyramid
  /// direction to normal force.
  public: double mu = 1.0;

  /// \brief Coefficient of friction in second friction pyramid direction,
  /// the unitless maximum ratio of force in second friction pyramid
  /// direction to normal force.
  public: double mu2 = 1.0;

  /// \brief Unit vector specifying first friction pyramid direction in
  /// collision-fixed reference frame.
  public: gz::math::Vector3d fdir1 = {0, 0, 0};

  /// \brief Force dependent slip in first friction pyramid direction,
  /// equivalent to inverse of viscous damping coefficient
  /// with units of m/s/N.
  public: double slip1 = 0.0;

  /// \brief Force dependent slip in second friction pyramid direction,
  /// equivalent to inverse of viscous damping coefficient
  /// with units of m/s/N.
  public: double slip2 = 0.0;
};

class sdf::Friction::Implementation
{
  /// \brief The object storing contact parameters
  public: sdf::ODE ode;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

class sdf::Surface::Implementation
{
  /// \brief The object storing contact parameters
  public: sdf::Friction friction;

  /// \brief The object storing contact parameters
  public: sdf::Contact contact;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};


/////////////////////////////////////////////////
ODE::ODE()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors ODE::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a ODE, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <ode>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "ode")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a ODE, but the provided SDF element is not a "
        "<ode>."});
    return errors;
  }

  this->dataPtr->mu = _sdf->Get<double>(errors, "mu", this->dataPtr->mu).first;
  this->dataPtr->mu2 = _sdf->Get<double>(
      errors, "mu2", this->dataPtr->mu2).first;
  this->dataPtr->slip1 = _sdf->Get<double>(
      errors, "slip1", this->dataPtr->slip1).first;
  this->dataPtr->slip2 = _sdf->Get<double>(
      errors, "slip2", this->dataPtr->slip2).first;
  this->dataPtr->fdir1 = _sdf->Get<gz::math::Vector3d>(errors, "fdir1",
        this->dataPtr->fdir1).first;

  return errors;
}

/////////////////////////////////////////////////
double ODE::Mu() const
{
  return this->dataPtr->mu;
}

/////////////////////////////////////////////////
void ODE::SetMu(double _mu)
{
  this->dataPtr->mu = _mu;
}

/////////////////////////////////////////////////
double ODE::Mu2() const
{
  return this->dataPtr->mu2;
}

/////////////////////////////////////////////////
void ODE::SetMu2(double _mu2)
{
  this->dataPtr->mu2 = _mu2;
}

/////////////////////////////////////////////////
const gz::math::Vector3d &ODE::Fdir1() const
{
  return this->dataPtr->fdir1;
}

/////////////////////////////////////////////////
void ODE::SetFdir1(const gz::math::Vector3d &_fdir)
{
  this->dataPtr->fdir1 = _fdir;
}

/////////////////////////////////////////////////
double ODE::Slip1() const
{
  return this->dataPtr->slip1;
}

/////////////////////////////////////////////////
void ODE::SetSlip1(double _slip1)
{
  this->dataPtr->slip1 = _slip1;
}

/////////////////////////////////////////////////
double ODE::Slip2() const
{
  return this->dataPtr->slip2;
}

/////////////////////////////////////////////////
void ODE::SetSlip2(double _slip2)
{
  this->dataPtr->slip2 = _slip2;
}

/////////////////////////////////////////////////
sdf::ElementPtr ODE::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
Friction::Friction()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Friction::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Friction, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <friction>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "friction")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Friction, but the provided SDF element is not a "
        "<friction>."});
    return errors;
  }

  if (_sdf->HasElement("ode"))
  {
    Errors err = this->dataPtr->ode.Load(_sdf->GetElement("ode", errors));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Friction::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
void Friction::SetODE(const sdf::ODE &_ode)
{
  this->dataPtr->ode = _ode;
}

/////////////////////////////////////////////////
const sdf::ODE *Friction::ODE() const
{
  return &this->dataPtr->ode;
}

/////////////////////////////////////////////////
Contact::Contact()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Contact::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Contact, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <contact>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "contact")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Contact, but the provided SDF element is not a "
        "<contact>."});
    return errors;
  }

  if (_sdf->HasElement("collide_bitmask"))
  {
    this->dataPtr->collideBitmask =
        static_cast<uint16_t>(_sdf->Get<unsigned int>(
        errors, "collide_bitmask"));
  }

  // \todo(nkoenig) Parse the remaining collide properties.
  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Contact::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
uint16_t Contact::CollideBitmask() const
{
  return this->dataPtr->collideBitmask;
}

/////////////////////////////////////////////////
void Contact::SetCollideBitmask(const uint16_t _bitmask)
{
  this->dataPtr->collideBitmask = _bitmask;
}

/////////////////////////////////////////////////
Surface::Surface()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Surface::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Surface, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <surface>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "surface")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Surface, but the provided SDF element is not a "
        "<surface>."});
    return errors;
  }

  if (_sdf->HasElement("contact"))
  {
    Errors err = this->dataPtr->contact.Load(
        _sdf->GetElement("contact", errors));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  if (_sdf->HasElement("friction"))
  {
    Errors err = this->dataPtr->friction.Load(
        _sdf->GetElement("friction", errors));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  // \todo(nkoenig) Parse the remaining surface properties.
  return errors;
}
/////////////////////////////////////////////////
sdf::ElementPtr Surface::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const sdf::Contact *Surface::Contact() const
{
  return &this->dataPtr->contact;
}

/////////////////////////////////////////////////
void Surface::SetFriction(const sdf::Friction &_friction)
{
  this->dataPtr->friction = _friction;
}

/////////////////////////////////////////////////
const sdf::Friction *Surface::Friction() const
{
  return &this->dataPtr->friction;
}

/////////////////////////////////////////////////
void Surface::SetContact(const sdf::Contact &_contact)
{
  this->dataPtr->contact = _contact;
}

/////////////////////////////////////////////////
sdf::ElementPtr Surface::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Surface::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("surface.sdf", elem);

  sdf::ElementPtr contactElem = elem->GetElement("contact", _errors);
  contactElem->GetElement("collide_bitmask", _errors)->Set(
      _errors, this->dataPtr->contact.CollideBitmask());

  sdf::ElementPtr frictionElem = elem->GetElement("friction", _errors);

  sdf::ElementPtr ode = frictionElem->GetElement("ode", _errors);
  ode->GetElement("mu", _errors)->Set(
      _errors, this->dataPtr->friction.ODE()->Mu());
  ode->GetElement("mu2", _errors)->Set(
      _errors, this->dataPtr->friction.ODE()->Mu2());
  ode->GetElement("slip1", _errors)->Set(
      _errors, this->dataPtr->friction.ODE()->Slip1());
  ode->GetElement("slip2", _errors)->Set(
      _errors, this->dataPtr->friction.ODE()->Slip2());
  ode->GetElement("fdir1", _errors)->Set(
      _errors, this->dataPtr->friction.ODE()->Fdir1());

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view Surface::SchemaFile() 
{
    static char kSchemaFile[] = "surface.sdf";
    return kSchemaFile;
}

