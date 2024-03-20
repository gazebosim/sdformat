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

class sdf::BulletFriction::Implementation
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};

  /// \brief Coefficient of friction in first friction pyramid direction,
  /// the unitless maximum ratio of force in first friction pyramid
  /// direction to normal force.
  public: double friction{1.0};

  /// \brief Coefficient of friction in second friction pyramid direction,
  /// the unitless maximum ratio of force in second friction pyramid
  /// direction to normal force.
  public: double friction2{1.0};

  /// \brief Unit vector specifying first friction pyramid direction in
  /// collision-fixed reference frame.
  public: gz::math::Vector3d fdir1{0, 0, 0};

  /// \brief Rolling friction coefficient.
  public: double rollingFriction{1.0};
};

class sdf::Torsional::Implementation
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};

  /// \brief Torsional friction coefficient. Uunitless maximum ratio of
  /// tangential stress to normal stress.
  public: double coefficient{1.0};

  /// \brief If this flag is true, torsional friction is calculated using the
  /// "patch_radius" parameter. If this flag is set to false,
  /// "surface_radius" (R) and contact depth (d) are used to compute the patch
  /// radius as sqrt(R*d).
  public: bool usePatchRadius{true};

  /// \brief Radius of contact patch surface.
  public: double patchRadius{0.0};

  /// \brief Surface radius on the point of contact.
  public: double surfaceRadius{0.0};

  /// \brief Force dependent slip for torsional friction.
  /// equivalent to inverse of viscous damping coefficient with units of
  /// rad/s/(Nm). A slip value of 0 is infinitely viscous.
  public: double odeSlip{0.0};
};

class sdf::Friction::Implementation
{
  /// \brief The object storing ode parameters
  public: sdf::ODE ode;

  /// \brief The object storing bullet friction parameters
  public: std::optional<sdf::BulletFriction> bullet;

  /// \brief The object storing torsional parameters
  public: std::optional<sdf::Torsional> torsional;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

class sdf::Surface::Implementation
{
  /// \brief The object storing friction parameters
  public: sdf::Friction friction;

  /// \brief The object storing contact parameters
  public: sdf::Contact contact;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

/////////////////////////////////////////////////
Torsional::Torsional()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Torsional::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a BulletFriction, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <torsional>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "torsional")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a BulletFriction, but the provided SDF element is not a "
        "<ode>."});
    return errors;
  }

  this->dataPtr->coefficient = _sdf->Get<double>(
      "coefficient", this->dataPtr->coefficient).first;
  this->dataPtr->usePatchRadius = _sdf->Get<bool>(
      "use_patch_radius", this->dataPtr->usePatchRadius).first;
  this->dataPtr->patchRadius = _sdf->Get<double>(
      "patch_radius", this->dataPtr->patchRadius).first;
  this->dataPtr->surfaceRadius = _sdf->Get<double>(
      "surface_radius", this->dataPtr->surfaceRadius).first;

  if (_sdf->HasElement("ode"))
  {
    this->dataPtr->odeSlip = _sdf->GetElement("ode")->Get<double>(
        "slip", this->dataPtr->odeSlip).first;
  }

  return errors;
}

/////////////////////////////////////////////////
double Torsional::Coefficient() const
{
  return this->dataPtr->coefficient;
}

/////////////////////////////////////////////////
void Torsional::SetCoefficient(double _coefficient)
{
  this->dataPtr->coefficient = _coefficient;
}

/////////////////////////////////////////////////
bool Torsional::UsePatchRadius() const
{
  return this->dataPtr->usePatchRadius;
}

/////////////////////////////////////////////////
void Torsional::SetUsePatchRadius(bool _usePatchRadius)
{
  this->dataPtr->usePatchRadius = _usePatchRadius;
}

/////////////////////////////////////////////////
double Torsional::PatchRadius() const
{
  return this->dataPtr->patchRadius;
}

/////////////////////////////////////////////////
void Torsional::SetPatchRadius(double _radius)
{
  this->dataPtr->patchRadius = _radius;
}

/////////////////////////////////////////////////
double Torsional::SurfaceRadius() const
{
  return this->dataPtr->surfaceRadius;
}

/////////////////////////////////////////////////
void Torsional::SetSurfaceRadius(double _radius)
{
  this->dataPtr->surfaceRadius = _radius;
}

/////////////////////////////////////////////////
double Torsional::ODESlip() const
{
  return this->dataPtr->odeSlip;
}

/////////////////////////////////////////////////
void Torsional::SetODESlip(double _slip)
{
  this->dataPtr->odeSlip = _slip;
}

/////////////////////////////////////////////////
sdf::ElementPtr Torsional::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
BulletFriction::BulletFriction()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors BulletFriction::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a BulletFriction, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <bullet>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "bullet")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a BulletFriction, but the provided SDF element is not a "
        "<ode>."});
    return errors;
  }

  this->dataPtr->friction = _sdf->Get<double>(
      "friction", this->dataPtr->friction).first;
  this->dataPtr->friction2 = _sdf->Get<double>(
      "friction2", this->dataPtr->friction2).first;
  this->dataPtr->fdir1 = _sdf->Get<gz::math::Vector3d>("fdir1",
        this->dataPtr->fdir1).first;
  this->dataPtr->rollingFriction = _sdf->Get<double>(
      "rolling_friction", this->dataPtr->rollingFriction).first;

  return errors;
}

/////////////////////////////////////////////////
double BulletFriction::Friction() const
{
  return this->dataPtr->friction;
}

/////////////////////////////////////////////////
void BulletFriction::SetFriction(double _friction)
{
  this->dataPtr->friction = _friction;
}

/////////////////////////////////////////////////
double BulletFriction::Friction2() const
{
  return this->dataPtr->friction2;
}

/////////////////////////////////////////////////
void BulletFriction::SetFriction2(double _friction2)
{
  this->dataPtr->friction2 = _friction2;
}

/////////////////////////////////////////////////
const gz::math::Vector3d &BulletFriction::Fdir1() const
{
  return this->dataPtr->fdir1;
}

/////////////////////////////////////////////////
void BulletFriction::SetFdir1(const gz::math::Vector3d &_fdir)
{
  this->dataPtr->fdir1 = _fdir;
}

/////////////////////////////////////////////////
double BulletFriction::RollingFriction() const
{
  return this->dataPtr->rollingFriction;
}

/////////////////////////////////////////////////
void BulletFriction::SetRollingFriction(double _rollingFriction)
{
  this->dataPtr->rollingFriction = _rollingFriction;
}

/////////////////////////////////////////////////
sdf::ElementPtr BulletFriction::Element() const
{
  return this->dataPtr->sdf;
}

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

  this->dataPtr->mu = _sdf->Get<double>("mu", this->dataPtr->mu).first;
  this->dataPtr->mu2 = _sdf->Get<double>("mu2", this->dataPtr->mu2).first;
  this->dataPtr->slip1 = _sdf->Get<double>("slip1", this->dataPtr->slip1).first;
  this->dataPtr->slip2 = _sdf->Get<double>("slip2", this->dataPtr->slip2).first;
  this->dataPtr->fdir1 = _sdf->Get<gz::math::Vector3d>("fdir1",
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
    Errors err = this->dataPtr->ode.Load(_sdf->GetElement("ode"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  if (_sdf->HasElement("bullet"))
  {
    this->dataPtr->bullet.emplace();
    Errors err = this->dataPtr->bullet->Load(_sdf->GetElement("bullet"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  if (_sdf->HasElement("torsional"))
  {
    this->dataPtr->torsional.emplace();
    Errors err = this->dataPtr->torsional->Load(_sdf->GetElement("torsional"));
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
void Friction::SetBulletFriction(const sdf::BulletFriction &_bullet)
{
  this->dataPtr->bullet = _bullet;
}

/////////////////////////////////////////////////
const sdf::BulletFriction *Friction::BulletFriction() const
{
  return optionalToPointer(this->dataPtr->bullet);
}

/////////////////////////////////////////////////
void Friction::SetTorsional(const sdf::Torsional &_torsional)
{
  this->dataPtr->torsional = _torsional;
}

/////////////////////////////////////////////////
const sdf::Torsional *Friction::Torsional() const
{
  return optionalToPointer(this->dataPtr->torsional);
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
        static_cast<uint16_t>(_sdf->Get<unsigned int>("collide_bitmask"));
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
    Errors err = this->dataPtr->contact.Load(_sdf->GetElement("contact"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  if (_sdf->HasElement("friction"))
  {
    Errors err = this->dataPtr->friction.Load(_sdf->GetElement("friction"));
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
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("surface.sdf", elem);

  sdf::ElementPtr contactElem = elem->GetElement("contact");
  contactElem->GetElement("collide_bitmask")->Set(
      this->dataPtr->contact.CollideBitmask());

  sdf::ElementPtr frictionElem = elem->GetElement("friction");
  sdf::ElementPtr ode = frictionElem->GetElement("ode");
  ode->GetElement("mu")->Set(this->dataPtr->friction.ODE()->Mu());
  ode->GetElement("mu2")->Set(this->dataPtr->friction.ODE()->Mu2());
  ode->GetElement("slip1")->Set(this->dataPtr->friction.ODE()->Slip1());
  ode->GetElement("slip2")->Set(this->dataPtr->friction.ODE()->Slip2());
  ode->GetElement("fdir1")->Set(this->dataPtr->friction.ODE()->Fdir1());

  if (this->dataPtr->friction.BulletFriction())
  {
    sdf::ElementPtr bullet = frictionElem->GetElement("bullet");
    bullet->GetElement("friction")->Set(
      this->dataPtr->friction.BulletFriction()->Friction());
    bullet->GetElement("friction2")->Set(
      this->dataPtr->friction.BulletFriction()->Friction2());
    bullet->GetElement("fdir1")->Set(
      this->dataPtr->friction.BulletFriction()->Fdir1());
    bullet->GetElement("rolling_friction")->Set(
      this->dataPtr->friction.BulletFriction()->RollingFriction());
  }

  if (this->dataPtr->friction.Torsional())
  {
    sdf::ElementPtr torsional = frictionElem->GetElement("torsional");
    torsional->GetElement("coefficient")->Set(
      this->dataPtr->friction.Torsional()->Coefficient());
    torsional->GetElement("use_patch_radius")->Set(
      this->dataPtr->friction.Torsional()->UsePatchRadius());
    torsional->GetElement("patch_radius")->Set(
      this->dataPtr->friction.Torsional()->PatchRadius());
    torsional->GetElement("surface_radius")->Set(
      this->dataPtr->friction.Torsional()->SurfaceRadius());

    torsional->GetElement("ode")->GetElement("slip")->Set(
      this->dataPtr->friction.Torsional()->ODESlip());
  }

  return elem;
}
