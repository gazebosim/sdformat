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
#ifndef SDF_SURFACE_HH_
#define SDF_SURFACE_HH_

#include <gz/math/Vector3.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \brief Contact information for a surface.
  class SDFORMAT_VISIBLE Contact
  {
    /// \brief Default constructor
    public: Contact();

    /// \brief Load the contact based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the collide bitmask parameter.
    /// \return The collide bitmask parameter.
    public: uint16_t CollideBitmask() const;

    /// \brief Set the collide bitmask parameter.
    public: void SetCollideBitmask(const uint16_t _bitmask);

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief ODE information for a friction.
  class SDFORMAT_VISIBLE ODE
  {
    /// \brief Default constructor
    public: ODE();

    /// \brief Load the ODE based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Set the Mu
    /// \returns ODE mu
    public: double Mu() const;

    /// \brief Set Mu
    /// \param[in] _mu ODE mu
    public: void SetMu(double _mu);

    /// \brief Get the Mu2
    /// \returns ODE mu2
    public: double Mu2() const;

    /// \brief Set Mu2
    /// \param[in] _mu2 ODE mu2
    public: void SetMu2(double _mu2);

    /// \brief Get the fdir
    /// \returns ODE fdir
    public: const gz::math::Vector3d &Fdir1() const;

    /// \brief Set fdir
    /// \param[in] _fdir ODE fdir
    public: void SetFdir1(const gz::math::Vector3d &_fdir);

    /// \brief Get the slip1
    /// \returns ODE slip1
    public: double Slip1() const;

    /// \brief Set Slip1
    /// \param[in] _slip1 ODE Slip1
    public: void SetSlip1(double _slip1);

    /// \brief Get the Slip2
    /// \returns ODE Slip2
    public: double Slip2() const;

    /// \brief Set Slip2
    /// \param[in] _slip2 ODE Slip2
    public: void SetSlip2(double _slip2);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief BulletFriction information for a friction.
  class SDFORMAT_VISIBLE BulletFriction
  {
    /// \brief Default constructor
    public: BulletFriction();

    /// \brief Load BulletFriction friction based on a element pointer. This is
    /// *not* the usual entry point. Typical usage of the SDF DOM is through
    /// the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the friction coefficient in first friction pyramid direction.
    /// \returns Friction coefficient
    public: double Friction() const;

    /// \brief Set friction coefficient in first friction pyramid direction.
    /// \param[in] _fricton Friction coefficient
    public: void SetFriction(double _friction);

    /// \brief Get the friction coefficient in second friction pyramid
    /// direction.
    /// \return Second friction coefficient
    public: double Friction2() const;

    /// \brief Set friction coefficient in second friction pyramid direction.
    /// \param[in] _fricton Friction coefficient
    public: void SetFriction2(double _friction);

    /// \brief Get the first friction pyramid direction in collision-fixed
    /// reference
    /// \return First friction pyramid direction.
    public: const gz::math::Vector3d &Fdir1() const;

    /// \brief Set the first friction pyramid direction in collision-fixed
    /// reference
    /// \param[in] _fdir First friction pyramid direction.
    public: void SetFdir1(const gz::math::Vector3d &_fdir);

    /// \brief Get the rolling friction coefficient
    /// \return Rolling friction coefficient
    public: double RollingFriction() const;

    /// \brief Set the rolling friction coefficient
    /// \param[in] _slip1 Rolling friction coefficient
    public: void SetRollingFriction(double _friction);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Torsional friction
  class SDFORMAT_VISIBLE Torsional
  {
    /// \brief Default constructor
    public: Torsional();

    /// \brief Load torsional friction based on a element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the torsional friction coefficient.
    /// \return Torsional friction coefficient
    public: double Coefficient() const;

    /// \brief Set the torsional friction coefficient.
    /// \param[in] _fricton Torsional friction coefficient
    public: void SetCoefficient(double _coefficient);

    /// \brief Get whether the patch radius is used to calculate torsional
    /// friction.
    /// \return True if patch radius is used.
    public: bool UsePatchRadius() const;

    /// \brief Set whether to use patch radius for torsional friction
    /// calculation.
    /// \param[in] _usePatchRadius True to use patch radius.
    /// False to use surface radius.
    public: void SetUsePatchRadius(bool _usePatchRadius);

    /// \brief Get the radius of contact patch surface.
    /// \return Patch radius
    public: double PatchRadius() const;

    /// \brief Set the radius of contact patch surface.
    /// \param[in] _radius Patch radius
    public: void SetPatchRadius(double _radius);

    /// \brief Get the surface radius on the contact point
    /// \return Surface radius
    public: double SurfaceRadius() const;

    /// \brief Set the surface radius on the contact point.
    /// \param[in] _radius Surface radius
    public: void SetSurfaceRadius(double _radius);

    /// \brief Get the ODE force dependent slip for torsional friction
    /// \return Force dependent slip for torsional friction.
    public: double ODESlip() const;

    /// \brief Set the ODE force dependent slip for torsional friction
    /// \param[in] _slip Force dependent slip for torsional friction.
    public: void SetODESlip(double _slip);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Friction information for a surface.
  class SDFORMAT_VISIBLE Friction
  {
    /// \brief Default constructor
    public: Friction();

    /// \brief Load the friction based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the associated ODE object
    /// \returns Pointer to the associated ODE object,
    /// nullptr if the Surface doesn't contain a ODE element.
    public: const sdf::ODE *ODE() const;

    /// \brief Set the associated ODE object.
    /// \param[in] _ode The ODE object.
    public: void SetODE(const sdf::ODE &_ode);

    /// \brief Get the associated BulletFriction object
    /// \return Pointer to the associated BulletFriction object,
    /// nullptr if the Surface doesn't contain a BulletFriction element.
    public: const sdf::BulletFriction *BulletFriction() const;

    /// \brief Set the associated BulletFriction object.
    /// \param[in] _bullet The BulletFriction object.
    public: void SetBulletFriction(const sdf::BulletFriction &_bullet);

    /// \brief Get the torsional friction
    /// \return Pointer to the torsional friction
    /// nullptr if the Surface doesn't contain a torsional friction element.
    public: const sdf::Torsional *Torsional() const;

    /// \brief Set the torsional friction
    /// \param[in] _torsional The torsional friction.
    public: void SetTorsional(const sdf::Torsional &_torsional);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Surface information for a collision.
  class SDFORMAT_VISIBLE Surface
  {
    /// \brief Default constructor
    public: Surface();

    /// \brief Get the schema file name accessor
    public: static inline std::string_view SchemaFile();

    /// \brief Load the surface based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the associated contact object
    /// \returns Pointer to the associated Contact object,
    /// nullptr if the Surface doesn't contain a Contact element.
    public: const sdf::Contact *Contact() const;

    /// \brief Set the associated contact object.
    /// \param[in] _cont The contact object.
    public: void SetContact(const sdf::Contact &_contact);

    /// \brief Get the associated friction object
    /// \returns Pointer to the associated friction object,
    /// nullptr if the Surface doesn't contain a friction element.
    public: const sdf::Friction *Friction() const;

    /// \brief Set the associated friction object.
    /// \param[in] _friction The friction object.
    public: void SetFriction(const sdf::Friction &_friction);

    /// \brief Create and return an SDF element filled with data from this
    /// surface.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated surface values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// surface.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated surface values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}

#endif
