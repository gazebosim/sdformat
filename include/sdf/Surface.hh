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
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Get the collide bitmask parameter.
    /// \return The collide bitmask parameter.
    public: [[nodiscard]] uint16_t CollideBitmask() const;

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
    public: [[nodiscard]] double Mu() const;

    /// \brief Set Mu
    /// \param[in] _mu ODE mu
    public: void SetMu(double _mu);

    /// \brief Get the Mu2
    /// \returns ODE mu2
    public: [[nodiscard]] double Mu2() const;

    /// \brief Set Mu2
    /// \param[in] _mu2 ODE mu2
    public: void SetMu2(double _mu2);

    /// \brief Get the fdir
    /// \returns ODE fdir
    public: [[nodiscard]] const gz::math::Vector3d &Fdir1() const;

    /// \brief Set fdir
    /// \param[in] _fdir ODE fdir
    public: void SetFdir1(const gz::math::Vector3d &_fdir);

    /// \brief Get the slip1
    /// \returns ODE slip1
    public: [[nodiscard]] double Slip1() const;

    /// \brief Set Slip1
    /// \param[in] _slip1 ODE Slip1
    public: void SetSlip1(double _slip1);

    /// \brief Get the Slip2
    /// \returns ODE Slip2
    public: [[nodiscard]] double Slip2() const;

    /// \brief Set Slip2
    /// \param[in] _slip2 ODE Slip2
    public: void SetSlip2(double _slip2);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
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
    public: [[nodiscard]] const sdf::ODE *ODE() const;

    /// \brief Set the associated ODE object.
    /// \param[in] _ode The ODE object.
    public: void SetODE(const sdf::ODE &_ode);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };

  /// \brief Surface information for a collision.
  class SDFORMAT_VISIBLE Surface
  {
    /// \brief Default constructor
    public: Surface();

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
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Get the associated contact object
    /// \returns Pointer to the associated Contact object,
    /// nullptr if the Surface doesn't contain a Contact element.
    public: [[nodiscard]] const sdf::Contact *Contact() const;

    /// \brief Set the associated contact object.
    /// \param[in] _cont The contact object.
    public: void SetContact(const sdf::Contact &_contact);

    /// \brief Get the associated friction object
    /// \returns Pointer to the associated friction object,
    /// nullptr if the Surface doesn't contain a friction element.
    public: [[nodiscard]] const sdf::Friction *Friction() const;

    /// \brief Set the associated friction object.
    /// \param[in] _friction The friction object.
    public: void SetFriction(const sdf::Friction &_friction);

    /// \brief Create and return an SDF element filled with data from this
    /// surface.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated surface values.
    public: [[nodiscard]] sdf::ElementPtr ToElement() const;

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
