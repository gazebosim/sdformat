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

#include <ignition/utils/ImplPtr.hh>
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
    IGN_UTILS_IMPL_PTR(dataPtr)
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
    public: sdf::ElementPtr Element() const;

    /// \brief Get the associated contact object
    /// \returns Pointer to the associated Contact object,
    /// nullptr if the Surface doesn't contain a Contact element.
    public: const sdf::Contact *Contact() const;

    /// \brief Set the associated contact object.
    /// \param[in] _cont The contact object.
    public: void SetContact(const sdf::Contact &_contact);

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}

#endif
