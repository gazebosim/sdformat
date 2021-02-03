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
#include "sdf/Surface.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

using namespace sdf;

class sdf::Contact::Implementation
{
  // \brief The bitmask used to filter collisions.
  public: uint16_t collideBitmask = 0xff;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

class sdf::Surface::Implementation
{
  /// \brief The object storing contact parameters
  public: sdf::Contact contact;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};
};

/////////////////////////////////////////////////
Contact::Contact()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
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
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
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
void Surface::SetContact(const sdf::Contact &_contact)
{
  this->dataPtr->contact = _contact;
}
