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
#include "sdf/Box.hh"
#include "sdf/parser.hh"

using namespace sdf;

// Private data class
class sdf::Box::Implementation
{
  // Size of the box
  public: gz::math::Boxd box{gz::math::Vector3d::One};

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Box::Box()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Box::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a box, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a box element
  if (_sdf->GetName() != "box")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a box geometry, but the provided SDF "
        "element is not a <box>."});
    return errors;
  }

  if (_sdf->HasElement("size"))
  {
    std::pair<gz::math::Vector3d, bool> pair =
      _sdf->Get<gz::math::Vector3d>("size", this->dataPtr->box.Size());

    if (!pair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <size> data for a <box> geometry. "
          "Using a size of 1, 1, 1 "});
    }
    this->dataPtr->box.SetSize(pair.first);
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Box geometry is missing a <size> child element. "
        "Using a size of 1, 1, 1."});
  }

  return errors;
}

//////////////////////////////////////////////////
gz::math::Vector3d Box::Size() const
{
  return this->dataPtr->box.Size();
}

//////////////////////////////////////////////////
void Box::SetSize(const gz::math::Vector3d &_size)
{
  this->dataPtr->box.SetSize(_size);
}

/////////////////////////////////////////////////
sdf::ElementPtr Box::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
const gz::math::Boxd &Box::Shape() const
{
  return this->dataPtr->box;
}

/////////////////////////////////////////////////
gz::math::Boxd &Box::Shape()
{
  return this->dataPtr->box;
}

/////////////////////////////////////////////////
sdf::ElementPtr Box::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("box_shape.sdf", elem);

  sdf::ElementPtr sizeElem = elem->GetElement("size");
  sizeElem->Set(this->Size());

  return elem;
}
