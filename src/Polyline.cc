/*
 * Copyright 2022 Open Source Robotics Foundation
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
#include <string>

#include "sdf/parser.hh"
#include "sdf/Polyline.hh"
#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::Polyline::Implementation
{
  /// \brief Height in meters
  public: double height{1.0};

  /// \brief 2D points.
  public: std::vector<gz::math::Vector2d> points;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Polyline::Polyline()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Polyline::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
      "Attempting to load a polyline, but the provided SDF element is null."});
    return errors;
  }

  // We need a polyline element
  if (_sdf->GetName() != "polyline")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a polyline geometry, but the provided SDF "
        "element is not a <polyline>."});
    return errors;
  }

  if (_sdf->HasElement("height"))
  {
    auto height = _sdf->Get<double>(errors, "height", this->dataPtr->height);

    if (!height.second)
    {
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Invalid <height> data for a <polyline> geometry. "
          "Using a height of " + std::to_string(height.first) + "."});
    }
    this->dataPtr->height = height.first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Polyline geometry is missing a <height> child element. "
          "Using a height of " + std::to_string(this->dataPtr->height) + "."});
  }

  for (auto pointElem = _sdf->GetElement("point", errors);
       pointElem != nullptr;
       pointElem = pointElem->GetNextElement("point"))
  {
    this->dataPtr->points.push_back(
        pointElem->Get<gz::math::Vector2d>(errors));
  }

  return errors;
}

//////////////////////////////////////////////////
double Polyline::Height() const
{
  return this->dataPtr->height;
}

//////////////////////////////////////////////////
void Polyline::SetHeight(double _height)
{
  this->dataPtr->height = _height;
}

/////////////////////////////////////////////////
uint64_t Polyline::PointCount() const
{
  return this->dataPtr->points.size();
}

/////////////////////////////////////////////////
const gz::math::Vector2d *Polyline::PointByIndex(uint64_t _index) const
{
  if (_index < this->dataPtr->points.size())
    return &this->dataPtr->points[_index];
  return nullptr;
}

/////////////////////////////////////////////////
gz::math::Vector2d *Polyline::PointByIndex(uint64_t _index)
{
  if (_index < this->dataPtr->points.size())
    return &this->dataPtr->points[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Polyline::AddPoint(const gz::math::Vector2d &_point)
{
  if (this->dataPtr->points.size() == this->dataPtr->points.max_size())
  {
    return false;
  }
  this->dataPtr->points.push_back(_point);
  return true;
}

/////////////////////////////////////////////////
void Polyline::ClearPoints()
{
  this->dataPtr->points.clear();
}

/////////////////////////////////////////////////
const std::vector<gz::math::Vector2d> &Polyline::Points() const
{
  return this->dataPtr->points;
}

/////////////////////////////////////////////////
sdf::ElementPtr Polyline::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
sdf::ElementPtr Polyline::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Polyline::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("polyline_shape.sdf", elem);

  auto heightElem = elem->GetElement("height", _errors);
  heightElem->Set<double>(_errors, this->Height());

  for (auto &point : this->dataPtr->points)
  {
    auto pointElem = elem->AddElement("point", _errors);
    pointElem->Set<gz::math::Vector2d>(_errors, point);
  }

  return elem;
}
