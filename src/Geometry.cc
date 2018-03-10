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
#include "sdf/Geometry.hh"

using namespace sdf;

// Private data class
class sdf::GeometryPrivate
{
  public: GeometryType type = GeometryType::INVALID;
};

/////////////////////////////////////////////////
Geometry::Geometry()
  : dataPtr(new GeometryPrivate)
{
}

/////////////////////////////////////////////////
Geometry::~Geometry()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
GeometryType Geometry::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Geometry::SetType(const GeometryType _type)
{
  this->dataPtr->type = _type;
}
