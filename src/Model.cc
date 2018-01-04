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
#include <iostream>

#include "sdf/Model.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::WorldPrivate
{
};

/////////////////////////////////////////////////
Model::Model()
  : dataPtr(new WorldPrivate)
{
}

/////////////////////////////////////////////////
Model::~Model()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Model::Load(ElementPtr _sdf)
{
}

/////////////////////////////////////////////////
void Model::DebugPrint(const std::string &_prefix = "") const
{
}

/////////////////////////////////////////////////
std::string Model::Name() const
{
}

/////////////////////////////////////////////////
void Model::SetName(const std::string &_name) const
{
}
