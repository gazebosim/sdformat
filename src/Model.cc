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
#include "sdf/Model.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::ModelPrivate
{
  /// \brief Name of the model.
  public: std::string name = "";

  /// \brief The links specified in this model.
  public: std::vector<Link> links;

  /// \brief The joints specified in this model.
  public: std::vector<Joint> joints;
};

/////////////////////////////////////////////////
Model::Model()
  : dataPtr(new ModelPrivate)
{
}

/////////////////////////////////////////////////
Model::Model(Model &&_model)
{
  this->dataPtr = _model.dataPtr;
  _model.dataPtr = nullptr;
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
  Errors errors;

  // Check that the provided SDF element is a <model>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "model")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Model, but the provided SDF element is not a "
        "<model>."});
    return errors;
  }

  // Read the models's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A model name is required, but the name is not set."});
  }

  // Load all the links.
  Errors linkLoadErrors = loadUniqueRepeated<Link>(_sdf, "link",
    this->dataPtr->links);
  errors.insert(errors.end(), linkLoadErrors.begin(), linkLoadErrors.end());

  // Load all the joints.
  Errors jointLoadErrors = loadUniqueRepeated<Joint>(_sdf, "joint",
    this->dataPtr->joints);
  errors.insert(errors.end(), jointLoadErrors.begin(), jointLoadErrors.end());

  return errors;
}

/////////////////////////////////////////////////
std::string Model::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Model::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
uint64_t Model::LinkCount() const
{
  return this->dataPtr->links.size();
}

/////////////////////////////////////////////////
const Link *Model::LinkByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->links.size())
    return &this->dataPtr->links[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::LinkNameExists(const std::string &_name) const
{
  for (auto const &l : this->dataPtr->links)
  {
    if (l.Name() == _name)
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
uint64_t Model::JointCount() const
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
const Joint *Model::JointByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->joints.size())
    return &this->dataPtr->joints[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Model::JointNameExists(const std::string &_name) const
{
  for (auto const &j : this->dataPtr->joints)
  {
    if (j.Name() == _name)
    {
      return true;
    }
  }
  return false;
}
