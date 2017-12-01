/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <map>
#include <ignition/math/Pose3.hh>
#include "Utils.hh"
#include "sdf/dom/Joint.hh"
#include "sdf/dom/Link.hh"
#include "sdf/dom/Model.hh"

using namespace sdf;

/// \brief Private data class for model
class sdf::ModelPrivate
{
  /// The set of links that belong to the model.
  public: std::map<std::string, sdf::Link> links;

  /// The set of joints that belong to the model.
  public: std::map<std::string, sdf::Joint> joints;

  /// The set of nested models that belong to the model.
  public: std::map<std::string, sdf::Model> models;

  /// \brief True indicates the model is static (immovable).
  public: bool isStatic;

  /// \brief True indicates the model should perform self-collision checks.
  public: bool selfCollide;

  /// \brief True indicates the model should auto-disable. This is
  /// a feature of some physics engine, where objects that stop moving are
  /// no longer computed until a collision occurs.
  public: bool autoDisable;

  /// \brief True indicates that wind should affect this model.
  public: bool enableWind;
};

/////////////////////////////////////////////////
Model::Model()
  : Entity(), dataPtr(new ModelPrivate)
{
}

/////////////////////////////////////////////////
Model::~Model()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
bool Model::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (!this->LoadName(_sdf))
  {
    std::cerr << "A model name is required, but is not set.\n";
    result = false;
  }

  // A model doesn't require a pose. No need to check the return value.
  this->LoadPose(_sdf);

  this->dataPtr->isStatic = _sdf->Get<bool>("static", "false").first;
  this->dataPtr->selfCollide = _sdf->Get<bool>("self_collide", "false").first;
  this->dataPtr->autoDisable =
    _sdf->Get<bool>("allow_auto_disable", "true").first;
  this->dataPtr->enableWind = _sdf->Get<bool>("enable_wind", "false").first;

  // Read all the links
  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link");
    while (elem)
    {
      sdf::Link link;
      if (link.Load(elem))
      {
        this->dataPtr->links.insert(std::make_pair(link.Name(),
                                    std::move(link)));
      }
      else
      {
         result = false;
      }

      elem = elem->GetNextElement("link");
    }
  }

  // Read all the links
  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("joint");
    while (elem)
    {
      sdf::Joint joint;
      if (joint.Load(elem))
      {
        this->dataPtr->joints.insert(std::make_pair(joint.Name(),
                                     std::move(joint)));
      }
      else
      {
        result = false;
      }
      elem = elem->GetNextElement("joint");
    }
  }

  // Read all the nested models
  result = result && loadModels(_sdf, this->dataPtr->models);

  return result;
}

/////////////////////////////////////////////////
void Model::Print(const std::string &_prefix) const
{
  std::cout << _prefix << " ## Model: " << this->Name() << "\n"
            << _prefix << "   * Pose:  " << this->Pose() << "\n"
            << _prefix << "   * Frame:  " << this->Frame() << "\n"
            << _prefix << "   * Static:  " << this->dataPtr->isStatic << "\n"
            << _prefix << "   * Enable wind:  "
            << this->dataPtr->enableWind << "\n"
            << _prefix << "   * Self collide:  "
            << this->dataPtr->selfCollide << "\n"
            << _prefix << "   * Auto disable:  "
            << this->dataPtr->autoDisable << "\n"
            << _prefix << "   * Link count:  "
            << this->dataPtr->links.size() << "\n"
            << _prefix << "   * Joint count: "
            << this->dataPtr->joints.size() << "\n"
            << _prefix << "   * Nested model count: "
            << this->dataPtr->models.size() << "\n";

  for (auto const &link: this->dataPtr->links)
  {
    link.second.Print(_prefix + "  ");
  }

  for (auto const &joint: this->dataPtr->joints)
  {
    joint.second.Print(_prefix + "  ");
  }

  for (auto const &model: this->dataPtr->models)
  {
    model.second.Print(_prefix + "  ");
  }
}

/////////////////////////////////////////////////
uint64_t Model::LinkCount() const
{
  return this->dataPtr->links.size();
}

/////////////////////////////////////////////////
const Link *Model::FindLink(const std::string &_name) const
{
  const auto iter = this->dataPtr->links.find(_name);
  if (iter != this->dataPtr->links.end())
    return &iter->second;
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t Model::JointCount() const
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
const Joint *Model::FindJoint(const std::string &_name) const
{
  const auto iter = this->dataPtr->joints.find(_name);
  if (iter != this->dataPtr->joints.end())
    return &iter->second;
  return nullptr;
}

/////////////////////////////////////////////////
uint64_t Model::ModelCount() const
{
  return this->dataPtr->models.size();
}

/////////////////////////////////////////////////
const Model *Model::FindModel(const std::string &_name) const
{
  const auto iter = this->dataPtr->models.find(_name);
  if (iter != this->dataPtr->models.end())
    return &iter->second;
  return nullptr;
}
