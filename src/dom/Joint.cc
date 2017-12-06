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
#include "Utils.hh"
#include "sdf/dom/Joint.hh"

using namespace sdf;

namespace sdf
{
  /// \brief Names that correspond to the JointType enum.
  static std::string JointTypenames[] =
  {
    "ball",
    "fixed",
    "gearbox",
    "prismatic",
    "revolute",
    "revolute2",
    "screw",
    "universal",
    "unknown"
  };

  /// \brief Private data for Joint.
  class JointPrivate
  {
    /// \brief The joint type.
    public: JointType type = JointType::UNKNOWN;
  };
}

/////////////////////////////////////////////////
Joint::Joint()
  :Entity(), dataPtr(new JointPrivate)
{
}

/////////////////////////////////////////////////
Joint::~Joint()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
bool Joint::Load(sdf::ElementPtr _sdf)
{
  if (_sdf->GetName() != "joint")
  {
    std::cerr << "Attempting to load a Joint, but the provided "
      << "SDF element is not a <joint>\n";

    // This is an error that cannot be recovered, so return false.
    return false;
  }

  bool result = true;

  if (!this->LoadName(_sdf))
  {
    std::cerr << "A link name is required, but is not set.\n";
    result = false;
  }

  // A Pose is not required, so no need to output a error if it is not
  // present
  this->LoadPose(_sdf);

  // Read the type
  std::pair<std::string, bool> typePair = _sdf->Get<std::string>("type", "");
  this->dataPtr->type = JointType::UNKNOWN;
  if (!typePair.second)
  {
    std::cerr << "No type specified for joint[" << this->Name() << "].\n";
    result = false;
  }
  else
  {
    if (typePair.first == "ball")
      this->dataPtr->type = JointType::BALL;
    else if (typePair.first == "fixed")
      this->dataPtr->type = JointType::FIXED;
    else if (typePair.first == "gearbox")
      this->dataPtr->type = JointType::GEARBOX;
    else if (typePair.first == "prismatic")
      this->dataPtr->type = JointType::PRISMATIC;
    else if (typePair.first == "revolute")
      this->dataPtr->type = JointType::REVOLUTE;
    else if (typePair.first == "revolute2")
      this->dataPtr->type = JointType::REVOLUTE2;
    else if (typePair.first == "screw")
      this->dataPtr->type = JointType::SCREW;
    else if (typePair.first == "universal")
      this->dataPtr->type = JointType::UNIVERSAL;
    else
    {
      std::cerr << "Unknown or unsupported joint type["
                << typePair.first <<  "]\n";
      result = false;
    }
  }

  return result;
}

/////////////////////////////////////////////////
JointType Joint::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
std::string Joint::Typename() const
{
  return sdf::JointTypenames[static_cast<int>(this->dataPtr->type)];
}

/////////////////////////////////////////////////
void Joint::SetType(const JointType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
void Joint::Print(const std::string &_prefix) const
{
  std::cout << _prefix << "# Joint: " << this->Name() << "\n"
            << _prefix << "  * Type: " << this->Typename() << "\n"
            << _prefix << "  * Pose:  " << this->Pose() << "\n"
            << _prefix << "  * Frame:  " << this->Frame() << "\n";
}
