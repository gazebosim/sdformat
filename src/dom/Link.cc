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
#include "sdf/dom/Link.hh"

using namespace sdf;

/// \brief Private data for Link
class sdf::LinkPrivate
{
};

/////////////////////////////////////////////////
Link::Link()
  : Entity(), dataPtr(new LinkPrivate)
{
}

/////////////////////////////////////////////////
Link::~Link()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
bool Link::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (!this->LoadName(_sdf))
  {
    std::cerr << "A link name is required, but is not set.\n";
    result = false;
  }

  this->LoadPose(_sdf);

  return true;
}

/////////////////////////////////////////////////
void Link::Print(const std::string &_prefix) const
{
  std::cout << _prefix << "# Link: " << this->Name() << "\n"
            << _prefix << "  * Pose:  " << this->Pose() << "\n"
            << _prefix << "  * Frame:  " << this->Frame() << "\n";
}
