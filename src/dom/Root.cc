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

#include "sdf/sdf_config.h"
#include "sdf/parser.hh"
#include "sdf/dom/Root.hh"

using namespace sdf;

/// \brief Private data for sdf::Root
class sdf::RootPrivate
{
  /// \brief Version string
  public: std::string version = "";
};

/////////////////////////////////////////////////
Root::Root()
  : dataPtr(new RootPrivate)
{
}

/////////////////////////////////////////////////
Root::~Root()
{
  delete this->dataPtr;
}

/////////////////////////////////////////////////
bool Root::Load(const std::string &_filename)
{
  // Read an SDF file, and store the result in sdfParsed.
  sdf::SDFPtr sdfParsed = sdf::readFile(_filename);

  if (sdfParsed)
    return this->Load(sdfParsed->Root());

  std::cerr << "Unable to read file[" << _filename << "]\n";
  return false;
}

/////////////////////////////////////////////////
bool Root::Load(const sdf::SDFPtr _sdf)
{
  return this->Load(_sdf->Root());
}

/////////////////////////////////////////////////
bool Root::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  // Get the SDF version
  std::pair<std::string, bool> versionPair =
    _sdf->Get<std::string>("version", SDF_VERSION);

  // Check that the version exists.
  if (!versionPair.second)
  {
    std::cerr << "SDF does not have a version.\n";
    result = false;
  }
  else
    this->dataPtr->version = versionPair.first;

  return result;
}

/////////////////////////////////////////////////
std::string Root::Version() const
{
  return this->dataPtr->version;
}

/////////////////////////////////////////////////
void Root::SetVersion(const std::string &_version)
{
  this->dataPtr->version = _version;
}

/////////////////////////////////////////////////
void Root::Print(const std::string &_prefix) const
{
  std::cout << "SDF Version: " << this->Version() << "\n";
}
