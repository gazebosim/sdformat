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

#include "sdf/Root.hh"
#include "sdf/sdf_config.h"
#include "sdf/parser.hh"

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
Error Root::Load(const std::string &_filename)
{
  // Read an SDF file, and store the result in sdfParsed.
  sdf::SDFPtr sdfParsed = sdf::readFile(_filename);

  if (sdfParsed)
  {
    // Get the SDF version
    std::pair<std::string, bool> versionPair =
      sdfParsed->Root()->Get<std::string>("version", SDF_VERSION);

    // Check that the version exists.
    if (!versionPair.second)
    {
      return Error(ErrorCode::NO_ATTRIBUTE, "SDF does not have a version.");
    }

    this->dataPtr->version = versionPair.first;
    return Error();
  }

  return Error(ErrorCode::READ_FILE, "Unable to read file:" + _filename);
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
void Root::DebugPrint(const std::string &_prefix) const
{
  std::cout << _prefix << "SDF Version: " << this->Version() << "\n";
}
