/*
 * Copyright 2020 Open Source Robotics Foundation
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

#include "sdf/Filesystem.hh"
#include "sdf/Types.hh"

#include "sdf/ParserConfig.hh"

using namespace sdf;

class sdf::ParserConfig::Implementation
{
  public: ParserConfig::SchemeToPathMap uriPathMap;
  public: std::function<std::string(const std::string &)> findFileCB;
};


/////////////////////////////////////////////////
ParserConfig::ParserConfig()
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
ParserConfig &ParserConfig::GlobalConfig()
{
  static auto *defaultConfig = new ParserConfig;
  return *defaultConfig;
}

/////////////////////////////////////////////////
const std::function<std::string(const std::string &)> &
ParserConfig::FindFileCallback() const
{
  return this->dataPtr->findFileCB;
}


/////////////////////////////////////////////////
void ParserConfig::SetFindCallback(
    std::function<std::string(const std::string &)> _cb)
{
  this->dataPtr->findFileCB = _cb;
}

/////////////////////////////////////////////////
const ParserConfig::SchemeToPathMap &ParserConfig::URIPathMap() const
{
  return this->dataPtr->uriPathMap;
}

/////////////////////////////////////////////////
void ParserConfig::AddURIPath(const std::string &_uri, const std::string &_path)
{
  // Split _path on colons.
  // Add each part of the colon separated path to the global URI map.
#ifdef _WIN32
  constexpr char multiplePathSeparator[] = ";";
#else
  constexpr char multiplePathSeparator[] = ":";
#endif

  for (const auto &part : sdf::split(_path, multiplePathSeparator))
  {
    // Only add valid paths
    if (!part.empty() && sdf::filesystem::is_directory(part))
    {
      this->dataPtr->uriPathMap[_uri].push_back(part);
    }
  }
}
