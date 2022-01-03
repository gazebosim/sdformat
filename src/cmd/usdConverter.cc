/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <pxr/usd/usd/stage.h>

#include "sdf/sdf.hh"
#include "sdf_usd_parser/world.hh"

//////////////////////////////////////////////////
/// \brief Get the full path of a file
/// \param[in] _path Where to being searching for the file
/// \param[in] _name The name of the file to find
/// \return The full path to the file named _name. Empty string is returned if
/// the file could not be found.
std::string findFileByName(const std::string &_path, const std::string &_name)
{
  for (ignition::common::DirIter file(_path);
    file != ignition::common::DirIter(); ++file)
  {
    std::string current(*file);

    if (ignition::common::isDirectory(current))
    {
      std::string result = findFileByName(current, _name);
      if (result.empty())
      {
        continue;
      }
      else
      {
        return result;
      }
    }

    if (!ignition::common::isFile(current))
      continue;

    auto fileName = ignition::common::basename(current);

    if (fileName == _name)
    {
      return current;
    }
  }
  return "";
}

//////////////////////////////////////////////////
/// \brief Get the full path of a file based on the extension
/// \param[in] _path Where to being searching for the file
/// \param[in] _extension Where to being searching for the file
/// \return The full path to the file with an extension _extension. Empty
/// string is returned if the file could not be found.
std::string findFileByExtension(
  const std::string &_path, const std::string &_extension,
  bool insertDirectories = false)
{
  if (insertDirectories)
  {
    for (ignition::common::DirIter file(_path);
      file != ignition::common::DirIter(); ++file)
    {
      std::string current(*file);
      if (ignition::common::isDirectory(current))
      {
        auto systemPaths = ignition::common::systemPaths();
        systemPaths->AddFilePaths(current);
        findFileByExtension(current, "", true);
      }
    }
  }

  for (ignition::common::DirIter file(_path);
    file != ignition::common::DirIter(); ++file)
  {
    std::string current(*file);

    if (ignition::common::isDirectory(current))
    {
      std::string result = findFileByExtension(current, _extension);
      if (result.empty())
      {
        continue;
      }
      else
      {
        return result;
      }
    }

    if (!ignition::common::isFile(current))
      continue;

    auto fileName = ignition::common::basename(current);
    auto fileExtensionIndex = fileName.rfind(".");
    auto fileExtension = fileName.substr(fileExtensionIndex + 1);

    if (fileExtension == _extension)
    {
      return current;
    }
  }
  return "";
}

//////////////////////////////////////////////////
/// \brief This functions is used by sdf::setFindCallback to find
/// the resources defined in the URI
/// \param[in] _uri URI of the file to find
/// \return The full path to the uri. Empty
/// string is returned if the file could not be found.
std::string FindResources(const std::string &_uri)
{
  ignition::common::URI uri(_uri);
  std::string path;
  std::string home;
  if (!ignition::common::env("HOME", home, false))
  {
    std::cerr << "The HOME environment variable was not defined, "
              << "so the resource [" << _uri << "] could not be found\n";
    return "";
  }
  if (uri.Scheme() == "http" || uri.Scheme() == "https")
  {
    std::vector<std::string> tokens = ignition::common::split(uri.Path().Str(), "/");
    std::string server = tokens[0];
    std::string versionServer = tokens[1];
    std::string owner = ignition::common::lowercase(tokens[2]);
    std::string type = ignition::common::lowercase(tokens[3]);
    std::string modelName = ignition::common::lowercase(tokens[4]);
    path = ignition::common::joinPaths(
      home, ".ignition", "fuel", server, owner, type, modelName);
  }
  else
  {
    path = ignition::common::joinPaths(home, ".ignition", "fuel");
  }

  auto fileName = ignition::common::basename(uri.Path().Str());
  auto fileExtensionIndex = fileName.rfind(".");
  if (fileExtensionIndex == std::string::npos)
  {
    return findFileByExtension(path, "sdf", true);
  }
  else
  {
    return findFileByName(path, fileName);
  }
  return "";
}

//////////////////////////////////////////////////
/// \brief This functions is used by sdf::addFindFileURICallback to find
/// the resources defined in the URI
/// \param[in] _uri URI of the file to find
/// \return The full path to the uri. Empty
/// string is returned if the file could not be found.
std::string FindResourceUri(const ignition::common::URI &_uri)
{
  return FindResources(_uri.Str());
}

int main(int argc, const char* argv[])
{
  if (argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " <sdf-path> <usd-path>\n";
    return -1;
  }

  sdf::Root root;

  // Configure SDF to fetch assets from ignition fuel.
  sdf::setFindCallback(std::bind(&FindResources, std::placeholders::_1));
  ignition::common::addFindFileURICallback(
    std::bind(&FindResourceUri, std::placeholders::_1));

  auto errors = root.Load(argv[1]);
  if (!errors.empty())
  {
    std::cerr << "Errors encountered:\n";
    for (const auto &e : errors)
      std::cout << e << "\n";
    return -2;
  }

  // only support SDF files with exactly 1 world for now
  if (root.WorldCount() != 1u)
  {
    std::cerr << argv[1] << " does not have exactly 1 world\n";
    return -3;
  }

  auto world = root.WorldByIndex(0u);
  if (!world)
  {
    std::cerr << "Error retrieving the world from " << argv[1] << "\n";
    return -4;
  }

  auto stage = pxr::UsdStage::CreateInMemory();

  const auto worldPath = std::string("/" + world->Name());
  if (!usd::ParseSdfWorld(*world, stage, worldPath))
  {
    std::cerr << "Error parsing world [" << world->Name() << "]\n";
    //return -5;
  }

  if (!stage->GetRootLayer()->Export(argv[2]))
  {
    std::cerr << "Issue saving USD to " << argv[2] << "\n";
    return -6;
  }
}
