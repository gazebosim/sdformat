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

#include <pxr/usd/usd/stage.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#include "sdf/sdf.hh"
#include "sdf_usd_parser/world.hh"

// //////////////////////////////////////////////////
// // Getting the first .sdf file in the path
// std::string findFuelResourceSdf(const std::string &_path)
// {
//   if (!common::exists(_path))
//     return "";
//
//   for (common::DirIter file(_path); file != common::DirIter(); ++file)
//   {
//     std::string current(*file);
//     if (!common::isFile(current))
//       continue;
//
//     auto fileName = common::basename(current);
//     auto fileExtensionIndex = fileName.rfind(".");
//     auto fileExtension = fileName.substr(fileExtensionIndex + 1);
//
//     if (fileExtension == "sdf")
//     {
//       return current;
//     }
//   }
//   return "";
// }

//////////////////////////////////////////////////
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
std::string findFileByExtension(const std::string &_path)
{
  for (ignition::common::DirIter file(_path);
    file != ignition::common::DirIter(); ++file)
  {
    std::string current(*file);

    if (ignition::common::isDirectory(current))
    {
      auto systemPaths = ignition::common::systemPaths();
      systemPaths->AddFilePaths(current);
      // std::cerr << "AddFilePaths " << current << '\n';
      std::string result = findFileByExtension(current);
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

    if (fileExtension == "sdf")
    {
      return current;
    }
  }
  return "";
}

//////////////////////////////////////////////////
std::string FindResources(const std::string &_uri)
{
  ignition::common::URI uri(_uri);
  std::string path;
  std::string home;
  ignition::common::env("HOME", home, false);
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
    std::string result = findFileByExtension(path);
    return result;
  }
  else
  {
    std::string result = findFileByName(path, fileName);
    return result;
  }
  return "";
}

//////////////////////////////////////////////////
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
