/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <cctype>
#include <string>

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

#include <gz/utils/cli/CLI.hpp>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/sdf.hh"
#include "sdf/usd/sdf_parser/World.hh"
#include "../sdf_parser/Model.hh"
#include "../UsdUtils.hh"

//////////////////////////////////////////////////
/// \brief Enumeration of available commands
enum class Command
{
  kNone,
};

//////////////////////////////////////////////////
/// \brief Structure to hold all available topic options
struct Options
{
  /// \brief Command to execute
  Command command{Command::kNone};

  /// \brief input filename
  std::string inputFilename{"input.sdf"};

  /// \brief output filename
  std::string outputFilename{"output.usd"};
};

//////////////////////////////////////////////////
/// \brief Get the full path of a file
/// \param[in] _path Where to begin searching for the file
/// \param[in] _name The name of the file to find
/// \return The full path to the file named _name. Empty string is returned if
/// the file could not be found.
std::string findFileByName(const std::string &_path, const std::string &_name)
{
  for (gz::common::DirIter file(_path);
    file != gz::common::DirIter(); ++file)
  {
    std::string current(*file);

    if (gz::common::isDirectory(current))
    {
      std::string result = findFileByName(current, _name);
      if (result.empty())
      {
        continue;
      }
      return result;
    }

    if (!gz::common::isFile(current))
      continue;

    auto fileName = gz::common::basename(current);

    if (fileName == _name)
    {
      return current;
    }
  }
  return "";
}

//////////////////////////////////////////////////
/// \brief Get the full path of a file based on the extension
/// \param[in] _path Where to begin searching for the file
/// \param[in] _extension The extension of the file
/// \param[in] _insertDirectories Whether subdirectories should be inserted as
/// needed when looking for the file (true) or not (false)
/// \return The full path to the file with an extension _extension. Empty
/// string is returned if the file could not be found.
std::string findFileByExtension(
  const std::string &_path, const std::string &_extension,
  bool _insertDirectories = false)
{
  if (_insertDirectories)
  {
    for (gz::common::DirIter file(_path);
      file != gz::common::DirIter(); ++file)
    {
      std::string current(*file);
      if (gz::common::isDirectory(current))
      {
        auto systemPaths = gz::common::systemPaths();
        systemPaths->AddFilePaths(current);
        findFileByExtension(current, "", true);
      }
    }
  }

  for (gz::common::DirIter file(_path);
    file != gz::common::DirIter(); ++file)
  {
    std::string current(*file);

    if (gz::common::isDirectory(current))
    {
      std::string result = findFileByExtension(current, _extension);
      if (result.empty())
      {
        continue;
      }
      return result;
    }

    if (!gz::common::isFile(current))
      continue;

    auto fileName = gz::common::basename(current);
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
  gz::common::URI uri(_uri);
  std::string path;
  std::string home;
  if (!gz::common::env("HOME", home, false))
  {
    std::cerr << "The HOME environment variable was not defined, "
              << "so the resource [" << _uri << "] could not be found\n";
    return "";
  }
  if (uri.Scheme() == "http" || uri.Scheme() == "https")
  {
    std::vector<std::string> tokens =
      gz::common::split(uri.Path().Str(), "/");
    const std::string server = tokens[0];
    const std::string versionServer = tokens[1];
    const std::string owner = gz::common::lowercase(tokens[2]);
    const std::string type = gz::common::lowercase(tokens[3]);
    const std::string modelName = gz::common::lowercase(tokens[4]);
    path = gz::common::joinPaths(
      home, ".gz", "fuel", server, owner, type, modelName);
  }
  else
  {
    path = gz::common::joinPaths(home, ".gz", "fuel");
  }

  auto fileName = gz::common::basename(uri.Path().Str());
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
/// \brief This function is used by gz::common::addFindFileURICallback to
/// find the resources defined in the URI
/// \param[in] _uri URI of the file to find
/// \return The full path to the uri. Empty
/// string is returned if the file could not be found.
std::string FindResourceUri(const gz::common::URI &_uri)
{
  return FindResources(_uri.Str());
}

void runCommand(const Options &_opt)
{
  // Configure SDF to fetch assets from ignition fuel.
  sdf::setFindCallback(std::bind(&FindResources, std::placeholders::_1));
  gz::common::addFindFileURICallback(
    std::bind(&FindResourceUri, std::placeholders::_1));

  sdf::Root root;
  auto errors = root.Load(_opt.inputFilename);
  if (!errors.empty())
  {
    std::cerr << "Errors encountered:\n";
    for (const auto &e : errors)
      std::cout << e << "\n";
    exit(-2);
  }

  // only support SDF files with exactly 1 world or 1 model for now
  if (root.WorldCount() != 1u)
  {
    auto model = root.Model();
    if (model != nullptr)
    {
      std::string pathInputFile =
        gz::common::parentPath(_opt.inputFilename);
      if (pathInputFile.empty() || pathInputFile == _opt.inputFilename)
      {
        pathInputFile = gz::common::cwd();
      }
      auto systemPaths = gz::common::systemPaths();
      systemPaths->AddFilePaths(pathInputFile);

      // This loop here will add all the directories inside the sdf file.
      // For example: If we download a model from fuel, textures might live in
      // in the same path but in a different folder: materials/textures,
      // this loop will add these two folders to the systempaths allowing the
      // cmd to find the resources.
      std::vector<std::string> pathList = {pathInputFile};
      while (!pathList.empty())
      {
        std::string pathToAdd = pathList.back();
        pathList.pop_back();
        for (gz::common::DirIter file(pathToAdd);
          file != gz::common::DirIter(); ++file)
        {
          std::string current(*file);
          if (gz::common::isDirectory(current))
          {
            systemPaths->AddFilePaths(current);
            pathList.push_back(current);
          }
        }
      }

      auto stage = pxr::UsdStage::CreateInMemory();
      std::string modelName = model->Name();
      modelName = sdf::usd::validPath(modelName);
      auto modelPath = std::string("/" + modelName);
      auto usdErrors = sdf::usd::ParseSdfModel(
        *model,
        stage,
        modelPath,
        pxr::SdfPath(modelPath));
      if (!usdErrors.empty())
      {
        std::cerr << "The following errors occurred when parsing model ["
                  << modelName << "]:" << std::endl;
        for (const auto &e : usdErrors)
          std::cout << e << "\n";
        exit(-5);
      }

      if (!stage->GetRootLayer()->Export(_opt.outputFilename))
      {
        std::cerr << "Issue saving USD to " << _opt.outputFilename << "\n";
        exit(-6);
      }
      return;
    }
    else
    {
      std::cerr << _opt.inputFilename << " does not have exactly 1 world "
                << "or 1 model\n";
      exit(-3);
    }
  }

  auto world = root.WorldByIndex(0u);
  if (!world)
  {
    std::cerr << "Error retrieving the world from "
              << _opt.inputFilename << "\n";
    exit(-4);
  }

  auto stage = pxr::UsdStage::CreateInMemory();

  auto worldPath = std::string("/" + world->Name());
  worldPath = sdf::usd::validPath(worldPath);
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  if (!usdErrors.empty())
  {
    std::cerr << "The following errors occurred when parsing world ["
              << world->Name() << "]:" << std::endl;
    for (const auto &e : usdErrors)
      std::cout << e << "\n";
    exit(-5);
  }

  if (!stage->GetRootLayer()->Export(_opt.outputFilename))
  {
    std::cerr << "Issue saving USD to " << _opt.outputFilename << "\n";
    exit(-6);
  }
}

void addFlags(CLI::App &_app)
{
  auto opt = std::make_shared<Options>();

  _app.add_option("input",
    opt->inputFilename,
    "Input filename. Defaults to input.sdf unless otherwise specified."
    "Input file might be a world or a model");

  _app.add_option("output",
    opt->outputFilename,
    "Output filename. Defaults to output.usd unless otherwise specified.");

  _app.callback([opt](){
    runCommand(*opt);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"SDF to USD converter"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("--version", [](){
    std::cout << SDF_VERSION_FULL << std::endl;
    throw CLI::Success();
  });

  addFlags(app);
  CLI11_PARSE(app, argc, argv);

  return 0;
}
