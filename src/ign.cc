/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <string.h>
#include <vector>

#include "sdf/sdf_config.h"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/parser.hh"
#include "sdf/PrintConfig.hh"
#include "sdf/system_util.hh"

#include "ignition/math/Inertial.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "ign.hh"

//////////////////////////////////////////////////
extern "C" SDFORMAT_VISIBLE int cmdCheck(const char *_path)
{
  int result = 0;

  sdf::Root root;
  sdf::Errors errors = root.Load(_path);
  if (!errors.empty())
  {
    for (auto &error : errors)
    {
      std::cerr << error << std::endl;
    }
    return -1;
  }

  if (!sdf::checkCanonicalLinkNames(&root))
  {
    result = -1;
  }

  if (!sdf::checkJointParentChildLinkNames(&root))
  {
    result = -1;
  }

  if (!sdf::checkFrameAttachedToGraph(&root))
  {
    result = -1;
  }

  if (!sdf::checkPoseRelativeToGraph(&root))
  {
    result = -1;
  }

  if (!sdf::recursiveSiblingUniqueNames(root.Element()))
  {
    result = -1;
  }

  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    return -1;
  }

  sdf::SDFPtr sdf(new sdf::SDF());

  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    return -1;
  }

  if (!sdf::readFile(_path, sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed.\n";
    return -1;
  }

  if (result == 0)
  {
    std::cout << "Valid.\n";
  }
  return result;
}

//////////////////////////////////////////////////
extern "C" SDFORMAT_VISIBLE char *ignitionVersion()
{
#ifdef _MSC_VER
  return _strdup(SDF_VERSION_FULL);
#else
  return strdup(SDF_VERSION_FULL);
#endif
}

//////////////////////////////////////////////////
/// \brief Print the full description of the SDF spec.
/// \return 0 on success, -1 if SDF could not be initialized.
extern "C" SDFORMAT_VISIBLE int cmdDescribe(const char *_version)
{
  sdf::SDFPtr sdf(new sdf::SDF());

  if (nullptr != _version)
  {
    sdf->Version(_version);
  }
  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    return -1;
  }

  sdf->PrintDescription();

  return 0;
}

//////////////////////////////////////////////////
extern "C" SDFORMAT_VISIBLE int cmdPrint(const char *_path,
    int inDegrees, int snapToDegrees, float snapTolerance)
{
  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    return -1;
  }

  sdf::SDFPtr sdf(new sdf::SDF());

  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    return -1;
  }

  if (!sdf::readFile(_path, sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed.\n";
    return -1;
  }

  sdf::PrintConfig config;
  if (inDegrees!= 0)
  {
    config.SetRotationInDegrees(true);
  }
  if (snapToDegrees > 0)
  {
    config.SetRotationSnapToDegrees(static_cast<unsigned int>(snapToDegrees),
                                    static_cast<double>(snapTolerance));
  }

  sdf->PrintValues(config);
  return 0;
}

//////////////////////////////////////////////////
extern "C" SDFORMAT_VISIBLE int cmdPrintPreserveIncludes(const char *_path)
{
  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    return -1;
  }

  sdf::SDFPtr sdf(new sdf::SDF());

  if (!sdf::init(sdf))
  {
    std::cerr << "Error: SDF schema initialization failed.\n";
    return -1;
  }

  if (!sdf::readFile(_path, sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed.\n";
    return -1;
  }

  sdf::PrintConfig config;
  config.SetPreserveIncludes(true);
  sdf->PrintValues(config);

  return 0;
}

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE int cmdGraph(
    const char *_graphType, const char *_path)
{
  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    return -1;
  }

  sdf::Root root;
  sdf::Errors errors = root.Load(_path);
  if (!errors.empty())
  {
    std::cerr << errors << std::endl;
  }

  if (std::strcmp(_graphType, "pose") == 0)
  {
    auto ownedGraph = std::make_shared<sdf::PoseRelativeToGraph>();
    sdf::ScopedGraph<sdf::PoseRelativeToGraph> graph(ownedGraph);
    if (root.WorldCount() > 0)
    {
      errors = sdf::buildPoseRelativeToGraph(graph, root.WorldByIndex(0));
    }
    else if (root.Model() != nullptr)
    {
      errors =
        sdf::buildPoseRelativeToGraph(graph, root.Model());
    }

    if (!errors.empty())
    {
      std::cerr << errors << std::endl;
    }
    std::cout << graph.Graph() << std::endl;
  }
  else if (std::strcmp(_graphType, "frame") == 0)
  {
    auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
    sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
    if (root.WorldCount() > 0)
    {
      errors = sdf::buildFrameAttachedToGraph(graph, root.WorldByIndex(0));
    }
    else if (root.Model() != nullptr)
    {
      errors =
        sdf::buildFrameAttachedToGraph(graph, root.Model());
    }

    if (!errors.empty())
    {
      std::cerr << errors << std::endl;
    }
    std::cout << graph.Graph() << std::endl;
  }
  else
  {
    std::cerr << R"(Only "pose" and "frame" graph types are supported)"
              << std::endl;
  }

  return 0;
}

//////////////////////////////////////////////////
extern "C" SDFORMAT_VISIBLE int cmdInertialStats(
    const char *_path)
{
  if (!sdf::filesystem::exists(_path))
  {
    std::cerr << "Error: File [" << _path << "] does not exist.\n";
    return -1;
  }

  sdf::Root root;
  sdf::Errors errors = root.Load(_path);
  if (!errors.empty())
  {
    std::cerr << errors << std::endl;
  }

  if (root.WorldCount() > 0)
  {
    std::cerr << "Error: Expected a model file but received a world file."
            << std::endl;
    return -1;
  }

  const sdf::Model *model = root.Model();
  if (!model)
  {
    std::cerr << "Error: Could not find the model." << std::endl;
    return -1;
  }

  if (model->ModelCount() > 0)
  {
    std::cout << "Warning: Inertial properties of links in nested"
            " models will not be included." << std::endl;
  }

  ignition::math::Inertiald totalInertial;

  for (uint64_t i = 0; i < model->LinkCount(); i++)
  {
    ignition::math::Inertiald currentLinkInertial;
    model->LinkByIndex(i)->ResolveInertial(currentLinkInertial, "__model__");

    totalInertial += currentLinkInertial;
  }

  auto totalMass = totalInertial.MassMatrix().Mass();
  auto xCentreOfMass = totalInertial.Pose().Pos().X();
  auto yCentreOfMass = totalInertial.Pose().Pos().Y();
  auto zCentreOfMass = totalInertial.Pose().Pos().Z();

  std::cout << "Inertial statistics for model: " << model->Name() << std::endl;
  std::cout << "---" << std::endl;
  std::cout << "Total mass of the model: " << totalMass << std::endl;
  std::cout << "---" << std::endl;

  std::cout << "Centre of mass in model frame: " << std::endl;
  std::cout << "X: " << xCentreOfMass << std::endl;
  std::cout << "Y: " << yCentreOfMass << std::endl;
  std::cout << "Z: " << zCentreOfMass << std::endl;
  std::cout << "---" << std::endl;

  std::cout << "Moment of inertia matrix: " << std::endl;

  // Pretty print the MOI matrix
  std::stringstream ss;
  ss << totalInertial.Moi();

  std::string s;
  size_t maxLength = 0u;
  std::vector<std::string> moiVector;
  while ( std::getline(ss, s, ' ' ) )
  {
    moiVector.push_back(s);
    if (s.size() > maxLength)
    {
      maxLength = s.size();
    }
  }

  for (int i = 0; i < 9; i++)
  {
    size_t spacePadding = maxLength - moiVector[i].size();
    // Print the matrix element
    std::cout << moiVector[i];
    for (size_t j = 0; j < spacePadding; j++)
    {
      std::cout << " ";
    }
    // Add space for the next element
    std::cout << "  ";
    // Add '\n' if the next row is about to start
    if ((i+1)%3 == 0)
    {
      std::cout << "\n";
    }
  }
  std::cout << "---" << std::endl;

  return 0;
}
