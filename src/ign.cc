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
#include <string.h>

#include "sdf/sdf_config.h"
#include "sdf/Filesystem.hh"
#include "sdf/Root.hh"
#include "sdf/parser.hh"
#include "sdf/system_util.hh"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "ign.hh"

//////////////////////////////////////////////////
// cppcheck-suppress unusedFunction
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
// cppcheck-suppress unusedFunction
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
// cppcheck-suppress unusedFunction
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
// cppcheck-suppress unusedFunction
extern "C" SDFORMAT_VISIBLE int cmdPrint(const char *_path)
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

  sdf->PrintValues();

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
