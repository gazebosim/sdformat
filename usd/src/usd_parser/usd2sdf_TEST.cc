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
#include <set>
#include <string>

#include <gtest/gtest.h>

#include <gz/common/Filesystem.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/common/Util.hh>

#include <gz/utils/ExtraTestMacros.hh>

#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "test_config.h"
#include "test_utils.hh"

#ifdef _WIN32
  #define popen  _popen
  #define pclose _pclose
#endif

static std::string usd2sdfCommand()
{
  return gz::common::joinPaths(std::string(PROJECT_BINARY_DIR), "bin",
      "usd2sdf");
}

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(version_cmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string output =
    custom_exec_str(usd2sdfCommand() + " --version");
  EXPECT_EQ(output, std::string(SDF_VERSION_FULL) + "\n");
}

/////////////////////////////////////////////////
TEST(check_cmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  const auto tmp = gz::common::createTempDirectory("usd",
      gz::common::tempDirectoryPath());

  auto systemPaths = gz::common::systemPaths();
  systemPaths->AddFilePaths(gz::common::joinPaths(
    sdf::testing::TestFile("usd"), "materials", "textures"));
  // Check a good SDF file
  {
    const std::string path = sdf::testing::TestFile("usd", "upAxisZ.usda");
    const auto outputSdfFilePath =
      gz::common::joinPaths(tmp, "upAxisZ.sdf");
    EXPECT_FALSE(gz::common::isFile(outputSdfFilePath));
    const std::string output =
      custom_exec_str(usd2sdfCommand() + " " + path + " " + outputSdfFilePath);

    // make sure that a sdf file was generated
    ASSERT_TRUE(gz::common::isFile(outputSdfFilePath)) << output;

    // check the contents of the generated SDF file
    sdf::Root root;
    const auto errors = root.Load(outputSdfFilePath);
    EXPECT_TRUE(errors.empty());

    // check the value of the gravity element
    ASSERT_EQ(1u, root.WorldCount());
    const auto world = root.WorldByIndex(0u);
    ASSERT_NE(nullptr, world);
    EXPECT_DOUBLE_EQ(0.0, world->Gravity()[0]);
    EXPECT_DOUBLE_EQ(0.0, world->Gravity()[1]);
    EXPECT_DOUBLE_EQ(-0.098, world->Gravity()[2]);

    auto plugins = world->Plugins();
    EXPECT_EQ(4u, plugins.size());
    EXPECT_EQ("gz::gazebo::systems::Physics", plugins[0].Name());
    EXPECT_EQ("ignition-gazebo-physics-system", plugins[0].Filename());

    EXPECT_EQ("gz::gazebo::systems::Sensors", plugins[1].Name());
    EXPECT_EQ("ignition-gazebo-sensors-system", plugins[1].Filename());

    EXPECT_EQ("gz::gazebo::systems::UserCommands", plugins[2].Name());
    EXPECT_EQ("ignition-gazebo-user-commands-system", plugins[2].Filename());

    EXPECT_EQ("gz::gazebo::systems::SceneBroadcaster", plugins[3].Name());
    EXPECT_EQ(
      "ignition-gazebo-scene-broadcaster-system", plugins[3].Filename());

    // the world should have lights attached to it
    std::set<std::string> savedLightNames;
    for (unsigned int i = 0; i < world->LightCount(); ++i)
      savedLightNames.insert(world->LightByIndex(i)->Name());
    EXPECT_EQ(3u, savedLightNames.size());

    EXPECT_NE(savedLightNames.end(), savedLightNames.find("defaultLight"));
    EXPECT_NE(savedLightNames.end(), savedLightNames.find("diskLight"));
    EXPECT_NE(savedLightNames.end(), savedLightNames.find("sun"));

    // make sure all models in the USD file were correctly parsed to SDF
    std::set<std::string> savedModelNames;
    for (unsigned int i = 0; i < world->ModelCount(); ++i)
      savedModelNames.insert(world->ModelByIndex(i)->Name());
    EXPECT_EQ(6u, savedModelNames.size());

    EXPECT_NE(savedModelNames.end(), savedModelNames.find("ground_plane"));
    EXPECT_NE(savedModelNames.end(), savedModelNames.find("box"));
    EXPECT_NE(savedModelNames.end(), savedModelNames.find("cylinder"));
    EXPECT_NE(savedModelNames.end(), savedModelNames.find("sphere"));
    EXPECT_NE(savedModelNames.end(), savedModelNames.find("capsule"));
    EXPECT_NE(savedModelNames.end(), savedModelNames.find("ellipsoid"));

    // check for static/non-static models
    ASSERT_NE(nullptr, world->ModelByName("ground_plane"));
    EXPECT_TRUE(world->ModelByName("ground_plane")->Static());
    ASSERT_NE(nullptr, world->ModelByName("box"));
    EXPECT_FALSE(world->ModelByName("box")->Static());

    // Check that models have the right links.
    // If a link should have a light attached to it, check that as well
    std::function<void(const std::string &, const std::string &,
        const std::string &)> checkLink =
      [&world](const std::string &_modelName, const std::string &_linkName,
          const std::string &_lightLinkName)
      {
        const auto modelPtr = world->ModelByName(_modelName);
        ASSERT_NE(nullptr, modelPtr);
        EXPECT_EQ(1u, modelPtr->LinkCount());
        const auto modelLink = modelPtr->LinkByName(_linkName);
        ASSERT_NE(nullptr, modelLink);

        if (!_lightLinkName.empty())
          EXPECT_NE(nullptr, modelLink->LightByName(_lightLinkName));
        else
          EXPECT_EQ(0u, modelLink->LightCount());
      };
    checkLink("ground_plane", "link", "");
    checkLink("box", "box_link", "boxModelLight");
    checkLink("cylinder", "cylinder_link", "");
    checkLink("sphere", "sphere_link", "");
    checkLink("capsule", "capsule_link", "");
    checkLink("ellipsoid", "ellipsoid_link", "");
  }
}
