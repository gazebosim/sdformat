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

#include <ignition/common/Filesystem.hh>
#include <ignition/common/TempDirectory.hh>
#include <ignition/common/Util.hh>

#include <ignition/utils/ExtraTestMacros.hh>

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
  return ignition::common::joinPaths(std::string(PROJECT_BINARY_DIR), "bin",
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
  const auto tmp = ignition::common::createTempDirectory("usd",
      ignition::common::tempDirectoryPath());

  auto systemPaths = ignition::common::systemPaths();
  systemPaths->AddFilePaths(ignition::common::joinPaths(
    sdf::testing::TestFile("usd"), "materials", "textures"));
  // Check a good SDF file
  {
    const std::string path = sdf::testing::TestFile("usd", "upAxisZ.usda");
    const auto outputSdfFilePath =
      ignition::common::joinPaths(tmp, "upAxisZ.sdf");
    EXPECT_FALSE(ignition::common::isFile(outputSdfFilePath));
    const std::string output =
      custom_exec_str(usd2sdfCommand() + " " + path + " " + outputSdfFilePath);

    // make sure that a sdf file was generated
    ASSERT_TRUE(ignition::common::isFile(outputSdfFilePath)) << output;

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
    EXPECT_EQ("ignition::gazebo::systems::Physics", plugins[0].Name());
    EXPECT_EQ("ignition-gazebo-physics-system", plugins[0].Filename());

    EXPECT_EQ("ignition::gazebo::systems::Sensors", plugins[1].Name());
    EXPECT_EQ("ignition-gazebo-sensors-system", plugins[1].Filename());

    EXPECT_EQ("ignition::gazebo::systems::UserCommands", plugins[2].Name());
    EXPECT_EQ("ignition-gazebo-user-commands-system", plugins[2].Filename());

    EXPECT_EQ("ignition::gazebo::systems::SceneBroadcaster", plugins[3].Name());
    EXPECT_EQ(
      "ignition-gazebo-scene-broadcaster-system", plugins[3].Filename());

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

    // check that models have the right links
    std::function<void(const std::string &, const std::string &)> checkLink =
      [&world](const std::string &_modelName, const std::string &_linkName)
      {
        const auto modelPtr = world->ModelByName(_modelName);
        ASSERT_NE(nullptr, modelPtr);
        EXPECT_EQ(1u, modelPtr->LinkCount());
        EXPECT_NE(nullptr, modelPtr->LinkByName(_linkName));
      };
    checkLink("ground_plane", "link");
    checkLink("box", "box_link");
    checkLink("cylinder", "cylinder_link");
    checkLink("sphere", "sphere_link");
    checkLink("capsule", "capsule_link");
    checkLink("ellipsoid", "ellipsoid_link");
  }
}
