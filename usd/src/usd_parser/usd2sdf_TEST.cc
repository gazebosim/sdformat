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
#include <string>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/TempDirectory.hh>

#include <ignition/utilities/ExtraTestMacros.hh>

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

    EXPECT_EQ(4u, world->Plugins().size());
    // TODO(anyone) Check the remaining contents of outputUsdFilePath
    // when the parser is implemented
  }
}
