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

#include "test_config.h"
#include "test_utils.hh"

#ifdef _WIN32
  #define popen  _popen
  #define pclose _pclose
#endif

static std::string sdf2usdCommand()
{
  return std::string(IGN_PATH) + "/sdf2usd";
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
TEST(check_cmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase = ignition::common::joinPaths(pathBase, "test", "sdf");

  auto tmpDir = ignition::common::tempDirectoryPath();
  auto tmp = ignition::common::createTempDirectory("usd", tmpDir);
  // Check a good SDF file
  {
    std::string path = ignition::common::joinPaths(pathBase,
      "/shapes_world.sdf");
    std::string output =
      custom_exec_str(sdf2usdCommand() + " " + path + " " +
        ignition::common::joinPaths(tmp, "shapes.usd"));
    // make sure that a shapes.usd file was generated
    EXPECT_TRUE(ignition::common::isFile(
      ignition::common::joinPaths(tmp, "shapes.usd")));
    // TODO(ahcorde): Check the output when the parser is implemented
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
