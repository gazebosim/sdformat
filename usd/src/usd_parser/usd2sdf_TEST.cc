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
  // Check a good SDF file
  {
    std::string output =
      custom_exec_str(usd2sdfCommand() + " --version");

    EXPECT_EQ(output, std::string(SDF_VERSION_FULL) + "\n");

    // TODO(ahcorde): Check the contents of outputUsdFilePath when the parser
    // is implemented
  }
}

/////////////////////////////////////////////////
TEST(check_cmd, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  auto tmpDir = ignition::common::tempDirectoryPath();
  auto tmp = ignition::common::createTempDirectory("usd", tmpDir);
  // Check a good SDF file
  {
    std::string path = sdf::testing::TestFile("usd", "upAxisZ.usda");
    const auto outputUsdFilePath =
      ignition::common::joinPaths(tmp, "upAxisZ.sdf");
    EXPECT_FALSE(ignition::common::isFile(outputUsdFilePath));
    std::string output =
      custom_exec_str(usd2sdfCommand() + " " + path + " " + outputUsdFilePath);

    // make sure that a shapes.usd file was generated
    EXPECT_TRUE(ignition::common::isFile(outputUsdFilePath)) << output;

    // TODO(anyone): Check the contents of outputUsdFilePath when the parser
    // is implemented
  }
}
