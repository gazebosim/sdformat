/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include "sdf/sdf.hh"

#include "test_config.h"

const std::string SDF_SIMPLE_TEST_FILE =
  std::string(PROJECT_SOURCE_PATH) + "/test/integration/simple.sdf";
const std::string URDF_SIMPLE_RESULT_FILE =
  std::string(PROJECT_SOURCE_PATH) + "/test/integration/simple.urdf";

/////////////////////////////////////////////////
TEST(ConvertToURDF, ConvertSimple)
{
  std::string result;
  std::cout << SDF_SIMPLE_TEST_FILE << "\n";
  EXPECT_TRUE(
      sdf::ConvertToURDF::ConvertFile(SDF_SIMPLE_TEST_FILE.c_str(), result));

  std::ifstream fileStrm(URDF_SIMPLE_RESULT_FILE, std::ios::in);
  std::ostringstream expected;
  expected << fileStrm.rdbuf();
  fileStrm.close();

  EXPECT_EQ(expected.str(), result);
}
