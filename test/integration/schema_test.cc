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

#include <cstdlib>
#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

const std::string SDF_ROOT_SCHEMA =
  sdf::filesystem::append(PROJECT_BINARY_DIR, "sdf", SDF_PROTOCOL_VERSION,
                          "root.xsd");

const std::string SDF_TEST_PR2 =
  sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration", "model",
                          "pr2.sdf");

const std::string SDF_TEST_TURTLEBOT =
  sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration", "model",
                          "turtlebot.sdf");

const std::string SDF_TEST_PENDULUM =
  sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration", "model",
                          "double_pendulum.sdf");

const std::string SDF_TEST_NESTED_MODEL =
  sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
                          "nested_canonical_link.sdf");

const std::string SDF_TEST_STATE =
  sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration", "model",
                          "world_with_state.sdf");


class SDFSchemaGenerator : public testing::Test
{
  public:
    void runXMLlint(const std::string & model)
    {
      std::string xmllintCmd = "xmllint --noout --schema " +
                                SDF_ROOT_SCHEMA + " " + model;
      std::cout << "CMD[" << xmllintCmd << "]\n";
      if (system(xmllintCmd.c_str()) != 0)
      {
        FAIL() << "Fail in parsing the model";
      }
      else
      {
        SUCCEED();
      }
    }
};

/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestDoublePendulum)
{
  runXMLlint(SDF_TEST_PENDULUM);
}

/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestPR2Model)
{
  runXMLlint(SDF_TEST_PR2);
}

/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestTurtleBotModel)
{
  runXMLlint(SDF_TEST_TURTLEBOT);
}

/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestNestedModel)
{
  runXMLlint(SDF_TEST_NESTED_MODEL);
}

/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestState)
{
  runXMLlint(SDF_TEST_STATE);
}
