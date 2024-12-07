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

#include "sdf/parser.hh"

#include "test_config.hh"


class SDFSchemaGenerator : public testing::Test
{
  public:
    void runXMLlint(const std::string & model)
    {
      const auto sdfRootSchema = sdf::filesystem::append(SDF_ROOT_SCHEMA);
      std::string xmllintCmd = "xmllint --noout --schema " +
                                sdfRootSchema + " " + model;
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
  const std::string sdfTestPendulum =
      sdf::testing::TestFile("integration", "model", "double_pendulum.sdf");

  runXMLlint(sdfTestPendulum);
}

/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestPR2Model)
{
  const std::string sdfTestPr2 =
      sdf::testing::TestFile("integration", "model", "pr2.sdf");

  runXMLlint(sdfTestPr2);
}


/////////////////////////////////////////////////
TEST_F(SDFSchemaGenerator, TestTurtleBotModel)
{
  const std::string sdfTestTurtlebot =
      sdf::testing::TestFile("integration", "model", "turtlebot.sdf");

  runXMLlint(sdfTestTurtlebot);
}
