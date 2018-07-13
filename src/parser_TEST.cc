/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <string>
#include <sstream>

#include "test_config.h"
#include "sdf/sdf.hh"
#include "sdf/parser.hh"
#include "sdf/Element.hh"

/////////////////////////////////////////////////
TEST(parser, initStringTrim)
{
  sdf::SDFPtr sdf(new sdf::SDF());
  std::ostringstream stream;
  stream << "<element name=\"visual\" required=\"*\" "
         <<          "type=\"string\" default=\"_default_value_\">"
         << "  <attribute name=\"name\" type=\"string\" required=\" 1\""
         << "             default=\"__default__\">"
         << "    <description>test</description>"
         << "  </attribute>"
         << "</element>";

  EXPECT_TRUE(sdf::initString(stream.str(), sdf));
  sdf::ElementPtr root = sdf->Root();
  EXPECT_TRUE(root != nullptr);

  EXPECT_EQ("visual", root->GetName());
  EXPECT_EQ("*", root->GetRequired());
  sdf::ParamPtr value = root->GetValue();
  ASSERT_NE(nullptr, value);
  EXPECT_EQ("string", value->GetTypeName());
  EXPECT_EQ("_default_value_", value->GetDefaultAsString());

  sdf::ParamPtr attr = root->GetAttribute("name");
  EXPECT_TRUE(attr != nullptr);
  EXPECT_TRUE(attr->GetRequired());
}

/////////////////////////////////////////////////
TEST(Parser, ParseERB)
{
  std::stringstream testFile;
  testFile << PROJECT_SOURCE_PATH << "/test/integration/erb_test.sdf.erb";

  char *pathCStr = getenv("SDF_PATH");
  std::stringstream path;
  path << PROJECT_SOURCE_PATH << "/sdf/" << SDF_VERSION;
  setenv("SDF_PATH", path.str().c_str(), 1);

  // Parse the ERB file
  std::string parsed;
  EXPECT_TRUE(sdf::erbFile(testFile.str(), parsed));

  // Make sure erb completion took place.
  EXPECT_TRUE(parsed.find("<pose>0 0 0.005 0 0 0</pose>") != std::string::npos);

  // Read the parsed string into an SDF object
  sdf::SDFPtr p(new sdf::SDF());
  sdf::init(p);
  EXPECT_TRUE(sdf::readString(parsed, p));

  // There should be a model
  EXPECT_TRUE(p->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = p->Root()->GetElement("model");
  ASSERT_TRUE(modelElem != NULL);

  // The model should have a link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_TRUE(linkElem != NULL);

  // The link should have a pose
  EXPECT_TRUE(linkElem->HasElement("pose"));
  sdf::ElementPtr poseElem = linkElem->GetElement("pose");
  ASSERT_TRUE(poseElem != NULL);

  // The pose.pos.z should equal 0.005
  auto pose = linkElem->Get<ignition::math::Pose3d>("pose");
  EXPECT_DOUBLE_EQ(pose.Pos().Z(), 0.005);

  if (pathCStr)
  {
    setenv("SDF_PATH", pathCStr, 1);
  }
  else
  {
    unsetenv("SDF_PATH");
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
