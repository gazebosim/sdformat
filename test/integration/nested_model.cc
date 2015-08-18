/*
 * Copyright 2015 Open Source Robotics Foundation
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
#include <boost/filesystem.hpp>
#include <string>
#include "sdf/sdf.hh"

#include "test_config.h"

////////////////////////////////////////
// sdf model, version 1.4, use_parent_model_frame tag missing
// expect tag to be inserted with value true
TEST(NestedModel, NestedModel)
{
  std::ostringstream stream;
  std::string version = "1.5";
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='top_level_model'>"
    << "  <link name='parent'/>"
    << "  <link name='child'/>"
    << "  <model name='nested_model'>"
    << "    <link name='nested_link01'/>"
    << "  </model>"
    << "  <joint name='top_level_joint' type='revolute'>"
    << "    <parent>parent</parent>"
    << "    <child>child</child>"
    << "    <axis>"
    << "      <xyz>1 0 0</xyz>"
    << "    </axis>"
    << "  </joint>"
    << "</model>"
    << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing

  // top level model
  EXPECT_TRUE(sdfParsed.Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "top_level_model");

  // top level links
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linklElem = modelElem->GetElement("link");
  EXPECT_TRUE(linklElem->HasAttribute("name"));
  EXPECT_EQ(linklElem->Get<std::string>("name"), "parent");
  linklElem = linklElem->GetNextElement("link");
  EXPECT_TRUE(linklElem != NULL);
  EXPECT_TRUE(linklElem->HasAttribute("name"));
  EXPECT_EQ(linklElem->Get<std::string>("name"), "child");

  // nested model
  EXPECT_TRUE(modelElem->HasElement("model"));
  sdf::ElementPtr nestedModelElem = modelElem->GetElement("model");

  // nested model link
  EXPECT_TRUE(nestedModelElem->HasElement("link"));
  sdf::ElementPtr nestedLinkElem = nestedModelElem->GetElement("link");
  EXPECT_TRUE(nestedLinkElem->HasAttribute("name"));
  EXPECT_EQ(nestedLinkElem->Get<std::string>("name"), "nested_link01");

  // top level model joint
  EXPECT_TRUE(modelElem->HasElement("joint"));
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem->HasAttribute("name"));
  EXPECT_EQ(jointElem->Get<std::string>("name"), "top_level_joint");
  EXPECT_TRUE(jointElem->HasAttribute("type"));
  EXPECT_EQ(jointElem->Get<std::string>("type"), "revolute");

  // joint links
  EXPECT_TRUE(jointElem->HasElement("parent"));
  EXPECT_EQ(jointElem->Get<std::string>("parent"), "parent");
  EXPECT_TRUE(jointElem->HasElement("child"));
  EXPECT_EQ(jointElem->Get<std::string>("child"), "child");

  // joint axis
  EXPECT_TRUE(jointElem->HasElement("axis"));
  sdf::ElementPtr axisElem = jointElem->GetElement("axis");

  EXPECT_TRUE(axisElem->HasElement("xyz"));
  EXPECT_EQ(axisElem->Get<ignition::math::Vector3d>("xyz"),
      ignition::math::Vector3d(1, 0, 0));
}
