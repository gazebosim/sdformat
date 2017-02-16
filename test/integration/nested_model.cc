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
#include <string>
#include "sdf/sdf.hh"

#include "test_config.h"

////////////////////////////////////////
// Test parsing nested model with joint
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

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // top level model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
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

////////////////////////////////////////
// Test parsing nested model states
TEST(NestedModel, State)
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='model_00'>"
    << "  <pose>0 0 0.5 0 0 0</pose>"
    << "  <link name='link_00'>"
    << "    <pose>0 0 0.5 0 0 0</pose>"
    << "    <velocity>0.001 0 0 0 0 0</velocity>"
    << "    <acceleration>0 0.006121 0 0.012288 0 0.001751</acceleration>"
    << "    <wrench>0 0.006121 0 0 0 0</wrench>"
    << "  </link>"
    << "  <model name='model_01'>"
    << "    <pose>1 0 0.5 0 0 0</pose>"
    << "    <link name='link_01'>"
    << "      <pose>1.25 0 0.5 0 0 0</pose>"
    << "      <velocity>0 -0.001 0 0 0 0</velocity>"
    << "      <acceleration>0 0.000674 0 -0.001268 0 0</acceleration>"
    << "      <wrench>0 0.000674 0 0 0 0</wrench>"
    << "    </link>"
    << "    <model name='model_02'>"
    << "      <pose>1 1 0.5 0 0 0</pose>"
    << "      <link name='link_02'>"
    << "        <pose>1.25 1 0.5 0 0 0</pose>"
    << "        <velocity>0 0 0.001 0 0 0</velocity>"
    << "        <acceleration>0 0 0 0 0 0</acceleration>"
    << "        <wrench>0 0 0 0 0 0</wrench>"
    << "      </link>"
    << "    </model>"
    << "  </model>"
    << "</model>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(sdfStr.str(), sdfParsed));

  // load the state sdf
  EXPECT_TRUE(sdfParsed->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));

  sdf::ElementPtr modelStateElem = stateElem->GetElement("model");

  // model sdf
  EXPECT_TRUE(modelStateElem->HasAttribute("name"));
  EXPECT_EQ(modelStateElem->Get<std::string>("name"), "model_00");
  EXPECT_TRUE(modelStateElem->HasElement("pose"));
  EXPECT_EQ(modelStateElem->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(!modelStateElem->HasElement("joint"));

  // link sdf
  EXPECT_TRUE(modelStateElem->HasElement("link"));
  sdf::ElementPtr linkStateElem = modelStateElem->GetElement("link");
  EXPECT_TRUE(linkStateElem->HasAttribute("name"));
  EXPECT_EQ(linkStateElem->Get<std::string>("name"), "link_00");
  EXPECT_TRUE(linkStateElem->HasElement("pose"));
  EXPECT_EQ(linkStateElem->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(linkStateElem->HasElement("velocity"));
  EXPECT_EQ(linkStateElem->Get<ignition::math::Pose3d>("velocity"),
      ignition::math::Pose3d(0.001, 0, 0, 0, 0, 0));
  EXPECT_TRUE(linkStateElem->HasElement("acceleration"));
  EXPECT_EQ(linkStateElem->Get<ignition::math::Pose3d>("acceleration"),
      ignition::math::Pose3d(0, 0.006121, 0, 0.012288, 0, 0.001751));
  EXPECT_TRUE(linkStateElem->HasElement("wrench"));
  EXPECT_EQ(linkStateElem->Get<ignition::math::Pose3d>("wrench"),
      ignition::math::Pose3d(0, 0.006121, 0, 0, 0, 0));

  // nested model sdf
  EXPECT_TRUE(modelStateElem->HasElement("model"));
  sdf::ElementPtr nestedModelStateElem =
      modelStateElem->GetElement("model");
  EXPECT_TRUE(nestedModelStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedModelStateElem->Get<std::string>("name"), "model_01");
  EXPECT_TRUE(nestedModelStateElem->HasElement("pose"));
  EXPECT_EQ(nestedModelStateElem->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d(1, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(!nestedModelStateElem->HasElement("joint"));

  // nested model's link sdf
  EXPECT_TRUE(nestedModelStateElem->HasElement("link"));
  sdf::ElementPtr nestedLinkStateElem =
      nestedModelStateElem->GetElement("link");
  EXPECT_TRUE(nestedLinkStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedLinkStateElem->Get<std::string>("name"), "link_01");
  EXPECT_TRUE(nestedLinkStateElem->HasElement("pose"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d(1.25, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("velocity"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("velocity"),
      ignition::math::Pose3d(0, -0.001, 0, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("acceleration"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("acceleration"),
      ignition::math::Pose3d(0, 0.000674, 0, -0.001268, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("wrench"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("wrench"),
      ignition::math::Pose3d(0, 0.000674, 0, 0, 0, 0));

  // double nested model sdf
  EXPECT_TRUE(nestedModelStateElem->HasElement("model"));
  nestedModelStateElem = nestedModelStateElem->GetElement("model");
  EXPECT_TRUE(nestedModelStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedModelStateElem->Get<std::string>("name"), "model_02");
  EXPECT_TRUE(nestedModelStateElem->HasElement("pose"));
  EXPECT_EQ(nestedModelStateElem->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d(1, 1, 0.5, 0, 0, 0));
  EXPECT_TRUE(!nestedModelStateElem->HasElement("joint"));

  // double nested model's link sdf
  EXPECT_TRUE(nestedModelStateElem->HasElement("link"));
  nestedLinkStateElem = nestedModelStateElem->GetElement("link");
  EXPECT_TRUE(nestedLinkStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedLinkStateElem->Get<std::string>("name"), "link_02");
  EXPECT_TRUE(nestedLinkStateElem->HasElement("pose"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d(1.25, 1, 0.5, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("velocity"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("velocity"),
      ignition::math::Pose3d(0, 0, 0.001, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("acceleration"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("acceleration"),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("wrench"));
  EXPECT_EQ(nestedLinkStateElem->Get<ignition::math::Pose3d>("wrench"),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
}
