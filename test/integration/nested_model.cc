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

#include <sstream>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

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
  EXPECT_TRUE(linklElem != nullptr);
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
  EXPECT_EQ(axisElem->Get<gz::math::Vector3d>("xyz"),
    gz::math::Vector3d(1, 0, 0));
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
  EXPECT_EQ(modelStateElem->Get<gz::math::Pose3d>("pose"),
    gz::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(!modelStateElem->HasElement("joint"));

  // link sdf
  EXPECT_TRUE(modelStateElem->HasElement("link"));
  sdf::ElementPtr linkStateElem = modelStateElem->GetElement("link");
  EXPECT_TRUE(linkStateElem->HasAttribute("name"));
  EXPECT_EQ(linkStateElem->Get<std::string>("name"), "link_00");
  EXPECT_TRUE(linkStateElem->HasElement("pose"));
  EXPECT_EQ(linkStateElem->Get<gz::math::Pose3d>("pose"),
    gz::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(linkStateElem->HasElement("velocity"));
  EXPECT_EQ(linkStateElem->Get<gz::math::Pose3d>("velocity"),
    gz::math::Pose3d(0.001, 0, 0, 0, 0, 0));
  EXPECT_TRUE(linkStateElem->HasElement("acceleration"));
  EXPECT_EQ(linkStateElem->Get<gz::math::Pose3d>("acceleration"),
    gz::math::Pose3d(0, 0.006121, 0, 0.012288, 0, 0.001751));
  EXPECT_TRUE(linkStateElem->HasElement("wrench"));
  EXPECT_EQ(linkStateElem->Get<gz::math::Pose3d>("wrench"),
    gz::math::Pose3d(0, 0.006121, 0, 0, 0, 0));

  // nested model sdf
  EXPECT_TRUE(modelStateElem->HasElement("model"));
  sdf::ElementPtr nestedModelStateElem =
    modelStateElem->GetElement("model");
  EXPECT_TRUE(nestedModelStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedModelStateElem->Get<std::string>("name"), "model_01");
  EXPECT_TRUE(nestedModelStateElem->HasElement("pose"));
  EXPECT_EQ(nestedModelStateElem->Get<gz::math::Pose3d>("pose"),
    gz::math::Pose3d(1, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(!nestedModelStateElem->HasElement("joint"));

  // nested model's link sdf
  EXPECT_TRUE(nestedModelStateElem->HasElement("link"));
  sdf::ElementPtr nestedLinkStateElem =
    nestedModelStateElem->GetElement("link");
  EXPECT_TRUE(nestedLinkStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedLinkStateElem->Get<std::string>("name"), "link_01");
  EXPECT_TRUE(nestedLinkStateElem->HasElement("pose"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("pose"),
    gz::math::Pose3d(1.25, 0, 0.5, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("velocity"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("velocity"),
    gz::math::Pose3d(0, -0.001, 0, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("acceleration"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("acceleration"),
    gz::math::Pose3d(0, 0.000674, 0, -0.001268, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("wrench"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("wrench"),
    gz::math::Pose3d(0, 0.000674, 0, 0, 0, 0));

  // double nested model sdf
  EXPECT_TRUE(nestedModelStateElem->HasElement("model"));
  nestedModelStateElem = nestedModelStateElem->GetElement("model");
  EXPECT_TRUE(nestedModelStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedModelStateElem->Get<std::string>("name"), "model_02");
  EXPECT_TRUE(nestedModelStateElem->HasElement("pose"));
  EXPECT_EQ(nestedModelStateElem->Get<gz::math::Pose3d>("pose"),
    gz::math::Pose3d(1, 1, 0.5, 0, 0, 0));
  EXPECT_TRUE(!nestedModelStateElem->HasElement("joint"));

  // double nested model's link sdf
  EXPECT_TRUE(nestedModelStateElem->HasElement("link"));
  nestedLinkStateElem = nestedModelStateElem->GetElement("link");
  EXPECT_TRUE(nestedLinkStateElem->HasAttribute("name"));
  EXPECT_EQ(nestedLinkStateElem->Get<std::string>("name"), "link_02");
  EXPECT_TRUE(nestedLinkStateElem->HasElement("pose"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("pose"),
    gz::math::Pose3d(1.25, 1, 0.5, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("velocity"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("velocity"),
    gz::math::Pose3d(0, 0, 0.001, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("acceleration"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("acceleration"),
    gz::math::Pose3d(0, 0, 0, 0, 0, 0));
  EXPECT_TRUE(nestedLinkStateElem->HasElement("wrench"));
  EXPECT_EQ(nestedLinkStateElem->Get<gz::math::Pose3d>("wrench"),
    gz::math::Pose3d(0, 0, 0, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a include element that has a pose element and includes a
// submodel
TEST(NestedModel, IncludeFlatteningNames)
{
  const std::string MODEL_PATH =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "model", "nested_names_test");

  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='top_level_model'>"
    << "  <include>"
    << "    <uri>" + MODEL_PATH +"</uri>"
    << "  </include>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_EQ(modelElem->Get<std::string>("name"), "top_level_model");

  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_EQ(linkElem->Get<std::string>("name"), "main_model_prefix::frame");

  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_EQ(jointElem->Get<std::string>("name"), "main_model_prefix::joint1");
  EXPECT_EQ(jointElem->Get<std::string>("parent"),
    "main_model_prefix::subnested_model");
  EXPECT_EQ(jointElem->Get<std::string>("child"),
    "main_model_prefix::subnested_model::link1") <<
    "Flattening logic for nested models failed (check parser.cc)";

  sdf::ElementPtr joint2Elem = jointElem->GetNextElement("joint");
  EXPECT_EQ(joint2Elem->Get<std::string>("name"), "main_model_prefix::joint2");
  EXPECT_EQ(joint2Elem->Get<std::string>("parent"),
    "main_model_prefix::subnested_model::link1") <<
    "Flattening logic for nested models failed (check parser.cc)";
  EXPECT_EQ(joint2Elem->Get<std::string>("child"),
    "main_model_prefix::joint1") <<
    "Flattening logic for nested models failed (check parser.cc)";
}

////////////////////////////////////////
// Test parsing models with joints nested via <include>
// Confirm that joint axis rotation is handled differently for 1.4 and 1.5+
TEST(NestedModel, NestedInclude)
{
  const std::string name = "double_pendulum_kinematics";
  const std::string MODEL_PATH = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  // this uses two models from the test/integration/model folder that are
  // identical except for the //sdf/@version attribute
  // * version 1.5: double_pendulum_with_base
  // * version 1.4: double_pendulum_with_base_14
  //
  // 1. double_pendulum_with_base is included directly into the world
  //    with the following pose
  const gz::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);
  // 2. it's also included into the model named "include_with_rotation"
  //    with the following pose
  const gz::math::Pose3d model2Pose(-10, 0, 0, 0, 0, IGN_PI/2);
  // 3. double_pendulum_with_base_14 is included into
  //    the model named "include_with_rotation_1.4" with the following pose
  const gz::math::Pose3d model3Pose(0, 10, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.5";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <include>"
    << "    <uri>" + MODEL_PATH + "</uri>"
    << "    <pose>" << model1Pose << "</pose>"
    << "  </include>"
    << "  <model name='include_with_rotation'>"
    << "    <include>"
    << "      <uri>" + MODEL_PATH + "</uri>"
    << "      <pose>" << model2Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "  <model name='include_with_rotation_1.4'>"
    << "    <include>"
    << "      <uri>" + MODEL_PATH + "_14</uri>"
    << "      <pose>" << model3Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfParsed);
  EXPECT_TRUE(errors.empty());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(version, world->Element()->OriginalVersion());

  const sdf::Model *model1 = world->ModelByName(name);
  const sdf::Model *model2 = world->ModelByName("include_with_rotation");
  const sdf::Model *model3 = world->ModelByName("include_with_rotation_1.4");
  ASSERT_NE(nullptr, model1);
  ASSERT_NE(nullptr, model2);
  ASSERT_NE(nullptr, model3);

  // model1Pose is used at //model[0]/pose
  // but model2Pose and model3Pose are applied to the link poses, so those
  // //model/pose values should be identity
  EXPECT_EQ(model1Pose, model1->RawPose());
  EXPECT_EQ(gz::math::Pose3d::Zero, model2->RawPose());
  EXPECT_EQ(gz::math::Pose3d::Zero, model3->RawPose());
  // expect empty //pose/@relative_to
  EXPECT_TRUE(model1->PoseRelativeTo().empty());
  EXPECT_TRUE(model2->PoseRelativeTo().empty());
  EXPECT_TRUE(model3->PoseRelativeTo().empty());

  // each model has 3 links, and the link names of the nested models have
  // been transformed
  EXPECT_EQ(3u, model1->LinkCount());
  EXPECT_EQ(3u, model2->LinkCount());
  EXPECT_EQ(3u, model3->LinkCount());
  const sdf::Link *baseLink1 = model1->LinkByName("base");
  const sdf::Link *baseLink2 = model2->LinkByName(name + "::base");
  const sdf::Link *baseLink3 = model3->LinkByName(name + "_14::base");
  ASSERT_NE(nullptr, baseLink1);
  ASSERT_NE(nullptr, baseLink2);
  ASSERT_NE(nullptr, baseLink3);
  const sdf::Link *lowerLink1 = model1->LinkByName("lower_link");
  const sdf::Link *lowerLink2 = model2->LinkByName(name + "::lower_link");
  const sdf::Link *lowerLink3 = model3->LinkByName(name + "_14::lower_link");
  ASSERT_NE(nullptr, lowerLink1);
  ASSERT_NE(nullptr, lowerLink2);
  ASSERT_NE(nullptr, lowerLink3);
  const sdf::Link *upperLink1 = model1->LinkByName("upper_link");
  const sdf::Link *upperLink2 = model2->LinkByName(name + "::upper_link");
  const sdf::Link *upperLink3 = model3->LinkByName(name + "_14::upper_link");
  ASSERT_NE(nullptr, upperLink1);
  ASSERT_NE(nullptr, upperLink2);
  ASSERT_NE(nullptr, upperLink3);

  // expect link poses from model2 and model3 to match raw link poses
  // from model1.
  EXPECT_EQ(baseLink2->RawPose(), baseLink1->RawPose());
  EXPECT_EQ(baseLink3->RawPose(), baseLink1->RawPose());
  EXPECT_EQ(lowerLink2->RawPose(), lowerLink1->RawPose());
  EXPECT_EQ(lowerLink3->RawPose(), lowerLink1->RawPose());
  EXPECT_EQ(upperLink2->RawPose(), upperLink1->RawPose());
  EXPECT_EQ(upperLink3->RawPose(), upperLink1->RawPose());
  // expect //pose/@relative_to to contain the name of the nested model frame
  // injected during parsing. model1 is not nested, so it's links should have
  // empty //pose/@relative_to
  EXPECT_TRUE(baseLink1->PoseRelativeTo().empty());
  EXPECT_EQ(name + "::__model__", baseLink2->PoseRelativeTo());
  EXPECT_EQ(name + "_14::__model__", baseLink3->PoseRelativeTo());
  EXPECT_TRUE(lowerLink1->PoseRelativeTo().empty());
  EXPECT_EQ(name + "::__model__", lowerLink2->PoseRelativeTo());
  EXPECT_EQ(name + "_14::__model__", lowerLink3->PoseRelativeTo());
  EXPECT_TRUE(upperLink1->PoseRelativeTo().empty());
  EXPECT_EQ(name + "::__model__", upperLink2->PoseRelativeTo());
  EXPECT_EQ(name + "_14::__model__", upperLink3->PoseRelativeTo());

  // each model has 2 joints, and the joint names of the nested models have
  // been transformed
  EXPECT_EQ(2u, model1->JointCount());
  EXPECT_EQ(2u, model2->JointCount());
  EXPECT_EQ(2u, model3->JointCount());
  auto *lowerJoint1 = model1->JointByName("lower_joint");
  auto *lowerJoint2 = model2->JointByName(name + "::lower_joint");
  auto *lowerJoint3 = model3->JointByName(name + "_14::lower_joint");
  ASSERT_NE(nullptr, lowerJoint1);
  ASSERT_NE(nullptr, lowerJoint2);
  ASSERT_NE(nullptr, lowerJoint3);
  auto *upperJoint1 = model1->JointByName("upper_joint");
  auto *upperJoint2 = model2->JointByName(name + "::upper_joint");
  auto *upperJoint3 = model3->JointByName(name + "_14::upper_joint");
  ASSERT_NE(nullptr, upperJoint1);
  ASSERT_NE(nullptr, upperJoint2);
  ASSERT_NE(nullptr, upperJoint3);

  const sdf::JointAxis *lowerAxis1 = lowerJoint1->Axis(0);
  const sdf::JointAxis *lowerAxis2 = lowerJoint2->Axis(0);
  const sdf::JointAxis *lowerAxis3 = lowerJoint3->Axis(0);
  ASSERT_NE(nullptr, lowerAxis1);
  ASSERT_NE(nullptr, lowerAxis2);
  ASSERT_NE(nullptr, lowerAxis3);
  const sdf::JointAxis *upperAxis1 = upperJoint1->Axis(0);
  const sdf::JointAxis *upperAxis2 = upperJoint2->Axis(0);
  const sdf::JointAxis *upperAxis3 = upperJoint3->Axis(0);
  ASSERT_NE(nullptr, upperAxis1);
  ASSERT_NE(nullptr, upperAxis2);
  ASSERT_NE(nullptr, upperAxis3);

  // expect //axis/xyz to be unchanged for model1 and model2
  EXPECT_EQ(gz::math::Vector3d::UnitX, lowerAxis1->Xyz());
  EXPECT_EQ(gz::math::Vector3d::UnitX, upperAxis1->Xyz());
  EXPECT_EQ(gz::math::Vector3d::UnitX, lowerAxis2->Xyz());
  EXPECT_EQ(gz::math::Vector3d::UnitX, upperAxis2->Xyz());
  // For model3, expect //axis/xyz to use the nested model frame in
  // //axis/xyz/@expressed_in since it came from SDFormat 1.4, which implies
  // //axis/xyz/@expressed_in == "__model__" inside the nested model
  EXPECT_EQ(gz::math::Vector3d::UnitX, lowerAxis3->Xyz());
  EXPECT_EQ(gz::math::Vector3d::UnitX, upperAxis3->Xyz());
  EXPECT_EQ(name + "_14::__model__", lowerAxis3->XyzExpressedIn());
  EXPECT_EQ(name + "_14::__model__", upperAxis3->XyzExpressedIn());
}

//////////////////////////////////////////////////
// Test parsing models with child models containg frames nested via <include>
TEST(NestedModel, NestedModelWithFrames)
{
  const std::string name = "test_model_with_frames";
  const std::string modelPath = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  const gz::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.7";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <include>"
    << "      <uri>" + modelPath + "</uri>"
    << "      <name>M1</name>"
    << "      <pose>" << model1Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfParsed);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *parentModel = world->ModelByIndex(0);
  ASSERT_NE(nullptr, parentModel);

  using gz::math::Pose3d;
  using gz::math::Vector3d;
  // Expected poses for frames, links and joints after nesting.
  Pose3d frame1ExpPose = model1Pose * Pose3d(0, 0, 0, IGN_PI/2, 0, 0);
  Pose3d frame2ExpPose = frame1ExpPose * Pose3d(0, 0, 0, 0, IGN_PI/4, 0);
  Pose3d link1ExpPose = frame1ExpPose;
  Pose3d link2ExpPose = frame1ExpPose * Pose3d(1, 0, 0, 0, 0, 0);
  Pose3d joint1ExpPose = link1ExpPose;
  Vector3d joint1AxisExpVector = frame2ExpPose.Rot() * Vector3d::UnitZ;
  Vector3d joint1Axis2ExpVector = frame2ExpPose.Rot() * Vector3d::UnitX;

  const auto *frame1 = parentModel->FrameByName("M1::F1");
  ASSERT_NE(nullptr, frame1);
  Pose3d frame1Pose;
  EXPECT_TRUE(frame1->SemanticPose().Resolve(frame1Pose).empty());
  EXPECT_EQ(frame1ExpPose, frame1Pose);

  const auto *frame2 = parentModel->FrameByName("M1::F2");
  ASSERT_NE(nullptr, frame2);
  Pose3d frame2Pose;
  EXPECT_TRUE(frame2->SemanticPose().Resolve(frame2Pose).empty());
  EXPECT_EQ(frame2ExpPose, frame2Pose);

  const auto *link1 = parentModel->LinkByName("M1::L1");
  ASSERT_NE(nullptr, link1);
  Pose3d link1Pose;
  EXPECT_TRUE(link1->SemanticPose().Resolve(link1Pose).empty());
  EXPECT_EQ(link1ExpPose, link1Pose);

  const auto *visual1 = link1->VisualByName("V1");
  ASSERT_NE(nullptr, visual1);
  EXPECT_EQ("M1::F2", visual1->PoseRelativeTo());

  const auto *collision1 = link1->CollisionByName("C1");
  ASSERT_NE(nullptr, collision1);
  EXPECT_EQ("M1::__model__", collision1->PoseRelativeTo());

  const auto *link2 = parentModel->LinkByName("M1::L2");
  ASSERT_NE(nullptr, link2);
  Pose3d link2Pose;
  EXPECT_TRUE(link2->SemanticPose().Resolve(link2Pose).empty());
  EXPECT_EQ(link2ExpPose, link2Pose);

  const auto *joint1 = parentModel->JointByName("M1::J1");
  ASSERT_NE(nullptr, joint1);
  Pose3d joint1Pose;
  EXPECT_TRUE(joint1->SemanticPose().Resolve(joint1Pose, "__model__").empty());
  EXPECT_EQ(joint1ExpPose, joint1Pose);

  const auto joint1Axis = joint1->Axis(0);
  ASSERT_NE(nullptr, joint1Axis);
  Vector3d joint1AxisVector;
  EXPECT_TRUE(joint1Axis->ResolveXyz(joint1AxisVector, "__model__").empty());
  EXPECT_EQ(joint1AxisExpVector, joint1AxisVector);

  const auto joint1Axis2 = joint1->Axis(1);
  ASSERT_NE(nullptr, joint1Axis2);
  Vector3d joint1Axis2Vector;
  EXPECT_TRUE(joint1Axis2->ResolveXyz(joint1Axis2Vector, "__model__").empty());
  EXPECT_EQ(joint1Axis2ExpVector, joint1Axis2Vector);
}

// Function to remove any element in //joint/axis that is not <xyz>
static void removeNoneXyz(const sdf::ElementPtr &_elem)
{
  std::vector<sdf::ElementPtr> toRemove;
  for (auto el = _elem->GetFirstElement(); el; el = el->GetNextElement())
  {
    if (el->GetName() != "xyz")
    {
      toRemove.push_back(el);
    }
  }

  for (auto el : toRemove)
  {
    _elem->RemoveChild(el);
  }
}

// Remove elements in world that are not model for comparison with
// expectation. Also remove any element in //joint/axis that is not <xyz>
void prepareForDirectComparison(sdf::ElementPtr _worldElem)
{
  std::vector<sdf::ElementPtr> toRemove;
  for (auto elem = _worldElem->GetFirstElement(); elem;
       elem = elem->GetNextElement())
  {
    if (elem->GetName() != "model")
    {
      toRemove.push_back(elem);
    }
    else
    {
      if (elem->HasElement("joint"))
      {
        for (auto joint = elem->GetElement("joint"); joint;
             joint = joint->GetNextElement("joint"))
        {
          if (joint->HasElement("axis"))
          {
            removeNoneXyz(joint->GetElement("axis"));
          }
          if (joint->HasElement("axis2"))
          {
            removeNoneXyz(joint->GetElement("axis2"));
          }
        }
      }
    }
  }

  for (auto elem : toRemove)
  {
    _worldElem->RemoveChild(elem);
  }
}

//////////////////////////////////////////////////
// Test parsing models with child models containg frames nested via <include>
// Compare parsed SDF with expected string
TEST(NestedModel, NestedModelWithFramesDirectComparison)
{
  const std::string name = "test_model_with_frames";
  const std::string modelPath = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  const gz::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.7";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <include>"
    << "      <uri>" + modelPath + "</uri>"
    << "      <name>M1</name>"
    << "      <pose>" << model1Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  auto worldElem = sdfParsed->Root()->GetElement("world");
  prepareForDirectComparison(worldElem);

  // Compare with expected output
  const std::string expectedSdfPath =
      std::string(PROJECT_SOURCE_PATH) +
      "/test/integration/nested_model_with_frames_expected.sdf";
  std::fstream fs;
  fs.open(expectedSdfPath);
  EXPECT_TRUE(fs.is_open());
  std::stringstream expected;
  fs >> expected.rdbuf();
  EXPECT_EQ(expected.str(), sdfParsed->ToString());
}

//////////////////////////////////////////////////
// Test parsing models that have two levels of nesting with child models
// containg frames nested via <include>.
// Compare parsed SDF with expected string
TEST(NestedModel, TwoLevelNestedModelWithFramesDirectComparison)
{
  const std::string modelRootPath = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/";
  const std::string name = "test_nested_model_with_frames";
  const std::string modelPath = modelRootPath + name;

  const gz::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.7";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <include>"
    << "      <uri>" + modelPath + "</uri>"
    << "      <name>M1</name>"
    << "      <pose>" << model1Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  sdf::setFindCallback(
      [&](const std::string &_file)
      {
        return modelRootPath + _file;
      });

  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  auto worldElem = sdfParsed->Root()->GetElement("world");
  prepareForDirectComparison(worldElem);

  // Compare with expected output
  const std::string expectedSdfPath =
      std::string(PROJECT_SOURCE_PATH) +
      "/test/integration/two_level_nested_model_with_frames_expected.sdf";
  std::fstream fs;
  fs.open(expectedSdfPath);
  EXPECT_TRUE(fs.is_open());
  std::stringstream expected;
  fs >> expected.rdbuf();
  EXPECT_EQ(expected.str(), sdfParsed->ToString());
}

//////////////////////////////////////////////////
// Test parsing models nested with <include> and that reference sibling frames
// in their //include/pose/@relative_to attribute
TEST(NestedModel, NestedModelWithSiblingFrames)
{
  const std::string name = "test_model_with_frames";
  const std::string MODEL_PATH = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  const gz::math::Pose3d testFramePose(0, 5, 0, 0, 0, -IGN_PI/2);
  const gz::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.7";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <frame name='testFrame'>"
    << "     <pose>" << testFramePose << "</pose>"
    << "    </frame>"
    << "    <include>"
    << "      <uri>" + MODEL_PATH + "</uri>"
    << "      <name>M1</name>"
    << "      <pose relative_to='testFrame'>" << model1Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfParsed);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *parentModel = world->ModelByIndex(0);
  ASSERT_NE(nullptr, parentModel);

  const sdf::Frame *nestedModel1Frame =
      parentModel->FrameByName("M1::__model__");
  ASSERT_NE(nullptr, nestedModel1Frame);

  EXPECT_EQ("testFrame", nestedModel1Frame->PoseRelativeTo());

  using gz::math::Pose3d;
  using gz::math::Vector3d;
  // Expected poses
  Pose3d nestedModel1FrameExpPose = testFramePose * model1Pose;
  Pose3d frame1ExpPose =
      nestedModel1FrameExpPose * Pose3d(0, 0, 0, IGN_PI / 2, 0, 0);
  Pose3d frame2ExpPose = frame1ExpPose * Pose3d(0, 0, 0, 0, IGN_PI / 4, 0);
  Pose3d link1ExpPose = frame1ExpPose;
  Pose3d link2ExpPose = frame1ExpPose * Pose3d(1, 0, 0, 0, 0, 0);
  Pose3d joint1ExpPose = link1ExpPose;

  Pose3d nestedModel1FramePose;
  EXPECT_TRUE(
      nestedModel1Frame->SemanticPose().Resolve(nestedModel1FramePose).empty());
  EXPECT_EQ(nestedModel1FrameExpPose, nestedModel1FramePose);

  const auto *frame1 = parentModel->FrameByName("M1::F1");
  ASSERT_NE(nullptr, frame1);
  Pose3d frame1Pose;
  EXPECT_TRUE(frame1->SemanticPose().Resolve(frame1Pose).empty());
  EXPECT_EQ(frame1ExpPose, frame1Pose);

  const auto *frame2 = parentModel->FrameByName("M1::F2");
  ASSERT_NE(nullptr, frame2);
  Pose3d frame2Pose;
  EXPECT_TRUE(frame2->SemanticPose().Resolve(frame2Pose).empty());
  EXPECT_EQ(frame2ExpPose, frame2Pose);

  const auto *link1 = parentModel->LinkByName("M1::L1");
  ASSERT_NE(nullptr, link1);
  Pose3d link1Pose;
  EXPECT_TRUE(link1->SemanticPose().Resolve(link1Pose).empty());
  EXPECT_EQ(link1ExpPose, link1Pose);

  const auto *link2 = parentModel->LinkByName("M1::L2");
  ASSERT_NE(nullptr, link2);
  Pose3d link2Pose;
  EXPECT_TRUE(link2->SemanticPose().Resolve(link2Pose).empty());
  EXPECT_EQ(link2ExpPose, link2Pose);

  const auto *joint1 = parentModel->JointByName("M1::J1");
  ASSERT_NE(nullptr, joint1);
  Pose3d joint1Pose;
  EXPECT_TRUE(joint1->SemanticPose().Resolve(joint1Pose, "__model__").empty());
  EXPECT_EQ(joint1ExpPose, joint1Pose);
}

//////////////////////////////////////////////////
// Test parsing models with child models that contain only frames
// and are nested via <include>
TEST(NestedModel, NestedFrameOnlyModel)
{
  const std::string name = "frame_only_model";
  const std::string MODEL_PATH = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  const gz::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.7";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <include>"
    << "      <uri>" + MODEL_PATH + "</uri>"
    << "      <name>M1</name>"
    << "      <pose>" << model1Pose << "</pose>"
    << "    </include>"
    << "  </model>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfParsed);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *parentModel = world->ModelByIndex(0);
  ASSERT_NE(nullptr, parentModel);

  using gz::math::Pose3d;
  using gz::math::Vector3d;
  // Expected poses for frames after nesting.
  Pose3d frame1ExpPose = model1Pose * Pose3d(0, 0, 0, IGN_PI, 0, 0);
  Pose3d frame2ExpPose = frame1ExpPose * Pose3d(0, 0, 0, 0, IGN_PI, 0);

  const auto *frame1 = parentModel->FrameByName("M1::F1");
  ASSERT_NE(nullptr, frame1);
  Pose3d frame1Pose;
  EXPECT_TRUE(frame1->SemanticPose().Resolve(frame1Pose).empty());
  EXPECT_EQ(frame1ExpPose, frame1Pose);

  const auto *frame2 = parentModel->FrameByName("M1::F2");
  ASSERT_NE(nullptr, frame2);
  Pose3d frame2Pose;
  EXPECT_TRUE(frame2->SemanticPose().Resolve(frame2Pose).empty());
  EXPECT_EQ(frame2ExpPose, frame2Pose);
}
