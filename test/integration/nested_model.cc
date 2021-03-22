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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

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
  const ignition::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);
  // 2. it's also included into the model named "include_with_rotation"
  //    with the following pose
  const ignition::math::Pose3d model2Pose(-10, 0, 0, 0, 0, IGN_PI/2);
  // 3. double_pendulum_with_base_14 is included into
  //    the model named "include_with_rotation_1.4" with the following pose
  const ignition::math::Pose3d model3Pose(0, 10, 0, 0, 0, IGN_PI/2);

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
  EXPECT_EQ(ignition::math::Pose3d::Zero, model2->RawPose());
  EXPECT_EQ(ignition::math::Pose3d::Zero, model3->RawPose());
  // expect empty //pose/@relative_to
  EXPECT_TRUE(model1->PoseRelativeTo().empty());
  EXPECT_TRUE(model2->PoseRelativeTo().empty());
  EXPECT_TRUE(model3->PoseRelativeTo().empty());

  // each model has 3 links, and the link names of the nested models have
  // been transformed
  EXPECT_EQ(3u, model1->LinkCount());
  // TODO (addisu): Update the following two expectations to account for the
  // fact that included models are no longer flattened.
  // EXPECT_EQ(3u, model2->LinkCount());
  // EXPECT_EQ(3u, model3->LinkCount());
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

  // each model has 2 joints, and the joint names of the nested models have
  // been transformed
  EXPECT_EQ(2u, model1->JointCount());
  // TODO (addisu): Update the following two expectations to account for the
  // fact that included models are no longer flattened.
  // EXPECT_EQ(2u, model2->JointCount());
  // EXPECT_EQ(2u, model3->JointCount());
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
  EXPECT_EQ(ignition::math::Vector3d::UnitX, lowerAxis1->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, upperAxis1->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, lowerAxis2->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, upperAxis2->Xyz());
  // For model3, expect //axis/xyz to use the nested model frame in
  // //axis/xyz/@expressed_in since it came from SDFormat 1.4, which implies
  // //axis/xyz/@expressed_in == "__model__" inside the nested model
  EXPECT_EQ(ignition::math::Vector3d::UnitX, lowerAxis3->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, upperAxis3->Xyz());
}

//////////////////////////////////////////////////
// Test parsing models with child models containg frames nested via <include>
TEST(NestedModel, NestedModelWithFrames)
{
  const std::string name = "test_model_with_frames";
  const std::string modelPath = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  const ignition::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

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

  const sdf::Model *childModel = parentModel->ModelByIndex(0);
  ASSERT_NE(nullptr, childModel);

  using ignition::math::Pose3d;
  using ignition::math::Vector3d;
  // Expected poses for frames, links and joints after nesting.
  Pose3d frame1ExpPose = model1Pose * Pose3d(0, 0, 0, IGN_PI/2, 0, 0);
  Pose3d frame2ExpPose = frame1ExpPose * Pose3d(0, 0, 0, 0, IGN_PI/4, 0);
  Pose3d link1ExpPose = frame1ExpPose;
  Pose3d link2ExpPose = frame1ExpPose * Pose3d(1, 0, 0, 0, 0, 0);
  Pose3d joint1ExpPose = link1ExpPose;
  Vector3d joint1AxisExpVector = frame2ExpPose.Rot() * Vector3d::UnitZ;
  Vector3d joint1Axis2ExpVector = frame2ExpPose.Rot() * Vector3d::UnitX;

  Pose3d frame1Pose;
  auto parentSemPose = parentModel->SemanticPose();
  EXPECT_TRUE(parentSemPose.Resolve(frame1Pose, "ParentModel::M1::F1").empty());
  EXPECT_EQ(frame1ExpPose, frame1Pose.Inverse());

  Pose3d frame2Pose;
  EXPECT_TRUE(parentSemPose.Resolve(frame2Pose, "ParentModel::M1::F2").empty());
  EXPECT_EQ(frame2ExpPose, frame2Pose.Inverse());

  const auto *link1 = parentModel->LinkByName("M1::L1");
  ASSERT_NE(nullptr, link1);
  Pose3d link1Pose;
  EXPECT_TRUE(parentSemPose.Resolve(link1Pose, "ParentModel::M1::L1").empty());
  EXPECT_EQ(link1ExpPose, link1Pose.Inverse());

  const auto *visual1 = link1->VisualByName("V1");
  ASSERT_NE(nullptr, visual1);
  EXPECT_EQ("F2", visual1->PoseRelativeTo());

  const auto *collision1 = link1->CollisionByName("C1");
  ASSERT_NE(nullptr, collision1);
  EXPECT_EQ("__model__", collision1->PoseRelativeTo());

  const auto *link2 = parentModel->LinkByName("M1::L2");
  ASSERT_NE(nullptr, link2);
  Pose3d link2Pose;
  EXPECT_TRUE(parentSemPose.Resolve(link2Pose, "ParentModel::M1::L2").empty());
  EXPECT_EQ(link2ExpPose, link2Pose.Inverse());

  const auto *joint1 = parentModel->JointByName("M1::J1");
  ASSERT_NE(nullptr, joint1);
  Pose3d joint1Pose;
  EXPECT_TRUE(parentSemPose.Resolve(joint1Pose, "ParentModel::M1::J1").empty());
  EXPECT_EQ(joint1ExpPose, joint1Pose.Inverse());

  const auto joint1Axis = joint1->Axis(0);
  ASSERT_NE(nullptr, joint1Axis);
  Vector3d joint1AxisVector;
  EXPECT_TRUE(joint1Axis->ResolveXyz(joint1AxisVector, "__model__").empty());
  EXPECT_EQ(joint1AxisExpVector, model1Pose.Rot() * joint1AxisVector);

  const auto joint1Axis2 = joint1->Axis(1);
  ASSERT_NE(nullptr, joint1Axis2);
  Vector3d joint1Axis2Vector;
  EXPECT_TRUE(joint1Axis2->ResolveXyz(joint1Axis2Vector, "__model__").empty());
  EXPECT_EQ(joint1Axis2ExpVector, model1Pose.Rot() * joint1Axis2Vector);
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

  for (const auto &el : toRemove)
  {
    _elem->RemoveChild(el);
  }
}

void prepareModelForDirectComparison(sdf::ElementPtr _modelElem)
{
  if (_modelElem->HasElement("joint"))
  {
    for (auto joint = _modelElem->GetElement("joint"); joint;
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
  if (_modelElem->HasElement("model"))
  {
    for (auto elem = _modelElem->GetElement("model"); elem;
        elem = elem->GetNextElement("model"))
    {
      prepareModelForDirectComparison(elem);
    }
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
      prepareModelForDirectComparison(elem);
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

  const ignition::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

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
  sdf::Errors errors;
  ASSERT_TRUE(
      sdf::readStringWithoutConversion(stream.str(), sdfParsed, errors));

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
// Test DOM APIs with partially flattened model
TEST(NestedModel, PartiallyFlattened)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
      "partially_flattened.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  // Get the outer model
  const sdf::Model *outerModel = world->ModelByIndex(0);
  ASSERT_NE(nullptr, outerModel);
  EXPECT_EQ("ParentModel", outerModel->Name());

  EXPECT_TRUE(outerModel->LinkNameExists("M1::L1"));
  EXPECT_TRUE(outerModel->LinkNameExists("M1::L2"));
  EXPECT_TRUE(outerModel->LinkNameExists("M1::L3"));
  EXPECT_TRUE(outerModel->LinkNameExists("M1::L4"));

  EXPECT_TRUE(outerModel->JointNameExists("M1::J1"));
  EXPECT_TRUE(outerModel->JointNameExists("M1::J2"));
  EXPECT_TRUE(outerModel->JointNameExists("M1::J3"));

  EXPECT_TRUE(outerModel->FrameNameExists("M1::F1"));
  EXPECT_TRUE(outerModel->FrameNameExists("M1::F2"));

  EXPECT_TRUE(outerModel->ModelNameExists("M1::M2"));

  EXPECT_EQ(1u, outerModel->ModelCount());

  // Get the middle model
  const sdf::Model *midModel = outerModel->ModelByIndex(0);
  ASSERT_NE(nullptr, midModel);
  EXPECT_EQ(1u, midModel->ModelCount());
  EXPECT_EQ("M1", midModel->Name());
  EXPECT_EQ("L1", midModel->CanonicalLinkName());
  EXPECT_EQ(ignition::math::Pose3d(10, 0, 0, 0, -0, 1.5708),
            midModel->SemanticPose().RawPose());
  EXPECT_EQ("__model__", midModel->SemanticPose().RelativeTo());

  EXPECT_EQ(4u, midModel->LinkCount());
  EXPECT_NE(nullptr, midModel->LinkByIndex(0));
  EXPECT_NE(nullptr, midModel->LinkByIndex(1));
  EXPECT_NE(nullptr, midModel->LinkByIndex(2));
  EXPECT_NE(nullptr, midModel->LinkByIndex(3));
  EXPECT_EQ(nullptr, midModel->LinkByIndex(4));

  EXPECT_TRUE(midModel->LinkNameExists("L1"));
  EXPECT_TRUE(midModel->LinkNameExists("L2"));
  EXPECT_TRUE(midModel->LinkNameExists("L3"));
  EXPECT_TRUE(midModel->LinkNameExists("L4"));

  EXPECT_EQ(3u, midModel->JointCount());
  EXPECT_TRUE(midModel->JointNameExists("J1"));
  EXPECT_TRUE(midModel->JointNameExists("J2"));
  EXPECT_TRUE(midModel->JointNameExists("J3"));

  EXPECT_EQ(2u, midModel->FrameCount());
  EXPECT_TRUE(midModel->FrameNameExists("F1"));
  EXPECT_TRUE(midModel->FrameNameExists("F2"));

  EXPECT_EQ(1u, midModel->ModelCount());
  EXPECT_NE(nullptr, midModel->ModelByIndex(0));
  EXPECT_EQ(nullptr, midModel->ModelByIndex(1));

  EXPECT_TRUE(midModel->ModelNameExists("M2"));

  // Get the inner model of midModel
  const sdf::Model *innerModel = midModel->ModelByIndex(0);
  ASSERT_NE(nullptr, innerModel);
  EXPECT_EQ("M2", innerModel->Name());
  EXPECT_EQ(innerModel, midModel->ModelByName("M2"));
  EXPECT_EQ(innerModel, outerModel->ModelByName("M1::M2"));

  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, -0, 0),
            innerModel->SemanticPose().RawPose());
  EXPECT_EQ("F1", innerModel->SemanticPose().RelativeTo());

  EXPECT_EQ(1u, innerModel->LinkCount());
  EXPECT_NE(nullptr, innerModel->LinkByIndex(0));
  EXPECT_EQ(nullptr, innerModel->LinkByIndex(1));

  EXPECT_TRUE(innerModel->LinkNameExists("L5"));
  EXPECT_NE(nullptr, innerModel->LinkByName("L5"));
  EXPECT_TRUE(outerModel->LinkNameExists("M1::M2::L5"));

  EXPECT_EQ(0u, innerModel->JointCount());

  EXPECT_EQ(0u, innerModel->FrameCount());

  EXPECT_EQ(0u, innerModel->ModelCount());
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

  const ignition::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

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

  sdf::Errors errors;
  ASSERT_TRUE(
      sdf::readStringWithoutConversion(stream.str(), sdfParsed, errors));

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

  const ignition::math::Pose3d testFramePose(0, 5, 0, 0, 0, -IGN_PI/2);
  const ignition::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.7";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <frame name='parentModelFrame'/>"
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
  const sdf::Frame *parentModelFrame = parentModel->FrameByIndex(0);
  ASSERT_NE(nullptr, parentModelFrame);

  const sdf::Model *nestedModel1 = parentModel->ModelByName("M1");
  ASSERT_NE(nullptr, nestedModel1);

  EXPECT_EQ("testFrame", nestedModel1->PoseRelativeTo());

  using ignition::math::Pose3d;
  using ignition::math::Vector3d;
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
      nestedModel1->SemanticPose().Resolve(nestedModel1FramePose).empty());
  EXPECT_EQ(nestedModel1FrameExpPose, nestedModel1FramePose);

  auto parentSemPose = parentModel->SemanticPose();

  const auto *frame1 = parentModel->FrameByName("M1::F1");
  ASSERT_NE(nullptr, frame1);
  Pose3d frame1Pose;
  EXPECT_TRUE(parentSemPose.Resolve(frame1Pose, "ParentModel::M1::F1").empty());
  EXPECT_EQ(frame1ExpPose, frame1Pose.Inverse());

  const auto *frame2 = parentModel->FrameByName("M1::F2");
  ASSERT_NE(nullptr, frame2);
  Pose3d frame2Pose;
  EXPECT_TRUE(parentSemPose.Resolve(frame2Pose, "ParentModel::M1::F2").empty());
  EXPECT_EQ(frame2ExpPose, frame2Pose.Inverse());

  const auto *link1 = parentModel->LinkByName("M1::L1");
  ASSERT_NE(nullptr, link1);
  Pose3d link1Pose;
  EXPECT_TRUE(parentSemPose.Resolve(link1Pose, "ParentModel::M1::L1").empty());
  EXPECT_EQ(link1ExpPose, link1Pose.Inverse());

  const auto *link2 = parentModel->LinkByName("M1::L2");
  ASSERT_NE(nullptr, link2);
  Pose3d link2Pose;
  EXPECT_TRUE(parentSemPose.Resolve(link2Pose, "ParentModel::M1::L2").empty());
  EXPECT_EQ(link2ExpPose, link2Pose.Inverse());

  const auto *joint1 = parentModel->JointByName("M1::J1");
  ASSERT_NE(nullptr, joint1);
  Pose3d joint1Pose;
  EXPECT_TRUE(parentSemPose.Resolve(joint1Pose, "ParentModel::M1::J1").empty());
  EXPECT_EQ(joint1ExpPose, joint1Pose.Inverse());
}

//////////////////////////////////////////////////
// Test parsing models with child models that contain only frames
// and are nested via <include>
TEST(NestedModel, NestedFrameOnlyModel)
{
  const std::string name = "frame_only_model";
  const std::string MODEL_PATH = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/" + name;

  const ignition::math::Pose3d model1Pose(10, 0, 0, 0, 0, IGN_PI/2);

  std::ostringstream stream;
  std::string version = "1.8";
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "  <model name='ParentModel'>"
    << "    <static>true</static>"
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
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *parentModel = world->ModelByIndex(0);
  ASSERT_NE(nullptr, parentModel);

  auto parentSemPose = parentModel->SemanticPose();

  using ignition::math::Pose3d;
  using ignition::math::Vector3d;
  // Expected poses for frames after nesting.
  Pose3d frame1ExpPose = model1Pose * Pose3d(0, 0, 0, IGN_PI, 0, 0);
  Pose3d frame2ExpPose = frame1ExpPose * Pose3d(0, 0, 0, 0, IGN_PI, 0);

  Pose3d frame1Pose;
  EXPECT_TRUE(parentSemPose.Resolve(frame1Pose, "ParentModel::M1::F1").empty());
  EXPECT_EQ(frame1ExpPose, frame1Pose.Inverse());

  Pose3d frame2Pose;
  EXPECT_TRUE(parentSemPose.Resolve(frame2Pose, "ParentModel::M1::F2").empty());
  EXPECT_EQ(frame2ExpPose, frame2Pose.Inverse());
}

class PlacementFrame: public ::testing::Test
{
  protected: using Pose3d = ignition::math::Pose3d;

  protected: void SetUp() override
  {
    const std::string modelRootPath = sdf::filesystem::append(
        PROJECT_SOURCE_PATH, "test", "integration", "model");

    const std::string testModelPath = sdf::filesystem::append(
        PROJECT_SOURCE_PATH, "test", "sdf", "placement_frame.sdf");

    sdf::setFindCallback(
        [&](const std::string &_file)
        {
          return sdf::filesystem::append(modelRootPath, _file);
        });
    sdf::Errors errors = this->root.Load(testModelPath);
    for (const auto &e : errors)
    {
      std::cout << e.Message() << std::endl;
    }
    if (!errors.empty())
    {
      std::cout << this->root.Element()->ToString("") << std::endl;
    }
    EXPECT_TRUE(errors.empty());

    this->world = this->root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
  }

  public: template <typename FrameType>
  const FrameType *GetFrameByName(
      const sdf::Model *_model, const std::string &_testFrameName)
  {
    // cppcheck-suppress syntaxError
    if constexpr (std::is_same_v<FrameType, sdf::Frame>)
    {
      return _model->FrameByName(_testFrameName);
    }
    else if constexpr (std::is_same_v<FrameType, sdf::Link>)
    {
      return _model->LinkByName(_testFrameName);
    }
    else if constexpr (std::is_same_v<FrameType, sdf::Joint>)
    {
      return _model->JointByName(_testFrameName);
    }
    else
    {
      return nullptr;
    }
  }

  public: sdf::SemanticPose GetChildEntitySemanticPose(
      const sdf::World *_world, const std::string _entityName)
  {
    return _world->ModelByName(_entityName)->SemanticPose();
  }

  public: template <typename FrameType>
  void TestExpectedWorldPose(const std::string &_testModelName,
                             const std::string &_testFrameName)
  {
    const Pose3d placementPose(0, 10, 0, IGN_PI_2, 0, 0);
    const sdf::Model *testModel = this->world->ModelByName(_testModelName);
    ASSERT_NE(nullptr, testModel);

    // Pose of model in world frame
    Pose3d modelPoseWorld;
    {
      sdf::Errors errors =
          testModel->SemanticPose().Resolve(modelPoseWorld, "world");
      EXPECT_TRUE(errors.empty()) << errors[0].Message();
    }

    const auto *testFrame =
        this->GetFrameByName<FrameType>(testModel, _testFrameName);
    ASSERT_NE(nullptr, testFrame);

    // Pose of frame in its parent model frame.
    Pose3d frameRelPose;
    {
      sdf::Errors errors =
          testFrame->SemanticPose().Resolve(frameRelPose, "__model__");
      EXPECT_TRUE(errors.empty()) << errors[0].Message();
    }

    Pose3d framePoseWorld = modelPoseWorld * frameRelPose;
    EXPECT_EQ(placementPose, framePoseWorld);
  }

  public: template <typename FrameType>
  void TestExpectedModelPose(const std::string &_parentModelName,
                             const std::string &_testFrameName)
  {
    const Pose3d placementPose(0, 10, 0, IGN_PI_2, 0, 0);
    const sdf::Model *parentModel = this->world->ModelByName(_parentModelName);
    ASSERT_NE(nullptr, parentModel);

    const auto *testFrame = this->GetFrameByName<FrameType>(
        parentModel, _testFrameName);
    ASSERT_NE(nullptr, testFrame);

    Pose3d testFramePose;
    {
      sdf::Errors errors = parentModel->SemanticPose().Resolve(
          testFramePose, parentModel->Name() + "::" + _testFrameName);
      EXPECT_TRUE(errors.empty()) << errors;
    }

    // The expected pose of the test frame is precisely the placement pose
    EXPECT_EQ(placementPose, testFramePose.Inverse());
  }

  protected: sdf::Root root;
  protected: const sdf::World *world{nullptr};
};

//////////////////////////////////////////////////
TEST_F(PlacementFrame, WorldInclude)
{
  // Test that link names can be used for <placement_frame>
  this->TestExpectedWorldPose<sdf::Link>("placement_frame_using_link", "L4");

  // Test that frame names can be used for <placement_frame>
  this->TestExpectedWorldPose<sdf::Frame>("placement_frame_using_frame", "F2");

  // Test that joint names can be used for <placement_frame>
  this->TestExpectedWorldPose<sdf::Joint>("placement_frame_using_joint", "J2");

  // Test that the pose of an included model with placement_frame can use the
  // relative_to attribute
  this->TestExpectedWorldPose<sdf::Link>(
      "include_with_placement_frame_and_pose_relative_to", "L4");
}

//////////////////////////////////////////////////
TEST_F(PlacementFrame, ModelInclude)
{
  // Test that link names can be used for <placement_frame>
  this->TestExpectedModelPose<sdf::Link>(
      "parent_model_include", "placement_frame_using_link::L4");

  // Test that frame names can be used for <placement_frame>
  this->TestExpectedModelPose<sdf::Frame>(
      "parent_model_include", "placement_frame_using_frame::F2");

  // Test that joint names can be used for <placement_frame>
  this->TestExpectedModelPose<sdf::Joint>(
      "parent_model_include", "placement_frame_using_joint::J2");

  // Test that the pose of an included model with placement_frame can use the
  // relative_to attribute
  this->TestExpectedModelPose<sdf::Link>("parent_model_include",
      "nested_include_with_placement_frame_and_pose_relative_to::L4");
}

//////////////////////////////////////////////////
TEST_F(PlacementFrame, ModelPlacementFrameAttribute)
{
  // Test that link names can be used for <placement_frame>
  this->TestExpectedWorldPose<sdf::Link>(
      "model_with_link_placement_frame", "L4");

  // Test that frame names can be used for <placement_frame>
  this->TestExpectedWorldPose<sdf::Frame>(
      "model_with_frame_placement_frame", "F2");

  // Test that joint names can be used for <placement_frame>
  this->TestExpectedWorldPose<sdf::Joint>(
      "model_with_joint_placement_frame", "J2");

  // Test that the pose of a model with a placement_frame attribute can use the
  // relative_to attribute
  this->TestExpectedWorldPose<sdf::Link>(
      "model_with_placement_frame_and_pose_relative_to", "L4");
}

//////////////////////////////////////////////////
TEST(NestedReference, PoseRelativeTo)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_relative_to_nested_reference.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = ignition::math::Pose3d;
  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  auto parentSemPose = model->SemanticPose();
  // The child models starting from M2 all reference an entity inside M1. Check
  // that resolving poses works when referencing such entities.
  {
    Pose pose;
    const sdf::Model *testModel = model->ModelByName("M2");
    ASSERT_NE(nullptr, testModel);
    EXPECT_TRUE(parentSemPose.Resolve(pose, "parent_model::M2").empty());
    EXPECT_EQ(Pose(1, 0, -2, 0, IGN_PI_2, 0), pose.Inverse());
  }
  {
    Pose pose;
    const sdf::Model *testModel = model->ModelByName("M3");
    ASSERT_NE(nullptr, testModel);
    EXPECT_TRUE(parentSemPose.Resolve(pose, "parent_model::M3").empty());
    EXPECT_EQ(Pose(1, 1, -3, 0, IGN_PI_2, 0), pose.Inverse());
  }
  {
    Pose pose;
    const sdf::Model *testModel = model->ModelByName("M4");
    ASSERT_NE(nullptr, testModel);
    EXPECT_TRUE(parentSemPose.Resolve(pose, "parent_model::M4").empty());
    EXPECT_EQ(Pose(2, 0, -4, 0, IGN_PI_2, 0), pose.Inverse());
  }
  {
    Pose pose;
    const sdf::Model *testModel = model->ModelByName("M5");
    ASSERT_NE(nullptr, testModel);
    EXPECT_TRUE(parentSemPose.Resolve(pose, "parent_model::M5").empty());
    EXPECT_EQ(Pose(2, 0, -6, 0, IGN_PI_2, 0), pose.Inverse());
  }
  {
    Pose pose;
    const sdf::Model *testModel = model->ModelByName("M6");
    ASSERT_NE(nullptr, testModel);
    EXPECT_TRUE(parentSemPose.Resolve(pose, "parent_model::M6").empty());
    EXPECT_EQ(Pose(1, 1, -6, IGN_PI_2, IGN_PI_2, 0), pose.Inverse());
  }
  {
    Pose pose;
    const sdf::Model *testModel = model->ModelByName("M7");
    ASSERT_NE(nullptr, testModel);
    EXPECT_TRUE(parentSemPose.Resolve(pose, "parent_model::M7").empty());
    EXPECT_EQ(Pose(1, -6, -1, 0, 0, -IGN_PI_2), pose.Inverse());
  }
}

//////////////////////////////////////////////////
TEST(NestedReference, PoseRelativeToInWorld)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_relative_to_nested_reference.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  using Pose = ignition::math::Pose3d;

  // The //world/frame elements reference an entity inside M1. Check
  // that resolving poses works when referencing such entities.
  {
    Pose pose;
    const sdf::Frame *testFrame = world->FrameByName("F2");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_TRUE(testFrame->SemanticPose().Resolve(pose).empty());
    EXPECT_EQ(Pose(1, 0, -2, 0, IGN_PI_2, 0), pose);
  }
  {
    Pose pose;
    const sdf::Frame *testFrame = world->FrameByName("F3");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_TRUE(testFrame->SemanticPose().Resolve(pose).empty());
    EXPECT_EQ(Pose(1, 1, -3, 0, IGN_PI_2, 0), pose);
  }
  {
    Pose pose;
    const sdf::Frame *testFrame = world->FrameByName("F4");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_TRUE(testFrame->SemanticPose().Resolve(pose).empty());
    EXPECT_EQ(Pose(2, 0, -4, 0, IGN_PI_2, 0), pose);
  }
  {
    Pose pose;
    const sdf::Frame *testFrame = world->FrameByName("F5");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_TRUE(testFrame->SemanticPose().Resolve(pose).empty());
    EXPECT_EQ(Pose(2, 0, -6, 0, IGN_PI_2, 0), pose);
  }
  {
    Pose pose;
    const sdf::Frame *testFrame = world->FrameByName("F6");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_TRUE(testFrame->SemanticPose().Resolve(pose).empty());
    EXPECT_EQ(Pose(1, 1, -6, IGN_PI_2, IGN_PI_2, 0), pose);
  }
  {
    Pose pose;
    const sdf::Frame *testFrame = world->FrameByName("F7");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_TRUE(testFrame->SemanticPose().Resolve(pose).empty());
    EXPECT_EQ(Pose(1, -6, -1, 0, 0, -IGN_PI_2), pose);
  }
}

/////////////////////////////////////////////////
TEST(NestedReference, PlacementFrameAttribute)
{
  const std::string testFile =
      sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
          "model", "model_with_nested_placement_frame_attribute", "model.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  auto *model = root.Model();
  ASSERT_NE(nullptr, model);

  ignition::math::Pose3d pose;
  errors = model->SemanticPose().Resolve(pose);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_EQ(ignition::math::Pose3d(0, -2, 10, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(NestedReference, PlacementFrameElement)
{
  const std::string modelRootPath = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "integration", "model");

  sdf::setFindCallback([&](const std::string &_file)
  {
    return sdf::filesystem::append(modelRootPath, _file);
  });

  // Test with //world/include without overriding the placement_frame
  {
    std::ostringstream stream;
    stream << R"(
    <sdf version='1.8'>
      <world name='default'>
        <include>
          <uri>model_with_nested_placement_frame_attribute</uri>
        </include>
      </world>
    </sdf>)";
    // Load the SDF file
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(stream.str());
    EXPECT_TRUE(errors.empty()) << errors;

    auto *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    auto *model = world->ModelByIndex(0);
    ASSERT_NE(nullptr, model);

    ignition::math::Pose3d pose;
    errors = model->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(ignition::math::Pose3d(0, -2, 10, 0, 0, 0), pose);
  }

  // Test with //world/include overriding the placement_frame
  {
    const ignition::math::Pose3d placementPose(0, 10, 0, IGN_PI_2, 0, 0);
    std::ostringstream stream;
    stream << R"(
    <sdf version='1.8'>
      <world name='default'>
        <frame name="world_frame"/>
        <include>
          <uri>model_with_nested_placement_frame_attribute</uri>
          <placement_frame>child_model::link3</placement_frame>
          <pose>)" << placementPose << R"(</pose>"
        </include>
      </world>
    </sdf>)";
    // Load the SDF file
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(stream.str());
    EXPECT_TRUE(errors.empty()) << errors;

    auto *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);

    auto *world_frame = world->FrameByName("world_frame");
    ASSERT_NE(nullptr, world_frame);
    ignition::math::Pose3d pose;
    errors = world_frame->SemanticPose().Resolve(
        pose, "model_placement_frame::child_model::link3");
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(placementPose, pose.Inverse());
  }


  // Test with //model/include without overriding the placement_frame
  {
    std::ostringstream stream;
    stream << R"(
    <sdf version='1.8'>
      <model name='parent_model'>
        <include>
          <uri>model_with_nested_placement_frame_attribute</uri>
        </include>
      </model>
    </sdf>)";
    // Load the SDF file
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(stream.str());
    EXPECT_TRUE(errors.empty()) << errors;

    auto *parentModel = root.Model();
    ASSERT_NE(nullptr, parentModel);
    auto *model = parentModel->ModelByIndex(0);
    ASSERT_NE(nullptr, model);

    ignition::math::Pose3d pose;
    errors = model->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(ignition::math::Pose3d(0, -2, 10, 0, 0, 0), pose);
  }

  // Test with //model/include overriding the placement_frame
  {
    const ignition::math::Pose3d placementPose(0, 10, 0, IGN_PI_2, 0, 0);
    std::ostringstream stream;
    stream << R"(
    <sdf version='1.8'>
      <model name='parent_model'>
        <frame name="parent_model_frame"/>
        <include>
          <uri>model_with_nested_placement_frame_attribute</uri>
          <placement_frame>child_model::link3</placement_frame>
          <pose>)" << placementPose << R"(</pose>"
        </include>
      </model>
    </sdf>)";
    // Load the SDF file
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(stream.str());
    EXPECT_TRUE(errors.empty()) << errors;

    auto *parentModel = root.Model();
    ASSERT_NE(nullptr, parentModel);
    auto *model = parentModel ->ModelByIndex(0);
    ASSERT_NE(nullptr, model);

    auto *parentModelFrame = parentModel->FrameByName("parent_model_frame");
    auto *link = model->LinkByName("child_model::link3");
    ASSERT_NE(nullptr, link);
    ignition::math::Pose3d pose;
    errors = parentModelFrame->SemanticPose().Resolve(
        pose, "model_placement_frame::child_model::link3");
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(placementPose, pose.Inverse());
  }
}

/////////////////////////////////////////////////
TEST(NestedModel, IncludeElements)
{
  const std::string modelRootPath = sdf::filesystem::append(
      PROJECT_SOURCE_PATH, "test", "integration", "model");

  sdf::ParserConfig config;
  config.SetFindCallback(
      [&](const std::string &_file)
      {
        return sdf::filesystem::append(modelRootPath, _file);
      });

  auto checkIncludeElem = [](sdf::ElementPtr _includeElem,
                              const std::string &_uri, const std::string &_name)
  {
    ASSERT_NE(nullptr, _includeElem);
    ASSERT_TRUE(_includeElem->HasElement("uri"));
    EXPECT_EQ(_uri, _includeElem->Get<std::string>("uri"));

    ASSERT_TRUE(_includeElem->HasElement("name"));
    EXPECT_EQ(_name, _includeElem->Get<std::string>("name"));

    ASSERT_TRUE(_includeElem->HasElement("pose"));
    EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0),
        _includeElem->Get<ignition::math::Pose3d>("pose"));

    ASSERT_TRUE(_includeElem->HasElement("placement_frame"));
    EXPECT_EQ("link", _includeElem->Get<std::string>("placement_frame"));

    ASSERT_TRUE(_includeElem->HasElement("plugin"));
    auto pluginElem = _includeElem->GetElement("plugin");
    ASSERT_TRUE(pluginElem->HasAttribute("name"));
    EXPECT_EQ("test_plugin", pluginElem->Get<std::string>("name"));
    ASSERT_TRUE(pluginElem->HasAttribute("filename"));
    EXPECT_EQ("test_plugin_file", pluginElem->Get<std::string>("filename"));
  };

  // Test with //world/include
  {
    std::ostringstream stream;
    stream << R"(
    <sdf version='1.8'>
      <world name='default'>
        <include>
          <uri>box</uri>
          <name>test_box</name>
          <pose>1 2 3 0 0 0</pose>
          <placement_frame>link</placement_frame>
          <plugin name='test_plugin' filename='test_plugin_file'/>
        </include>
        <model name='test2'>
          <include>
            <uri>test_model</uri>
            <name>test_model</name>
            <pose>1 2 3 0 0 0</pose>
            <placement_frame>link</placement_frame>
            <plugin name='test_plugin' filename='test_plugin_file'/>
          </include>
        </model>
      </world>
    </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(stream.str(), config);
    EXPECT_TRUE(errors.empty()) << errors;

    auto *world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    auto *box = world->ModelByIndex(0);
    ASSERT_NE(nullptr, box);

    checkIncludeElem(
        box->Element()->GetIncludeElement(), "box", "test_box");

    auto *test2 = world->ModelByIndex(1);
    ASSERT_NE(nullptr, test2);
    EXPECT_EQ(nullptr, test2->Element()->GetIncludeElement());

    auto *testModel2 = test2->ModelByIndex(0);
    ASSERT_NE(nullptr, testModel2);
    checkIncludeElem(
        testModel2->Element()->GetIncludeElement(), "test_model", "test_model");
  }
}
