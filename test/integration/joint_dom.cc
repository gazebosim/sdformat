/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMJoint, NotAJoint)
{
  // Create an Element that is not a joint
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Joint joint;
  sdf::Errors errors = joint.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Joint") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMJoint, NoName)
{
  // Create a "joint" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("joint");

  sdf::Joint joint;
  sdf::Errors errors = joint.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_TRUE(errors[0].Message().find("joint name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMJoint, DoublePendulum)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  // The double pendulum should have two joints.
  EXPECT_EQ(2u, model->JointCount());

  // Try to get an invalid joint by name
  EXPECT_TRUE(model->JointByName("invalid_joint") == nullptr);

  // Get the two joints
  const sdf::Joint *upperJoint = model->JointByName("upper_joint");
  ASSERT_NE(nullptr, upperJoint);
  const sdf::Joint *lowerJoint = model->JointByName("lower_joint");
  ASSERT_NE(nullptr, lowerJoint);

  // Check the parent and child link values
  EXPECT_EQ("base", upperJoint->ParentLinkName());
  EXPECT_EQ("upper_link", upperJoint->ChildLinkName());
  EXPECT_EQ("upper_link", lowerJoint->ParentLinkName());
  EXPECT_EQ("lower_link", lowerJoint->ChildLinkName());

  // Check that the pose relative_to values are empty
  EXPECT_TRUE(upperJoint->PoseRelativeTo().empty());
  EXPECT_TRUE(lowerJoint->PoseRelativeTo().empty());

  // The two joinst should not have a second axis.
  EXPECT_TRUE(upperJoint->Axis(1) == nullptr);
  EXPECT_TRUE(upperJoint->Axis(2) == nullptr);
  EXPECT_TRUE(lowerJoint->Axis(1) == nullptr);
  EXPECT_TRUE(lowerJoint->Axis(2) == nullptr);

  // Get the first axis for each joint
  const sdf::JointAxis *upperAxis = upperJoint->Axis(0);
  ASSERT_NE(nullptr, upperAxis);
  const sdf::JointAxis *lowerAxis = upperJoint->Axis(0);
  ASSERT_NE(nullptr, lowerAxis);

  // Check the xyz values for both axis.
  EXPECT_EQ(ignition::math::Vector3d::UnitX, upperAxis->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, lowerAxis->Xyz());
}

//////////////////////////////////////////////////
TEST(DOMJoint, Complete)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "joint_complete.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  std::vector<ignition::math::Pose3d> jointPoses =
  {
    {1, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1},
    {2, 0, 0, 0, 0, 0},
    {0, 2, 0, 0, 0, 0},
    {0, 0, 2, 0, 0, 0},
  };

  for (size_t i = 0; i < jointPoses.size(); ++i)
  {
    EXPECT_EQ(jointPoses[i], model->JointByIndex(i)->RawPose()) << i;
  }

  // Check thread_pitch for screw joint
  {
    const sdf::Joint *joint = model->JointByName("screw_joint");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, joint->Element());
    EXPECT_DOUBLE_EQ(20, joint->ThreadPitch());
  }
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointPoseRelativeTo)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_joint_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = ignition::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_joint_relative_to", model->Name());
  EXPECT_EQ(4u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_NE(nullptr, model->LinkByIndex(2));
  EXPECT_NE(nullptr, model->LinkByIndex(3));
  EXPECT_EQ(nullptr, model->LinkByIndex(4));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("P1"));
  ASSERT_TRUE(model->LinkNameExists("P2"));
  ASSERT_TRUE(model->LinkNameExists("C1"));
  ASSERT_TRUE(model->LinkNameExists("C2"));
  EXPECT_TRUE(model->LinkByName("P1")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("P2")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("C1")->PoseRelativeTo().empty());
  EXPECT_EQ("J2", model->LinkByName("C2")->PoseRelativeTo());

  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI/2, 0), model->LinkByName("P1")->RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, -IGN_PI/2, 0), model->LinkByName("C1")->RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, IGN_PI/2, 0), model->LinkByName("P2")->RawPose());
  EXPECT_EQ(Pose(4, 0, 0, 0, 0, 0), model->LinkByName("C2")->RawPose());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(2u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_NE(nullptr, model->JointByIndex(1));
  EXPECT_EQ(nullptr, model->JointByIndex(2));
  ASSERT_TRUE(model->JointNameExists("J1"));
  ASSERT_TRUE(model->JointNameExists("J2"));
  EXPECT_TRUE(model->JointByName("J1")->PoseRelativeTo().empty());
  EXPECT_EQ("P2", model->JointByName("J2")->PoseRelativeTo());

  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), model->JointByName("J1")->RawPose());
  EXPECT_EQ(Pose(0, 0, 2, 0, 0, 0), model->JointByName("J2")->RawPose());

  // Test ResolveFrame to get each link and joint pose in the model frame.
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("P1")->
      SemanticPose().Resolve("__model__", pose).empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("C1")->
      SemanticPose().Resolve("__model__", pose).empty());
  EXPECT_EQ(Pose(2, 0, 0, 0, -IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    model->JointByName("J1")->
      SemanticPose().Resolve("__model__", pose).empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, -IGN_PI/2, 0), pose);

  EXPECT_TRUE(
    model->LinkByName("P2")->
      SemanticPose().Resolve("__model__", pose).empty());
  EXPECT_EQ(Pose(3, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    model->JointByName("J2")->
      SemanticPose().Resolve("__model__", pose).empty());
  EXPECT_EQ(Pose(5, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("C2")->
      SemanticPose().Resolve("__model__", pose).empty());
  EXPECT_EQ(Pose(5, 0, -4, 0, IGN_PI/2, 0), pose);

  // resolve pose of J1 relative to C1, J2 relative to P2
  // these should match the numbers in the model file
  EXPECT_TRUE(
    model->JointByName("J1")->SemanticPose().Resolve("C1", pose).empty());
  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("J2")->SemanticPose().Resolve("P2", pose).empty());
  EXPECT_EQ(Pose(0, 0, 2, 0, 0, 0), pose);

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadInvalidJointPoseRelativeTo)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_invalid_joint_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_invalid_joint_relative_to", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("P"));
  ASSERT_TRUE(model->LinkNameExists("C"));
  EXPECT_TRUE(model->LinkByName("P")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("C")->PoseRelativeTo().empty());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(2u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_NE(nullptr, model->JointByIndex(1));
  EXPECT_EQ(nullptr, model->JointByIndex(2));
  ASSERT_TRUE(model->JointNameExists("J"));
  ASSERT_TRUE(model->JointNameExists("Jcycle"));
  EXPECT_EQ("A", model->JointByName("J")->PoseRelativeTo());
  EXPECT_EQ("Jcycle", model->JointByName("Jcycle")->PoseRelativeTo());

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}
