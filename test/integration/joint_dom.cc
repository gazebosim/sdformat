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
#include "sdf/ForceTorque.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "test_config.hh"

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
    sdf::testing::TestFile("sdf", "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.Model();
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
  EXPECT_EQ("base", upperJoint->ParentName());
  EXPECT_EQ("upper_link", upperJoint->ChildName());
  EXPECT_EQ("upper_link", lowerJoint->ParentName());
  EXPECT_EQ("lower_link", lowerJoint->ChildName());

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
  EXPECT_EQ(gz::math::Vector3d::UnitX, upperAxis->Xyz());
  EXPECT_EQ(gz::math::Vector3d::UnitX, lowerAxis->Xyz());
}

//////////////////////////////////////////////////
TEST(DOMJoint, Complete)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_complete.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  std::vector<gz::math::Pose3d> jointPoses =
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
TEST(DOMJoint, LoadJointParentWorld)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_parent_world.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("joint_parent_world", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_EQ(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("link"));
  EXPECT_TRUE(model->LinkByName("link")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), model->LinkByName("link")->RawPose());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(1u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_EQ(nullptr, model->JointByIndex(1));
  ASSERT_TRUE(model->JointNameExists("joint"));
  EXPECT_EQ("link", model->JointByName("joint")->ChildName());
  EXPECT_EQ("world", model->JointByName("joint")->ParentName());
  std::string resolvedLinkName;
  EXPECT_TRUE(
    model->JointByName("joint")->ResolveChildLink(resolvedLinkName).empty());
  EXPECT_EQ("link", resolvedLinkName);
  EXPECT_TRUE(
    model->JointByName("joint")->ResolveParentLink(resolvedLinkName).empty());
  EXPECT_EQ("world", resolvedLinkName);

  EXPECT_EQ(Pose(0, 0, 3, 0, 0, 0), model->JointByName("joint")->RawPose());
  EXPECT_TRUE(model->JointByName("joint")->PoseRelativeTo().empty());

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadInvalidJointChildWorld)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_child_world.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  ASSERT_EQ(10u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::JOINT_CHILD_LINK_INVALID);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "Joint with name[joint] specified invalid child link [world]"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::JOINT_CHILD_LINK_INVALID);
  EXPECT_NE(std::string::npos,
    errors[1].Message().find(
      "Child frame with name[world] specified by joint with name[joint] "
      "not found in model with name[joint_child_world]"));
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointParentModelFrame)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_parent_model_frame.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("joint_parent_model_frame", model->Name());
  EXPECT_EQ(2u, model->LinkCount());

  EXPECT_EQ(1u, model->JointCount());
  auto *joint = model->JointByName("joint");
  ASSERT_NE(nullptr, joint);
  EXPECT_EQ("child_link", model->JointByName("joint")->ChildName());
  EXPECT_EQ("__model__", model->JointByName("joint")->ParentName());
  std::string resolvedLinkName;
  EXPECT_TRUE(joint->ResolveParentLink(resolvedLinkName).empty());
  EXPECT_EQ("base_link", resolvedLinkName);

  gz::math::Pose3d pose;
  EXPECT_TRUE(
    joint->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(gz::math::Pose3d(1, 0, 0, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointChildModelFrame)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_child_model_frame.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("joint_child_model_frame", model->Name());
  EXPECT_EQ(2u, model->LinkCount());

  EXPECT_EQ(1u, model->JointCount());
  auto *joint = model->JointByName("joint");
  ASSERT_NE(nullptr, joint);
  EXPECT_EQ("__model__", model->JointByName("joint")->ChildName());
  EXPECT_EQ("parent_link", model->JointByName("joint")->ParentName());
  std::string resolvedLinkName;
  EXPECT_TRUE(joint->ResolveChildLink(resolvedLinkName).empty());
  EXPECT_EQ("base_link", resolvedLinkName);

  gz::math::Pose3d pose;
  EXPECT_TRUE(
    joint->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(gz::math::Pose3d(0, 0, 0, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointParentFrame)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_parent_frame.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("joint_parent_frame", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("parent_link"));
  ASSERT_TRUE(model->LinkNameExists("child_link"));
  EXPECT_TRUE(model->LinkByName("parent_link")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("child_link")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0),
            model->LinkByName("parent_link")->RawPose());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0),
            model->LinkByName("child_link")->RawPose());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(1u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_EQ(nullptr, model->JointByIndex(1));
  ASSERT_TRUE(model->JointNameExists("joint"));
  EXPECT_EQ("child_link", model->JointByName("joint")->ChildName());
  EXPECT_EQ("parent_frame", model->JointByName("joint")->ParentName());

  std::string resolvedLinkName;
  EXPECT_TRUE(
    model->JointByName("joint")->ResolveChildLink(resolvedLinkName).empty());
  EXPECT_EQ("child_link", resolvedLinkName);
  EXPECT_TRUE(
    model->JointByName("joint")->ResolveParentLink(resolvedLinkName).empty());
  EXPECT_EQ("parent_link", resolvedLinkName);

  EXPECT_TRUE(model->JointByName("joint")->PoseRelativeTo().empty());
  EXPECT_EQ(Pose(0, 1, 0, 0, 0, 0), model->JointByName("joint")->RawPose());

  EXPECT_EQ(1u, model->FrameCount());
  EXPECT_NE(nullptr, model->FrameByIndex(0));
  EXPECT_EQ(nullptr, model->FrameByIndex(1));

  ASSERT_TRUE(model->FrameNameExists("parent_frame"));

  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0),
            model->FrameByName("parent_frame")->RawPose());

  // Test Resolve to get each link, joint and frame pose in model frame.
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("parent_link")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("child_link")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("joint")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 1, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->FrameByName("parent_frame")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 1, 0, 0, 0), pose);

  // joint frame relative to parent and child links
  EXPECT_TRUE(
    model->JointByName("joint")->
      SemanticPose().Resolve(pose, "child_link").empty());
  EXPECT_EQ(Pose(0, 1, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("joint")->
      SemanticPose().Resolve(pose, "parent_link").empty());
  EXPECT_EQ(Pose(0, 1, 9, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointChildFrame)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_child_frame.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("joint_child_frame", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("parent_link"));
  ASSERT_TRUE(model->LinkNameExists("child_link"));
  EXPECT_TRUE(model->LinkByName("parent_link")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("child_link")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0),
            model->LinkByName("parent_link")->RawPose());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0),
            model->LinkByName("child_link")->RawPose());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(1u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_EQ(nullptr, model->JointByIndex(1));
  ASSERT_TRUE(model->JointNameExists("joint"));
  EXPECT_EQ("child_frame", model->JointByName("joint")->ChildName());
  EXPECT_EQ("parent_link", model->JointByName("joint")->ParentName());

  std::string resolvedLinkName;
  EXPECT_TRUE(
    model->JointByName("joint")->ResolveChildLink(resolvedLinkName).empty());
  EXPECT_EQ("child_link", resolvedLinkName);
  EXPECT_TRUE(
    model->JointByName("joint")->ResolveParentLink(resolvedLinkName).empty());
  EXPECT_EQ("parent_link", resolvedLinkName);

  EXPECT_TRUE(model->JointByName("joint")->PoseRelativeTo().empty());
  EXPECT_EQ(Pose(0, 1, 0, 0, 0, 0), model->JointByName("joint")->RawPose());

  EXPECT_EQ(1u, model->FrameCount());
  EXPECT_NE(nullptr, model->FrameByIndex(0));
  EXPECT_EQ(nullptr, model->FrameByIndex(1));

  ASSERT_TRUE(model->FrameNameExists("child_frame"));

  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0),
            model->FrameByName("child_frame")->RawPose());

  // Test Resolve to get each link, joint and frame pose in model frame.
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("parent_link")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("child_link")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("joint")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 1, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->FrameByName("child_frame")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 10, 0, 0, 0), pose);

  // joint frame relative to parent and child links
  EXPECT_TRUE(
    model->JointByName("joint")->
      SemanticPose().Resolve(pose, "child_link").empty());
  EXPECT_EQ(Pose(1, 1, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("joint")->
      SemanticPose().Resolve(pose, "parent_link").empty());
  EXPECT_EQ(Pose(1, 1, 9, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadWorldJointChildFrame)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_joint_child_frame.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = gz::math::Pose3d;

  // Get the world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("world_joint_child_frame", world->Name());
  EXPECT_EQ(2u, world->ModelCount());
  EXPECT_NE(nullptr, world->ModelByIndex(0));
  EXPECT_NE(nullptr, world->ModelByIndex(1));
  EXPECT_EQ(nullptr, world->ModelByIndex(2));

  ASSERT_TRUE(world->ModelNameExists("parent_model"));
  ASSERT_TRUE(world->ModelNameExists("child_model"));
  EXPECT_TRUE(world->ModelByName("parent_model")->PoseRelativeTo().empty());
  EXPECT_TRUE(world->ModelByName("child_model")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0),
            world->ModelByName("parent_model")->RawPose());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0),
            world->ModelByName("child_model")->RawPose());

  EXPECT_EQ(2u, world->JointCount());
  EXPECT_NE(nullptr, world->JointByIndex(0));
  EXPECT_NE(nullptr, world->JointByIndex(1));
  EXPECT_EQ(nullptr, world->JointByIndex(2));
  ASSERT_TRUE(world->JointNameExists("J1"));
  ASSERT_TRUE(world->JointNameExists("J2"));
  EXPECT_EQ("child_frame", world->JointByName("J1")->ChildName());
  EXPECT_EQ("parent_model::L", world->JointByName("J1")->ParentName());
  EXPECT_EQ("child_model::L", world->JointByName("J2")->ChildName());
  EXPECT_EQ("parent_model::L", world->JointByName("J2")->ParentName());

  std::string resolvedLinkName;
  EXPECT_TRUE(
    world->JointByName("J1")->ResolveChildLink(resolvedLinkName).empty());
  EXPECT_EQ("child_model::L", resolvedLinkName);
  EXPECT_TRUE(
    world->JointByName("J1")->ResolveParentLink(resolvedLinkName).empty());
  EXPECT_EQ("parent_model::L", resolvedLinkName);
  EXPECT_TRUE(
    world->JointByName("J2")->ResolveChildLink(resolvedLinkName).empty());
  EXPECT_EQ("child_model::L", resolvedLinkName);
  EXPECT_TRUE(
    world->JointByName("J2")->ResolveParentLink(resolvedLinkName).empty());
  EXPECT_EQ("parent_model::L", resolvedLinkName);

  EXPECT_TRUE(world->JointByName("J1")->PoseRelativeTo().empty());
  EXPECT_TRUE(world->JointByName("J2")->PoseRelativeTo().empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), world->JointByName("J1")->RawPose());
  EXPECT_EQ(Pose(10, 0, 0, 0, 0, 0), world->JointByName("J2")->RawPose());

  EXPECT_EQ(1u, world->FrameCount());
  EXPECT_NE(nullptr, world->FrameByIndex(0));
  EXPECT_EQ(nullptr, world->FrameByIndex(1));

  ASSERT_TRUE(world->FrameNameExists("child_frame"));

  EXPECT_EQ(Pose(0, 1, 0, 0, 0, 0),
            world->FrameByName("child_frame")->RawPose());

  // Test Resolve to get each model, joint and frame pose in world frame.
  Pose pose;
  EXPECT_TRUE(
    world->ModelByName("parent_model")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->ModelByName("child_model")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(0, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->JointByName("J1")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(1, 1, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->JointByName("J2")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(10, 0, 10, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->FrameByName("child_frame")->
      SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(Pose(0, 1, 10, 0, 0, 0), pose);

  // joint frame relative to parent and child models
  EXPECT_TRUE(
    world->JointByName("J1")->
      SemanticPose().Resolve(pose, "child_frame").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->JointByName("J1")->
      SemanticPose().Resolve(pose, "parent_model::L").empty());
  EXPECT_EQ(Pose(1, 1, 9, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->JointByName("J2")->
      SemanticPose().Resolve(pose, "child_model::L").empty());
  EXPECT_EQ(Pose(10, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    world->JointByName("J2")->
      SemanticPose().Resolve(pose, "parent_model::L").empty());
  EXPECT_EQ(Pose(10, 0, 9, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(DOMJoint, WorldJointChildWorldNotPermitted)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_joint_child_world.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::JOINT_CHILD_LINK_INVALID);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "Joint with name[J2] specified invalid child link [world]"));
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointPoseRelativeTo)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_joint_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
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

  EXPECT_EQ(Pose(1, 0, 0, 0, GZ_PI/2, 0), model->LinkByName("P1")->RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, -GZ_PI/2, 0), model->LinkByName("C1")->RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, GZ_PI/2, 0), model->LinkByName("P2")->RawPose());
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

  // Test Resolve to get each link and joint pose in the model frame.
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("P1")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, GZ_PI/2, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("C1")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(2, 0, 0, 0, -GZ_PI/2, 0), pose);
  EXPECT_TRUE(
    model->JointByName("J1")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, -GZ_PI/2, 0), pose);

  EXPECT_TRUE(
    model->LinkByName("P2")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(3, 0, 0, 0, GZ_PI/2, 0), pose);
  EXPECT_TRUE(
    model->JointByName("J2")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(5, 0, 0, 0, GZ_PI/2, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("C2")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(5, 0, -4, 0, GZ_PI/2, 0), pose);

  // resolve pose of J1 relative to C1, J2 relative to P2
  // these should match the numbers in the model file
  EXPECT_TRUE(
    model->JointByName("J1")->SemanticPose().Resolve(pose, "C1").empty());
  EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("J2")->SemanticPose().Resolve(pose, "P2").empty());
  EXPECT_EQ(Pose(0, 0, 2, 0, 0, 0), pose);

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadInvalidJointPoseRelativeTo)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf",
        "model_invalid_joint_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(5u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_CYCLE);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "relative_to name[Jcycle] is identical to joint name[Jcycle], causing "
      "a graph cycle"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_INVALID);
  EXPECT_NE(std::string::npos,
    errors[1].Message().find(
      "relative_to name[A] specified by joint with name[J] does not match a "
      "nested model, link, joint, or frame name in model"));
  // errors[2]
  // errors[3]
  // errors[4]
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadInvalidChild)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_invalid_child.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(8u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::JOINT_CHILD_LINK_INVALID);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "Child frame with name[invalid] specified by joint with name[joint] not "
      "found"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR);
  EXPECT_NE(std::string::npos,
      errors[1].Message().find("FrameAttachedToGraph error, Non-LINK vertex "
                               "with name [joint_invalid_child::joint] is "
                               "disconnected"));
  // errors[2]
  // errors[3]
  // errors[4]
  // errors[5]
  // errors[6]
  // errors[7]
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadLinkJointSameName17Invalid)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf",
        "model_link_joint_same_name.sdf");

  // Read with sdf::readFile, which converts from 1.6 to latest
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  sdf::readFile(testFile, sdf);

  // Load the SDF file from the converted string and expect errors
  sdf::Root root;
  auto errors = root.LoadSdfString(sdf->Root()->ToString(""));
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(9u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::DUPLICATE_NAME);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "Joint with non-unique name [attachment] detected in model with name "
      "[link_joint_same_name]."));
  EXPECT_EQ(errors[3].Code(), sdf::ErrorCode::DUPLICATE_NAME);
  EXPECT_NE(std::string::npos,
    errors[3].Message().find(
      "Joint with non-unique name [attachment] detected in model with name "
      "[link_joint_same_name]."));
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadLinkJointSameName16Valid)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_link_joint_same_name.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_TRUE(errors.empty());

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("link_joint_same_name", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_EQ(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("base"));
  ASSERT_TRUE(model->LinkNameExists("attachment"));
  EXPECT_TRUE(model->LinkByName("base")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("attachment")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), model->LinkByName("base")->RawPose());
  EXPECT_EQ(Pose(0, 2, 0, 0, 0, 0), model->LinkByName("attachment")->RawPose());

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(1u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_EQ(nullptr, model->JointByIndex(1));
  // attachment joint name should be changed
  EXPECT_FALSE(model->JointNameExists("attachment"));
  ASSERT_TRUE(model->JointNameExists("attachment_joint"));
  EXPECT_TRUE(model->JointByName("attachment_joint")->PoseRelativeTo().empty());

  EXPECT_EQ(Pose(0, 0, 3, 0, 0, 0),
      model->JointByName("attachment_joint")->RawPose());

  // Test Resolve to get each link and joint pose in the model frame.
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("base")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("attachment")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 2, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("attachment_joint")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 2, 3, 0, 0, 0), pose);

  // Resolve poses relative to different frames
  EXPECT_TRUE(
    model->LinkByName("attachment")->
      SemanticPose().Resolve(pose, "base").empty());
  EXPECT_EQ(Pose(-1, 2, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->JointByName("attachment_joint")->
      SemanticPose().Resolve(pose, "base").empty());
  EXPECT_EQ(Pose(-1, 2, 3, 0, 0, 0), pose);

  EXPECT_TRUE(
    model->JointByName("attachment_joint")->
      SemanticPose().Resolve(pose, "attachment").empty());
  EXPECT_EQ(Pose(0, 0, 3, 0, 0, 0), pose);
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadURDFJointPoseRelativeTo)
{
  const std::string testFile =
    sdf::testing::TestFile("integration", "provide_feedback.urdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_TRUE(errors.empty());

  using Pose = gz::math::Pose3d;
  using Vector3 = gz::math::Vector3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("provide_feedback_test", model->Name());
  EXPECT_EQ(3u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_NE(nullptr, model->LinkByIndex(2));
  EXPECT_EQ(nullptr, model->LinkByIndex(3));

  ASSERT_TRUE(model->LinkNameExists("link0"));
  ASSERT_TRUE(model->LinkNameExists("link1"));
  ASSERT_TRUE(model->LinkNameExists("link2"));

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(3u, model->JointCount());
  EXPECT_NE(nullptr, model->JointByIndex(0));
  EXPECT_NE(nullptr, model->JointByIndex(1));
  EXPECT_NE(nullptr, model->JointByIndex(2));
  EXPECT_EQ(nullptr, model->JointByIndex(3));
  ASSERT_TRUE(model->JointNameExists("jointw0"));
  ASSERT_TRUE(model->JointNameExists("joint01"));
  ASSERT_TRUE(model->JointNameExists("joint12"));

  // Confirm each link's pose relative to parent joint is identity
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("link0")->
      SemanticPose().Resolve(pose, "jointw0").empty());
  EXPECT_EQ(Pose::Zero, pose);
  EXPECT_TRUE(
    model->LinkByName("link1")->
      SemanticPose().Resolve(pose, "joint01").empty());
  EXPECT_EQ(Pose::Zero, pose);
  EXPECT_TRUE(
    model->LinkByName("link2")->
      SemanticPose().Resolve(pose, "joint12").empty());
  EXPECT_EQ(Pose::Zero, pose);

  // Confirm joint pose relative to parent link matches origin tag
  EXPECT_TRUE(
    model->JointByName("jointw0")->
      SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(0, 0, 1.0, 0, 0, 1.57079632679), pose);
  EXPECT_TRUE(
    model->JointByName("joint01")->
      SemanticPose().Resolve(pose, "link0").empty());
  EXPECT_EQ(Pose(0, 0, -1.0, 0, 0, 1.57079632679), pose);
  EXPECT_TRUE(
    model->JointByName("joint12")->
      SemanticPose().Resolve(pose, "link1").empty());
  EXPECT_EQ(Pose(0, -3.0, 0, -1.57079632679, 0, -1.57079632679), pose);

  // Confirm joint axis relative to joint frame matches //axis/@xyz
  Vector3 vec3;
  EXPECT_TRUE(
    model->JointByName("jointw0")->Axis()->ResolveXyz(vec3, "jointw0").empty());
  EXPECT_EQ(Vector3(1.0, 0, 0), vec3);
  EXPECT_TRUE(
    model->JointByName("joint01")->Axis()->ResolveXyz(vec3, "joint01").empty());
  EXPECT_EQ(Vector3(1.0, 0, 0), vec3);
  EXPECT_TRUE(
    model->JointByName("joint12")->Axis()->ResolveXyz(vec3, "joint12").empty());
  EXPECT_EQ(Vector3(0, 1.0, 0), vec3);
}

/////////////////////////////////////////////////
TEST(DOMJoint, LoadJointNestedParentChild)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "joint_nested_parent_child.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  {
    const sdf::Joint *j1 = model->JointByName("J1");
    ASSERT_NE(nullptr, j1);
    EXPECT_EQ("M1::L1", j1->ParentName());
    EXPECT_EQ("L1", j1->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j1->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::L1", resolvedLinkName);
    EXPECT_TRUE(j1->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j1->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(0, 0, 9, 0, GZ_PI_2, 0), pose);
  }
  {
    const sdf::Joint *j2 = model->JointByName("J2");
    ASSERT_NE(nullptr, j2);
    EXPECT_EQ("F1", j2->ParentName());
    EXPECT_EQ("L1", j2->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j2->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::L1", resolvedLinkName);
    EXPECT_TRUE(j2->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j2->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(0, 1, 10, 0, GZ_PI_2, 0), pose);
  }
  {
    const sdf::Joint *j3 = model->JointByName("J3");
    ASSERT_NE(nullptr, j3);
    EXPECT_EQ("L1", j3->ParentName());
    EXPECT_EQ("M1::L2", j3->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j3->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);
    EXPECT_TRUE(j3->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::L2", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j3->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(1, 1, 1, 0, 0, 0), pose);
  }
  {
    const sdf::Joint *j4 = model->JointByName("J4");
    ASSERT_NE(nullptr, j4);
    EXPECT_EQ("L1", j4->ParentName());
    EXPECT_EQ("M1::F1", j4->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j4->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);
    EXPECT_TRUE(j4->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::L1", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j4->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(1, 0, 1, 0, 0, 0), pose);
  }
  {
    const sdf::Joint *j5 = model->JointByName("J5");
    ASSERT_NE(nullptr, j5);
    EXPECT_EQ("L1", j5->ParentName());
    EXPECT_EQ("M1::M2", j5->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j5->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);
    EXPECT_TRUE(j5->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::M2::L1", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j5->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(0, -1, 1, GZ_PI_2, 0, 0), pose);
  }
  {
    const sdf::Joint *j6 = model->JointByName("J6");
    ASSERT_NE(nullptr, j6);
    EXPECT_EQ("M1::__model__", j6->ParentName());
    EXPECT_EQ("L1", j6->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j6->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::L1", resolvedLinkName);
    EXPECT_TRUE(j6->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j6->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(1, 0, 10, 0, GZ_PI_2, 0), pose);
  }
  {
    const sdf::Joint *j7 = model->JointByName("J7");
    ASSERT_NE(nullptr, j7);
    EXPECT_EQ("L1", j7->ParentName());
    EXPECT_EQ("M1::__model__", j7->ChildName());

    std::string resolvedLinkName;
    EXPECT_TRUE(j7->ResolveParentLink(resolvedLinkName).empty());
    EXPECT_EQ("L1", resolvedLinkName);
    EXPECT_TRUE(j7->ResolveChildLink(resolvedLinkName).empty());
    EXPECT_EQ("M1::L1", resolvedLinkName);

    Pose pose;
    EXPECT_TRUE(j7->SemanticPose().Resolve(pose, "__model__").empty());
    EXPECT_EQ(Pose(0, 0, 1, 0, 0, 0), pose);
  }
}


//////////////////////////////////////////////////
TEST(DOMJoint, Sensors)
{
  const std::string testFile =
      sdf::testing::TestFile("sdf", "joint_sensors.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model", model->Name());

  // Get the first joint
  const sdf::Joint *joint = model->JointByIndex(0);
  ASSERT_NE(nullptr, joint);
  EXPECT_EQ("joint", joint->Name());
  EXPECT_EQ(1u, joint->SensorCount());

  gz::math::Pose3d pose;
  // Get the force_torque sensor
  const sdf::Sensor *forceTorqueSensor =
    joint->SensorByName("force_torque_sensor");
  ASSERT_NE(nullptr, forceTorqueSensor);
  EXPECT_EQ("force_torque_sensor", forceTorqueSensor->Name());
  EXPECT_EQ(sdf::SensorType::FORCE_TORQUE, forceTorqueSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
      forceTorqueSensor->RawPose());
  EXPECT_TRUE(
      forceTorqueSensor->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0), pose);
  EXPECT_TRUE(forceTorqueSensor->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0), pose);

  auto forceTorqueSensorObj = forceTorqueSensor->ForceTorqueSensor();
  ASSERT_NE(nullptr, forceTorqueSensorObj);
  EXPECT_EQ(sdf::ForceTorqueFrame::PARENT, forceTorqueSensorObj->Frame());
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD,
      forceTorqueSensorObj->MeasureDirection());

  EXPECT_DOUBLE_EQ(0.0, forceTorqueSensorObj->ForceXNoise().Mean());
  EXPECT_DOUBLE_EQ(0.1, forceTorqueSensorObj->ForceXNoise().StdDev());
  EXPECT_DOUBLE_EQ(1.0, forceTorqueSensorObj->ForceYNoise().Mean());
  EXPECT_DOUBLE_EQ(1.1, forceTorqueSensorObj->ForceYNoise().StdDev());
  EXPECT_DOUBLE_EQ(2.0, forceTorqueSensorObj->ForceZNoise().Mean());
  EXPECT_DOUBLE_EQ(2.1, forceTorqueSensorObj->ForceZNoise().StdDev());

  EXPECT_DOUBLE_EQ(3.0, forceTorqueSensorObj->TorqueXNoise().Mean());
  EXPECT_DOUBLE_EQ(3.1, forceTorqueSensorObj->TorqueXNoise().StdDev());
  EXPECT_DOUBLE_EQ(4.0, forceTorqueSensorObj->TorqueYNoise().Mean());
  EXPECT_DOUBLE_EQ(4.1, forceTorqueSensorObj->TorqueYNoise().StdDev());
  EXPECT_DOUBLE_EQ(5.0, forceTorqueSensorObj->TorqueZNoise().Mean());
  EXPECT_DOUBLE_EQ(5.1, forceTorqueSensorObj->TorqueZNoise().StdDev());

}
