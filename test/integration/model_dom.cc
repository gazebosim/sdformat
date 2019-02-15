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
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "test_config.h"

using namespace ignition::math;

//////////////////////////////////////////////////
TEST(DOMModel, NotAModel)
{
  // Create an Element that is not a model
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Model model;
  sdf::Errors errors = model.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Model") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMModel, NoName)
{
  // Create a "model" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("model");

  sdf::Model model;
  sdf::Errors errors = model.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("model name is required") !=
               std::string::npos);
}

/////////////////////////////////////////////////
TEST(DOMModel, LoadLinkCheck)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("default", world->Name());

  // Get the first model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("ground_plane", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByIndex(0));
  ASSERT_FALSE(nullptr == model->LinkByName("link"));
  EXPECT_EQ(model->LinkByName("link")->Name(), model->LinkByIndex(0)->Name());
  EXPECT_TRUE(nullptr == model->LinkByIndex(1));
  EXPECT_TRUE(model->LinkNameExists("link"));
  EXPECT_FALSE(model->LinkNameExists("links"));
}

/////////////////////////////////////////////////
TEST(DOMModel, LoadDuplicateLinks)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_duplicate_links.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::DUPLICATE_NAME, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "link with name[link] already exists"));
}

/////////////////////////////////////////////////
TEST(DOMModel, LoadDuplicateJoints)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_duplicate_joints.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::DUPLICATE_NAME, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "joint with name[joint] already exists"));
}

/////////////////////////////////////////////////
TEST(DOMModel, LoadLinkJointWithSameName)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_link_joint_same_name.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("link_joint_same_name", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_EQ(1u, model->JointCount());

  const sdf::Link *linkBase = model->LinkByName("base");
  ASSERT_TRUE(linkBase != nullptr);
  const sdf::Link *linkAttachment = model->LinkByName("attachment");
  ASSERT_TRUE(linkAttachment != nullptr);
  const sdf::Joint *jointAttachment = model->JointByName("attachment");
  ASSERT_TRUE(jointAttachment != nullptr);

  EXPECT_EQ(model->Name(), linkBase->PoseFrame());
  EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), linkBase->Pose());
  EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), linkBase->PoseInFrame(model->Name()));

  EXPECT_EQ(model->Name(), linkAttachment->PoseFrame());
  EXPECT_EQ(Pose3d(0, 2, 0, 0, 0, 0), linkAttachment->Pose());
  // this link has the same name as a joint
  // duplicate name in frame graph causes PoseInFrame to return infinite pose
  EXPECT_FALSE(linkAttachment->PoseInFrame(model->Name()).IsFinite());

  EXPECT_EQ(linkAttachment->Name(), jointAttachment->PoseFrame());
  EXPECT_EQ(Pose3d(0, 0, 3, 0, 0, 0), jointAttachment->Pose());
  // this joint has the same name as a link
  // duplicate name in frame graph causes PoseInFrame to return infinite pose
  EXPECT_FALSE(jointAttachment->PoseInFrame(model->Name()).IsFinite());

  // the attachment_offset frame is ambiguous since its pose frame "attachment"
  // is defined by two separate vertices in the frame graph.
  // so PoseInFrame returns an infinite pose
  EXPECT_FALSE(jointAttachment->PoseInFrame("attachment_offset").IsFinite());
}

/////////////////////////////////////////////////
TEST(DOMModel, LoadDoublePendulum)
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
  EXPECT_EQ("double_pendulum_with_base", model->Name());
  EXPECT_EQ(3u, model->LinkCount());
  EXPECT_FALSE(nullptr == model->LinkByIndex(0));
  EXPECT_FALSE(nullptr == model->LinkByIndex(1));
  EXPECT_FALSE(nullptr == model->LinkByIndex(2));
  EXPECT_TRUE(nullptr == model->LinkByIndex(3));
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0), model->Pose());
  EXPECT_EQ("", model->PoseFrame());

  EXPECT_TRUE(model->LinkNameExists("base"));
  EXPECT_TRUE(model->LinkNameExists("upper_link"));
  EXPECT_TRUE(model->LinkNameExists("lower_link"));

  EXPECT_EQ(2u, model->JointCount());
  EXPECT_FALSE(nullptr == model->JointByIndex(0));
  EXPECT_FALSE(nullptr == model->JointByIndex(1));
  EXPECT_TRUE(nullptr == model->JointByIndex(2));

  EXPECT_TRUE(model->JointNameExists("upper_joint"));
  EXPECT_TRUE(model->JointNameExists("lower_joint"));
}

//////////////////////////////////////////////////
TEST(DOMModel, FourBar)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "four_bar.sdf");
  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_TRUE(model != nullptr);

  const sdf::Link *linkOne = model->LinkByName("link1");
  ASSERT_TRUE(linkOne != nullptr);
  const sdf::Link *linkTwo = model->LinkByName("link2");
  ASSERT_TRUE(linkTwo != nullptr);
  const sdf::Link *linkThree = model->LinkByName("link3");
  ASSERT_TRUE(linkThree != nullptr);
  const sdf::Link *linkFour = model->LinkByName("link4");
  ASSERT_TRUE(linkFour != nullptr);

  const sdf::Joint *jointOne = model->JointByName("joint1");
  ASSERT_TRUE(jointOne != nullptr);
  const sdf::Joint *jointTwo = model->JointByName("joint2");
  ASSERT_TRUE(jointTwo != nullptr);
  const sdf::Joint *jointThree = model->JointByName("joint3");
  ASSERT_TRUE(jointThree != nullptr);
  const sdf::Joint *jointFour = model->JointByName("joint4");
  ASSERT_TRUE(jointFour != nullptr);

  // Link 1
  EXPECT_EQ(Pose3d(0, 0.2, 0.05, 0, 0, 0), linkOne->Pose());
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), linkOne->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(-0.2, 0.2, 0, 0, 0, 0), linkOne->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(0, 0.4, 0, 0, 0, 0), linkOne->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0.2, 0.2, 0, 0, 0, 0), linkOne->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(-0.2, 0, 0, 0, 0, 0), linkOne->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(-0.2, 0.4, 0, 0, 0, 0), linkOne->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0.2, 0.4, 0, 0, 0, 0), linkOne->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0.2, 0.2, 0.05, 0, 0, 0), linkOne->PoseInFrame("joint4"));

  // Link 2
  EXPECT_EQ(Pose3d(0.2, 0, 0.05, 0, 0, 0), linkTwo->Pose());
  EXPECT_EQ(Pose3d(0.2, -0.2, 0, 0, 0, 0), linkTwo->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), linkTwo->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(0.2, 0.2, 0, 0, 0, 0), linkTwo->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0.4, 0, 0, 0, 0, 0), linkTwo->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(0.2, -0.2, 0.05, 0, 0, 0), linkTwo->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(0, 0.2, 0, 0, 0, 0), linkTwo->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0.4, 0.2, 0, 0, 0, 0), linkTwo->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0.4, -0.2, 0, 0, 0, 0), linkTwo->PoseInFrame("joint4"));

  // Link 3
  EXPECT_EQ(Pose3d(0, -0.2, 0.05, 0, 0, 0), linkThree->Pose());
  EXPECT_EQ(Pose3d(0, -0.4, 0, 0, 0, 0), linkThree->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(-0.2, -0.2, 0, 0, 0, 0), linkThree->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), linkThree->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0.2, -0.2, 0, 0, 0, 0), linkThree->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(-0.2, -0.4, 0, 0, 0, 0), linkThree->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(-0.2, -0.2, 0.05, 0, 0, 0),
            linkThree->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0.2, 0, 0, 0, 0, 0), linkThree->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0.2, -0.4, 0, 0, 0, 0), linkThree->PoseInFrame("joint4"));

  // Link 4
  EXPECT_EQ(Pose3d(-0.2, 0, 0.05, 0, 0, 0), linkFour->Pose());
  EXPECT_EQ(Pose3d(-0.2, -0.2, 0, 0, 0, 0), linkFour->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(-0.4, 0, 0, 0, 0, 0), linkFour->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(-0.2, 0.2, 0, 0, 0, 0), linkFour->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), linkFour->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(-0.4, -0.2, 0, 0, 0, 0), linkFour->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(-0.4, 0.2, 0, 0, 0, 0), linkFour->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(-0.2, 0.2, 0.05, 0, 0, 0), linkFour->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0, -0.2, 0, 0, 0, 0), linkFour->PoseInFrame("joint4"));

  // Joint 1
  EXPECT_EQ(Pose3d(0, 0.2, 0, 0, 0, 0), jointOne->Pose());
  EXPECT_EQ(Pose3d(0.2, 0, 0, 0, 0, 0), jointOne->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(0, 0.2, 0, 0, 0, 0), jointOne->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(0.2, 0.4, 0, 0, 0, 0), jointOne->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0.4, 0.2, 0, 0, 0, 0), jointOne->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), jointOne->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(0, 0.4, 0, 0, 0, 0), jointOne->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0.4, 0.4, 0, 0, 0, 0), jointOne->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0.4, 0, 0, 0, 0, 0), jointOne->PoseInFrame("joint4"));

  // Joint 2
  EXPECT_EQ(Pose3d(0.2, 0, 0, 0, 0, 0), jointTwo->Pose());
  EXPECT_EQ(Pose3d(0.2, -0.4, 0, 0, 0, 0), jointTwo->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(0, -0.2, 0, 0, 0, 0), jointTwo->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(0.2, 0, 0, 0, 0, 0), jointTwo->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0.4, -0.2, 0, 0, 0, 0), jointTwo->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(0, -0.4, 0, 0, 0, 0), jointTwo->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), jointTwo->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0.4, 0, 0, 0, 0, 0), jointTwo->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0.4, -0.4, 0, 0, 0, 0), jointTwo->PoseInFrame("joint4"));

  // Joint 3
  EXPECT_EQ(Pose3d(0, -0.2, 0, 0, 0, 0), jointThree->Pose());
  EXPECT_EQ(Pose3d(-0.2, -0.4, 0, 0, 0, 0), jointThree->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(-0.4, -0.2, 0, 0, 0, 0), jointThree->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(-0.2, 0, 0, 0, 0, 0), jointThree->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0, -0.2, 0, 0, 0, 0), jointThree->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(-0.4, -0.4, 0, 0, 0, 0), jointThree->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(-0.4, 0, 0, 0, 0, 0), jointThree->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), jointThree->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0, -0.4, 0, 0, 0, 0), jointThree->PoseInFrame("joint4"));

  // Joint 4
  EXPECT_EQ(Pose3d(-0.2, 0, 0, 0, 0, 0), jointFour->Pose());
  EXPECT_EQ(Pose3d(-0.2, 0, 0, 0, 0, 0), jointFour->PoseInFrame("link1"));
  EXPECT_EQ(Pose3d(-0.4, 0.2, 0, 0, 0, 0), jointFour->PoseInFrame("link2"));
  EXPECT_EQ(Pose3d(-0.2, 0.4, 0, 0, 0, 0), jointFour->PoseInFrame("link3"));
  EXPECT_EQ(Pose3d(0, 0.2, 0, 0, 0, 0), jointFour->PoseInFrame("link4"));

  EXPECT_EQ(Pose3d(-0.4, 0, 0, 0, 0, 0), jointFour->PoseInFrame("joint1"));
  EXPECT_EQ(Pose3d(-0.4, 0.4, 0, 0, 0, 0), jointFour->PoseInFrame("joint2"));
  EXPECT_EQ(Pose3d(0, 0.4, 0, 0, 0, 0), jointFour->PoseInFrame("joint3"));
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), jointFour->PoseInFrame("joint4"));
}

/////////////////////////////////////////////////
TEST(DOMModel, ToStringSameAsSDF)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "double_pendulum.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ(1u, root.ModelCount());

  sdf::SDFPtr sdf(new sdf::SDF());
  ASSERT_TRUE(sdf::init(sdf));
  ASSERT_TRUE(sdf::readFile(testFile, sdf));
  EXPECT_EQ(sdf->Root()->ToString(""), root.Element()->ToString(""));
}
