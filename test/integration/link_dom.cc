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
#include "sdf/Collision.hh"
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMLink, NotALink)
{
  // Create an Element that is not a link
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Link link;
  std::shared_ptr<sdf::FrameGraph> frameGraph(new sdf::FrameGraph);
  sdf::Errors errors = link.Load(element, frameGraph);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Link") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMLink, NoName)
{
  // Create a "link" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("link");

  sdf::Link link;
  std::shared_ptr<sdf::FrameGraph> frameGraph(new sdf::FrameGraph);
  sdf::Errors errors = link.Load(element, frameGraph);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("link name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMLink, LoadVisualCollision)
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

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("link", link->Name());

  // Get the first visual
  EXPECT_EQ(1u, link->VisualCount());
  EXPECT_TRUE(link->VisualNameExists("visual"));
  EXPECT_FALSE(link->VisualNameExists("visuals"));
  const sdf::Visual *visual = link->VisualByIndex(0);
  ASSERT_NE(nullptr, visual);
  EXPECT_EQ("visual", visual->Name());

  // Get the first collision
  EXPECT_EQ(1u, link->CollisionCount());
  EXPECT_TRUE(link->CollisionNameExists("collision"));
  EXPECT_FALSE(link->CollisionNameExists("collisions"));
  const sdf::Collision *collision = link->CollisionByIndex(0);
  ASSERT_NE(nullptr, collision);
  EXPECT_EQ("collision", collision->Name());
}

//////////////////////////////////////////////////
TEST(DOMLink, InertialDoublePendulum)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *baseLink = model->LinkByIndex(0);
  ASSERT_NE(nullptr, baseLink);
  EXPECT_EQ(ignition::math::Pose3d::Zero, baseLink->Pose());
  EXPECT_EQ("", baseLink->PoseFrame());

  const ignition::math::Inertiald inertial = baseLink->Inertial();
  EXPECT_DOUBLE_EQ(100.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Z());

  const sdf::Link *upperLink = model->LinkByIndex(1);
  ASSERT_NE(nullptr, upperLink);
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 2.1, -1.5708, 0, 0),
      upperLink->Pose());
  EXPECT_EQ("", upperLink->PoseFrame());

  const ignition::math::Inertiald inertialUpper = upperLink->Inertial();
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.Pose().Pos().X());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.Pose().Pos().Y());
  EXPECT_DOUBLE_EQ(0.5, inertialUpper.Pose().Pos().Z());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  const sdf::Link *lowerLink = model->LinkByIndex(2);
  ASSERT_TRUE(lowerLink != nullptr);
  EXPECT_EQ(ignition::math::Pose3d(0.25, 1.0, 2.1, -2, 0, 0),
      lowerLink->Pose());
  EXPECT_EQ("", lowerLink->PoseFrame());
}

//////////////////////////////////////////////////
TEST(DOMLink, InertialComplete)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "inertial_complete.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const ignition::math::Inertiald inertial = link->Inertial();
  EXPECT_DOUBLE_EQ(17.982, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(0.125569, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0972062, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.117937, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0008, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(-0.000499757,
      inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(-0.0005, inertial.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.01, inertial.Pose().Pos().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.Pose().Pos().Y());
  EXPECT_DOUBLE_EQ(0.02, inertial.Pose().Pos().Z());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());
}

//////////////////////////////////////////////////
TEST(DOMLink, InertialInvalid)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "inertial_invalid.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::LINK_INERTIA_INVALID);
  EXPECT_EQ(errors[0].Message(), "A link named link has invalid inertia.");

  // TODO: make this failure less severe?
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_EQ(model, nullptr);
}

//////////////////////////////////////////////////
TEST(DOMLink, LinkChain)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "link_chain.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_TRUE(model != nullptr);

  const sdf::Link *linkOne = model->LinkByIndex(0);
  const sdf::Link *linkTwo = model->LinkByIndex(1);

  ignition::math::Pose3d pose1 = linkOne->Pose("link_chain");
  std::cout << pose1 << std::endl;

  /*ignition::math::Pose3d pose2 = linkTwo->Pose("one");
  std::cout << pose2 << std::endl;
  */
}
