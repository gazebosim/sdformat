/*
 * Copyright 2023 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include <gz/math/Vector3.hh>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Material.hh"
#include "sdf/Model.hh"
#include "sdf/Projector.hh"
#include "sdf/Pbr.hh"
#include "sdf/Filesystem.hh"
#include "test_config.hh"

//////////////////////////////////////////////////
TEST(DOMWorld, LoadProjector)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  gz::math::Pose3d pose;

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(gz::math::Pose3d(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("frame1", model->PoseRelativeTo());
  EXPECT_TRUE(model->SemanticPose().Resolve(pose, "frame1").empty());
  errors = model->SemanticPose().Resolve(pose, "frame1");
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_EQ(gz::math::Pose3d(0, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(model->SemanticPose().Resolve(pose, "world").empty());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), pose);
  EXPECT_TRUE(model->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), pose);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  const sdf::Projector *linkProjector = link->ProjectorByIndex(0);
  ASSERT_NE(nullptr, linkProjector);
  EXPECT_EQ("projector", linkProjector->Name());
  EXPECT_TRUE(linkProjector->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), pose);

  EXPECT_DOUBLE_EQ(0.03, linkProjector->NearClip());
  EXPECT_DOUBLE_EQ(3.0, linkProjector->FarClip());
  EXPECT_EQ(gz::math::Angle(0.8), linkProjector->HorizontalFov());
  EXPECT_EQ(0x01, linkProjector->VisibilityFlags());
  EXPECT_EQ("materials/textures/projector.png", linkProjector->Texture());

  ASSERT_EQ(1u, linkProjector->Plugins().size());
  EXPECT_EQ("projector_plugin", linkProjector->Plugins()[0].Name());
  EXPECT_EQ("test/file/projector", linkProjector->Plugins()[0].Filename());
}
