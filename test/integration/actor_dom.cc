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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Actor.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMWorld, LoadActors)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  EXPECT_EQ(2u, world->ActorCount());
  EXPECT_NE(nullptr, world->ActorByIndex(0));
  EXPECT_NE(nullptr, world->ActorByIndex(1));
  EXPECT_EQ(nullptr, world->ActorByIndex(2));
  EXPECT_FALSE(world->ActorNameExists(""));
  EXPECT_TRUE(world->ActorNameExists("actor_1"));
  EXPECT_TRUE(world->ActorNameExists("actor_2"));

  const sdf::Actor *actor_1 = world->ActorByIndex(0);
  EXPECT_EQ("actor_1", actor_1->Name());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), actor_1->Pose());
  EXPECT_EQ("", actor_1->PoseFrame());
  EXPECT_TRUE(actor_1->ActorStatic());
  EXPECT_EQ(1u, actor_1->AnimationCount());
  EXPECT_NE(nullptr, actor_1->AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor_1->AnimationByIndex(1));
  EXPECT_EQ("walk.dae", actor_1->AnimationByIndex(0)->Filename());
  EXPECT_DOUBLE_EQ(1.0, actor_1->AnimationByIndex(0)->Scale());
  EXPECT_TRUE(actor_1->AnimationByIndex(0)->InterpolateX());
  EXPECT_FALSE(actor_1->AnimationNameExists(""));
  EXPECT_TRUE(actor_1->AnimationNameExists("walking"));
  EXPECT_EQ("walk.dae", actor_1->SkinFilename());
  EXPECT_DOUBLE_EQ(1.0, actor_1->SkinScale());
  EXPECT_EQ(1u, actor_1->TrajectoryCount());
  EXPECT_NE(nullptr, actor_1->TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor_1->TrajectoryByIndex(1));
  EXPECT_EQ(0u, actor_1->TrajectoryByIndex(0)->Id());
  EXPECT_EQ("walking", actor_1->TrajectoryByIndex(0)->Type());
  EXPECT_EQ(4u, actor_1->TrajectoryByIndex(0)->WaypointCount());
  EXPECT_TRUE(actor_1->TrajectoryIdExists(0));
  EXPECT_FALSE(actor_1->TrajectoryIdExists(1));
  EXPECT_TRUE(actor_1->ScriptLoop());
  EXPECT_DOUBLE_EQ(1.0, actor_1->ScriptDelayStart());
  EXPECT_TRUE(actor_1->ScriptAutoStart());

  const sdf::Actor *actor_2 = world->ActorByIndex(1);
  EXPECT_EQ("actor_2", actor_2->Name());
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0), actor_2->Pose());
  EXPECT_EQ("", actor_2->PoseFrame());
  EXPECT_TRUE(actor_2->ActorStatic());
  EXPECT_EQ(1u, actor_2->AnimationCount());
  EXPECT_NE(nullptr, actor_2->AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor_2->AnimationByIndex(1));
  EXPECT_EQ("run.dae", actor_2->AnimationByIndex(0)->Filename());
  EXPECT_DOUBLE_EQ(2.0, actor_2->AnimationByIndex(0)->Scale());
  EXPECT_FALSE(actor_2->AnimationByIndex(0)->InterpolateX());
  EXPECT_FALSE(actor_2->AnimationNameExists(""));
  EXPECT_FALSE(actor_2->AnimationNameExists("walking"));
  EXPECT_TRUE(actor_2->AnimationNameExists("run"));
  EXPECT_EQ("run.dae", actor_2->SkinFilename());
  EXPECT_DOUBLE_EQ(0.5, actor_2->SkinScale());
  EXPECT_EQ(1u, actor_2->TrajectoryCount());
  EXPECT_NE(nullptr, actor_2->TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor_2->TrajectoryByIndex(1));
  EXPECT_EQ(1u, actor_2->TrajectoryByIndex(0)->Id());
  EXPECT_EQ("run", actor_2->TrajectoryByIndex(0)->Type());
  EXPECT_EQ(5u, actor_2->TrajectoryByIndex(0)->WaypointCount());
  EXPECT_FALSE(actor_2->TrajectoryIdExists(0));
  EXPECT_TRUE(actor_2->TrajectoryIdExists(1));
  EXPECT_FALSE(actor_2->ScriptLoop());
  EXPECT_DOUBLE_EQ(2.7, actor_2->ScriptDelayStart());
  EXPECT_FALSE(actor_2->ScriptAutoStart());
}
