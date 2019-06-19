/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ignition/math/Pose3.hh>
#include "sdf/Actor.hh"

/////////////////////////////////////////////////
TEST(DOMActor, DefaultConstruction)
{
  sdf::Actor actor;
  EXPECT_EQ("__default__", actor.Name());
  EXPECT_EQ(ignition::math::Pose3d::Zero, actor.Pose());
  EXPECT_EQ("", actor.PoseFrame());
  EXPECT_TRUE(actor.ActorStatic());
  EXPECT_EQ(nullptr, actor.Element());
  EXPECT_EQ("__default__", actor.SkinFilename());
  EXPECT_DOUBLE_EQ(1.0, actor.SkinScale());

  EXPECT_EQ(0u, actor.AnimationCount());
  EXPECT_EQ(nullptr, actor.AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor.AnimationByIndex(1));
  EXPECT_FALSE(actor.AnimationNameExists(""));
  EXPECT_FALSE(actor.AnimationNameExists("default"));

  EXPECT_TRUE(actor.ScriptLoop());
  EXPECT_DOUBLE_EQ(0.0, actor.ScriptDelayStart());
  EXPECT_TRUE(actor.ScriptAutoStart());

  EXPECT_EQ(0u, actor.TrajectoryCount());
  EXPECT_EQ(nullptr, actor.TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor.TrajectoryByIndex(1));
  EXPECT_FALSE(actor.TrajectoryIdExists(0));
  EXPECT_FALSE(actor.TrajectoryIdExists(1));

  EXPECT_EQ(nullptr, actor.Element());
}

/////////////////////////////////////////////////
TEST(DOMActor, CopyConstructor)
{
  sdf::Actor actor;
  actor.SetName("test_copy_actor");
  actor.SetPose({3, 2, 1, 0, IGN_PI, 0});
  actor.SetPoseFrame("ground_plane");
  actor.SetActorStatic(true);
  actor.SetSkinFilename("walk.dae");
  actor.SetSkinScale(2.0);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.8);
  actor.SetScriptAutoStart(false);

  sdf::Actor actor2(actor);
  EXPECT_EQ("test_copy_actor", actor2.Name());
  EXPECT_EQ(ignition::math::Pose3d(3, 2, 1, 0, IGN_PI, 0), actor2.Pose());
  EXPECT_EQ("ground_plane", actor2.PoseFrame());
  EXPECT_TRUE(actor2.ActorStatic());

  EXPECT_EQ(0u, actor2.AnimationCount());
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(1));
  EXPECT_FALSE(actor2.AnimationNameExists(""));
  EXPECT_FALSE(actor2.AnimationNameExists("default"));

  EXPECT_EQ("walk.dae", actor2.SkinFilename());
  EXPECT_DOUBLE_EQ(2.0, actor2.SkinScale());

  EXPECT_EQ(0u, actor2.TrajectoryCount());
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(1));
  EXPECT_FALSE(actor2.TrajectoryIdExists(0));
  EXPECT_FALSE(actor2.TrajectoryIdExists(1));

  EXPECT_TRUE(actor2.ScriptLoop());
  EXPECT_DOUBLE_EQ(2.8, actor2.ScriptDelayStart());
  EXPECT_FALSE(actor2.ScriptAutoStart());
}

/////////////////////////////////////////////////
TEST(DOMActor, CopyAssignmentOperator)
{
  sdf::Actor actor;
  actor.SetName("test_actor_assignment");
  actor.SetPose({3, 2, 1, 0, IGN_PI, 0});
  actor.SetPoseFrame("ground_plane");
  actor.SetActorStatic(true);
  actor.SetSkinFilename("walk.dae");
  actor.SetSkinScale(2.0);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.8);
  actor.SetScriptAutoStart(false);

  sdf::Actor actor2;
  actor2 = actor;
  EXPECT_EQ("test_actor_assignment", actor2.Name());
  EXPECT_EQ(ignition::math::Pose3d(3, 2, 1, 0, IGN_PI, 0), actor2.Pose());
  EXPECT_EQ("ground_plane", actor2.PoseFrame());
  EXPECT_TRUE(actor2.ActorStatic());

  EXPECT_EQ(0u, actor2.AnimationCount());
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(1));
  EXPECT_FALSE(actor2.AnimationNameExists(""));
  EXPECT_FALSE(actor2.AnimationNameExists("default"));

  EXPECT_EQ("walk.dae", actor2.SkinFilename());
  EXPECT_DOUBLE_EQ(2.0, actor2.SkinScale());

  EXPECT_EQ(0u, actor2.TrajectoryCount());
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(1));
  EXPECT_FALSE(actor2.TrajectoryIdExists(0));
  EXPECT_FALSE(actor2.TrajectoryIdExists(1));

  EXPECT_TRUE(actor2.ScriptLoop());
  EXPECT_DOUBLE_EQ(2.8, actor2.ScriptDelayStart());
  EXPECT_FALSE(actor2.ScriptAutoStart());
}

/////////////////////////////////////////////////
TEST(DOMActor, MoveConstructor)
{
  sdf::Actor actor;
  actor.SetName("test_actor_assignment");
  actor.SetPose({3, 2, 1, 0, IGN_PI, 0});
  actor.SetPoseFrame("ground_plane");
  actor.SetActorStatic(true);
  actor.SetSkinFilename("walk.dae");
  actor.SetSkinScale(2.0);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.8);
  actor.SetScriptAutoStart(false);

  sdf::Actor actor2(std::move(actor));
  EXPECT_EQ("test_actor_assignment", actor2.Name());
  EXPECT_EQ(ignition::math::Pose3d(3, 2, 1, 0, IGN_PI, 0), actor2.Pose());
  EXPECT_EQ("ground_plane", actor2.PoseFrame());
  EXPECT_TRUE(actor2.ActorStatic());

  EXPECT_EQ(0u, actor2.AnimationCount());
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(1));
  EXPECT_FALSE(actor2.AnimationNameExists(""));
  EXPECT_FALSE(actor2.AnimationNameExists("default"));

  EXPECT_EQ("walk.dae", actor2.SkinFilename());
  EXPECT_DOUBLE_EQ(2.0, actor2.SkinScale());

  EXPECT_EQ(0u, actor2.TrajectoryCount());
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(1));
  EXPECT_FALSE(actor2.TrajectoryIdExists(0));
  EXPECT_FALSE(actor2.TrajectoryIdExists(1));

  EXPECT_TRUE(actor2.ScriptLoop());
  EXPECT_DOUBLE_EQ(2.8, actor2.ScriptDelayStart());
  EXPECT_FALSE(actor2.ScriptAutoStart());
}

/////////////////////////////////////////////////
TEST(DOMActor, MoveAssignment)
{
  sdf::Actor actor;
  actor.SetName("test_actor_assignment");
  actor.SetPose({3, 2, 1, 0, IGN_PI, 0});
  actor.SetPoseFrame("ground_plane");
  actor.SetActorStatic(true);
  actor.SetSkinFilename("walk.dae");
  actor.SetSkinScale(2.0);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.8);
  actor.SetScriptAutoStart(false);

  sdf::Actor actor2;
  actor2 = std::move(actor);
  EXPECT_EQ("test_actor_assignment", actor2.Name());
  EXPECT_EQ(ignition::math::Pose3d(3, 2, 1, 0, IGN_PI, 0), actor2.Pose());
  EXPECT_EQ("ground_plane", actor2.PoseFrame());
  EXPECT_TRUE(actor2.ActorStatic());

  EXPECT_EQ(0u, actor2.AnimationCount());
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor2.AnimationByIndex(1));
  EXPECT_FALSE(actor2.AnimationNameExists(""));
  EXPECT_FALSE(actor2.AnimationNameExists("default"));

  EXPECT_EQ("walk.dae", actor2.SkinFilename());
  EXPECT_DOUBLE_EQ(2.0, actor2.SkinScale());

  EXPECT_EQ(0u, actor2.TrajectoryCount());
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor2.TrajectoryByIndex(1));
  EXPECT_FALSE(actor2.TrajectoryIdExists(0));
  EXPECT_FALSE(actor2.TrajectoryIdExists(1));

  EXPECT_TRUE(actor2.ScriptLoop());
  EXPECT_DOUBLE_EQ(2.8, actor2.ScriptDelayStart());
  EXPECT_FALSE(actor2.ScriptAutoStart());
}

/////////////////////////////////////////////////
TEST(DOMActor, CopyAssignmentAfterMove)
{
  sdf::Actor actor1;
  actor1.SetName("actor1");
  actor1.SetPose({3, 2, 1, 0, IGN_PI, 0});
  actor1.SetPoseFrame("ground_plane_1");
  actor1.SetActorStatic(true);
  actor1.SetSkinFilename("walk.dae");
  actor1.SetSkinScale(2.0);
  actor1.SetScriptLoop(true);
  actor1.SetScriptDelayStart(2.8);
  actor1.SetScriptAutoStart(false);

  sdf::Actor actor2;
  actor2.SetName("actor2");
  actor2.SetPose({1, 2, 3, 0, IGN_PI, 0});
  actor2.SetPoseFrame("ground_plane_2");
  actor2.SetActorStatic(false);
  actor2.SetSkinFilename("run.dae");
  actor2.SetSkinScale(0.5);
  actor2.SetScriptLoop(false);
  actor2.SetScriptDelayStart(0.8);
  actor2.SetScriptAutoStart(true);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Actor tmp = std::move(actor1);
  actor1 = actor2;
  actor2 = tmp;

  EXPECT_EQ("actor2", actor1.Name());
  EXPECT_EQ("actor1", actor2.Name());

  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, IGN_PI, 0), actor1.Pose());
  EXPECT_EQ(ignition::math::Pose3d(3, 2, 1, 0, IGN_PI, 0), actor2.Pose());

  EXPECT_EQ("ground_plane_2", actor1.PoseFrame());
  EXPECT_EQ("ground_plane_1", actor2.PoseFrame());

  EXPECT_FALSE(actor1.ActorStatic());
  EXPECT_TRUE(actor2.ActorStatic());

  EXPECT_EQ("run.dae", actor1.SkinFilename());
  EXPECT_EQ("walk.dae", actor2.SkinFilename());

  EXPECT_DOUBLE_EQ(0.5, actor1.SkinScale());
  EXPECT_DOUBLE_EQ(2.0, actor2.SkinScale());

  EXPECT_FALSE(actor1.ScriptLoop());
  EXPECT_TRUE(actor2.ScriptLoop());

  EXPECT_DOUBLE_EQ(0.8, actor1.ScriptDelayStart());
  EXPECT_DOUBLE_EQ(2.8, actor2.ScriptDelayStart());

  EXPECT_TRUE(actor1.ScriptAutoStart());
  EXPECT_FALSE(actor2.ScriptAutoStart());
}
