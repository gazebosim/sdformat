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
#include <gz/math/Pose3.hh>
#include "sdf/Actor.hh"
#include "sdf/Plugin.hh"

#include <filesystem>

/////////////////////////////////////////////////
sdf::Animation CreateDummyAnimation()
{
  sdf::Animation animation;
  animation.SetName("animation");
  animation.SetFilename("animation_filename");
  animation.SetFilePath("animation_filepath");
  animation.SetScale(1.234);
  animation.SetInterpolateX(true);
  return animation;
}

/////////////////////////////////////////////////
bool AnimationsEqual(const sdf::Animation &_anim1, const sdf::Animation &_anim2)
{
  constexpr double EPS = 1e-6;
  return (_anim1.Name() == _anim2.Name()) &&
    (_anim1.Filename() == _anim2.Filename()) &&
    (_anim1.FilePath() == _anim2.FilePath()) &&
    (std::abs(_anim1.Scale() - _anim2.Scale()) < EPS) &&
    (_anim1.InterpolateX() == _anim2.InterpolateX());
}

/////////////////////////////////////////////////
sdf::Trajectory CreateDummyTrajectory()
{
  sdf::Trajectory trajectory;
  sdf::Waypoint waypoint;
  waypoint.SetTime(0.12);
  waypoint.SetPose({3, 2, 1, 0, GZ_PI, 0});
  trajectory.SetId(1234);
  trajectory.SetType("trajectory_type");
  trajectory.SetTension(4.567);
  trajectory.AddWaypoint(waypoint);
  return trajectory;
}

/////////////////////////////////////////////////
bool TrajectoriesEqual(const sdf::Trajectory &_traj1,
    const sdf::Trajectory &_traj2)
{
  constexpr double EPS = 1e-6;
  bool waypointsEqual = true;
  if (_traj1.WaypointCount() != _traj2.WaypointCount())
  {
    return false;
  }
  for (uint64_t wp_idx = 0; wp_idx < _traj1.WaypointCount(); ++wp_idx)
  {
    auto wp1 = _traj1.WaypointByIndex(wp_idx);
    auto wp2 = _traj2.WaypointByIndex(wp_idx);
    waypointsEqual &= (std::abs(wp1->Time() - wp2->Time()) < EPS) &&
      wp1->Pose() == wp2->Pose();
  }
  return waypointsEqual &&
    (_traj1.Id() == _traj2.Id()) &&
    (_traj1.Type() == _traj2.Type()) &&
    (std::abs(_traj1.Tension() - _traj2.Tension()) < EPS);
}

/////////////////////////////////////////////////
sdf::Actor CreateDummyActor()
{
  sdf::Actor actor;
  actor.SetName("test_dummy_actor");
  actor.SetRawPose({3, 2, 1, 0, GZ_PI, 0});
  actor.SetPoseRelativeTo("ground_plane");
  actor.SetSkinFilename("walk.dae");
  actor.SetSkinScale(2.0);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.8);
  actor.SetScriptAutoStart(false);
  actor.SetFilePath("/home/path/to/model.sdf");
  // Add a dummy trajectory and animation as well
  actor.AddTrajectory(CreateDummyTrajectory());
  actor.AddAnimation(CreateDummyAnimation());
  return actor;
}

/////////////////////////////////////////////////
bool ActorsEqual(const sdf::Actor &_actor1, const sdf::Actor &_actor2)
{
  constexpr double EPS = 1e-6;
  // Check animations, trajectories and properties
  bool animationsEqual = true;
  if (_actor1.AnimationCount() != _actor2.AnimationCount())
  {
    return false;
  }
  for (uint64_t anim_idx = 0; anim_idx < _actor1.AnimationCount(); ++anim_idx)
  {
    animationsEqual &= AnimationsEqual(
        *_actor1.AnimationByIndex(anim_idx),
        *_actor2.AnimationByIndex(anim_idx));
  }
  bool trajectoriesEqual = true;
  if (_actor1.TrajectoryCount() != _actor2.TrajectoryCount())
  {
    return false;
  }
  for (uint64_t traj_idx = 0; traj_idx < _actor1.TrajectoryCount(); ++traj_idx)
  {
    trajectoriesEqual &= TrajectoriesEqual(
        *_actor1.TrajectoryByIndex(traj_idx),
        *_actor2.TrajectoryByIndex(traj_idx));
  }
  return animationsEqual && trajectoriesEqual &&
    (_actor1.Name() == _actor2.Name()) &&
    (_actor1.RawPose() == _actor2.RawPose()) &&
    (_actor1.PoseRelativeTo() == _actor2.PoseRelativeTo()) &&
    (_actor1.SkinFilename() == _actor2.SkinFilename()) &&
    (std::abs(_actor1.SkinScale() - _actor2.SkinScale()) < EPS) &&
    (_actor1.ScriptLoop() == _actor2.ScriptLoop()) &&
    (std::abs(_actor1.ScriptDelayStart() - _actor2.ScriptDelayStart()) < EPS) &&
    (_actor1.ScriptAutoStart() == _actor2.ScriptAutoStart()) &&
    (_actor1.FilePath() == _actor2.FilePath()) &&
    (_actor1.LinkCount() == _actor2.LinkCount()) &&
    (_actor1.JointCount() == _actor2.JointCount()) &&
    (_actor1.Element() == _actor2.Element());
}

/////////////////////////////////////////////////
TEST(DOMActor, DefaultConstruction)
{
  sdf::Actor actor;
  EXPECT_EQ("__default__", actor.Name());
  EXPECT_EQ(gz::math::Pose3d::Zero, actor.RawPose());
  EXPECT_EQ("", actor.PoseRelativeTo());
  EXPECT_EQ(nullptr, actor.Element());
  EXPECT_EQ("__default__", actor.SkinFilename());
  EXPECT_DOUBLE_EQ(1.0, actor.SkinScale());
  EXPECT_TRUE(actor.FilePath().empty());

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

  EXPECT_EQ(0u, actor.LinkCount());
  EXPECT_EQ(nullptr, actor.LinkByIndex(0));
  EXPECT_EQ(nullptr, actor.LinkByIndex(1));
  EXPECT_FALSE(actor.LinkNameExists(""));

  EXPECT_EQ(0u, actor.JointCount());
  EXPECT_EQ(nullptr, actor.JointByIndex(0));
  EXPECT_EQ(nullptr, actor.JointByIndex(1));
  EXPECT_FALSE(actor.JointNameExists(""));

  EXPECT_EQ(nullptr, actor.Element());
}

/////////////////////////////////////////////////
TEST(DOMActor, CopyConstructor)
{
  sdf::Actor actor = CreateDummyActor();
  sdf::Actor actor2(actor);
  EXPECT_TRUE(ActorsEqual(actor, actor2));
}

/////////////////////////////////////////////////
TEST(DOMActor, CopyAssignmentOperator)
{
  sdf::Actor actor = CreateDummyActor();
  sdf::Actor actor2;
  actor2 = actor;
  EXPECT_TRUE(ActorsEqual(actor, actor2));
}

/////////////////////////////////////////////////
TEST(DOMActor, MoveConstructor)
{
  sdf::Actor actor = CreateDummyActor();
  sdf::Actor actor2(std::move(actor));
  EXPECT_TRUE(ActorsEqual(CreateDummyActor(), actor2));
}

/////////////////////////////////////////////////
TEST(DOMActor, MoveAssignment)
{
  sdf::Actor actor = CreateDummyActor();
  sdf::Actor actor2;
  actor2 = std::move(actor);
  EXPECT_TRUE(ActorsEqual(CreateDummyActor(), actor2));
}

/////////////////////////////////////////////////
TEST(DOMActor, CopyAssignmentAfterMove)
{
  sdf::Actor actor1 = CreateDummyActor();
  sdf::Actor actor2;

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Actor tmp = std::move(actor1);
  actor1 = actor2;
  actor2 = tmp;

  EXPECT_TRUE(ActorsEqual(sdf::Actor(), actor1));
  EXPECT_TRUE(ActorsEqual(CreateDummyActor(), actor2));
}

//////////////////////////////////////////////////
TEST(DOMActor, Add)
{
  sdf::Actor actor;

  // Animation
  EXPECT_EQ(0u, actor.AnimationCount());

  sdf::Animation anim1;
  anim1.SetName("animation1");
  anim1.SetFilename("animation_filename1");

  actor.AddAnimation(anim1);
  EXPECT_EQ(1u, actor.AnimationCount());
  EXPECT_EQ("animation1", actor.AnimationByIndex(0)->Name());
  EXPECT_EQ("animation_filename1", actor.AnimationByIndex(0)->Filename());

  sdf::Animation anim2;
  anim2.SetName("animation2");
  anim2.SetFilename("animation_filename2");

  actor.AddAnimation(anim2);
  EXPECT_EQ(2u, actor.AnimationCount());
  EXPECT_EQ("animation2", actor.AnimationByIndex(1)->Name());
  EXPECT_EQ("animation_filename2", actor.AnimationByIndex(1)->Filename());

  // Waypoint
  sdf::Trajectory traj1;
  EXPECT_EQ(0u, traj1.WaypointCount());

  sdf::Waypoint way1;
  way1.SetTime(0.123);

  traj1.AddWaypoint(way1);
  EXPECT_EQ(1u, traj1.WaypointCount());
  EXPECT_DOUBLE_EQ(0.123, traj1.WaypointByIndex(0)->Time());

  sdf::Waypoint way2;
  way2.SetTime(0.456);

  traj1.AddWaypoint(way2);
  EXPECT_EQ(2u, traj1.WaypointCount());
  EXPECT_DOUBLE_EQ(0.456, traj1.WaypointByIndex(1)->Time());

  // Trajectory
  EXPECT_EQ(0u, actor.TrajectoryCount());

  traj1.SetId(123u);
  traj1.SetType("trajectory1");

  actor.AddTrajectory(traj1);
  EXPECT_EQ(1u, actor.TrajectoryCount());
  EXPECT_EQ(123u, actor.TrajectoryByIndex(0)->Id());
  EXPECT_EQ("trajectory1", actor.TrajectoryByIndex(0)->Type());

  sdf::Trajectory traj2;
  traj2.SetId(456u);
  traj2.SetType("trajectory2");

  actor.AddTrajectory(traj2);
  EXPECT_EQ(2u, actor.TrajectoryCount());
  EXPECT_EQ(456u, actor.TrajectoryByIndex(1)->Id());
  EXPECT_EQ("trajectory2", actor.TrajectoryByIndex(1)->Type());
}

//////////////////////////////////////////////////
TEST(DOMAnimation, DefaultConstruction)
{
  sdf::Animation anim;

  EXPECT_EQ(anim.Name(), "__default__");
  EXPECT_EQ(anim.Filename(), "__default__");
  EXPECT_EQ(anim.FilePath(), "");
  EXPECT_DOUBLE_EQ(anim.Scale(), 1.0);
  EXPECT_EQ(anim.InterpolateX(), false);
}

//////////////////////////////////////////////////
TEST(DOMAnimation, CopyConstructor)
{
  sdf::Animation anim1 = CreateDummyAnimation();
  sdf::Animation anim2(anim1);
  EXPECT_TRUE(AnimationsEqual(anim1, anim2));
}

//////////////////////////////////////////////////
TEST(DOMAnimation, CopyAssignmentOperator)
{
  sdf::Animation anim1 = CreateDummyAnimation();
  sdf::Animation anim2;
  anim2 = anim1;
  EXPECT_TRUE(AnimationsEqual(anim1, anim2));
}

//////////////////////////////////////////////////
TEST(DOMAnimation, MoveConstructor)
{
  sdf::Animation anim1 = CreateDummyAnimation();
  sdf::Animation anim2(std::move(anim1));
  EXPECT_TRUE(AnimationsEqual(CreateDummyAnimation(), anim2));
}

//////////////////////////////////////////////////
TEST(DOMAnimation, MoveAssignment)
{
  sdf::Animation anim1 = CreateDummyAnimation();
  sdf::Animation anim2;
  anim2 = std::move(anim1);
  EXPECT_TRUE(AnimationsEqual(CreateDummyAnimation(), anim2));
}

/////////////////////////////////////////////////
TEST(DOMAnimation, CopyAssignmentAfterMove)
{
  sdf::Animation anim1 = CreateDummyAnimation();
  sdf::Animation anim2;

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Animation tmp = std::move(anim1);
  anim1 = anim2;
  anim2 = tmp;

  EXPECT_TRUE(AnimationsEqual(sdf::Animation(), anim1));
  EXPECT_TRUE(AnimationsEqual(CreateDummyAnimation(), anim2));
}

//////////////////////////////////////////////////
TEST(DOMWaypoint, DefaultConstruction)
{
  sdf::Waypoint waypoint;
  EXPECT_DOUBLE_EQ(waypoint.Time(), 0.0);
  EXPECT_EQ(waypoint.Pose(), gz::math::Pose3d::Zero);
}

//////////////////////////////////////////////////
TEST(DOMWaypoint, CopyConstructor)
{
  sdf::Waypoint waypoint1;
  waypoint1.SetTime(1.23);
  waypoint1.SetPose({3, 2, 1, 0, GZ_PI, 0});

  sdf::Waypoint waypoint2(waypoint1);
  EXPECT_DOUBLE_EQ(waypoint1.Time(), waypoint2.Time());
  EXPECT_EQ(waypoint1.Pose(), waypoint2.Pose());
}

//////////////////////////////////////////////////
TEST(DOMWaypoint, CopyAssignmentOperator)
{
  sdf::Waypoint waypoint1;
  waypoint1.SetTime(1.23);
  waypoint1.SetPose({3, 2, 1, 0, GZ_PI, 0});

  sdf::Waypoint waypoint2;
  waypoint2 = waypoint1;
  EXPECT_DOUBLE_EQ(waypoint1.Time(), waypoint2.Time());
  EXPECT_EQ(waypoint1.Pose(), waypoint2.Pose());
}

//////////////////////////////////////////////////
TEST(DOMWaypoint, MoveConstructor)
{
  sdf::Waypoint waypoint1;
  gz::math::Pose3d pose1(3, 2, 1, 0, GZ_PI, 0);
  waypoint1.SetTime(1.23);
  waypoint1.SetPose(pose1);

  sdf::Waypoint waypoint2(std::move(waypoint1));
  EXPECT_DOUBLE_EQ(1.23, waypoint2.Time());
  EXPECT_EQ(pose1, waypoint2.Pose());
}

//////////////////////////////////////////////////
TEST(DOMWaypoint, MoveAssignment)
{
  sdf::Waypoint waypoint1;
  gz::math::Pose3d pose1(3, 2, 1, 0, GZ_PI, 0);
  waypoint1.SetTime(1.23);
  waypoint1.SetPose(pose1);

  sdf::Waypoint waypoint2;
  waypoint2 = std::move(waypoint1);
  EXPECT_DOUBLE_EQ(1.23, waypoint2.Time());
  EXPECT_EQ(pose1, waypoint2.Pose());
}

/////////////////////////////////////////////////
TEST(DOMWaypoint, CopyAssignmentAfterMove)
{
  sdf::Waypoint waypoint1;
  gz::math::Pose3d pose1(3, 2, 1, 0, GZ_PI, 0);
  waypoint1.SetTime(1.23);
  waypoint1.SetPose(pose1);
  sdf::Waypoint waypoint2;
  gz::math::Pose3d pose2(1, 2, 3, 1, 2, GZ_PI);
  waypoint2.SetTime(3.45);
  waypoint2.SetPose(pose2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Waypoint tmp = std::move(waypoint1);
  waypoint1 = waypoint2;
  waypoint2 = tmp;

  EXPECT_DOUBLE_EQ(1.23, waypoint2.Time());
  EXPECT_EQ(pose1, waypoint2.Pose());
  EXPECT_DOUBLE_EQ(3.45, waypoint1.Time());
  EXPECT_EQ(pose2, waypoint1.Pose());
}

//////////////////////////////////////////////////
TEST(DOMTrajectory, DefaultConstruction)
{
  sdf::Trajectory trajectory;

  EXPECT_EQ(trajectory.Id(), 0u);
  EXPECT_EQ(trajectory.Type(), "__default__");
  EXPECT_DOUBLE_EQ(trajectory.Tension(), 0.0);
  EXPECT_EQ(trajectory.WaypointCount(), 0u);
}

//////////////////////////////////////////////////
TEST(DOMTrajectory, CopyConstructor)
{
  sdf::Trajectory trajectory1 = CreateDummyTrajectory();
  sdf::Trajectory trajectory2(trajectory1);
  EXPECT_TRUE(TrajectoriesEqual(trajectory1, trajectory2));
}

//////////////////////////////////////////////////
TEST(DOMTrajectory, CopyAssignmentOperator)
{
  sdf::Trajectory trajectory1 = CreateDummyTrajectory();
  sdf::Trajectory trajectory2;
  trajectory2 = trajectory1;
  EXPECT_TRUE(TrajectoriesEqual(trajectory1, trajectory2));
}

//////////////////////////////////////////////////
TEST(DOMTrajectory, MoveConstructor)
{
  sdf::Trajectory trajectory1 = CreateDummyTrajectory();
  sdf::Trajectory trajectory2(std::move(trajectory1));
  EXPECT_TRUE(TrajectoriesEqual(CreateDummyTrajectory(), trajectory2));
}

//////////////////////////////////////////////////
TEST(DOMTrajectory, MoveAssignment)
{
  sdf::Trajectory trajectory1 = CreateDummyTrajectory();
  sdf::Trajectory trajectory2;
  trajectory2 = std::move(trajectory1);
  EXPECT_TRUE(TrajectoriesEqual(CreateDummyTrajectory(), trajectory2));
}

/////////////////////////////////////////////////
TEST(DOMTrajectory, CopyAssignmentAfterMove)
{
  sdf::Trajectory trajectory1 = CreateDummyTrajectory();
  sdf::Trajectory trajectory2;

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Trajectory tmp = std::move(trajectory1);
  trajectory1 = trajectory2;
  trajectory2 = tmp;

  EXPECT_TRUE(TrajectoriesEqual(sdf::Trajectory(), trajectory1));
  EXPECT_TRUE(TrajectoriesEqual(CreateDummyTrajectory(), trajectory2));
}

/////////////////////////////////////////////////
TEST(DOMActor, ToElement)
{
  sdf::Actor actor;

  actor.SetName("my-actor");
  actor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  actor.SetSkinFilename("my-skinfilename");
  actor.SetSkinScale(1.2);
  actor.SetScriptLoop(true);
  actor.SetScriptDelayStart(2.4);
  actor.SetScriptAutoStart(true);

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 1; ++i)
    {
      sdf::Link link;
      link.SetName("link" + std::to_string(i));
      EXPECT_TRUE(actor.AddLink(link));
      EXPECT_FALSE(actor.AddLink(link));
    }
    if (j == 0)
      actor.ClearLinks();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 2; ++i)
    {
      sdf::Joint joint;
      joint.SetName("joint" + std::to_string(i));
      EXPECT_TRUE(actor.AddJoint(joint));
      EXPECT_FALSE(actor.AddJoint(joint));
    }
    if (j == 0)
      actor.ClearJoints();
  }

  sdf::Trajectory trajectory;
  trajectory.SetId(1);
  trajectory.SetType("type");
  trajectory.SetTension(1.0);
  sdf::Waypoint waypointA, waypointB;
  waypointA.SetTime(0.0);
  waypointA.SetPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  trajectory.AddWaypoint(waypointA);
  waypointB.SetTime(1.0);
  waypointB.SetPose(gz::math::Pose3d(2, 4, 6, 0.1, 0.2, 0.3));
  trajectory.AddWaypoint(waypointB);
  actor.AddTrajectory(trajectory);

  sdf::Animation animation;
  animation.SetName("my-animation");
  animation.SetFilename("my-filename");
  animation.SetScale(1.2);
  animation.SetInterpolateX(true);

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");
  actor.AddPlugin(plugin);

  sdf::ElementPtr elem = actor.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Actor actor2;
  actor2.Load(elem);

  EXPECT_EQ(actor.Name(), actor2.Name());
  EXPECT_EQ(actor.RawPose(), actor2.RawPose());
  EXPECT_DOUBLE_EQ(actor.SkinScale(), actor2.SkinScale());
  EXPECT_EQ(actor.ScriptLoop(), actor2.ScriptLoop());
  EXPECT_DOUBLE_EQ(actor.ScriptDelayStart(), actor2.ScriptDelayStart());

  EXPECT_EQ(actor.LinkCount(), actor2.LinkCount());
  for (uint64_t i = 0; i < actor2.LinkCount(); ++i)
    EXPECT_NE(nullptr, actor2.LinkByIndex(i));

  EXPECT_EQ(actor.JointCount(), actor2.JointCount());
  for (uint64_t i = 0; i < actor2.JointCount(); ++i)
    EXPECT_NE(nullptr, actor2.JointByIndex(i));

  ASSERT_EQ(1u, actor2.TrajectoryCount());
  const sdf::Trajectory *trajectory2 = actor2.TrajectoryByIndex(0u);
  ASSERT_NE(nullptr, trajectory2);
  EXPECT_EQ(trajectory.Id(), trajectory2->Id());
  EXPECT_EQ(trajectory.Type(), trajectory2->Type());
  EXPECT_DOUBLE_EQ(trajectory.Tension(), trajectory2->Tension());

  ASSERT_EQ(2u, trajectory2->WaypointCount());
  const sdf::Waypoint *waypointA2 = trajectory2->WaypointByIndex(0u);
  ASSERT_NE(nullptr, waypointA2);
  EXPECT_DOUBLE_EQ(waypointA.Time(), waypointA2->Time());
  EXPECT_EQ(waypointA.Pose(), waypointA2->Pose());

  const sdf::Waypoint *waypointB2 = trajectory2->WaypointByIndex(1u);
  ASSERT_NE(nullptr, waypointB2);
  EXPECT_DOUBLE_EQ(waypointB.Time(), waypointB2->Time());
  EXPECT_EQ(waypointB.Pose(), waypointB2->Pose());

  ASSERT_EQ(1u, actor2.Plugins().size());
  EXPECT_EQ("name1", actor2.Plugins()[0].Name());
  EXPECT_EQ("filename1", actor2.Plugins()[0].Filename());
}

/////////////////////////////////////////////////
TEST(DOMActor, Plugins)
{
  sdf::Actor actor;
  EXPECT_TRUE(actor.Plugins().empty());

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");

  actor.AddPlugin(plugin);
  ASSERT_EQ(1u, actor.Plugins().size());

  plugin.SetName("name2");
  actor.AddPlugin(plugin);
  ASSERT_EQ(2u, actor.Plugins().size());

  EXPECT_EQ("name1", actor.Plugins()[0].Name());
  EXPECT_EQ("name2", actor.Plugins()[1].Name());

  actor.ClearPlugins();
  EXPECT_TRUE(actor.Plugins().empty());
}
