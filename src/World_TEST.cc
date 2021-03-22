/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <ignition/math/Color.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/World.hh"

/////////////////////////////////////////////////
TEST(DOMWorld, Construction)
{
  sdf::World world;
  EXPECT_EQ(nullptr, world.Element());
  EXPECT_TRUE(world.Name().empty());
  EXPECT_EQ(ignition::math::Vector3d(0, 0, -9.80665), world.Gravity());
  EXPECT_EQ(ignition::math::Vector3d(5.5645e-6, 22.8758e-6, -42.3884e-6),
            world.MagneticField());
  EXPECT_STREQ("default", world.AudioDevice().c_str());
  EXPECT_EQ(ignition::math::Vector3d::Zero, world.WindLinearVelocity());

  EXPECT_EQ(0u, world.ModelCount());
  EXPECT_EQ(nullptr, world.ModelByIndex(0));
  EXPECT_EQ(nullptr, world.ModelByIndex(1));
  EXPECT_FALSE(world.ModelNameExists(""));
  EXPECT_FALSE(world.ModelNameExists("default"));
  EXPECT_FALSE(world.ModelNameExists("a::b"));
  EXPECT_FALSE(world.ModelNameExists("a::b::c"));
  EXPECT_FALSE(world.ModelNameExists("::::"));
  EXPECT_EQ(nullptr, world.ModelByName(""));
  EXPECT_EQ(nullptr, world.ModelByName("default"));
  EXPECT_EQ(nullptr, world.ModelByName("a::b"));
  EXPECT_EQ(nullptr, world.ModelByName("a::b::c"));
  EXPECT_EQ(nullptr, world.ModelByName("::::"));

  EXPECT_EQ(0u, world.FrameCount());
  EXPECT_EQ(nullptr, world.FrameByIndex(0));
  EXPECT_EQ(nullptr, world.FrameByIndex(1));
  EXPECT_FALSE(world.FrameNameExists(""));
  EXPECT_FALSE(world.FrameNameExists("default"));

  EXPECT_EQ(0u, world.FrameCount());
  EXPECT_EQ(nullptr, world.FrameByIndex(0));
  EXPECT_EQ(nullptr, world.FrameByIndex(1));
  EXPECT_FALSE(world.FrameNameExists(""));
  EXPECT_FALSE(world.FrameNameExists("default"));

  EXPECT_EQ(1u, world.PhysicsCount());
}

/////////////////////////////////////////////////
TEST(DOMWorld, CopyConstructor)
{
  sdf::World world;
  sdf::Atmosphere atmosphere;
  atmosphere.SetPressure(0.1);
  world.SetAtmosphere(atmosphere);
  world.SetAudioDevice("test_audio_device");
  world.SetGravity({1, 0, 0});

  sdf::Gui gui;
  gui.SetFullscreen(true);
  world.SetGui(gui);

  sdf::Scene scene;
  scene.SetGrid(true);
  world.SetScene(scene);

  world.SetMagneticField({0, 1, 0});
  world.SetName("test_world");

  world.SetWindLinearVelocity({0, 0, 1});

  sdf::World world2(world);

  ASSERT_TRUE(nullptr != world.Atmosphere());
  EXPECT_DOUBLE_EQ(0.1, world.Atmosphere()->Pressure());
  EXPECT_EQ("test_audio_device", world.AudioDevice());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, world.Gravity());

  ASSERT_TRUE(nullptr != world.Gui());
  EXPECT_EQ(gui.Fullscreen(), world.Gui()->Fullscreen());

  ASSERT_TRUE(nullptr != world.Scene());
  EXPECT_EQ(scene.Grid(), world.Scene()->Grid());

  EXPECT_EQ(ignition::math::Vector3d::UnitY, world.MagneticField());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, world.WindLinearVelocity());
  EXPECT_EQ("test_world", world.Name());

  ASSERT_TRUE(nullptr != world2.Atmosphere());
  EXPECT_DOUBLE_EQ(0.1, world2.Atmosphere()->Pressure());
  EXPECT_EQ("test_audio_device", world2.AudioDevice());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, world2.Gravity());

  ASSERT_TRUE(nullptr != world2.Gui());
  EXPECT_EQ(gui.Fullscreen(), world2.Gui()->Fullscreen());

  ASSERT_TRUE(nullptr != world2.Scene());
  EXPECT_EQ(scene.Grid(), world2.Scene()->Grid());

  EXPECT_EQ(ignition::math::Vector3d::UnitY, world2.MagneticField());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, world2.WindLinearVelocity());
  EXPECT_EQ("test_world", world2.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, CopyAssignmentOperator)
{
  sdf::World world;
  sdf::Atmosphere atmosphere;
  atmosphere.SetPressure(0.1);
  world.SetAtmosphere(atmosphere);
  world.SetAudioDevice("test_audio_device");
  world.SetGravity({1, 0, 0});

  sdf::Gui gui;
  gui.SetFullscreen(true);
  world.SetGui(gui);

  sdf::Scene scene;
  scene.SetGrid(true);
  world.SetScene(scene);

  world.SetMagneticField({0, 1, 0});
  world.SetName("test_world");

  world.SetWindLinearVelocity({0, 0, 1});

  sdf::World world2;
  world2 = world;

  ASSERT_TRUE(nullptr != world.Atmosphere());
  EXPECT_DOUBLE_EQ(0.1, world.Atmosphere()->Pressure());
  EXPECT_EQ("test_audio_device", world.AudioDevice());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, world.Gravity());

  ASSERT_TRUE(nullptr != world.Gui());
  EXPECT_EQ(gui.Fullscreen(), world.Gui()->Fullscreen());

  ASSERT_TRUE(nullptr != world.Scene());
  EXPECT_EQ(scene.Grid(), world.Scene()->Grid());

  EXPECT_EQ(ignition::math::Vector3d::UnitY, world.MagneticField());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, world.WindLinearVelocity());
  EXPECT_EQ("test_world", world.Name());

  ASSERT_TRUE(nullptr != world2.Atmosphere());
  EXPECT_DOUBLE_EQ(0.1, world2.Atmosphere()->Pressure());
  EXPECT_EQ("test_audio_device", world2.AudioDevice());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, world2.Gravity());

  ASSERT_TRUE(nullptr != world2.Gui());
  EXPECT_EQ(gui.Fullscreen(), world2.Gui()->Fullscreen());

  ASSERT_TRUE(nullptr != world2.Scene());
  EXPECT_EQ(scene.Grid(), world2.Scene()->Grid());

  EXPECT_EQ(ignition::math::Vector3d::UnitY, world2.MagneticField());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, world2.WindLinearVelocity());
  EXPECT_EQ("test_world", world2.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, MoveConstructor)
{
  sdf::World world;
  sdf::Atmosphere atmosphere;
  atmosphere.SetPressure(0.1);
  world.SetAtmosphere(atmosphere);
  world.SetAudioDevice("test_audio_device");
  world.SetGravity({1, 0, 0});

  sdf::Gui gui;
  gui.SetFullscreen(true);
  world.SetGui(gui);

  sdf::Scene scene;
  scene.SetGrid(true);
  world.SetScene(scene);

  world.SetMagneticField({0, 1, 0});
  world.SetName("test_world");

  world.SetWindLinearVelocity({0, 0, 1});

  sdf::World world2(std::move(world));

  ASSERT_TRUE(nullptr != world2.Atmosphere());
  EXPECT_DOUBLE_EQ(0.1, world2.Atmosphere()->Pressure());
  EXPECT_EQ("test_audio_device", world2.AudioDevice());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, world2.Gravity());

  ASSERT_TRUE(nullptr != world2.Gui());
  EXPECT_EQ(gui.Fullscreen(), world2.Gui()->Fullscreen());

  ASSERT_TRUE(nullptr != world2.Scene());
  EXPECT_EQ(scene.Grid(), world2.Scene()->Grid());

  EXPECT_EQ(ignition::math::Vector3d::UnitY, world2.MagneticField());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, world2.WindLinearVelocity());
  EXPECT_EQ("test_world", world2.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, MoveAssignmentOperator)
{
  sdf::World world;
  sdf::Atmosphere atmosphere;
  atmosphere.SetPressure(0.1);
  world.SetAtmosphere(atmosphere);
  world.SetAudioDevice("test_audio_device");
  world.SetGravity({1, 0, 0});

  sdf::Gui gui;
  gui.SetFullscreen(true);
  world.SetGui(gui);

  sdf::Scene scene;
  scene.SetGrid(true);
  world.SetScene(scene);

  world.SetMagneticField({0, 1, 0});
  world.SetName("test_world");

  world.SetWindLinearVelocity({0, 0, 1});

  sdf::World world2;
  world2 = std::move(world);

  ASSERT_TRUE(nullptr != world2.Atmosphere());
  EXPECT_DOUBLE_EQ(0.1, world2.Atmosphere()->Pressure());
  EXPECT_EQ("test_audio_device", world2.AudioDevice());
  EXPECT_EQ(ignition::math::Vector3d::UnitX, world2.Gravity());

  ASSERT_TRUE(nullptr != world2.Gui());
  EXPECT_EQ(gui.Fullscreen(), world2.Gui()->Fullscreen());

  ASSERT_TRUE(nullptr != world2.Scene());
  EXPECT_EQ(scene.Grid(), world2.Scene()->Grid());

  EXPECT_EQ(ignition::math::Vector3d::UnitY, world2.MagneticField());
  EXPECT_EQ(ignition::math::Vector3d::UnitZ, world2.WindLinearVelocity());
  EXPECT_EQ("test_world", world2.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, CopyAssignmentAfterMove)
{
  sdf::World world1;
  world1.SetName("world1");

  sdf::World world2;
  world2.SetName("world2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::World tmp = std::move(world1);
  world1 = world2;
  world2 = tmp;

  EXPECT_EQ("world2", world1.Name());
  EXPECT_EQ("world1", world2.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, Set)
{
  sdf::World world;
  EXPECT_TRUE(world.Name().empty());

  world.SetName("default");
  EXPECT_EQ("default", world.Name());

  world.SetAudioDevice("/dev/audio");
  EXPECT_EQ("/dev/audio", world.AudioDevice());

  world.SetWindLinearVelocity({0, 1 , 2});
  EXPECT_EQ(ignition::math::Vector3d(0, 1, 2), world.WindLinearVelocity());

  world.SetGravity({1, -2, 4});
  EXPECT_EQ(ignition::math::Vector3d(1, -2, 4), world.Gravity());

  world.SetMagneticField({1.2, -2.3, 4.5});
  EXPECT_EQ(ignition::math::Vector3d(1.2, -2.3, 4.5), world.MagneticField());
}

/////////////////////////////////////////////////
TEST(DOMWorld, SetGui)
{
  sdf::Gui gui;
  gui.SetFullscreen(true);

  sdf::World world;
  EXPECT_EQ(nullptr, world.Gui());

  world.SetGui(gui);
  ASSERT_NE(nullptr, world.Gui());
  EXPECT_TRUE(world.Gui()->Fullscreen());
}

/////////////////////////////////////////////////
TEST(DOMWorld, SetScene)
{
  sdf::World world;
  EXPECT_EQ(nullptr, world.Scene());

  sdf::Scene scene;
  scene.SetAmbient(ignition::math::Color::Blue);
  scene.SetBackground(ignition::math::Color::Red);
  scene.SetGrid(true);
  scene.SetShadows(true);
  scene.SetOriginVisual(true);
  world.SetScene(scene);

  ASSERT_NE(nullptr, world.Scene());
  EXPECT_EQ(ignition::math::Color::Blue, world.Scene()->Ambient());
  EXPECT_EQ(ignition::math::Color::Red, world.Scene()->Background());
  EXPECT_TRUE(world.Scene()->Grid());
  EXPECT_TRUE(world.Scene()->Shadows());
  EXPECT_TRUE(world.Scene()->OriginVisual());
}
