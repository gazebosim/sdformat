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
#include "sdf/Frame.hh"
#include "sdf/Light.hh"
#include "sdf/Actor.hh"
#include "sdf/Model.hh"
#include "sdf/Physics.hh"
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

  auto errors = world.ValidateGraphs();
  EXPECT_EQ(2u, errors.size()) << errors;
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_GRAPH_ERROR);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "FrameAttachedToGraph error: scope does not point to a valid graph"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_GRAPH_ERROR);
  EXPECT_NE(std::string::npos,
    errors[1].Message().find(
      "PoseRelativeToGraph error: scope does not point to a valid graph"));
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
  world.SetSphericalCoordinates(ignition::math::SphericalCoordinates());

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
  ASSERT_TRUE(nullptr != world.SphericalCoordinates());
  EXPECT_EQ(ignition::math::SphericalCoordinates::EARTH_WGS84,
      world.SphericalCoordinates()->Surface());
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

/////////////////////////////////////////////////
TEST(DOMWorld, AddModel)
{
  sdf::World world;
  EXPECT_EQ(0u, world.ModelCount());

  sdf::Model model;
  model.SetName("model1");
  EXPECT_TRUE(world.AddModel(model));
  EXPECT_EQ(1u, world.ModelCount());
  EXPECT_FALSE(world.AddModel(model));
  EXPECT_EQ(1u, world.ModelCount());

  world.ClearModels();
  EXPECT_EQ(0u, world.ModelCount());

  EXPECT_TRUE(world.AddModel(model));
  EXPECT_EQ(1u, world.ModelCount());
  const sdf::Model *modelFromWorld = world.ModelByIndex(0);
  ASSERT_NE(nullptr, modelFromWorld);
  EXPECT_EQ(modelFromWorld->Name(), model.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, AddModifyFrame)
{
  sdf::World world;
  EXPECT_EQ(0u, world.FrameCount());

  sdf::Frame frame;
  frame.SetName("frame1");
  EXPECT_TRUE(world.AddFrame(frame));
  EXPECT_EQ(1u, world.FrameCount());
  EXPECT_FALSE(world.AddFrame(frame));
  EXPECT_EQ(1u, world.FrameCount());

  world.ClearFrames();
  EXPECT_EQ(0u, world.FrameCount());

  EXPECT_TRUE(world.AddFrame(frame));
  EXPECT_EQ(1u, world.FrameCount());
  const sdf::Frame *frameFromWorld = world.FrameByIndex(0);
  ASSERT_NE(nullptr, frameFromWorld);
  EXPECT_EQ(frameFromWorld->Name(), frame.Name());

  sdf::Frame *mutableFrame = world.FrameByIndex(0);
  mutableFrame->SetName("newName1");
  EXPECT_EQ(mutableFrame->Name(), world.FrameByIndex(0)->Name());

  sdf::Frame *mutableFrameByName = world.FrameByName("frame1");
  EXPECT_EQ(nullptr, mutableFrameByName);
  mutableFrameByName = world.FrameByName("newName1");
  ASSERT_NE(nullptr, mutableFrameByName);
  EXPECT_EQ("newName1", world.FrameByName("newName1")->Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, AddActor)
{
  sdf::World world;
  EXPECT_EQ(0u, world.ActorCount());

  sdf::Actor actor;
  actor.SetName("actor1");
  EXPECT_TRUE(world.AddActor(actor));
  EXPECT_EQ(1u, world.ActorCount());
  EXPECT_FALSE(world.AddActor(actor));
  EXPECT_EQ(1u, world.ActorCount());

  world.ClearActors();
  EXPECT_EQ(0u, world.ActorCount());

  EXPECT_TRUE(world.AddActor(actor));
  EXPECT_EQ(1u, world.ActorCount());
  const sdf::Actor *actorFromWorld = world.ActorByIndex(0);
  ASSERT_NE(nullptr, actorFromWorld);
  EXPECT_EQ(actorFromWorld->Name(), actor.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, AddLight)
{
  sdf::World world;
  EXPECT_EQ(0u, world.LightCount());

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(world.AddLight(light));
  EXPECT_EQ(1u, world.LightCount());
  EXPECT_FALSE(world.AddLight(light));
  EXPECT_EQ(1u, world.LightCount());

  world.ClearLights();
  EXPECT_EQ(0u, world.LightCount());

  EXPECT_TRUE(world.AddLight(light));
  EXPECT_EQ(1u, world.LightCount());
  const sdf::Light *lightFromWorld = world.LightByIndex(0);
  ASSERT_NE(nullptr, lightFromWorld);
  EXPECT_EQ(lightFromWorld->Name(), light.Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, ToElement)
{
  sdf::World world;

  world.SetName("my-world");
  world.SetAudioDevice("my-audio");
  world.SetWindLinearVelocity(ignition::math::Vector3d(1, 2, 3));
  world.SetGravity(ignition::math::Vector3d(-1, 5, 10));
  world.SetMagneticField(ignition::math::Vector3d(2.0, 0.1, 0.5));
  world.SetSphericalCoordinates(ignition::math::SphericalCoordinates());

  sdf::Atmosphere atmosphere;
  world.SetAtmosphere(atmosphere);

  sdf::Gui gui;
  world.SetGui(gui);

  sdf::Scene scene;
  world.SetScene(scene);

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 2; ++i)
    {
      sdf::Model model;
      model.SetName("model" + std::to_string(i));
      EXPECT_TRUE(world.AddModel(model));
      EXPECT_FALSE(world.AddModel(model));
    }
    if (j == 0)
      world.ClearModels();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      sdf::Actor actor;
      actor.SetName("actor" + std::to_string(i));
      EXPECT_TRUE(world.AddActor(actor));
      EXPECT_FALSE(world.AddActor(actor));
    }
    if (j == 0)
      world.ClearActors();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 4; ++i)
    {
      sdf::Light light;
      light.SetName("light" + std::to_string(i));
      EXPECT_TRUE(world.AddLight(light));
      EXPECT_FALSE(world.AddLight(light));
    }
    if (j == 0)
      world.ClearLights();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 5; ++i)
    {
      sdf::Physics physics;
      physics.SetName("physics" + std::to_string(i));
      EXPECT_TRUE(world.AddPhysics(physics));
      EXPECT_FALSE(world.AddPhysics(physics));
    }
    if (j == 0)
      world.ClearPhysics();
  }

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");
  world.AddPlugin(plugin);

  sdf::ElementPtr elem = world.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::World world2;
  world2.Load(elem);

  EXPECT_EQ(world.Name(), world2.Name());
  EXPECT_EQ(world.AudioDevice(), world2.AudioDevice());
  EXPECT_EQ(world.WindLinearVelocity(), world2.WindLinearVelocity());
  EXPECT_EQ(world.Gravity(), world2.Gravity());
  EXPECT_EQ(world.MagneticField(), world2.MagneticField());
  EXPECT_EQ(*world.SphericalCoordinates(), *world2.SphericalCoordinates());

  const sdf::Atmosphere *atmosphere2 = world2.Atmosphere();
  ASSERT_NE(nullptr, atmosphere2);

  const sdf::Gui *gui2 = world2.Gui();
  ASSERT_NE(nullptr, gui2);

  const sdf::Scene *scene2 = world2.Scene();
  ASSERT_NE(nullptr, scene2);

  EXPECT_EQ(world.ModelCount(), world2.ModelCount());
  for (uint64_t i = 0; i < world2.ModelCount(); ++i)
    EXPECT_NE(nullptr, world2.ModelByIndex(i));

  EXPECT_EQ(world.LightCount(), world2.LightCount());
  for (uint64_t i = 0; i < world2.LightCount(); ++i)
    EXPECT_NE(nullptr, world2.LightByIndex(i));

  EXPECT_EQ(world.ActorCount(), world2.ActorCount());
  for (uint64_t i = 0; i < world2.ActorCount(); ++i)
    EXPECT_NE(nullptr, world2.ActorByIndex(i));

  EXPECT_EQ(world.PhysicsCount(), world2.PhysicsCount());
  for (uint64_t i = 0; i < world2.PhysicsCount(); ++i)
    EXPECT_NE(nullptr, world2.PhysicsByIndex(i));

  ASSERT_EQ(1u, world2.Plugins().size());
  EXPECT_EQ("name1", world2.Plugins()[0].Name());
  EXPECT_EQ("filename1", world2.Plugins()[0].Filename());
}

/////////////////////////////////////////////////
TEST(DOMWorld, MutableByIndex)
{
  sdf::World world;

  sdf::Model model;
  model.SetName("model1");
  EXPECT_TRUE(world.AddModel(model));

  sdf::Actor actor;
  actor.SetName("actor1");
  EXPECT_TRUE(world.AddActor(actor));

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(world.AddLight(light));

  sdf::Physics physics;
  physics.SetName("physics1");
  EXPECT_TRUE(world.AddPhysics(physics));

  sdf::Frame frame;
  frame.SetName("frame1");
  EXPECT_TRUE(world.AddFrame(frame));

  // Modify the model
  sdf::Model *m = world.ModelByIndex(0);
  ASSERT_NE(nullptr, m);
  EXPECT_EQ("model1", m->Name());
  m->SetName("model2");
  EXPECT_EQ("model2", world.ModelByIndex(0)->Name());

  // Modify the actor
  sdf::Actor *a = world.ActorByIndex(0);
  ASSERT_NE(nullptr, a);
  EXPECT_EQ("actor1", a->Name());
  a->SetName("actor2");
  EXPECT_EQ("actor2", world.ActorByIndex(0)->Name());

  // Modify the light
  sdf::Light *l = world.LightByIndex(0);
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("light1", l->Name());
  l->SetName("light2");
  EXPECT_EQ("light2", world.LightByIndex(0)->Name());

  // Modify the physics
  sdf::Physics *p = world.PhysicsByIndex(1);
  ASSERT_NE(nullptr, p);
  EXPECT_EQ("physics1", p->Name());
  p->SetName("physics2");
  EXPECT_EQ("physics2", world.PhysicsByIndex(1)->Name());

  // Modify the frame
  sdf::Frame *f = world.FrameByIndex(0);
  ASSERT_NE(nullptr, f);
  EXPECT_EQ("frame1", f->Name());
  f->SetName("frame2");
  EXPECT_EQ("frame2", world.FrameByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST(DOMWorld, MutableByName)
{
  sdf::World world;

  sdf::Model model;
  model.SetName("model1");
  EXPECT_TRUE(world.AddModel(model));

  sdf::Frame frame;
  frame.SetName("frame1");
  EXPECT_TRUE(world.AddFrame(frame));

  // Modify the model
  sdf::Model *m = world.ModelByName("model1");
  ASSERT_NE(nullptr, m);
  EXPECT_EQ("model1", m->Name());
  m->SetName("model2");
  EXPECT_FALSE(world.ModelByName("model1"));
  EXPECT_TRUE(world.ModelByName("model2"));

  // Modify the frame
  sdf::Frame *f = world.FrameByName("frame1");
  ASSERT_NE(nullptr, f);
  EXPECT_EQ("frame1", f->Name());
  f->SetName("frame2");
  EXPECT_FALSE(world.FrameByName("frame1"));
  EXPECT_TRUE(world.FrameByName("frame2"));
}

/////////////////////////////////////////////////
TEST(DOMWorld, Plugins)
{
  sdf::World world;
  EXPECT_TRUE(world.Plugins().empty());

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");

  world.AddPlugin(plugin);
  ASSERT_EQ(1u, world.Plugins().size());

  plugin.SetName("name2");
  world.AddPlugin(plugin);
  ASSERT_EQ(2u, world.Plugins().size());

  EXPECT_EQ("name1", world.Plugins()[0].Name());
  EXPECT_EQ("name2", world.Plugins()[1].Name());

  world.ClearPlugins();
  EXPECT_TRUE(world.Plugins().empty());
}
