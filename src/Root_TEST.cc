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
#include "sdf/Actor.hh"
#include "sdf/sdf_config.h"
#include "sdf/Collision.hh"
#include "sdf/Error.hh"
#include "sdf/Link.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/World.hh"
#include "sdf/Frame.hh"
#include "sdf/Root.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Types.hh"
#include "test_config.hh"

/////////////////////////////////////////////////
TEST(DOMRoot, Construction)
{
  sdf::Root root;
  EXPECT_EQ(nullptr, root.Element());
  EXPECT_EQ(SDF_VERSION, root.Version());
  EXPECT_FALSE(root.WorldNameExists("default"));
  EXPECT_TRUE(root.WorldByName("default") == nullptr);
  EXPECT_FALSE(root.WorldNameExists(""));
  EXPECT_EQ(0u, root.WorldCount());
  EXPECT_TRUE(root.WorldByIndex(0) == nullptr);
  EXPECT_TRUE(root.WorldByIndex(1) == nullptr);

  EXPECT_EQ(nullptr, root.Model());
  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Actor());
}

/////////////////////////////////////////////////
TEST(DOMRoot, MoveConstructor)
{
  sdf::Root root;
  root.SetVersion("test_root");

  sdf::Root root2(std::move(root));
  EXPECT_EQ("test_root", root2.Version());
}

/////////////////////////////////////////////////
TEST(DOMRoot, MoveAssignmentOperator)
{
  sdf::Root root;
  root.SetVersion("test_root");

  sdf::Root root2;
  root2 = std::move(root);
  EXPECT_EQ("test_root", root2.Version());
}

/////////////////////////////////////////////////
TEST(DOMRoot, WorldNamesFromFile)
{
  const auto path = sdf::testing::TestFile("sdf", "basic_shapes.sdf");
  sdf::Root root;
  std::vector<std::string> worldNames;
  auto errors = root.WorldNamesFromFile(path, worldNames);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(1u, worldNames.size());
  EXPECT_EQ("shapes_world", worldNames[0]);

  worldNames.clear();
  const auto path2 = sdf::testing::TestFile("sdf", "empty_invalid.sdf");
  errors = root.WorldNamesFromFile(path2, worldNames);
  EXPECT_EQ(0u, worldNames.size());
  EXPECT_FALSE(errors.empty());

  worldNames.clear();
  const auto path3 = sdf::testing::TestFile("sdf", "invalid_file_name.sdf");
  errors = root.WorldNamesFromFile(path3, worldNames);
  EXPECT_EQ(0u, worldNames.size());
  EXPECT_FALSE(errors.empty());
}

/////////////////////////////////////////////////
TEST(DOMRoot, StringModelSdfParse)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
    " <sdf version=\"1.8\">"
    "   <model name='shapes'>"
    "     <link name='link'>"
    "       <collision name='box_col'>"
    "         <geometry>"
    "           <box>"
    "             <size>3 4 5</size>"
    "           </box>"
    "         </geometry>"
    "       </collision>"
    "     </link>"
    "   </model>"
    " </sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_NE(nullptr, model->Element());

  EXPECT_EQ("shapes", model->Name());
  EXPECT_EQ(1u, model->LinkCount());

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  EXPECT_NE(nullptr, link->Element());
  EXPECT_EQ("link", link->Name());
  EXPECT_EQ(1u, link->CollisionCount());

  const sdf::Collision *collision = link->CollisionByIndex(0);
  ASSERT_NE(nullptr, collision);
  EXPECT_NE(nullptr, collision->Element());
  EXPECT_EQ("box_col", collision->Name());

  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(0u, root.WorldCount());

  // Test cloning
  sdf::Root root2 = root.Clone();

  const sdf::Model *model2 = root2.Model();
  ASSERT_NE(nullptr, model2);
  EXPECT_NE(nullptr, model2->Element());

  EXPECT_EQ("shapes", model2->Name());
  EXPECT_EQ(1u, model2->LinkCount());

  const sdf::Link *link2 = model2->LinkByIndex(0);
  ASSERT_NE(nullptr, link2);
  EXPECT_NE(nullptr, link2->Element());
  EXPECT_EQ("link", link2->Name());
  EXPECT_EQ(1u, link2->CollisionCount());

  const sdf::Collision *collision2 = link2->CollisionByIndex(0);
  ASSERT_NE(nullptr, collision2);
  EXPECT_NE(nullptr, collision2->Element());
  EXPECT_EQ("box_col", collision2->Name());

  EXPECT_EQ(nullptr, root2.Light());
  EXPECT_EQ(nullptr, root2.Actor());
  EXPECT_EQ(0u, root2.WorldCount());
}

/////////////////////////////////////////////////
TEST(DOMRoot, StringLightSdfParse)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
    " <sdf version=\"1.8\">"
    "   <light type='directional' name='sun'>"
    "     <direction>-0.5 0.1 -0.9</direction>"
    "   </light>"
    " </sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdf);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Light *light = root.Light();
  ASSERT_NE(nullptr, light);
  EXPECT_EQ("sun", light->Name());
  EXPECT_NE(nullptr, light->Element());

  EXPECT_EQ(nullptr, root.Model());
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(0u, root.WorldCount());
}

/////////////////////////////////////////////////
TEST(DOMRoot, StringActorSdfParse)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
    " <sdf version=\"1.8\">"
    "   <actor name='actor_test'>"
    "     <pose>0 0 1.0 0 0 0</pose>"
    "     <skin>"
    "       <filename>/fake/path/to/mesh.dae</filename>"
    "       <scale>1.0</scale>"
    "     </skin>"
    "     <animation name='run'>"
    "       <filename>/fake/path/to/mesh.dae</filename>"
    "       <scale>0.055</scale>"
    "       <interpolate_x>true</interpolate_x>"
    "     </animation>"
    "     <script>"
    "       <loop>true</loop>"
    "       <delay_start>5.0</delay_start>"
    "       <auto_start>true</auto_start>"
    "     </script>"
    "   </actor>"
    " </sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdf);
  EXPECT_TRUE(errors.empty());

  const sdf::Actor *actor = root.Actor();
  ASSERT_NE(nullptr, actor);
  EXPECT_EQ("actor_test", actor->Name());
  EXPECT_NE(nullptr, actor->Element());

  EXPECT_EQ(nullptr, root.Model());
  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(0u, root.WorldCount());
}

/////////////////////////////////////////////////
TEST(DOMRoot, Set)
{
  sdf::Root root;
  EXPECT_STREQ(SDF_VERSION, root.Version().c_str());
  root.SetVersion(SDF_PROTOCOL_VERSION);
  EXPECT_STREQ(SDF_PROTOCOL_VERSION, root.Version().c_str());
}

/////////////////////////////////////////////////
TEST(DOMRoot, FrameSemanticsOnMove)
{
  const std::string sdfString1 = R"(
    <sdf version="1.8">
      <world name="default">
        <frame name="frame1">
          <pose>0 1 0 0 0 0</pose>
        </frame>
      </world>
    </sdf>)";

  const std::string sdfString2 = R"(
    <sdf version="1.8">
      <world name="default">
        <frame name="frame2">
          <pose>1 1 0 0 0 0</pose>
        </frame>
      </world>
    </sdf>)";

  auto testFrame1 = [](const sdf::Root &_root)
  {
    auto *world = _root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    auto *frame = world->FrameByIndex(0);
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("frame1", frame->Name());
    gz::math::Pose3d pose;
    sdf::Errors errors = frame->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(gz::math::Pose3d(0, 1, 0, 0, 0, 0), pose);
  };

  auto testFrame2 = [](const sdf::Root &_root)
  {
    auto *world = _root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    auto *frame = world->FrameByIndex(0);
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("frame2", frame->Name());
    gz::math::Pose3d pose;
    sdf::Errors errors = frame->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(gz::math::Pose3d(1, 1, 0, 0, 0, 0), pose);
  };

  {
    sdf::Root root1;
    sdf::Errors errors = root1.LoadSdfString(sdfString1);
    EXPECT_TRUE(errors.empty()) << errors;
    testFrame1(root1);
  }

  {
    sdf::Root root2;
    sdf::Errors errors = root2.LoadSdfString(sdfString2);
    EXPECT_TRUE(errors.empty()) << errors;
    testFrame2(root2);
  }

  {
    sdf::Root root1;
    sdf::Errors errors = root1.LoadSdfString(sdfString1);
    EXPECT_TRUE(errors.empty()) << errors;

    // then root1 is moved into root2 via the move constructor
    sdf::Root root2(std::move(root1));
    testFrame1(root2);
  }

  {
    sdf::Root root1;
    sdf::Errors errors = root1.LoadSdfString(sdfString1);
    EXPECT_TRUE(errors.empty()) << errors;
    sdf::Root root2;
    errors = root2.LoadSdfString(sdfString2);
    EXPECT_TRUE(errors.empty()) << errors;

    testFrame1(root1);
    testFrame2(root2);

    // root1 is moved into root2 via the move assignment operator.
    root2 = std::move(root1);
    testFrame1(root2);
  }
}

/////////////////////////////////////////////////
TEST(DOMRoot, ResolveAutoInertialsWithSaveCalculationConfiguration)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
  " <sdf version=\"1.11\">"
  "   <model name='shapes'>"
  "     <link name='link'>"
  "       <inertial auto='true' />"
  "       <collision name='box_col'>"
  "         <density>1240.0</density>"
  "         <geometry>"
  "           <box>"
  "             <size>2 2 2</size>"
  "           </box>"
  "         </geometry>"
  "       </collision>"
  "     </link>"
  "   </model>"
  " </sdf>";

  sdf::Root root;
  sdf::ParserConfig sdfParserConfig;
  sdf::Errors errors = root.LoadSdfString(sdf, sdfParserConfig);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.Model();
  const sdf::Link *link = model->LinkByIndex(0);

  sdfParserConfig.SetCalculateInertialConfiguration(
    sdf::ConfigureResolveAutoInertials::SAVE_CALCULATION);

  sdf::Errors inertialErr;
  root.ResolveAutoInertials(inertialErr, sdfParserConfig);
  EXPECT_TRUE(inertialErr.empty());
  ASSERT_TRUE(link->AutoInertiaSaved());
}

/////////////////////////////////////////////////
TEST(DOMRoot, AddWorld)
{
  sdf::Root root;
  EXPECT_EQ(0u, root.WorldCount());

  sdf::World world;
  world.SetName("world1");
  sdf::Errors errors = root.AddWorld(world);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(1u, root.WorldCount());
  ASSERT_FALSE(root.AddWorld(world).empty());
  EXPECT_EQ(sdf::ErrorCode::DUPLICATE_NAME, root.AddWorld(world)[0].Code());
  EXPECT_EQ(1u, root.WorldCount());

  root.ClearWorlds();
  EXPECT_EQ(0u, root.WorldCount());

  EXPECT_TRUE(root.AddWorld(world).empty());
  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *worldFromRoot = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldFromRoot);
  EXPECT_EQ(worldFromRoot->Name(), world.Name());
}

/////////////////////////////////////////////////
TEST(DOMRoot, MutableByIndex)
{
  sdf::Root root;
  EXPECT_EQ(0u, root.WorldCount());

  sdf::World world;
  world.SetName("world1");
  EXPECT_TRUE(root.AddWorld(world).empty());
  EXPECT_EQ(1u, root.WorldCount());

  // Modify the world
  sdf::World *w = root.WorldByIndex(0);
  ASSERT_NE(nullptr, w);
  EXPECT_EQ("world1", w->Name());
  w->SetName("world2");
  EXPECT_EQ("world2", root.WorldByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST(DOMRoot, ToElementEmpty)
{
  sdf::Root root;

  sdf::ElementPtr elem = root.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Root root2;
  root2.LoadSdfString(elem->ToString(""));

  EXPECT_EQ(SDF_VERSION, root2.Version());
}

/////////////////////////////////////////////////
TEST(DOMRoot, ToElementModel)
{
  sdf::Root root;

  sdf::Actor actor1;
  actor1.SetName("actor1");
  root.SetActor(actor1);

  sdf::Light light1;
  light1.SetName("light1");
  root.SetLight(light1);

  sdf::Model model1;
  model1.SetName("model1");
  root.SetModel(model1);

  ASSERT_NE(nullptr, root.Model());
  ASSERT_EQ(nullptr, root.Light());
  ASSERT_EQ(nullptr, root.Actor());
  EXPECT_EQ(0u, root.WorldCount());

  // Convert to sdf::ElementPtr
  sdf::ElementPtr elem = root.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Root root2;
  root2.LoadSdfString(elem->ToString(""));

  EXPECT_EQ(SDF_VERSION, root2.Version());

  ASSERT_NE(nullptr, root2.Model());
  EXPECT_EQ("model1", root2.Model()->Name());

  ASSERT_EQ(nullptr, root2.Actor());
  ASSERT_EQ(nullptr, root2.Light());
  EXPECT_EQ(0u, root2.WorldCount());
}

/////////////////////////////////////////////////
TEST(DOMRoot, ToElementLight)
{
  sdf::Root root;

  sdf::Model model1;
  model1.SetName("model1");
  root.SetModel(model1);

  sdf::Actor actor1;
  actor1.SetName("actor1");
  root.SetActor(actor1);

  sdf::Light light1;
  light1.SetName("light1");
  root.SetLight(light1);

  ASSERT_NE(nullptr, root.Light());
  ASSERT_EQ(nullptr, root.Model());
  ASSERT_EQ(nullptr, root.Actor());
  EXPECT_EQ(0u, root.WorldCount());

  // Convert to sdf::ElementPtr
  sdf::ElementPtr elem = root.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Root root2;
  root2.LoadSdfString(elem->ToString(""));

  EXPECT_EQ(SDF_VERSION, root2.Version());

  ASSERT_NE(nullptr, root2.Light());
  EXPECT_EQ("light1", root2.Light()->Name());

  ASSERT_EQ(nullptr, root2.Model());
  ASSERT_EQ(nullptr, root2.Actor());
  EXPECT_EQ(0u, root2.WorldCount());
}

/////////////////////////////////////////////////
TEST(DOMRoot, ToElementActor)
{
  sdf::Root root;

  sdf::Model model1;
  model1.SetName("model1");
  root.SetModel(model1);

  sdf::Light light1;
  light1.SetName("light1");
  root.SetLight(light1);

  sdf::Actor actor1;
  actor1.SetName("actor1");
  root.SetActor(actor1);

  ASSERT_NE(nullptr, root.Actor());
  ASSERT_EQ(nullptr, root.Light());
  ASSERT_EQ(nullptr, root.Model());
  EXPECT_EQ(0u, root.WorldCount());

  // Convert to sdf::ElementPtr
  sdf::ElementPtr elem = root.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Root root2;
  root2.LoadSdfString(elem->ToString(""));

  EXPECT_EQ(SDF_VERSION, root2.Version());

  ASSERT_NE(nullptr, root2.Actor());
  EXPECT_EQ("actor1", root2.Actor()->Name());

  ASSERT_EQ(nullptr, root2.Model());
  ASSERT_EQ(nullptr, root2.Light());
  EXPECT_EQ(0u, root2.WorldCount());
}

/////////////////////////////////////////////////
TEST(DOMRoot, ToElementWorld)
{
  sdf::Root root;

  sdf::World world1;
  world1.SetName("world1");
  root.AddWorld(world1);

  sdf::World world2;
  world2.SetName("world2");
  root.AddWorld(world2);

  EXPECT_EQ(2u, root.WorldCount());

  // Convert to sdf::ElementPtr
  sdf::ElementPtr elem = root.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Root root2;
  root2.LoadSdfString(elem->ToString(""));

  EXPECT_EQ(SDF_VERSION, root2.Version());
  EXPECT_EQ(2u, root2.WorldCount());

  ASSERT_NE(nullptr, root2.WorldByIndex(0));
  EXPECT_EQ("world1", root2.WorldByIndex(0)->Name());

  ASSERT_NE(nullptr, root2.WorldByIndex(1));
  EXPECT_EQ("world2", root2.WorldByIndex(1)->Name());
}

/////////////////////////////////////////////////
TEST(DOMRoot, CopyConstructor)
{
  const std::string sdfString = R"(
    <sdf version="1.10">
      <world name="default">
        <frame name="frame1">
          <pose>0 1 0 0 0 0</pose>
        </frame>
      </world>
    </sdf>)";

  auto testFrame1 = [](const sdf::Root &_root)
  {
    EXPECT_EQ("1.11", _root.Version());

    const sdf::World *world = _root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    EXPECT_EQ("default", world->Name());
    const sdf::Frame *frame = world->FrameByIndex(0);
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("frame1", frame->Name());
    gz::math::Pose3d pose;
    sdf::Errors errors = frame->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(gz::math::Pose3d(0, 1, 0, 0, 0, 0), pose);
  };

  sdf::Root root;
  root.LoadSdfString(sdfString);
  testFrame1(root);

  // Copy constructor test
  {
    sdf::Root root2(root);
    testFrame1(root2);

    // Test conversion to element after copy constructor
    sdf::ElementPtr elem = root2.ToElement();
    ASSERT_NE(nullptr, elem);
    sdf::Root root3;
    root3.LoadSdfString(elem->ToString(""));
    testFrame1(root3);
  }

  // Copy assignment test
  {
    sdf::Root root2;
    root2 = root;
    testFrame1(root2);

    // Test conversion to element after copy constructor
    sdf::ElementPtr elem = root2.ToElement();
    ASSERT_NE(nullptr, elem);
    sdf::Root root3;
    root3.LoadSdfString(elem->ToString(""));
    testFrame1(root3);
  }

  // Copy assignment after move test
  {
    sdf::Root root2;

    // This is similar to what std::swap does except it uses std::move for each
    // assignment
    sdf::Root tmp = std::move(root);
    root = root2;
    root2 = tmp;

    // Test conversion to element after copy constructor
    sdf::ElementPtr elem = root2.ToElement();
    ASSERT_NE(nullptr, elem);
    sdf::Root root3;
    root3.LoadSdfString(elem->ToString(""));
    testFrame1(root3);
  }
}

/////////////////////////////////////////////////
TEST(DOMRoot, WorldByName)
{
  sdf::Root root;
  EXPECT_EQ(0u, root.WorldCount());

  sdf::World world;
  world.SetName("world1");
  EXPECT_TRUE(root.AddWorld(world).empty());
  EXPECT_EQ(1u, root.WorldCount());

  EXPECT_TRUE(root.WorldNameExists("world1"));
  const sdf::World *wPtr = root.WorldByName("world1");
  EXPECT_NE(nullptr, wPtr);

  // Modify the world
  sdf::World *w = root.WorldByName("world1");
  ASSERT_NE(nullptr, w);
  EXPECT_EQ("world1", w->Name());
  w->SetName("world2");
  ASSERT_TRUE(root.WorldNameExists("world2"));
  EXPECT_EQ("world2", root.WorldByName("world2")->Name());
}

/////////////////////////////////////////////////
TEST(DOMRoot, ClearActorLightModel)
{
  sdf::Root root;
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Model());

  sdf::Actor actor1;
  actor1.SetName("actor1");
  root.SetActor(actor1);
  EXPECT_NE(nullptr, root.Actor());
  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Model());
  root.ClearActorLightModel();
  EXPECT_EQ(nullptr, root.Actor());

  sdf::Light light1;
  light1.SetName("light1");
  root.SetLight(light1);
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_NE(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Model());
  root.ClearActorLightModel();
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Model());

  sdf::Model model1;
  model1.SetName("model1");
  root.SetModel(model1);
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(nullptr, root.Light());
  EXPECT_NE(nullptr, root.Model());
  root.ClearActorLightModel();
  EXPECT_EQ(nullptr, root.Model());
}
