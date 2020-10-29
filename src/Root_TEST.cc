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

/////////////////////////////////////////////////
TEST(DOMRoot, Construction)
{
  sdf::Root root;
  EXPECT_EQ(nullptr, root.Element());
  EXPECT_EQ("", root.Version());
  EXPECT_FALSE(root.WorldNameExists("default"));
  EXPECT_FALSE(root.WorldNameExists(""));
  EXPECT_EQ(0u, root.WorldCount());
  EXPECT_TRUE(root.WorldByIndex(0) == nullptr);
  EXPECT_TRUE(root.WorldByIndex(1) == nullptr);

  EXPECT_FALSE(root.ModelNameExists("default"));
  EXPECT_FALSE(root.ModelNameExists(""));
  EXPECT_EQ(0u, root.ModelCount());
  EXPECT_TRUE(root.ModelByIndex(0) == nullptr);
  EXPECT_TRUE(root.ModelByIndex(1) == nullptr);

  EXPECT_FALSE(root.LightNameExists("default"));
  EXPECT_FALSE(root.LightNameExists(""));
  EXPECT_EQ(0u, root.LightCount());
  EXPECT_TRUE(root.LightByIndex(0) == nullptr);
  EXPECT_TRUE(root.LightByIndex(1) == nullptr);

  EXPECT_FALSE(root.ActorNameExists("default"));
  EXPECT_FALSE(root.ActorNameExists(""));
  EXPECT_EQ(0u, root.ActorCount());
  EXPECT_TRUE(root.ActorByIndex(0) == nullptr);
  EXPECT_TRUE(root.ActorByIndex(1) == nullptr);
}

/////////////////////////////////////////////////
TEST(DOMRoot, CopyConstructor)
{
  sdf::Root root;
  root.SetVersion("test_root");

  sdf::Root root2(root);
  EXPECT_EQ("test_root", root2.Version());
}

/////////////////////////////////////////////////
TEST(DOMRoot, CopyAssignmentOperator)
{
  sdf::Root root;
  root.SetVersion("test_root");

  sdf::Root root2;
  root2 = root;
  EXPECT_EQ("test_root", root2.Version());
}

/////////////////////////////////////////////////
TEST(DOMRoot, MoveConstructor)
{
  sdf::Root root;
  root.SetVersion("test_root");

  sdf::Root root2(root);
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
TEST(DOMRoot, CopyAssignmentAfterMove)
{
  sdf::Root root1;
  root1.SetVersion("root1");

  sdf::Root root2;
  root2.SetVersion("root2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Root tmp = std::move(root1);
  root1 = root2;
  root2 = tmp;

  EXPECT_EQ("root2", root1.Version());
  EXPECT_EQ("root1", root2.Version());
}

/////////////////////////////////////////////////
TEST(DOMRoot, StringParse)
{
  std::string sdf = "<?xml version=\"1.0\"?>"
    " <sdf version=\"1.6\">"
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
    "   <light type='directional' name='sun'>"
    "     <direction>-0.5 0.1 -0.9</direction>"
    "   </light>"
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
  EXPECT_EQ(1u, root.ModelCount());
  EXPECT_EQ(1u, root.LightCount());
  EXPECT_NE(nullptr, root.Element());

  const sdf::Model *model = root.ModelByIndex(0);
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

  EXPECT_TRUE(root.LightNameExists("sun"));
  EXPECT_EQ(1u, root.LightCount());
  const sdf::Light *light = root.LightByIndex(0);
  ASSERT_NE(nullptr, light);
  EXPECT_NE(nullptr, light->Element());

  EXPECT_TRUE(root.ActorNameExists("actor_test"));
  EXPECT_EQ(1u, root.ActorCount());
  const sdf::Actor *actor = root.ActorByIndex(0);
  ASSERT_NE(nullptr, actor);
  EXPECT_NE(nullptr, actor->Element());
}

/////////////////////////////////////////////////
TEST(DOMRoot, Set)
{
  sdf::Root root;
  EXPECT_STREQ("", root.Version().c_str());
  root.SetVersion(SDF_PROTOCOL_VERSION);
  EXPECT_STREQ(SDF_PROTOCOL_VERSION, root.Version().c_str());
}

/////////////////////////////////////////////////
TEST(DOMRoot, FrameSemanticsOnCopyAndMove)
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
    ignition::math::Pose3d pose;
    sdf::Errors errors = frame->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(ignition::math::Pose3d(0, 1, 0, 0, 0, 0), pose);
  };

  auto testFrame2 = [](const sdf::Root &_root)
  {
    auto *world = _root.WorldByIndex(0);
    ASSERT_NE(nullptr, world);
    auto *frame = world->FrameByIndex(0);
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("frame2", frame->Name());
    ignition::math::Pose3d pose;
    sdf::Errors errors = frame->SemanticPose().Resolve(pose);
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_EQ(ignition::math::Pose3d(1, 1, 0, 0, 0, 0), pose);
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
    // root2 is copy constructed from root1
    sdf::Root root2(root1);
    testFrame1(root1);
    testFrame1(root2);
  }

  {
    sdf::Root root1;
    sdf::Errors errors = root1.LoadSdfString(sdfString1);
    EXPECT_TRUE(errors.empty()) << errors;
    sdf::Root root2;
    errors = root2.LoadSdfString(sdfString2);
    EXPECT_TRUE(errors.empty()) << errors;

    // root1 is copied into root2 via the assignment operator
    root2 = root1;
    testFrame1(root1);
    testFrame1(root2);
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

    // root1 is moved into root2 via the move assignment operator.
    root2 = std::move(root1);
    testFrame1(root2);
  }
}
