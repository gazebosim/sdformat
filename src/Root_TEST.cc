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
#include "sdf/Root.hh"
#include "test_config.h"

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
TEST(DOMRoot, WorldName)
{
  const auto path =
    sdf::testing::TestFile("sdf", "box_plane_low_friction_test.world");
  sdf::Root root;
  std::string worldName;
  auto errors = root.WorldName(path, worldName);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ("default", worldName);

  const auto path2 = sdf::testing::TestFile("sdf", "empty_invalid.sdf");
  errors = root.WorldName(path2, worldName);
  EXPECT_FALSE(errors.empty());

  const auto path3 = sdf::testing::TestFile("sdf", "invalid_file_name.sdf");
  errors = root.WorldName(path3, worldName);
  EXPECT_FALSE(errors.empty());
}
