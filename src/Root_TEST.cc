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
#include "sdf/sdf_config.h"
#include "sdf/Collision.hh"
#include "sdf/Error.hh"
#include "sdf/Link.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
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
}

/////////////////////////////////////////////////
TEST(DOMRoot, Set)
{
  sdf::Root root;
  EXPECT_STREQ("", root.Version().c_str());
  root.SetVersion(SDF_PROTOCOL_VERSION);
  EXPECT_STREQ(SDF_PROTOCOL_VERSION, root.Version().c_str());
}
