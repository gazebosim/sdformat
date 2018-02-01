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

#include <string>
#include <gtest/gtest.h>

#include "sdf/Collision.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMLink, NotALink)
{
  // Create an Element that is not a link
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Link link;
  sdf::Errors errors = link.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Link") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMLink, NoName)
{
  // Create a "link" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("link");

  sdf::Link link;
  sdf::Errors errors = link.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_TRUE(errors[0].Message().find("link name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMLink, LoadVisualCollision)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(world != nullptr);
  EXPECT_EQ("default", world->Name());

  // Get the first model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_TRUE(model != nullptr);
  EXPECT_EQ("ground_plane", model->Name());

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_TRUE(link != nullptr);
  EXPECT_EQ("link", link->Name());

  // Get the first visual
  EXPECT_EQ(1u, link->VisualCount());
  EXPECT_TRUE(link->VisualNameExists("visual"));
  EXPECT_FALSE(link->VisualNameExists("visuals"));
  const sdf::Visual *visual = link->VisualByIndex(0);
  ASSERT_TRUE(visual != nullptr);
  EXPECT_EQ("visual", visual->Name());

  // Get the first collision
  EXPECT_EQ(1u, link->CollisionCount());
  EXPECT_TRUE(link->CollisionNameExists("collision"));
  EXPECT_FALSE(link->CollisionNameExists("collisions"));
  const sdf::Collision *collision = link->CollisionByIndex(0);
  ASSERT_TRUE(collision != nullptr);
  EXPECT_EQ("collision", collision->Name());
}
