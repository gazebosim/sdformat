/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "sdf/Collision.hh"
#include "sdf/Link.hh"
#include "sdf/Visual.hh"

/////////////////////////////////////////////////
TEST(DOMLink, Construction)
{
  sdf::Link link;
  EXPECT_TRUE(link.Name().empty());

  link.SetName("test_link");
  EXPECT_EQ("test_link", link.Name());

  EXPECT_EQ(0u, link.VisualCount());
  EXPECT_EQ(nullptr, link.VisualByIndex(0));
  EXPECT_EQ(nullptr, link.VisualByIndex(1));
  EXPECT_FALSE(link.VisualNameExists(""));
  EXPECT_FALSE(link.VisualNameExists("default"));

  EXPECT_EQ(0u, link.CollisionCount());
  EXPECT_EQ(nullptr, link.CollisionByIndex(0));
  EXPECT_EQ(nullptr, link.CollisionByIndex(1));
  EXPECT_FALSE(link.CollisionNameExists(""));
  EXPECT_FALSE(link.CollisionNameExists("default"));
}
