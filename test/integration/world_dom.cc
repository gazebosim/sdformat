/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMWorld, NoName)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_noname.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());

  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
}

//////////////////////////////////////////////////
TEST(DOMWorld, Duplicate)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_duplicate.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Code() == sdf::ErrorCode::DUPLICATE_NAME);
}

//////////////////////////////////////////////////
TEST(DOMWorld, LoadIncorrectElement)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_complete.sdf");

  sdf::Errors errors;
  // Read an SDF file, and store the result in sdfParsed.
  sdf::SDFPtr sdfParsed = sdf::readFile(testFile, errors);
  ASSERT_TRUE(errors.empty());

  sdf::World world;
  errors = world.Load(sdfParsed->Root());
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a World") !=
      std::string::npos);
}


//////////////////////////////////////////////////
TEST(DOMWorld, Load)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_complete.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ(root.Version(), "1.6");
  EXPECT_EQ(root.WorldCount(), 1u);
  EXPECT_TRUE(root.WorldNameExists("default"));

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(world != nullptr);
  EXPECT_EQ(world->Name(), "default");
  EXPECT_EQ(world->AudioDevice(), "/dev/audio");
  EXPECT_EQ(world->WindLinearVelocity(), ignition::math::Vector3d(4, 5, 6));
  EXPECT_EQ(world->Gravity(), ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(world->MagneticField(), ignition::math::Vector3d(-1, 0.5, 10));
}
