/*
 * Copyright 2020 Open Source Robotics Foundation
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

#include "sdf/Actor.hh"
#include "sdf/Collision.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.h"

const auto g_testPath = sdf::filesystem::append(PROJECT_SOURCE_PATH, "test");
const auto g_modelsPath =
    sdf::filesystem::append(g_testPath, "integration", "model");

/////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::filesystem::append(g_testPath, "integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(IncludesTest, nested_multiple_models_error)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_models_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(
    std::string("Found more than one of model for <include>\n"),
    errors[0].Message());
}

//////////////////////////////////////////////////
TEST(IncludesTest, nested_multiple_actors_error)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_actors_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(
    std::string("Found more than one of actor for <include>\n"),
    errors[0].Message());
}

//////////////////////////////////////////////////
TEST(IncludesTest, nested_multiple_lights_error)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_lights_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(
    std::string("Found more than one of light for <include>\n"),
    errors[0].Message());
}

//////////////////////////////////////////////////
TEST(IncludesTest, nested_multiple_elements_error)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_elements_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(
    std::string(
      "Found more than one of <model> / <actor> / <light> for <include>\n"),
    errors[0].Message());
}
