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
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Light.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

#include "test_config.hh"

const auto g_testPath = sdf::testing::TestFile();
const auto g_modelsPath = sdf::testing::TestFile("integration", "model");
const auto g_sdfPath = sdf::testing::TestFile("sdf");

/////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::filesystem::append(g_modelsPath, _input);
}

//////////////////////////////////////////////////
// Check that an error is emitted if there are multiple models in an included
// file. Despite the error, the first model should be loaded.
TEST(IncludesTest, NestedMultipleModelsError)
{
  sdf::setFindCallback(findFileCb);

  const auto sdfFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_models_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_EQ("Found more than one model for <include>.", errors[0].Message());

  const auto * model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("nested_model", model->Name());

  ASSERT_EQ(1u, model->LinkCount());
  EXPECT_EQ("link1", model->LinkByIndex(0)->Name());

  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(nullptr, root.Light());
}

//////////////////////////////////////////////////
// Check that an error is emitted if there are multiple actors in an included
// file. Despite the error, the first actor should be loaded.
TEST(IncludesTest, NestedMultipleActorsError)
{
  sdf::setFindCallback(findFileCb);

  const auto sdfFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_actors_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_EQ("Found more than one actor for <include>.", errors[0].Message());

  EXPECT_EQ(nullptr, root.Model());
  EXPECT_EQ(nullptr, root.Light());

  const auto * actor = root.Actor();
  ASSERT_NE(nullptr, actor);
  EXPECT_EQ("nested_actor", actor->Name());
  ASSERT_EQ(1u, actor->LinkCount());
  EXPECT_EQ("link1", actor->LinkByIndex(0)->Name());
}

//////////////////////////////////////////////////
// Check that an error is emitted if there are multiple lights in an included
// file. Despite the error, the first light should be loaded.
TEST(IncludesTest, NestedMultipleLightsError)
{
  sdf::setFindCallback(findFileCb);

  const auto sdfFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_lights_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_EQ("Found more than one light for <include>.", errors[0].Message());

  EXPECT_EQ(nullptr, root.Model());
  EXPECT_EQ(nullptr, root.Actor());

  const auto * light = root.Light();
  ASSERT_NE(nullptr, light);
  EXPECT_EQ("nested_light", light->Name());
  EXPECT_EQ(gz::math::Vector3d(1, 0, 0), light->Direction());
}

//////////////////////////////////////////////////
// Check that an error is emitted if there are more than one of model, actor, or
// light in an included file. Despite the error, the
// the first model, actor, light should be loaded in that order of preference.
TEST(IncludesTest, NestedMultipleElementsError)
{
  sdf::setFindCallback(findFileCb);

  const auto sdfFile =
    sdf::filesystem::append(g_modelsPath, "nested_multiple_elements_error");

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_EQ(
      "Found other top level element <actor> in addition to <model> in include "
      "file.",
      errors[0].Message());

  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Actor());

  const auto * model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("nested_model", model->Name());
}

//////////////////////////////////////////////////
TEST(IncludesTest, NestedMultipleElementsErrorWorld)
{
  sdf::setFindCallback(findFileCb);

  const auto sdfFile =
    sdf::filesystem::append(
      g_sdfPath, "nested_multiple_elements_error_world.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(sdfFile);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());

  EXPECT_EQ(nullptr, root.Light());
  EXPECT_EQ(nullptr, root.Actor());
  EXPECT_EQ(nullptr, root.Model());

  ASSERT_EQ(1u, root.WorldCount());
  const auto * world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  ASSERT_EQ(1u, world->ModelCount());

  const auto model = world->ModelByIndex(0);
  EXPECT_EQ("nested_model", model->Name());
  ASSERT_EQ(1u, model->LinkCount());
  EXPECT_EQ("link", model->LinkByIndex(0)->Name());
}
