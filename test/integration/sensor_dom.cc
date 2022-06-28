/*
 * Copyright 2022 Open Source Robotics Foundation
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

#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Sensor.hh"
#include "sdf/World.hh"
#include "test_config.hh"
#include "test_utils.hh"

//////////////////////////////////////////////////
TEST(DOMSensor, SensorPlugins)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_complete.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(testFile, root.Element()->FilePath());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const sdf::Sensor *sensor = link->SensorByIndex(0);
  ASSERT_NE(nullptr, sensor);

  ASSERT_EQ(2u, sensor->Plugins().size());
  EXPECT_EQ("sensor_plugin1", sensor->Plugins()[0].Name());
  EXPECT_EQ("test/file/sensor1", sensor->Plugins()[0].Filename());
  EXPECT_EQ("sensor_plugin2", sensor->Plugins()[1].Name());
  EXPECT_EQ("test/file/sensor2", sensor->Plugins()[1].Filename());
}
