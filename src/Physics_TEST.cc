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
#include "sdf/Physics.hh"

/////////////////////////////////////////////////
TEST(DOMPhysics, DefaultConstruction)
{
  sdf::Physics physics;
  EXPECT_EQ(nullptr, physics.Element());
  EXPECT_TRUE(physics.Name().empty());

  physics.SetName("test_physics");
  EXPECT_EQ("test_physics", physics.Name());

  EXPECT_FALSE(physics.IsDefault());
  physics.SetDefault(true);
  EXPECT_TRUE(physics.IsDefault());

  EXPECT_EQ("ode", physics.EngineType());
  physics.SetEngineType("bullet");
  EXPECT_EQ("bullet", physics.EngineType());

  EXPECT_DOUBLE_EQ(0.001, physics.MaxStepSize());
  physics.SetMaxStepSize(1.234);
  EXPECT_DOUBLE_EQ(1.234, physics.MaxStepSize());

  EXPECT_DOUBLE_EQ(1.0, physics.RealTimeFactor());
  physics.SetRealTimeFactor(2.45);
  EXPECT_DOUBLE_EQ(2.45, physics.RealTimeFactor());
}
