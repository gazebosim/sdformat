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

/////////////////////////////////////////////////
TEST(DOMPhysics, CopyConstructor)
{
  sdf::Physics physics;
  physics.SetName("test_physics");

  sdf::Physics physics2(physics);
  EXPECT_EQ("test_physics", physics2.Name());
}

/////////////////////////////////////////////////
TEST(DOMPhysics, CopyAssignmentOperator)
{
  sdf::Physics physics;
  physics.SetName("test_physics");

  sdf::Physics physics2;
  physics2 = physics;
  EXPECT_EQ("test_physics", physics2.Name());
}

/////////////////////////////////////////////////
TEST(DOMPhysics, MoveConstructor)
{
  sdf::Physics physics;
  physics.SetName("test_physics");

  sdf::Physics physics2(physics);
  EXPECT_EQ("test_physics", physics2.Name());
}

/////////////////////////////////////////////////
TEST(DOMPhysics, MoveAssignmentOperator)
{
  sdf::Physics physics;
  physics.SetName("test_physics");

  sdf::Physics physics2;
  physics2 = std::move(physics);
  EXPECT_EQ("test_physics", physics2.Name());
}

/////////////////////////////////////////////////
TEST(DOMPhysics, CopyAssignmentAfterMove)
{
  sdf::Physics physics1;
  physics1.SetName("physics1");

  sdf::Physics physics2;
  physics2.SetName("physics2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Physics tmp = std::move(physics1);
  physics1 = physics2;
  physics2 = tmp;

  EXPECT_EQ("physics2", physics1.Name());
  EXPECT_EQ("physics1", physics2.Name());
}

/////////////////////////////////////////////////
TEST(DOMPhysics, ToElement)
{
  sdf::Physics physics;
  sdf::Errors errors;
  physics.SetName("my-bullet-engine");
  physics.SetDefault(true);
  physics.SetEngineType("bullet");
  physics.SetMaxStepSize(0.1);
  physics.SetRealTimeFactor(20.4);
  physics.SetMaxContacts(42);

  sdf::ElementPtr elem = physics.ToElement(errors);
  ASSERT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Physics physics2;
  physics2.Load(elem);

  // verify values after loading the element back
  EXPECT_EQ(physics.Name(), physics2.Name());
  EXPECT_EQ(physics.IsDefault(), physics2.IsDefault());
  EXPECT_EQ(physics.EngineType(), physics2.EngineType());
  EXPECT_DOUBLE_EQ(physics.MaxStepSize(), physics2.MaxStepSize());
  EXPECT_DOUBLE_EQ(physics.RealTimeFactor(), physics2.RealTimeFactor());
  EXPECT_EQ(physics.MaxContacts(), physics2.MaxContacts());
}
