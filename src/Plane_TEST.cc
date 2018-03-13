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
#include "sdf/Plane.hh"

/////////////////////////////////////////////////
TEST(DOMPlane, Construction)
{
  sdf::Plane plane;

  EXPECT_EQ(ignition::math::Vector3d::UnitZ, plane.Normal());
  EXPECT_EQ(ignition::math::Vector2d::One, plane.Size());

  plane.SetNormal({1, 0, 0});
  EXPECT_EQ(ignition::math::Vector3d::UnitX, plane.Normal());

  plane.SetNormal({1, 0, 1});
  EXPECT_EQ(ignition::math::Vector3d(0.707107, 0, 0.707107), plane.Normal());

  plane.SetSize({1.2, 3.4});
  EXPECT_EQ(ignition::math::Vector2d(1.2, 3.4), plane.Size());
}
