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
#include "sdf/World.hh"

/////////////////////////////////////////////////
TEST(DOMWorld, Construction)
{
  sdf::World world;
  EXPECT_TRUE(world.Name().empty());
  EXPECT_EQ(world.Gravity(), ignition::math::Vector3d(0, 0, -9.80665));
  EXPECT_EQ(world.MagneticField(),
      ignition::math::Vector3d(5.5645e-6, 22.8758e-6, -42.3884e-6));
  EXPECT_STREQ(world.AudioDevice().c_str(), "default");
  EXPECT_EQ(world.WindLinearVelocity(), ignition::math::Vector3d::Zero);
}

/////////////////////////////////////////////////
TEST(DOMWorld, Set)
{
  sdf::World world;
  EXPECT_TRUE(world.Name().empty());

  world.SetName("default");
  EXPECT_EQ(world.Name(), "default");

  world.SetAudioDevice("/dev/audio");
  EXPECT_EQ(world.AudioDevice(), "/dev/audio");

  world.SetWindLinearVelocity({0, 1 , 2});
  EXPECT_EQ(world.WindLinearVelocity(), ignition::math::Vector3d(0, 1, 2));

  world.SetGravity({1, -2, 4});
  EXPECT_EQ(world.Gravity(), ignition::math::Vector3d(1, -2, 4));

  world.SetMagneticField({1.2, -2.3, 4.5});
  EXPECT_EQ(world.MagneticField(), ignition::math::Vector3d(1.2, -2.3, 4.5));
}
