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
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMcollision, Construction)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());
  EXPECT_TRUE(collision.Name().empty());

  EXPECT_TRUE(collision.PoseFrame().empty());
  collision.SetName("test_collison");
  EXPECT_EQ("test_collison", collision.Name());
  EXPECT_TRUE(collision.PoseFrame().empty());

  EXPECT_EQ(ignition::math::Pose3d::Zero, collision.Pose());

  collision.SetPose({-10, -20, -30, 1.2, 0, 0});
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, 1.2, 0, 0),
            collision.Pose());

  EXPECT_TRUE(collision.SetPoseFrame("link"));
  EXPECT_EQ("link", collision.PoseFrame());
  EXPECT_FALSE(collision.SetPoseFrame(""));
  EXPECT_EQ("link", collision.PoseFrame());

  ASSERT_NE(nullptr, collision.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, collision.Geom()->Type());
  EXPECT_EQ(nullptr, collision.Geom()->BoxShape());
  EXPECT_EQ(nullptr, collision.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, collision.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, collision.Geom()->SphereShape());
}
