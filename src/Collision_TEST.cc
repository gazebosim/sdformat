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
#include "sdf/Surface.hh"

/////////////////////////////////////////////////
TEST(DOMcollision, Construction)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());
  EXPECT_TRUE(collision.Name().empty());

  collision.SetName("test_collison");
  EXPECT_EQ(collision.Name(), "test_collison");

  EXPECT_EQ(ignition::math::Pose3d::Zero, collision.RawPose());
  EXPECT_TRUE(collision.PoseRelativeTo().empty());
  {
    auto semanticPose = collision.SemanticPose();
    EXPECT_EQ(collision.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  collision.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            collision.RawPose());

  collision.SetPoseRelativeTo("link");
  EXPECT_EQ("link", collision.PoseRelativeTo());
  {
    auto semanticPose = collision.SemanticPose();
    EXPECT_EQ(collision.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  ASSERT_NE(nullptr, collision.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, collision.Geom()->Type());
  EXPECT_EQ(nullptr, collision.Geom()->BoxShape());
  EXPECT_EQ(nullptr, collision.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, collision.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, collision.Geom()->SphereShape());

  ASSERT_NE(nullptr, collision.Surface());
  ASSERT_NE(nullptr, collision.Surface()->Contact());
}

/////////////////////////////////////////////////
TEST(DOMCollision, MoveConstructor)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});

  sdf::Collision collision2(std::move(collision));
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyConstructor)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});

  sdf::Collision collision2(collision);
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, MoveAssignment)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});

  sdf::Collision collision2;
  collision2 = std::move(collision);
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyAssignment)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});

  sdf::Collision collision2;
  collision2 = collision;
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyAssignmentAfterMove)
{
  sdf::Collision collision1;
  collision1.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});

  sdf::Collision collision2;
  collision2.SetRawPose({-20, -30, -40, IGN_PI, IGN_PI, IGN_PI});

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Collision tmp = std::move(collision1);
  collision1 = collision2;
  collision2 = tmp;

  EXPECT_EQ(ignition::math::Pose3d(-20, -30, -40, IGN_PI, IGN_PI, IGN_PI),
            collision1.RawPose());
  EXPECT_EQ(ignition::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMcollision, SetGeometry)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());
  EXPECT_TRUE(collision.Name().empty());

  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);

  collision.SetGeom(geometry);

  ASSERT_NE(nullptr, collision.Geom());
  EXPECT_EQ(sdf::GeometryType::BOX, collision.Geom()->Type());
}

/////////////////////////////////////////////////
TEST(DOMcollision, SetSurface)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());

  sdf::Surface surface;
  ASSERT_NE(nullptr, surface.Contact());
  sdf::Contact contact;
  contact.SetCollideBitmask(0x2);
  surface.SetContact(contact);

  collision.SetSurface(surface);

  ASSERT_NE(nullptr, collision.Surface());
  ASSERT_NE(nullptr, collision.Surface()->Contact());
  EXPECT_EQ(collision.Surface()->Contact()->CollideBitmask(), 0x2);
}
