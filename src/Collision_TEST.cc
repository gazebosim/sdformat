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
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMcollision, Construction)
{
  sdf::Collision collision;
  EXPECT_EQ(nullptr, collision.Element());
  EXPECT_TRUE(collision.Name().empty());

  collision.SetName("test_collison");
  EXPECT_EQ(collision.Name(), "test_collison");

  EXPECT_EQ(gz::math::Pose3d::Zero, collision.RawPose());
  EXPECT_TRUE(collision.PoseRelativeTo().empty());
  {
    auto semanticPose = collision.SemanticPose();
    EXPECT_EQ(collision.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision.RawPose());

  collision.SetPoseRelativeTo("link");
  EXPECT_EQ("link", collision.PoseRelativeTo());
  {
    auto semanticPose = collision.SemanticPose();
    EXPECT_EQ(collision.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
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
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2(std::move(collision));
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyConstructor)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2(collision);
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, MoveAssignment)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2;
  collision2 = std::move(collision);
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyAssignment)
{
  sdf::Collision collision;
  collision.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2;
  collision2 = collision;
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
            collision2.RawPose());
}

/////////////////////////////////////////////////
TEST(DOMCollision, CopyAssignmentAfterMove)
{
  sdf::Collision collision1;
  collision1.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});

  sdf::Collision collision2;
  collision2.SetRawPose({-20, -30, -40, GZ_PI, GZ_PI, GZ_PI});

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Collision tmp = std::move(collision1);
  collision1 = collision2;
  collision2 = tmp;

  EXPECT_EQ(gz::math::Pose3d(-20, -30, -40, GZ_PI, GZ_PI, GZ_PI),
            collision1.RawPose());
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
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

/////////////////////////////////////////////////
TEST(DOMCollision, ToElement)
{
  sdf::Collision collision;

  collision.SetName("my-collision");

  sdf::Geometry geom;
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  sdf::Surface surface;
  sdf::Contact contact;
  contact.SetCollideBitmask(123u);
  surface.SetContact(contact);
  sdf::Friction friction;
  sdf::ODE ode;
  ode.SetMu(1.23);
  friction.SetODE(ode);
  surface.SetFriction(friction);
  collision.SetSurface(surface);

  sdf::ElementPtr elem = collision.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Collision collision2;
  collision2.Load(elem);
  const sdf::Surface *surface2 = collision2.Surface();

  EXPECT_EQ(collision.Name(), collision2.Name());
  EXPECT_EQ(collision.RawPose(), collision2.RawPose());
  EXPECT_NE(nullptr, collision2.Geom());
  ASSERT_NE(nullptr, surface2);
  ASSERT_NE(nullptr, surface2->Contact());
  EXPECT_EQ(123u, surface2->Contact()->CollideBitmask());
  ASSERT_NE(nullptr, surface2->Friction());
  ASSERT_NE(nullptr, surface2->Friction()->ODE());
  EXPECT_DOUBLE_EQ(1.23, surface2->Friction()->ODE()->Mu());
}

/////////////////////////////////////////////////
TEST(DOMCollision, ToElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  sdf::Collision collision;
  sdf::Errors errors;

  collision.SetName("my-collision");

  sdf::Geometry geom;
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  sdf::Surface surface;
  sdf::Contact contact;
  contact.SetCollideBitmask(123u);
  surface.SetContact(contact);
  sdf::Friction friction;
  sdf::ODE ode;
  ode.SetMu(1.23);
  friction.SetODE(ode);
  surface.SetFriction(friction);
  collision.SetSurface(surface);

  sdf::ElementPtr elem = collision.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Collision collision2;
  errors = collision2.Load(elem);
  EXPECT_TRUE(errors.empty());
  const sdf::Surface *surface2 = collision2.Surface();

  EXPECT_EQ(collision.Name(), collision2.Name());
  EXPECT_EQ(collision.RawPose(), collision2.RawPose());
  EXPECT_NE(nullptr, collision2.Geom());
  ASSERT_NE(nullptr, surface2);
  ASSERT_NE(nullptr, surface2->Contact());
  EXPECT_EQ(123u, surface2->Contact()->CollideBitmask());
  ASSERT_NE(nullptr, surface2->Friction());
  ASSERT_NE(nullptr, surface2->Friction()->ODE());
  EXPECT_DOUBLE_EQ(1.23, surface2->Friction()->ODE()->Mu());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
