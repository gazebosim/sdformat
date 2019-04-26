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
#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMVisual, Construction)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  visual.SetName("test_visual");
  EXPECT_EQ(visual.Name(), "test_visual");

  EXPECT_EQ(ignition::math::Pose3d::Zero, visual.Pose());
  EXPECT_TRUE(visual.PoseFrame().empty());

  visual.SetPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.Pose());

  visual.SetPoseFrame("link");
  EXPECT_EQ("link", visual.PoseFrame());

  ASSERT_NE(nullptr, visual.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, visual.Geom()->Type());
  EXPECT_EQ(nullptr, visual.Geom()->BoxShape());
  EXPECT_EQ(nullptr, visual.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, visual.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, visual.Geom()->SphereShape());

  EXPECT_EQ(nullptr, visual.Material());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyConstructor)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});

  visual.SetPoseFrame("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2(visual);

  EXPECT_EQ("test_visual", visual.Name());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.Pose());
  EXPECT_EQ("link", visual.PoseFrame());
  ASSERT_TRUE(nullptr != visual.Material());
  EXPECT_EQ(mat.Ambient(), visual.Material()->Ambient());

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.Pose());
  EXPECT_EQ("link", visual2.PoseFrame());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyAssignmentOperator)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});

  visual.SetPoseFrame("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2;
  visual2 = visual;

  EXPECT_EQ("test_visual", visual.Name());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual.Pose());
  EXPECT_EQ("link", visual.PoseFrame());
  ASSERT_TRUE(nullptr != visual.Material());
  EXPECT_EQ(mat.Ambient(), visual.Material()->Ambient());

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.Pose());
  EXPECT_EQ("link", visual2.PoseFrame());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
}

/////////////////////////////////////////////////
TEST(DOMVisual, MoveConstructor)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});

  visual.SetPoseFrame("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2(std::move(visual));

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.Pose());
  EXPECT_EQ("link", visual2.PoseFrame());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
}

/////////////////////////////////////////////////
TEST(DOMVisual, MoveAssignmentOperator)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetPose({0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2});

  visual.SetPoseFrame("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2;
  visual2 = std::move(visual);

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_EQ(ignition::math::Pose3d(0, -20, 30, IGN_PI_2, -IGN_PI, IGN_PI_2),
            visual2.Pose());
  EXPECT_EQ("link", visual2.PoseFrame());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyAssignmentAfterMove)
{
  sdf::Visual visual1;
  visual1.SetName("visual1");

  sdf::Visual visual2;
  visual2.SetName("visual2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Visual tmp = std::move(visual1);
  visual1 = visual2;
  visual2 = tmp;

  EXPECT_EQ("visual2", visual1.Name());
  EXPECT_EQ("visual1", visual2.Name());
}

/////////////////////////////////////////////////
TEST(DOMVisual, SetGeometry)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);

  visual.SetGeom(geometry);

  ASSERT_NE(nullptr, visual.Geom());
  EXPECT_EQ(sdf::GeometryType::BOX, visual.Geom()->Type());
}

/////////////////////////////////////////////////
TEST(DOMVisual, SetMaterial)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  sdf::Material material;
  material.SetAmbient(ignition::math::Color(0, 0.5, 0));
  material.SetDiffuse(ignition::math::Color(1, 0, 0));
  material.SetSpecular(ignition::math::Color(0.f, 0.1f, 0.9f));

  visual.SetMaterial(material);

  ASSERT_NE(nullptr, visual.Material());
  EXPECT_EQ(ignition::math::Color(0, 0.5, 0), visual.Material()->Ambient());
  EXPECT_EQ(ignition::math::Color(1, 0, 0), visual.Material()->Diffuse());
  EXPECT_EQ(ignition::math::Color(0.f, 0.1f, 0.9f),
            visual.Material()->Specular());
}
