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
#include "sdf/Polyline.hh"
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMVisual, Construction)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  visual.SetName("test_visual");
  EXPECT_EQ(visual.Name(), "test_visual");

  visual.SetCastShadows(false);
  EXPECT_FALSE(visual.CastShadows());

  // check default transparency is 0
  EXPECT_FLOAT_EQ(0.0, visual.Transparency());

  visual.SetTransparency(0.34f);
  EXPECT_FLOAT_EQ(0.34f, visual.Transparency());

  EXPECT_EQ(gz::math::Pose3d::Zero, visual.RawPose());
  EXPECT_TRUE(visual.PoseRelativeTo().empty());
  {
    auto semanticPose = visual.SemanticPose();
    EXPECT_EQ(visual.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  visual.SetRawPose({0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2});
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual.RawPose());

  visual.SetPoseRelativeTo("link");
  EXPECT_EQ("link", visual.PoseRelativeTo());
  {
    auto semanticPose = visual.SemanticPose();
    EXPECT_EQ(visual.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  ASSERT_NE(nullptr, visual.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, visual.Geom()->Type());
  EXPECT_EQ(nullptr, visual.Geom()->BoxShape());
  EXPECT_EQ(nullptr, visual.Geom()->ConeShape());
  EXPECT_EQ(nullptr, visual.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, visual.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, visual.Geom()->SphereShape());
  EXPECT_TRUE(visual.Geom()->PolylineShape().empty());

  EXPECT_EQ(nullptr, visual.Material());

  // visibility flags
  EXPECT_EQ(UINT32_MAX, visual.VisibilityFlags());
  visual.SetVisibilityFlags(1u);
  EXPECT_EQ(1u, visual.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyConstructor)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2(visual);

  EXPECT_EQ("test_visual", visual.Name());
  EXPECT_FALSE(visual.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual.Transparency());
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual.RawPose());
  EXPECT_EQ("link", visual.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual.Material());
  EXPECT_EQ(mat.Ambient(), visual.Material()->Ambient());
  EXPECT_EQ(2u, visual.VisibilityFlags());

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, CopyAssignmentOperator)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2;
  visual2 = visual;

  EXPECT_EQ("test_visual", visual.Name());
  EXPECT_FALSE(visual.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual.Transparency());
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual.RawPose());
  EXPECT_EQ("link", visual.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual.Material());
  EXPECT_EQ(mat.Ambient(), visual.Material()->Ambient());
  EXPECT_EQ(2u, visual.VisibilityFlags());

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, MoveConstructor)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2(std::move(visual));

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
}

/////////////////////////////////////////////////
TEST(DOMVisual, MoveAssignmentOperator)
{
  sdf::Visual visual;
  visual.SetName("test_visual");
  visual.SetCastShadows(false);
  visual.SetTransparency(0.345f);
  visual.SetRawPose({0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2});
  visual.SetVisibilityFlags(2u);

  visual.SetPoseRelativeTo("link");

  sdf::Material mat;
  mat.SetAmbient({0.1f, 0.1f, 0.1f});
  visual.SetMaterial(mat);

  sdf::Visual visual2;
  visual2 = std::move(visual);

  EXPECT_EQ("test_visual", visual2.Name());
  EXPECT_FALSE(visual2.CastShadows());
  EXPECT_FLOAT_EQ(0.345f, visual2.Transparency());
  EXPECT_EQ(gz::math::Pose3d(0, -20, 30, GZ_PI_2, -GZ_PI, GZ_PI_2),
            visual2.RawPose());
  EXPECT_EQ("link", visual2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != visual2.Material());
  EXPECT_EQ(mat.Ambient(), visual2.Material()->Ambient());
  EXPECT_EQ(2u, visual2.VisibilityFlags());
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
  material.SetAmbient(gz::math::Color(0, 0.5, 0));
  material.SetDiffuse(gz::math::Color(1, 0, 0));
  material.SetSpecular(gz::math::Color(0.f, 0.1f, 0.9f));

  visual.SetMaterial(material);

  ASSERT_NE(nullptr, visual.Material());
  EXPECT_EQ(gz::math::Color(0, 0.5, 0), visual.Material()->Ambient());
  EXPECT_EQ(gz::math::Color(1, 0, 0), visual.Material()->Diffuse());
  EXPECT_EQ(gz::math::Color(0.f, 0.1f, 0.9f),
            visual.Material()->Specular());
}

/////////////////////////////////////////////////
TEST(DOMVisual, SetLaserRetro)
{
  sdf::Visual visual;
  EXPECT_EQ(nullptr, visual.Element());
  EXPECT_TRUE(visual.Name().empty());

  visual.SetLaserRetro(150);

  EXPECT_TRUE(visual.HasLaserRetro());
  EXPECT_DOUBLE_EQ(150, visual.LaserRetro());
}

/////////////////////////////////////////////////
TEST(DOMVisual, ToElement)
{
  sdf::Visual visual;
  visual.SetName("my-visual");
  visual.SetCastShadows(true);
  visual.SetTransparency(0.2f);
  visual.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  visual.SetVisibilityFlags(1234u);
  visual.SetHasLaserRetro(true);
  visual.SetLaserRetro(1.2);

  sdf::Geometry geom;
  visual.SetGeom(geom);

  sdf::Material mat;
  visual.SetMaterial(mat);

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");
  visual.AddPlugin(plugin);

  sdf::ElementPtr elem = visual.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Visual visual2;
  visual2.Load(elem);

  EXPECT_EQ(visual.Name(), visual2.Name());
  EXPECT_EQ(visual.CastShadows(), visual2.CastShadows());
  EXPECT_DOUBLE_EQ(visual.Transparency(), visual2.Transparency());
  EXPECT_EQ(visual.RawPose(), visual2.RawPose());
  EXPECT_EQ(visual.VisibilityFlags(), visual2.VisibilityFlags());
  EXPECT_EQ(visual.HasLaserRetro(), visual2.HasLaserRetro());
  EXPECT_DOUBLE_EQ(visual.LaserRetro(), visual2.LaserRetro());
  EXPECT_NE(nullptr, visual2.Geom());
  EXPECT_NE(nullptr, visual2.Material());

  ASSERT_EQ(1u, visual2.Plugins().size());
  EXPECT_EQ("name1", visual2.Plugins()[0].Name());
  EXPECT_EQ("filename1", visual2.Plugins()[0].Filename());
}

/////////////////////////////////////////////////
TEST(DOMVisual, Plugins)
{
  sdf::Visual visual;
  EXPECT_TRUE(visual.Plugins().empty());

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");

  visual.AddPlugin(plugin);
  ASSERT_EQ(1u, visual.Plugins().size());

  plugin.SetName("name2");
  visual.AddPlugin(plugin);
  ASSERT_EQ(2u, visual.Plugins().size());

  EXPECT_EQ("name1", visual.Plugins()[0].Name());
  EXPECT_EQ("name2", visual.Plugins()[1].Name());

  visual.ClearPlugins();
  EXPECT_TRUE(visual.Plugins().empty());
}
