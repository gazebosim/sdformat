/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include "sdf/Projector.hh"

/////////////////////////////////////////////////
TEST(DOMProjector, Construction)
{
  sdf::Projector projector;

  EXPECT_EQ(nullptr, projector.Element());
  EXPECT_TRUE(projector.Name().empty());

  projector.SetName("test_projector");
  EXPECT_EQ(projector.Name(), "test_projector");

  EXPECT_DOUBLE_EQ(0.1, projector.NearClip());
  projector.SetNearClip(2.0);
  EXPECT_DOUBLE_EQ(2.0, projector.NearClip());

  EXPECT_DOUBLE_EQ(10.0, projector.FarClip());
  projector.SetFarClip(20.0);
  EXPECT_DOUBLE_EQ(20.0, projector.FarClip());

  EXPECT_EQ(gz::math::Angle(0.785), projector.HorizontalFov());
  projector.SetHorizontalFov(gz::math::Angle(GZ_PI * 0.5));
  EXPECT_EQ(gz::math::Angle(GZ_PI * 0.5), projector.HorizontalFov());

  EXPECT_EQ(UINT32_MAX, projector.VisibilityFlags());
  projector.SetVisibilityFlags(0x03);
  EXPECT_EQ(0x03, projector.VisibilityFlags());

  EXPECT_TRUE(projector.Texture().empty());
  projector.SetTexture("texture.png");
  EXPECT_EQ("texture.png", projector.Texture());

  EXPECT_EQ(gz::math::Pose3d::Zero, projector.RawPose());
  projector.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 1.5707));
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 1.5707), projector.RawPose());

  EXPECT_TRUE(projector.PoseRelativeTo().empty());
  projector.SetPoseRelativeTo("/test/relative");
  EXPECT_EQ("/test/relative", projector.PoseRelativeTo());

  EXPECT_TRUE(projector.Plugins().empty());
  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");

  projector.AddPlugin(plugin);
  ASSERT_EQ(1u, projector.Plugins().size());

  plugin.SetName("name2");
  projector.AddPlugin(plugin);
  ASSERT_EQ(2u, projector.Plugins().size());

  EXPECT_EQ("name1", projector.Plugins()[0].Name());
  EXPECT_EQ("name2", projector.Plugins()[1].Name());

  projector.ClearPlugins();
  EXPECT_TRUE(projector.Plugins().empty());
}

/////////////////////////////////////////////////
TEST(DOMProjector, ToElement)
{
  sdf::Projector projector;

  projector.SetName("my-projector");
  projector.SetNearClip(0.3);
  projector.SetFarClip(12.3);
  projector.SetHorizontalFov(gz::math::Angle(0.5));
  projector.SetVisibilityFlags(0x01);
  projector.SetTexture("test_texture.png");
  projector.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");
  projector.AddPlugin(plugin);

  sdf::ElementPtr elem = projector.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Projector projector2;
  projector2.Load(elem);

  EXPECT_EQ(projector.Name(), projector2.Name());
  EXPECT_DOUBLE_EQ(projector.NearClip(), projector2.NearClip());
  EXPECT_DOUBLE_EQ(projector.FarClip(), projector2.FarClip());
  EXPECT_EQ(projector.VisibilityFlags(), projector2.VisibilityFlags());
  EXPECT_EQ(projector.HorizontalFov(), projector2.HorizontalFov());
  EXPECT_EQ(projector.Texture(), projector2.Texture());
  EXPECT_EQ(projector.RawPose(), projector2.RawPose());

  ASSERT_EQ(1u, projector2.Plugins().size());
  EXPECT_EQ("name1", projector2.Plugins()[0].Name());
  EXPECT_EQ("filename1", projector2.Plugins()[0].Filename());
}
