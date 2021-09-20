/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/math/Pose3.hh>

#include "sdf/InterfaceElements.hh"

TEST(InterfaceElements, Construction)
{
  sdf::NestedInclude nestedInclude;
  EXPECT_EQ("", nestedInclude.Uri());
  EXPECT_EQ("", nestedInclude.ResolvedFileName());
  EXPECT_EQ("", nestedInclude.AbsoluteParentName());
  EXPECT_FALSE(nestedInclude.LocalModelName().has_value());
  EXPECT_FALSE(nestedInclude.IsStatic().has_value());
  EXPECT_FALSE(nestedInclude.IncludeRawPose().has_value());
  EXPECT_FALSE(nestedInclude.IncludePoseRelativeTo().has_value());
  EXPECT_FALSE(nestedInclude.PlacementFrame().has_value());
  EXPECT_EQ(nullptr, nestedInclude.IncludeElement());

  nestedInclude.SetUri("file://test_path");
  EXPECT_EQ("file://test_path", nestedInclude.Uri());

  nestedInclude.SetResolvedFileName("/path/to/file");
  EXPECT_EQ("/path/to/file", nestedInclude.ResolvedFileName());

  nestedInclude.SetAbsoluteParentName("A::B::C");
  EXPECT_EQ("A::B::C", nestedInclude.AbsoluteParentName());

  nestedInclude.SetLocalModelName("test_model");
  EXPECT_EQ("test_model", nestedInclude.LocalModelName());

  nestedInclude.SetIsStatic(true);
  EXPECT_EQ(true, nestedInclude.IsStatic());
  nestedInclude.SetIsStatic(false);
  EXPECT_EQ(false, nestedInclude.IsStatic());

  nestedInclude.SetIncludeRawPose({1, 2, 3, 0, 0, IGN_PI_4});
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, IGN_PI_4),
            nestedInclude.IncludeRawPose());

  nestedInclude.SetIncludePoseRelativeTo("test_frame");
  EXPECT_EQ("test_frame", nestedInclude.IncludePoseRelativeTo());

  nestedInclude.SetPlacementFrame("test_placement_frame");
  EXPECT_EQ("test_placement_frame", nestedInclude.PlacementFrame());
}
