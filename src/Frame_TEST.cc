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
#include "sdf/Frame.hh"
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMframe, Construction)
{
  sdf::Frame frame;
  EXPECT_EQ(nullptr, frame.Element());
  EXPECT_TRUE(frame.Name().empty());

  frame.SetName("test_frame");
  EXPECT_EQ(frame.Name(), "test_frame");

  EXPECT_TRUE(frame.AttachedTo().empty());
  EXPECT_EQ(gz::math::Pose3d::Zero, frame.RawPose());
  EXPECT_TRUE(frame.PoseRelativeTo().empty());
  {
    auto semanticPose = frame.SemanticPose();
    EXPECT_EQ(gz::math::Pose3d::Zero, semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  frame.SetAttachedTo("attachment");
  EXPECT_EQ("attachment", frame.AttachedTo());

  frame.SetRawPose({-10, -20, -30, IGN_PI, IGN_PI, IGN_PI});
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, IGN_PI, IGN_PI, IGN_PI),
            frame.RawPose());
  {
    auto semanticPose = frame.SemanticPose();
    EXPECT_EQ(frame.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("attachment", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  frame.SetPoseRelativeTo("link");
  EXPECT_EQ("link", frame.PoseRelativeTo());
  {
    auto semanticPose = frame.SemanticPose();
    EXPECT_EQ(frame.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  std::string body;
  EXPECT_FALSE(frame.ResolveAttachedToBody(body).empty());
  EXPECT_TRUE(body.empty());
}
