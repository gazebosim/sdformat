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
#include "test_utils.hh"

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

  frame.SetRawPose({-10, -20, -30, GZ_PI, GZ_PI, GZ_PI});
  EXPECT_EQ(gz::math::Pose3d(-10, -20, -30, GZ_PI, GZ_PI, GZ_PI),
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

/////////////////////////////////////////////////
TEST(DOMFrame, ToElement)
{
  // With 'attached-to'
  {
    sdf::Frame frame;

    frame.SetName("my-frame");
    frame.SetAttachedTo("attached-to-frame");
    frame.SetPoseRelativeTo("relative-to-frame");
    frame.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

    sdf::ElementPtr elem = frame.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Frame frame2;
    frame2.Load(elem);

    EXPECT_EQ(frame.Name(), frame.Name());
    EXPECT_EQ(frame.AttachedTo(), frame2.AttachedTo());
    EXPECT_EQ(frame.PoseRelativeTo(), frame2.PoseRelativeTo());
    EXPECT_EQ(frame.RawPose(), frame2.RawPose());
  }

  // Without 'attached-to'
  {
    sdf::Frame frame;

    frame.SetName("my-frame");
    frame.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
    EXPECT_TRUE(frame.AttachedTo().empty());

    sdf::ElementPtr elem = frame.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Frame frame2;
    frame2.Load(elem);

    EXPECT_EQ(frame.Name(), frame.Name());
    EXPECT_EQ(frame.RawPose(), frame2.RawPose());
    EXPECT_TRUE(frame.AttachedTo().empty());
    EXPECT_TRUE(frame2.AttachedTo().empty());
  }
}

/////////////////////////////////////////////////
TEST(DOMFrame, ToElementErrorOutput)
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

  sdf::Errors errors;

  // With 'attached-to'
  {
    sdf::Frame frame;

    frame.SetName("my-frame");
    frame.SetAttachedTo("attached-to-frame");
    frame.SetPoseRelativeTo("relative-to-frame");
    frame.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

    sdf::ElementPtr elem = frame.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Frame frame2;
    errors = frame2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(frame.Name(), frame.Name());
    EXPECT_EQ(frame.AttachedTo(), frame2.AttachedTo());
    EXPECT_EQ(frame.PoseRelativeTo(), frame2.PoseRelativeTo());
    EXPECT_EQ(frame.RawPose(), frame2.RawPose());
  }

  // Without 'attached-to'
  {
    sdf::Frame frame;

    frame.SetName("my-frame");
    frame.SetRawPose(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
    EXPECT_TRUE(frame.AttachedTo().empty());

    sdf::ElementPtr elem = frame.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Frame frame2;
    errors = frame2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(frame.Name(), frame.Name());
    EXPECT_EQ(frame.RawPose(), frame2.RawPose());
    EXPECT_TRUE(frame.AttachedTo().empty());
    EXPECT_TRUE(frame2.AttachedTo().empty());
  }

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
