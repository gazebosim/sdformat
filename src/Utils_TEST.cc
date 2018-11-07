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
#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;
using namespace ignition::math;

/////////////////////////////////////////////////
TEST(DOMUtils, PoseDefaultValues)
{
  ElementPtr element(new Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);
  element->AddAttribute("frame", "string", "", false);

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(frame.empty());
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseNoFrame)
{
  ElementPtr element(new Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(frame.empty());
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseWithFrame)
{
  ElementPtr element(new Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);
  element->AddAttribute("frame", "string", "", false);
  element->GetAttribute("frame")->SetFromString("frame_name");

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_EQ("frame_name", frame);
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseWithValue)
{
  ElementPtr element(new Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);
  element->AddAttribute("frame", "string", "", false);
  element->GetAttribute("frame")->SetFromString("another frame");
  element->GetValue()->SetFromString("1 2 3 0.1 0.2 0.3");

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), pose);
  EXPECT_EQ("another frame", frame);
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseInFrame)
{
  FrameGraph graph;
  EXPECT_EQ(std::nullopt, poseInFrame("", "", graph));
}
