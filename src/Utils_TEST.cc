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
#include "Utils.hh"

/////////////////////////////////////////////////
TEST(DOMUtils, PoseDefaultValues)
{
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);
  element->AddAttribute("relative_to", "string", "", false);

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(sdf::loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(frame.empty());
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseNoFrame)
{
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(sdf::loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(frame.empty());
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseNoValue)
{
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("pose");
  element->AddValue("pose", "", true);

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(sdf::loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(frame.empty());
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseWithFrame)
{
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);
  element->AddAttribute("relative_to", "string", "", false);
  element->GetAttribute("relative_to")->SetFromString("frame_name");

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(sdf::loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_EQ("frame_name", frame);
}

/////////////////////////////////////////////////
TEST(DOMUtils, PoseWithValue)
{
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("pose");
  element->AddValue("pose", "0 0 0 0 0 0", true);
  element->AddAttribute("relative_to", "string", "", false);
  element->GetAttribute("relative_to")->SetFromString("another frame");
  element->GetValue()->SetFromString("1 2 3 0.1 0.2 0.3");

  ignition::math::Pose3d pose;
  std::string frame;
  EXPECT_TRUE(sdf::loadPose(element, pose, frame));

  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3), pose);
  EXPECT_EQ("another frame", frame);
}

/////////////////////////////////////////////////
TEST(DOMUtils, ReservedNames)
{
  EXPECT_FALSE(sdf::isReservedName("model"));
  EXPECT_FALSE(sdf::isReservedName("link"));
  EXPECT_FALSE(sdf::isReservedName("joint"));
  EXPECT_FALSE(sdf::isReservedName("frame"));
  EXPECT_FALSE(sdf::isReservedName("collision"));
  EXPECT_FALSE(sdf::isReservedName("visual"));
  EXPECT_FALSE(sdf::isReservedName("not_reserved"));
  EXPECT_FALSE(sdf::isReservedName("_"));
  EXPECT_FALSE(sdf::isReservedName("__"));
  EXPECT_FALSE(sdf::isReservedName("___"));

  EXPECT_TRUE(sdf::isReservedName("____"));
  EXPECT_TRUE(sdf::isReservedName("world"));
  EXPECT_TRUE(sdf::isReservedName("__model__"));
  EXPECT_TRUE(sdf::isReservedName("__world__"));
  EXPECT_TRUE(sdf::isReservedName("__anything__"));
}

/////////////////////////////////////////////////
TEST(PolicyUtils, EnforcementPolicyErrors)
{
  sdf::Errors errors;
  const std::string emptyXmlPath = "/sdf/model";
  const std::string emptyFilePath = "Empty/file/path";
  sdf::Error error(
      sdf::ErrorCode::FILE_READ,
      "Unable to read a file",
      emptyXmlPath,
      emptyFilePath,
      10);
  ASSERT_EQ(error, true);
  ASSERT_TRUE(errors.empty());

  sdf::enforceConfigurablePolicyCondition(
      sdf::EnforcementPolicy::ERR,
      error,
      errors);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0], true);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_EQ(errors[0].Message(), "Unable to read a file");
  ASSERT_TRUE(errors[0].XmlPath().has_value());
  EXPECT_EQ(errors[0].XmlPath().value(), emptyXmlPath);
  ASSERT_TRUE(errors[0].FilePath().has_value());
  EXPECT_EQ(errors[0].FilePath().value(), emptyFilePath);
  ASSERT_TRUE(errors[0].LineNumber().has_value());
  EXPECT_EQ(errors[0].LineNumber().value(), 10);
}
