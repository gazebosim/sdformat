/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/math/Pose3.hh>

#include "test_config.h"
#include "test_utils.hh"

#include "USDJoints.hh"

#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/usd/usd_parser/USDData.hh"

/////////////////////////////////////////////////
TEST(USDJointTest, JointTest)
{
  const std::string filename =
    sdf::testing::TestFile("usd", "double_pendulum.usda");
  const auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  sdf::usd::USDData usdData(filename);
  usdData.Init();

  const auto upperJoint = stage->GetPrimAtPath(pxr::SdfPath(
    "/double_pendulum/double_pendulum_with_base/upper_joint"));
  ASSERT_TRUE(upperJoint);

  sdf::Joint joint1;
  auto errors = sdf::usd::ParseJoints(
    upperJoint, usdData, joint1);

  EXPECT_EQ(0u, errors.size());
  EXPECT_EQ(sdf::JointType::REVOLUTE, joint1.Type());
  EXPECT_EQ("upper_joint", joint1.Name());
  EXPECT_EQ("upper_link", joint1.ChildLinkName());
  EXPECT_EQ("base", joint1.ParentLinkName());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0.021, -1.5708, 0, 0),
            joint1.RawPose());

  auto axis = joint1.Axis(0);
  ASSERT_NE(nullptr, axis);
  EXPECT_EQ(ignition::math::Vector3d(1, 0, 0), axis->Xyz());

  const auto lowerJoint = stage->GetPrimAtPath(pxr::SdfPath(
    "/double_pendulum/double_pendulum_with_base/lower"));
  ASSERT_TRUE(lowerJoint);

  sdf::Joint joint2;
  errors = sdf::usd::ParseJoints(
    lowerJoint, usdData, joint2);

  EXPECT_EQ(0u, errors.size());
  EXPECT_EQ(sdf::JointType::REVOLUTE, joint2.Type());
  EXPECT_EQ("lower_joint", joint2.Name());
  EXPECT_EQ("lower_link", joint2.ChildLinkName());
  EXPECT_EQ("upper_link", joint2.ParentLinkName());
  EXPECT_EQ(ignition::math::Pose3d(0.0025, -0, 0.01, -0.4292, 0, 0),
            joint2.RawPose());
  axis = joint2.Axis(0);
  ASSERT_NE(nullptr, axis);
  EXPECT_EQ(ignition::math::Vector3d(1, 0, 0), axis->Xyz());
}
