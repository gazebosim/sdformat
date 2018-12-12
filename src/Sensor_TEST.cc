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
#include "sdf/Sensor.hh"

/////////////////////////////////////////////////
TEST(DOMSensor, Construction)
{
  sdf::Sensor sensor;
  EXPECT_EQ(nullptr, sensor.Element());
  EXPECT_EQ(sdf::SensorType::NONE, sensor.Type());

  sensor.SetType(sdf::SensorType::ALTIMETER);
  EXPECT_EQ(sdf::SensorType::ALTIMETER, sensor.Type());

  EXPECT_EQ(ignition::math::Pose3d::Zero, sensor.Pose());

  sensor.SetPose(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.Pose());

  EXPECT_TRUE(sensor.PoseFrame().empty());
  sensor.SetPoseFrame("a_frame");
  EXPECT_EQ("a_frame", sensor.PoseFrame());
}

/////////////////////////////////////////////////
TEST(DOMSensor, Load)
{
  sdf::Sensor sensor;
  sdf::Errors errors;

  // Null element name
  errors = sensor.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, sensor.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = sensor.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, sensor.Element());
}
