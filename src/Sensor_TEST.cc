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

  EXPECT_DOUBLE_EQ(0.0, sensor.UpdateRate());

  EXPECT_TRUE(sensor.Topic().empty());
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

/////////////////////////////////////////////////
TEST(DOMSensor, Type)
{
  sdf::Sensor sensor;

  EXPECT_EQ(sdf::SensorType::NONE, sensor.Type());
  EXPECT_EQ("none", sensor.TypeStr());

  std::vector<sdf::SensorType> types = {
    sdf::SensorType::NONE,
    sdf::SensorType::ALTIMETER,
    sdf::SensorType::CAMERA,
    sdf::SensorType::CONTACT,
    sdf::SensorType::DEPTH_CAMERA,
    sdf::SensorType::FORCE_TORQUE,
    sdf::SensorType::GPS,
    sdf::SensorType::GPU_LIDAR,
    sdf::SensorType::IMU,
    sdf::SensorType::LOGICAL_CAMERA,
    sdf::SensorType::MAGNETOMETER,
    sdf::SensorType::MULTICAMERA,
    sdf::SensorType::LIDAR,
    sdf::SensorType::RFID,
    sdf::SensorType::RFIDTAG,
    sdf::SensorType::SONAR,
    sdf::SensorType::WIRELESS_RECEIVER,
    sdf::SensorType::WIRELESS_TRANSMITTER
  };
  std::vector<std::string> typeStrs =
  {
    "none",
    "altimeter",
    "camera",
    "contact",
    "depth_camera",
    "force_torque",
    "gps",
    "gpu_lidar",
    "imu",
    "logical_camera",
    "magnetometer",
    "multicamera",
    "lidar",
    "rfid",
    "rfidtag",
    "sonar",
    "wireless_receiver",
    "wireless_transmitter"
  };


  for (size_t i = 0; i < types.size(); ++i)
  {
    sensor.SetType(types[i]);
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs [i], sensor.TypeStr());
  }
}
