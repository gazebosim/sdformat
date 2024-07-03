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
#include "sdf/Noise.hh"
#include "sdf/Magnetometer.hh"
#include "sdf/sdf.hh"
#include "sdf/Sensor.hh"

/////////////////////////////////////////////////
TEST(DOMSensor, Construction)
{
  sdf::Sensor sensor;
  sdf::Sensor sensor2;
  EXPECT_TRUE(sensor == sensor2);
  EXPECT_FALSE(sensor != sensor2);

  EXPECT_EQ(nullptr, sensor.Element());
  EXPECT_EQ(sdf::SensorType::NONE, sensor.Type());

  sensor.SetType(sdf::SensorType::ALTIMETER);
  EXPECT_EQ(sdf::SensorType::ALTIMETER, sensor.Type());

  EXPECT_EQ(gz::math::Pose3d::Zero, sensor.RawPose());
  EXPECT_TRUE(sensor.PoseRelativeTo().empty());
  {
    auto semanticPose = sensor.SemanticPose();
    EXPECT_EQ(sensor.RawPose(), semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  sensor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.RawPose());

  sensor.SetPoseRelativeTo("a_frame");
  EXPECT_EQ("a_frame", sensor.PoseRelativeTo());
  {
    auto semanticPose = sensor.SemanticPose();
    EXPECT_EQ(sensor.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("a_frame", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  EXPECT_TRUE(sensor.FrameId().empty());
  sensor.SetFrameId("new_frame_id");
  EXPECT_EQ("new_frame_id", sensor.FrameId());

  EXPECT_DOUBLE_EQ(0.0, sensor.UpdateRate());

  EXPECT_TRUE(sensor.Topic().empty());
  EXPECT_FALSE(sensor == sensor2);
  EXPECT_TRUE(sensor != sensor2);
}

/////////////////////////////////////////////////
TEST(DOMSensor, MoveConstructor)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetUpdateRate(0.123);
  sensor.SetFrameId("new_frame_id");

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2(std::move(sensor));

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor2.FrameId());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
  EXPECT_DOUBLE_EQ(0.123, sensor2.UpdateRate());
}

/////////////////////////////////////////////////
TEST(DOMSensor, CopyConstructor)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetUpdateRate(0.123);
  sensor.SetFrameId("new_frame_id");

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2(sensor);

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.RawPose());
  EXPECT_EQ("a_frame", sensor.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor.FrameId());
  ASSERT_TRUE(nullptr != sensor.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor.MagnetometerSensor()->XNoise().Mean());

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor2.FrameId());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
  EXPECT_DOUBLE_EQ(0.123, sensor2.UpdateRate());
}

/////////////////////////////////////////////////
TEST(DOMSensor, MoveAssignment)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetFrameId("new_frame_id");

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2;
  sensor2 = std::move(sensor);

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor2.FrameId());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
}

/////////////////////////////////////////////////
TEST(DOMSensor, CopyAssignment)
{
  sdf::Sensor sensor;
  sensor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetFrameId("new_frame_id");

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Sensor sensor2;
  sensor2 = sensor;

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor.RawPose());
  EXPECT_EQ("a_frame", sensor.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor.FrameId());
  ASSERT_TRUE(nullptr != sensor.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor.MagnetometerSensor()->XNoise().Mean());

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor.FrameId());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
}

/////////////////////////////////////////////////
TEST(DOMSensor, CopyAssignmentAfterMove)
{
  sdf::Sensor sensor1;
  sensor1.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor1.SetType(sdf::SensorType::MAGNETOMETER);
  sensor1.SetPoseRelativeTo("frame1");
  sensor1.SetFrameId("new_frame_id");

  sdf::Sensor sensor2;
  sensor2.SetRawPose(gz::math::Pose3d(4, 5, 6, 0, 0, 0));
  sensor2.SetType(sdf::SensorType::CAMERA);
  sensor2.SetPoseRelativeTo("frame2");
  sensor2.SetFrameId("new_frame_id2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Sensor tmp = std::move(sensor1);
  sensor1 = sensor2;
  sensor2 = tmp;

  EXPECT_EQ(sdf::SensorType::CAMERA, sensor1.Type());
  EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 0), sensor1.RawPose());
  EXPECT_EQ("frame2", sensor1.PoseRelativeTo());
  EXPECT_EQ("new_frame_id2", sensor1.FrameId());

  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("frame1", sensor2.PoseRelativeTo());
  EXPECT_EQ("new_frame_id", sensor2.FrameId());
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
    sdf::SensorType::BOUNDINGBOX_CAMERA,
    sdf::SensorType::CAMERA,
    sdf::SensorType::CONTACT,
    sdf::SensorType::DEPTH_CAMERA,
    sdf::SensorType::FORCE_TORQUE,
    sdf::SensorType::NAVSAT,
    sdf::SensorType::GPU_LIDAR,
    sdf::SensorType::IMU,
    sdf::SensorType::LOGICAL_CAMERA,
    sdf::SensorType::MAGNETOMETER,
    sdf::SensorType::MULTICAMERA,
    sdf::SensorType::LIDAR,
    sdf::SensorType::RFID,
    sdf::SensorType::RFIDTAG,
    sdf::SensorType::SEGMENTATION_CAMERA,
    sdf::SensorType::SONAR,
    sdf::SensorType::WIRELESS_RECEIVER,
    sdf::SensorType::WIRELESS_TRANSMITTER,
    sdf::SensorType::THERMAL_CAMERA,
    sdf::SensorType::CUSTOM,
    sdf::SensorType::WIDE_ANGLE_CAMERA
  };
  std::vector<std::string> typeStrs =
  {
    "none",
    "altimeter",
    "boundingbox_camera",
    "camera",
    "contact",
    "depth_camera",
    "force_torque",
    "navsat",
    "gpu_lidar",
    "imu",
    "logical_camera",
    "magnetometer",
    "multicamera",
    "lidar",
    "rfid",
    "rfidtag",
    "segmentation_camera",
    "sonar",
    "wireless_receiver",
    "wireless_transmitter",
    "thermal_camera",
    "custom",
    "wide_angle_camera"
  };

  for (size_t i = 0; i < types.size(); ++i)
  {
    sensor.SetType(types[i]);
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs[i], sensor.TypeStr());
  }

  for (size_t i = 0; i < typeStrs .size(); ++i)
  {
    EXPECT_TRUE(sensor.SetType(typeStrs[i]));
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs[i], sensor.TypeStr());

    EXPECT_FALSE(sensor.SetType("bad_sensor_type"));
    EXPECT_EQ(types[i], sensor.Type());
    EXPECT_EQ(typeStrs[i], sensor.TypeStr());
  }
}

/////////////////////////////////////////////////
TEST(DOMSensor, EnableMetrics)
{
  sdf::Sensor sensor;
  // Verify default value.
  EXPECT_EQ(false, sensor.EnableMetrics());

  // Set up a simple sdf to test enable metrics option
  std::ostringstream stream;
  stream << "<sdf version='1.5'>"
         << "  <model name='test_model'>"
         << "    <sensor name='test_sensor' type='none'>"
         << "      <enable_metrics>true</enable_metrics>"
         << "    </sensor>"
         << "  </model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  const sdf::ElementPtr sensorElem = sdfParsed.Root()->
    GetElement("model")->GetElement("sensor");
  sensor.Load(sensorElem);
  EXPECT_EQ(true, sensor.EnableMetrics());
  sensor.SetEnableMetrics(false);
  EXPECT_EQ(false, sensor.EnableMetrics());
}

/////////////////////////////////////////////////
TEST(DOMSensor, MutableSensors)
{
  // Altimeter
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::ALTIMETER);

    sdf::Altimeter alt;
    sensor.SetAltimeterSensor(alt);

    sdf::Altimeter *altMutable = sensor.AltimeterSensor();
    ASSERT_NE(nullptr, altMutable);
    EXPECT_DOUBLE_EQ(altMutable->VerticalPositionNoise().Mean(),
        sensor.AltimeterSensor()->VerticalPositionNoise().Mean());

    sdf::Noise noise;
    noise.SetMean(2.0);
    altMutable->SetVerticalPositionNoise(noise);
    EXPECT_DOUBLE_EQ(altMutable->VerticalPositionNoise().Mean(), 2.0);
    EXPECT_DOUBLE_EQ(
        sensor.AltimeterSensor()->VerticalPositionNoise().Mean(), 2.0);
  }

  // Air pressure
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::AIR_PRESSURE);

    sdf::AirPressure air;
    sensor.SetAirPressureSensor(air);

    sdf::AirPressure *airMutable = sensor.AirPressureSensor();
    ASSERT_NE(nullptr, airMutable);
    EXPECT_DOUBLE_EQ(airMutable->ReferenceAltitude(),
        sensor.AirPressureSensor()->ReferenceAltitude());

    airMutable->SetReferenceAltitude(2.0);
    EXPECT_DOUBLE_EQ(airMutable->ReferenceAltitude(), 2.0);
    EXPECT_DOUBLE_EQ(sensor.AirPressureSensor()->ReferenceAltitude(), 2.0);
  }

  // Camera
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::CAMERA);

    sdf::Camera cam;
    sensor.SetCameraSensor(cam);

    sdf::Camera *camMutable = sensor.CameraSensor();
    ASSERT_NE(nullptr, camMutable);
    EXPECT_DOUBLE_EQ(camMutable->NearClip(), sensor.CameraSensor()->NearClip());

    camMutable->SetNearClip(2.0);
    EXPECT_DOUBLE_EQ(camMutable->NearClip(), 2.0);
    EXPECT_DOUBLE_EQ(sensor.CameraSensor()->NearClip(), 2.0);
  }

  // Force torque
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::FORCE_TORQUE);

    sdf::ForceTorque ftq;
    sensor.SetForceTorqueSensor(ftq);

    sdf::ForceTorque *ftqMutable = sensor.ForceTorqueSensor();
    ASSERT_NE(nullptr, ftqMutable);
    EXPECT_DOUBLE_EQ(ftqMutable->ForceXNoise().Mean(),
        sensor.ForceTorqueSensor()->ForceXNoise().Mean());

    sdf::Noise noise;
    noise.SetMean(2.0);
    ftqMutable->SetForceXNoise(noise);
    EXPECT_DOUBLE_EQ(ftqMutable->ForceXNoise().Mean(), 2.0);
    EXPECT_DOUBLE_EQ(
        sensor.ForceTorqueSensor()->ForceXNoise().Mean(), 2.0);
  }

  // IMU
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::FORCE_TORQUE);

    sdf::Imu imu;
    sensor.SetImuSensor(imu);

    sdf::Imu *imuMutable = sensor.ImuSensor();
    ASSERT_NE(nullptr, imuMutable);
    EXPECT_DOUBLE_EQ(imuMutable->LinearAccelerationXNoise().Mean(),
        sensor.ImuSensor()->LinearAccelerationXNoise().Mean());

    sdf::Noise noise;
    noise.SetMean(2.0);
    imuMutable->SetLinearAccelerationXNoise(noise);
    EXPECT_DOUBLE_EQ(imuMutable->LinearAccelerationXNoise().Mean(), 2.0);
    EXPECT_DOUBLE_EQ(
        sensor.ImuSensor()->LinearAccelerationXNoise().Mean(), 2.0);
  }

  // Lidar
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::LIDAR);

    sdf::Lidar ldr;
    sensor.SetLidarSensor(ldr);

    sdf::Lidar *ldrMutable = sensor.LidarSensor();
    ASSERT_NE(nullptr, ldrMutable);
    EXPECT_DOUBLE_EQ(ldrMutable->LidarNoise().Mean(),
        sensor.LidarSensor()->LidarNoise().Mean());

    sdf::Noise noise;
    noise.SetMean(2.0);
    ldrMutable->SetLidarNoise(noise);
    EXPECT_DOUBLE_EQ(ldrMutable->LidarNoise().Mean(), 2.0);
    EXPECT_DOUBLE_EQ(
        sensor.LidarSensor()->LidarNoise().Mean(), 2.0);
  }

  // Magnetometer
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::MAGNETOMETER);

    sdf::Magnetometer mag;
    sensor.SetMagnetometerSensor(mag);

    sdf::Magnetometer *magMutable = sensor.MagnetometerSensor();
    ASSERT_NE(nullptr, magMutable);
    EXPECT_DOUBLE_EQ(magMutable->XNoise().Mean(),
        sensor.MagnetometerSensor()->XNoise().Mean());

    sdf::Noise noise;
    noise.SetMean(2.0);
    magMutable->SetXNoise(noise);
    EXPECT_DOUBLE_EQ(magMutable->XNoise().Mean(), 2.0);
    EXPECT_DOUBLE_EQ(sensor.MagnetometerSensor()->XNoise().Mean(), 2.0);
  }

  // NavSat
  {
    sdf::Sensor sensor;
    sensor.SetType(sdf::SensorType::NAVSAT);

    sdf::NavSat nav;
    sensor.SetNavSatSensor(nav);

    sdf::NavSat *navMutable = sensor.NavSatSensor();
    ASSERT_NE(nullptr, navMutable);
    EXPECT_DOUBLE_EQ(navMutable->HorizontalPositionNoise().Mean(),
        sensor.NavSatSensor()->HorizontalPositionNoise().Mean());

    sdf::Noise noise;
    noise.SetMean(2.0);
    navMutable->SetHorizontalPositionNoise(noise);
    EXPECT_DOUBLE_EQ(navMutable->HorizontalPositionNoise().Mean(), 2.0);
    EXPECT_DOUBLE_EQ(
        sensor.NavSatSensor()->HorizontalPositionNoise().Mean(), 2.0);
  }
}

/////////////////////////////////////////////////
TEST(DOMSensor, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Sensor sensor;
  sensor.SetName("my_sensor");
  sensor.SetFrameId("my_sensor_frame_id");
  sensor.SetRawPose(gz::math::Pose3d(1, 2, 3, 0, 0, 0));
  sensor.SetType(sdf::SensorType::MAGNETOMETER);
  sensor.SetPoseRelativeTo("a_frame");
  sensor.SetUpdateRate(0.123);

  sdf::Noise noise;
  noise.SetMean(0.1);
  sdf::Magnetometer mag;
  mag.SetXNoise(noise);
  sensor.SetMagnetometerSensor(mag);

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");
  sensor.AddPlugin(plugin);

  sdf::ElementPtr sensorElem = sensor.ToElement();
  EXPECT_NE(nullptr, sensorElem);
  EXPECT_EQ(nullptr, sensor.Element());

  // verify values after loading the element back
  sdf::Sensor sensor2;
  sensor2.Load(sensorElem);

  EXPECT_EQ("my_sensor", sensor2.Name());
  EXPECT_EQ("my_sensor_frame_id", sensor2.FrameId());
  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, sensor2.Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), sensor2.RawPose());
  EXPECT_EQ("a_frame", sensor2.PoseRelativeTo());
  ASSERT_TRUE(nullptr != sensor2.MagnetometerSensor());
  EXPECT_DOUBLE_EQ(mag.XNoise().Mean(),
                   sensor2.MagnetometerSensor()->XNoise().Mean());
  EXPECT_DOUBLE_EQ(0.123, sensor2.UpdateRate());

  ASSERT_EQ(1u, sensor2.Plugins().size());
  EXPECT_EQ("name1", sensor2.Plugins()[0].Name());
  EXPECT_EQ("filename1", sensor2.Plugins()[0].Filename());

  // make changes to DOM and verify ToElement produces updated values
  sensor2.SetUpdateRate(1.23);
  sdf::ElementPtr sensor2Elem = sensor2.ToElement();
  EXPECT_NE(nullptr, sensor2Elem);
  sdf::Sensor sensor3;
  sensor3.Load(sensor2Elem);
  EXPECT_DOUBLE_EQ(1.23, sensor3.UpdateRate());
}

/////////////////////////////////////////////////
TEST(DOMSensor, Plugins)
{
  sdf::Sensor sensor;
  EXPECT_TRUE(sensor.Plugins().empty());

  sdf::Plugin plugin;
  plugin.SetName("name1");
  plugin.SetFilename("filename1");

  sensor.AddPlugin(plugin);
  ASSERT_EQ(1u, sensor.Plugins().size());

  plugin.SetName("name2");
  sensor.AddPlugin(plugin);
  ASSERT_EQ(2u, sensor.Plugins().size());

  EXPECT_EQ("name1", sensor.Plugins()[0].Name());
  EXPECT_EQ("name2", sensor.Plugins()[1].Name());

  sensor.ClearPlugins();
  EXPECT_TRUE(sensor.Plugins().empty());
}
