/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "sdf/AirFlow.hh"
#include "sdf/AirPressure.hh"
#include "sdf/AirSpeed.hh"
#include "sdf/Altimeter.hh"
#include "sdf/Camera.hh"
#include "sdf/Element.hh"
#include "sdf/ForceTorque.hh"
#include "sdf/Imu.hh"
#include "sdf/Joint.hh"
#include "sdf/Lidar.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Magnetometer.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/Sensor.hh"
#include "sdf/World.hh"
#include "test_config.hh"

//////////////////////////////////////////////////
TEST(SDFDomConversion, Sensors)
{
  // this test loads the sensors.sdf test file, then
  // 1) converts each type of sensor DOM to Element
  // 2) loads the Element back to DOM,
  // 3) verify the values
  // Most of the verification code is adapted from link_dom.cc
  const std::string testFile = sdf::testing::TestFile("sdf", "sensors.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_TRUE(errors.empty());

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  // altimeter
  {
    const sdf::Sensor *sensor = link->SensorByIndex(0);
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto altimeterSensor = std::make_unique<sdf::Sensor>();
    altimeterSensor->Load(sensorElem);

    // verify values
    ASSERT_NE(nullptr, altimeterSensor);
    EXPECT_EQ("altimeter_sensor", altimeterSensor->Name());
    EXPECT_EQ(sdf::SensorType::ALTIMETER, altimeterSensor->Type());
    EXPECT_EQ(gz::math::Pose3d::Zero, altimeterSensor->RawPose());
    EXPECT_FALSE(altimeterSensor->EnableMetrics());
    const sdf::Altimeter *altSensor = altimeterSensor->AltimeterSensor();
    ASSERT_NE(nullptr, altSensor);
    EXPECT_DOUBLE_EQ(0.1, altSensor->VerticalPositionNoise().Mean());
    EXPECT_DOUBLE_EQ(0.2, altSensor->VerticalPositionNoise().StdDev());
    EXPECT_DOUBLE_EQ(2.3, altSensor->VerticalVelocityNoise().Mean());
    EXPECT_DOUBLE_EQ(4.5, altSensor->VerticalVelocityNoise().StdDev());
  }
  {
    // Get the air_speed sensor
    const sdf::Sensor *sensor = link->SensorByName("air_speed_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto airSpeedSensor = std::make_unique<sdf::Sensor>();
    airSpeedSensor->Load(sensorElem);

    ASSERT_NE(nullptr, airSpeedSensor);
    EXPECT_EQ("air_speed_sensor", airSpeedSensor->Name());
    EXPECT_EQ(sdf::SensorType::AIR_SPEED, airSpeedSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(11, 22, 33, 0, 0, 0),
        airSpeedSensor->RawPose());
    EXPECT_FALSE(airSpeedSensor->EnableMetrics());
    const sdf::AirSpeed *airSpeedSensorSDF = airSpeedSensor->AirSpeedSensor();
    ASSERT_NE(nullptr, airSpeedSensorSDF);
    EXPECT_DOUBLE_EQ(0.0, airSpeedSensorSDF->PressureNoise().Mean());
    EXPECT_DOUBLE_EQ(0.01, airSpeedSensorSDF->PressureNoise().StdDev());
  }

  {
    // Get the air_flow sensor
    const sdf::Sensor *sensor = link->SensorByName("air_flow_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto airFlowSensor = std::make_unique<sdf::Sensor>();
    airFlowSensor->Load(sensorElem);

    ASSERT_NE(nullptr, airFlowSensor);
    EXPECT_EQ("air_flow_sensor", airFlowSensor->Name());
    EXPECT_EQ(sdf::SensorType::AIR_FLOW, airFlowSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(2, 14, 96, 0, 0, 0),
        airFlowSensor->RawPose());
    EXPECT_FALSE(airFlowSensor->EnableMetrics());
    const sdf::AirFlow *airFlowSensorSDF = airFlowSensor->AirFlowSensor();
    ASSERT_NE(nullptr, airFlowSensorSDF);
    EXPECT_DOUBLE_EQ(0.0, airFlowSensorSDF->SpeedNoise().Mean());
    EXPECT_DOUBLE_EQ(0.01, airFlowSensorSDF->SpeedNoise().StdDev());
    EXPECT_DOUBLE_EQ(0.0, airFlowSensorSDF->DirectionNoise().Mean());
    EXPECT_DOUBLE_EQ(0.02, airFlowSensorSDF->DirectionNoise().StdDev());

  }

  // camera
  {
    const sdf::Sensor *sensor = link->SensorByName("camera_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto cameraSensor = std::make_unique<sdf::Sensor>();
    cameraSensor->Load(sensorElem);

    ASSERT_NE(nullptr, cameraSensor);
    EXPECT_EQ("camera_sensor", cameraSensor->Name());
    EXPECT_EQ(sdf::SensorType::CAMERA, cameraSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0),
        cameraSensor->RawPose());
    EXPECT_FALSE(cameraSensor->EnableMetrics());
    const sdf::Camera *camSensor = cameraSensor->CameraSensor();
    ASSERT_NE(nullptr, camSensor);
    EXPECT_EQ("my_camera", camSensor->Name());
    EXPECT_EQ(gz::math::Pose3d(0.1, 0.2, 0.3, 0, 0, 0),
              camSensor->RawPose());
    EXPECT_DOUBLE_EQ(0.75, camSensor->HorizontalFov().Radian());
    EXPECT_EQ(640u, camSensor->ImageWidth());
    EXPECT_EQ(480u, camSensor->ImageHeight());
    EXPECT_EQ(sdf::PixelFormatType::RGB_INT8, camSensor->PixelFormat());
    EXPECT_DOUBLE_EQ(0.2, camSensor->NearClip());
    EXPECT_DOUBLE_EQ(12.3, camSensor->FarClip());
    EXPECT_TRUE(camSensor->SaveFrames());
    EXPECT_EQ("/tmp/cam", camSensor->SaveFramesPath());
    EXPECT_DOUBLE_EQ(0.5, camSensor->ImageNoise().Mean());
    EXPECT_DOUBLE_EQ(0.1, camSensor->ImageNoise().StdDev());
    EXPECT_DOUBLE_EQ(0.1, camSensor->DistortionK1());
    EXPECT_DOUBLE_EQ(0.2, camSensor->DistortionK2());
    EXPECT_DOUBLE_EQ(0.3, camSensor->DistortionK3());
    EXPECT_DOUBLE_EQ(0.4, camSensor->DistortionP1());
    EXPECT_DOUBLE_EQ(0.5, camSensor->DistortionP2());
    EXPECT_EQ(gz::math::Vector2d(0.2, 0.4),
        camSensor->DistortionCenter());
    EXPECT_EQ("custom", camSensor->LensType());
    EXPECT_FALSE(camSensor->LensScaleToHfov());
    EXPECT_DOUBLE_EQ(1.1, camSensor->LensC1());
    EXPECT_DOUBLE_EQ(2.2, camSensor->LensC2());
    EXPECT_DOUBLE_EQ(3.3, camSensor->LensC3());
    EXPECT_DOUBLE_EQ(1.2, camSensor->LensFocalLength());
    EXPECT_EQ("sin", camSensor->LensFunction());
    EXPECT_DOUBLE_EQ(0.7505, camSensor->LensCutoffAngle().Radian());
    EXPECT_EQ(128, camSensor->LensEnvironmentTextureSize());
    EXPECT_DOUBLE_EQ(280, camSensor->LensIntrinsicsFx());
    EXPECT_DOUBLE_EQ(281, camSensor->LensIntrinsicsFy());
    EXPECT_DOUBLE_EQ(162, camSensor->LensIntrinsicsCx());
    EXPECT_DOUBLE_EQ(124, camSensor->LensIntrinsicsCy());
    EXPECT_DOUBLE_EQ(1.2, camSensor->LensIntrinsicsSkew());
  }

  // depth
  {
    const sdf::Sensor *sensor = link->SensorByName("depth_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto depthSensor = std::make_unique<sdf::Sensor>();
    depthSensor->Load(sensorElem);

    ASSERT_NE(nullptr, depthSensor);
    EXPECT_EQ("depth_sensor", depthSensor->Name());
    EXPECT_EQ(sdf::SensorType::DEPTH_CAMERA, depthSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(7, 8, 9, 0, 0, 0), depthSensor->RawPose());
    EXPECT_TRUE(depthSensor->EnableMetrics());
    const sdf::Camera *depthCamSensor = depthSensor->CameraSensor();
    ASSERT_NE(nullptr, depthCamSensor);
    EXPECT_EQ("my_depth_camera", depthCamSensor->Name());
  }

  // rgbd
  {
    const sdf::Sensor *sensor = link->SensorByName("rgbd_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto rgbdSensor = std::make_unique<sdf::Sensor>();
    rgbdSensor->Load(sensorElem);

    ASSERT_NE(nullptr, rgbdSensor);
    EXPECT_EQ("rgbd_sensor", rgbdSensor->Name());
    EXPECT_EQ(sdf::SensorType::RGBD_CAMERA, rgbdSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(37, 38, 39, 0, 0, 0),
        rgbdSensor->RawPose());
    EXPECT_FALSE(rgbdSensor->EnableMetrics());
    const sdf::Camera *rgbdCamSensor = rgbdSensor->CameraSensor();
    ASSERT_NE(nullptr, rgbdCamSensor);
    EXPECT_EQ("my_rgbd_camera", rgbdCamSensor->Name());
  }

  // thermal
  {
    const sdf::Sensor *sensor = link->SensorByName("thermal_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto thermalSensor = std::make_unique<sdf::Sensor>();
    thermalSensor->Load(sensorElem);

  ASSERT_NE(nullptr, thermalSensor);
    EXPECT_EQ("thermal_sensor", thermalSensor->Name());
    EXPECT_EQ(sdf::SensorType::THERMAL_CAMERA, thermalSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(37, 38, 39, 0, 0, 0),
              thermalSensor->RawPose());
    EXPECT_FALSE(thermalSensor->EnableMetrics());
    const sdf::Camera *thermalCamSensor = thermalSensor->CameraSensor();
    ASSERT_NE(nullptr, thermalCamSensor);
    EXPECT_EQ("my_thermal_camera", thermalCamSensor->Name());
  }

  // segmentation
  {
    const sdf::Sensor *sensor = link->SensorByName("segmentation_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto segmentationSensor = std::make_unique<sdf::Sensor>();
    segmentationSensor->Load(sensorElem);

    ASSERT_NE(nullptr, segmentationSensor);
    EXPECT_EQ("segmentation_sensor", segmentationSensor->Name());
    EXPECT_EQ(sdf::SensorType::SEGMENTATION_CAMERA, segmentationSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(37, 38, 39, 0, 0, 0),
              segmentationSensor->RawPose());
    const sdf::Camera *segmentationCameraSensor =
      segmentationSensor->CameraSensor();
    ASSERT_NE(nullptr, segmentationCameraSensor);
    EXPECT_EQ("my_segmentation_camera", segmentationCameraSensor->Name());
    EXPECT_TRUE(segmentationCameraSensor->HasSegmentationType());
    EXPECT_EQ("semantic", segmentationCameraSensor->SegmentationType());
  }

  // force torque
  {
    const sdf::Sensor *sensor = link->SensorByName("force_torque_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto forceTorqueSensor = std::make_unique<sdf::Sensor>();
    forceTorqueSensor->Load(sensorElem);

    ASSERT_NE(nullptr, forceTorqueSensor);
    EXPECT_EQ("force_torque_sensor", forceTorqueSensor->Name());
    EXPECT_EQ(sdf::SensorType::FORCE_TORQUE, forceTorqueSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
        forceTorqueSensor->RawPose());
    EXPECT_FALSE(forceTorqueSensor->EnableMetrics());
  }

  // gpu lidar
  {
    const sdf::Sensor *sensor = link->SensorByName("gpu_lidar_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto gpuLidarSensor = std::make_unique<sdf::Sensor>();
    gpuLidarSensor->Load(sensorElem);

    ASSERT_NE(nullptr, gpuLidarSensor);
    EXPECT_EQ("gpu_lidar_sensor", gpuLidarSensor->Name());
    EXPECT_EQ(sdf::SensorType::GPU_LIDAR, gpuLidarSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0),
              gpuLidarSensor->RawPose());
    EXPECT_FALSE(gpuLidarSensor->EnableMetrics());
    const sdf::Lidar *gpuLidar = gpuLidarSensor->LidarSensor();
    ASSERT_NE(nullptr, gpuLidar);
    EXPECT_EQ(640u, gpuLidar->HorizontalScanSamples());
    EXPECT_DOUBLE_EQ(1u, gpuLidar->HorizontalScanResolution());
    EXPECT_EQ(gz::math::Angle(-1.396263),
        gpuLidar->HorizontalScanMinAngle());
    EXPECT_EQ(gz::math::Angle(1.396263),
        gpuLidar->HorizontalScanMaxAngle());
    EXPECT_EQ(1u, gpuLidar->VerticalScanSamples());
    EXPECT_DOUBLE_EQ(0.01, gpuLidar->VerticalScanResolution());
    EXPECT_EQ(gz::math::Angle(0.1), gpuLidar->VerticalScanMinAngle());
    EXPECT_EQ(gz::math::Angle(0.2), gpuLidar->VerticalScanMaxAngle());

    EXPECT_DOUBLE_EQ(0.08, gpuLidar->RangeMin());
    EXPECT_DOUBLE_EQ(10.0, gpuLidar->RangeMax());
    EXPECT_DOUBLE_EQ(0.01, gpuLidar->RangeResolution());
  }

  // imu
  {
    const sdf::Sensor *sensor = link->SensorByName("imu_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto imuSensor = std::make_unique<sdf::Sensor>();
    imuSensor->Load(sensorElem);

    ASSERT_NE(nullptr, imuSensor);
    EXPECT_EQ("imu_sensor", imuSensor->Name());
    EXPECT_EQ(sdf::SensorType::IMU, imuSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 0), imuSensor->RawPose());
    EXPECT_FALSE(imuSensor->EnableMetrics());
    const sdf::Imu *imuSensorObj = imuSensor->ImuSensor();
    ASSERT_NE(nullptr, imuSensorObj);

    EXPECT_DOUBLE_EQ(0.0, imuSensorObj->LinearAccelerationXNoise().Mean());
    EXPECT_DOUBLE_EQ(0.1, imuSensorObj->LinearAccelerationXNoise().StdDev());
    EXPECT_DOUBLE_EQ(0.2,
        imuSensorObj->LinearAccelerationXNoise().DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(1.0,
        imuSensorObj->LinearAccelerationXNoise().DynamicBiasCorrelationTime());

    EXPECT_DOUBLE_EQ(1.0, imuSensorObj->LinearAccelerationYNoise().Mean());
    EXPECT_DOUBLE_EQ(1.1, imuSensorObj->LinearAccelerationYNoise().StdDev());
    EXPECT_DOUBLE_EQ(1.2,
        imuSensorObj->LinearAccelerationYNoise().DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(2.0,
        imuSensorObj->LinearAccelerationYNoise().DynamicBiasCorrelationTime());

    EXPECT_DOUBLE_EQ(2.0, imuSensorObj->LinearAccelerationZNoise().Mean());
    EXPECT_DOUBLE_EQ(2.1, imuSensorObj->LinearAccelerationZNoise().StdDev());
    EXPECT_DOUBLE_EQ(2.2,
        imuSensorObj->LinearAccelerationZNoise().DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(3.0,
        imuSensorObj->LinearAccelerationZNoise().DynamicBiasCorrelationTime());

    EXPECT_DOUBLE_EQ(3.0, imuSensorObj->AngularVelocityXNoise().Mean());
    EXPECT_DOUBLE_EQ(3.1, imuSensorObj->AngularVelocityXNoise().StdDev());
    EXPECT_DOUBLE_EQ(4.2,
        imuSensorObj->AngularVelocityXNoise().DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(4.0,
        imuSensorObj->AngularVelocityXNoise().DynamicBiasCorrelationTime());

    EXPECT_DOUBLE_EQ(4.0, imuSensorObj->AngularVelocityYNoise().Mean());
    EXPECT_DOUBLE_EQ(4.1, imuSensorObj->AngularVelocityYNoise().StdDev());
    EXPECT_DOUBLE_EQ(5.2,
        imuSensorObj->AngularVelocityYNoise().DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(5.0,
        imuSensorObj->AngularVelocityYNoise().DynamicBiasCorrelationTime());

    EXPECT_DOUBLE_EQ(5.0, imuSensorObj->AngularVelocityZNoise().Mean());
    EXPECT_DOUBLE_EQ(5.1, imuSensorObj->AngularVelocityZNoise().StdDev());
    EXPECT_DOUBLE_EQ(6.2,
        imuSensorObj->AngularVelocityZNoise().DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(6.0,
        imuSensorObj->AngularVelocityZNoise().DynamicBiasCorrelationTime());

    EXPECT_EQ("ENU", imuSensorObj->Localization());
    EXPECT_EQ("linka", imuSensorObj->CustomRpyParentFrame());
    EXPECT_EQ(gz::math::Vector3d::UnitY, imuSensorObj->CustomRpy());
    EXPECT_EQ("linkb", imuSensorObj->GravityDirXParentFrame());
    EXPECT_EQ(gz::math::Vector3d::UnitZ, imuSensorObj->GravityDirX());

    EXPECT_FALSE(imuSensorObj->OrientationEnabled());
  }

  // magnetometer
  {
    const sdf::Sensor *sensor = link->SensorByName("magnetometer_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto magnetometerSensor = std::make_unique<sdf::Sensor>();
    magnetometerSensor->Load(sensorElem);

    ASSERT_NE(nullptr, magnetometerSensor);
    EXPECT_EQ("magnetometer_sensor", magnetometerSensor->Name());
    EXPECT_EQ(sdf::SensorType::MAGNETOMETER, magnetometerSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
        magnetometerSensor->RawPose());
    EXPECT_FALSE(magnetometerSensor->EnableMetrics());
    const sdf::Magnetometer *magSensor =
        magnetometerSensor->MagnetometerSensor();
    ASSERT_NE(nullptr, magSensor);
    EXPECT_DOUBLE_EQ(0.1, magSensor->XNoise().Mean());
    EXPECT_DOUBLE_EQ(0.2, magSensor->XNoise().StdDev());
    EXPECT_DOUBLE_EQ(1.2, magSensor->YNoise().Mean());
    EXPECT_DOUBLE_EQ(2.3, magSensor->YNoise().StdDev());
    EXPECT_DOUBLE_EQ(3.4, magSensor->ZNoise().Mean());
    EXPECT_DOUBLE_EQ(5.6, magSensor->ZNoise().StdDev());
  }

  // air pressure
  {
    const sdf::Sensor *sensor = link->SensorByName("air_pressure_sensor");
    // convert to sdf element and load it back
    sdf::ElementPtr sensorElem = sensor->ToElement();
    auto airPressureSensor = std::make_unique<sdf::Sensor>();
    airPressureSensor->Load(sensorElem);

    ASSERT_NE(nullptr, airPressureSensor);
    EXPECT_EQ("air_pressure_sensor", airPressureSensor->Name());
    EXPECT_EQ(sdf::SensorType::AIR_PRESSURE, airPressureSensor->Type());
    EXPECT_EQ(gz::math::Pose3d(10, 20, 30, 0, 0, 0),
        airPressureSensor->RawPose());
    EXPECT_FALSE(airPressureSensor->EnableMetrics());
    const sdf::AirPressure *airSensor = airPressureSensor->AirPressureSensor();
    ASSERT_NE(nullptr, airSensor);
    EXPECT_DOUBLE_EQ(3.4, airSensor->PressureNoise().Mean());
    EXPECT_DOUBLE_EQ(5.6, airSensor->PressureNoise().StdDev());
    EXPECT_DOUBLE_EQ(123.4, airSensor->ReferenceAltitude());
  }
}

//////////////////////////////////////////////////
TEST(SDFDomConversion, Lights)
{
  // this test loads the lights.sdf test file, then
  // 1) converts Light DOM to Element
  // 2) loads the Element back to DOM,
  // 3) verify the values
  const std::string testFile = sdf::testing::TestFile("sdf", "lights.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_TRUE(errors.empty());

  // Get the world
  const sdf::World *world = root.WorldByIndex(0u);
  ASSERT_NE(nullptr, world);

  // Get lights
  // point
  {
    const sdf::Light *light = world->LightByIndex(0);
    sdf::ElementPtr lightElem = light->ToElement();
    auto pointLight = std::make_unique<sdf::Light>();
    pointLight->Load(lightElem);
    ASSERT_NE(nullptr, pointLight);
    EXPECT_EQ("point_light", pointLight->Name());
    EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());
    EXPECT_EQ(gz::math::Pose3d(10, 2, 100, 0, 0, 0),
        pointLight->RawPose());
    EXPECT_FALSE(pointLight->CastShadows());
    EXPECT_EQ(gz::math::Color(0.0f, 0.1f, 0.8f, 1.0f),
        pointLight->Diffuse());
    EXPECT_EQ(gz::math::Color(0.03f, 0.02f, 0.0f, 0.5f),
        pointLight->Specular());
    EXPECT_DOUBLE_EQ(1234.0, pointLight->AttenuationRange());
    // value should be clamped to [0, 1]
    EXPECT_DOUBLE_EQ(1.0, pointLight->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.01, pointLight->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(11.2, pointLight->QuadraticAttenuationFactor());
  }

  // spot
  {
    const sdf::Light *light = world->LightByIndex(1u);
    sdf::ElementPtr lightElem = light->ToElement();
    auto spotLight = std::make_unique<sdf::Light>();
    spotLight->Load(lightElem);
    ASSERT_NE(nullptr, spotLight);
    EXPECT_EQ("spot_light", spotLight->Name());
    EXPECT_EQ(sdf::LightType::SPOT, spotLight->Type());
    EXPECT_EQ(gz::math::Pose3d::Zero,
        spotLight->RawPose());
    EXPECT_TRUE(spotLight->CastShadows());
    EXPECT_DOUBLE_EQ(12.34, spotLight->AttenuationRange());
    EXPECT_DOUBLE_EQ(0.01, spotLight->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.02, spotLight->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.0001, spotLight->QuadraticAttenuationFactor());
    EXPECT_EQ(gz::math::Color(0.0f, 0.05f, 0.06f, 1.0f),
        spotLight->Diffuse());
    EXPECT_EQ(gz::math::Color(0.05f, 0.04f, 0.1f, 1.0f),
        spotLight->Specular());
    EXPECT_EQ(gz::math::Vector3d(1.0, 5.1, 2.1), spotLight->Direction());
  }

  // directional
  {
    const sdf::Light *light = world->LightByIndex(2u);
    sdf::ElementPtr lightElem = light->ToElement();
    auto dirLight = std::make_unique<sdf::Light>();
    dirLight->Load(lightElem);
    ASSERT_NE(nullptr, dirLight);
    EXPECT_EQ("directional_light", dirLight->Name());
    EXPECT_EQ(sdf::LightType::DIRECTIONAL, dirLight->Type());
    EXPECT_EQ(gz::math::Pose3d(1, 1, 2, 0, 0, 0),
        dirLight->RawPose());
    EXPECT_TRUE(dirLight->CastShadows());
    EXPECT_EQ(gz::math::Color(0.0f, 0.0f, 0.3f, 1.0f),
        dirLight->Diffuse());
    EXPECT_EQ(gz::math::Color(0.0f, 0.9f, 0.11f, 1.0f),
        dirLight->Specular());
    EXPECT_EQ(gz::math::Vector3d(5.0, 5.2, 6.0), dirLight->Direction());
  }

  // Get model with lights attached to its link
  const sdf::Model *model = world->ModelByName("model_lights");
  ASSERT_NE(nullptr, model);

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  // spot
  {
    const sdf::Light *light = link->LightByName("link_spot_light");
    sdf::ElementPtr lightElem = light->ToElement();
    auto spotLight = std::make_unique<sdf::Light>();
    spotLight->Load(lightElem);
    ASSERT_NE(nullptr, spotLight);
    EXPECT_EQ("link_spot_light", spotLight->Name());
    EXPECT_EQ(sdf::LightType::SPOT, spotLight->Type());
    EXPECT_EQ(gz::math::Pose3d::Zero,
        spotLight->RawPose());
    EXPECT_TRUE(spotLight->CastShadows());
    EXPECT_DOUBLE_EQ(12.35, spotLight->AttenuationRange());
    EXPECT_DOUBLE_EQ(0.1, spotLight->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.2, spotLight->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.001, spotLight->QuadraticAttenuationFactor());
    EXPECT_EQ(gz::math::Color(0.0f, 0.05f, 0.06f, 1.0f),
        spotLight->Diffuse());
    EXPECT_EQ(gz::math::Color(0.03f, 0.02f, 0.0f, 1.0f),
        spotLight->Specular());
    EXPECT_EQ(gz::math::Vector3d(1.0, 5.0, 2.0), spotLight->Direction());
  }

  // point
  {
    const sdf::Light *light = link->LightByName("link_point_light");
    sdf::ElementPtr lightElem = light->ToElement();
    auto pointLight = std::make_unique<sdf::Light>();
    pointLight->Load(lightElem);
    ASSERT_NE(nullptr, pointLight);
    EXPECT_EQ("link_point_light", pointLight->Name());
    EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());
    EXPECT_EQ(gz::math::Pose3d(10, 20, 100, 0, 0, 0),
        pointLight->RawPose());
    EXPECT_FALSE(pointLight->CastShadows());
    EXPECT_EQ(gz::math::Color(1.0f, 0.0f, 0.6f, 1.0f),
        pointLight->Diffuse());
    EXPECT_EQ(gz::math::Color(0.3f, 0.2f, 0.0f, 1.0f),
        pointLight->Specular());
    EXPECT_DOUBLE_EQ(1235.0, pointLight->AttenuationRange());
    EXPECT_DOUBLE_EQ(1.0, pointLight->LinearAttenuationFactor());
    // negative value should be clamped to 0
    EXPECT_DOUBLE_EQ(0.0, pointLight->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(10.2, pointLight->QuadraticAttenuationFactor());
  }

  // directional
  {
    const sdf::Light *light = link->LightByName("link_directional_light");
    sdf::ElementPtr lightElem = light->ToElement();
    auto dirLight = std::make_unique<sdf::Light>();
    dirLight->Load(lightElem);
    ASSERT_NE(nullptr, dirLight);
    EXPECT_EQ("link_directional_light", dirLight->Name());
    EXPECT_EQ(sdf::LightType::DIRECTIONAL, dirLight->Type());
    EXPECT_EQ(gz::math::Pose3d(0, 1, 2, 0, 0, 0),
        dirLight->RawPose());
    EXPECT_TRUE(dirLight->CastShadows());
    EXPECT_EQ(gz::math::Color(0.0f, 1.0f, 0.2f, 1.0f),
        dirLight->Diffuse());
    EXPECT_EQ(gz::math::Color(0.0f, 0.2f, 0.1f, 1.0f),
        dirLight->Specular());
    EXPECT_EQ(gz::math::Vector3d(4.0, 5.0, 6.0), dirLight->Direction());
  }
}

//////////////////////////////////////////////////
TEST(SDFDomConversion, Joints)
{
  // this test loads the joint_sensors.sdf test file, then
  // 1) converts each type of sensor DOM to Element
  // 2) loads the Element back to DOM,
  // 3) verify the values
  // Some of the verification code is adapted from joint_dom.cc
  const std::string testFile =
      sdf::testing::TestFile("sdf", "joint_sensors.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_TRUE(errors.empty());

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  // Get joint
  const sdf::Joint *j = model->JointByName("joint");
  ASSERT_NE(nullptr, j);

  // convert to sdf element and load it back
  sdf::ElementPtr jointElem = j->ToElement();
  auto joint = std::make_unique<sdf::Joint>();
  joint->Load(jointElem);
  ASSERT_NE(nullptr, joint);
  EXPECT_EQ("joint", joint->Name());
  EXPECT_EQ(1u, joint->SensorCount());

  EXPECT_EQ("link1", joint->ParentName());
  EXPECT_EQ("link2", joint->ChildName());
  EXPECT_EQ(sdf::JointType::FIXED, joint->Type());

  // Get the force_torque sensor
  const sdf::Sensor *forceTorqueSensor =
    joint->SensorByName("force_torque_sensor");
  ASSERT_NE(nullptr, forceTorqueSensor);
  EXPECT_EQ("force_torque_sensor", forceTorqueSensor->Name());
  EXPECT_EQ(sdf::SensorType::FORCE_TORQUE, forceTorqueSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
      forceTorqueSensor->RawPose());
  auto forceTorqueSensorObj = forceTorqueSensor->ForceTorqueSensor();
  ASSERT_NE(nullptr, forceTorqueSensorObj);
  EXPECT_EQ(sdf::ForceTorqueFrame::PARENT, forceTorqueSensorObj->Frame());
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD,
      forceTorqueSensorObj->MeasureDirection());

  EXPECT_DOUBLE_EQ(0.0, forceTorqueSensorObj->ForceXNoise().Mean());
  EXPECT_DOUBLE_EQ(0.1, forceTorqueSensorObj->ForceXNoise().StdDev());
  EXPECT_DOUBLE_EQ(1.0, forceTorqueSensorObj->ForceYNoise().Mean());
  EXPECT_DOUBLE_EQ(1.1, forceTorqueSensorObj->ForceYNoise().StdDev());
  EXPECT_DOUBLE_EQ(2.0, forceTorqueSensorObj->ForceZNoise().Mean());
  EXPECT_DOUBLE_EQ(2.1, forceTorqueSensorObj->ForceZNoise().StdDev());

  EXPECT_DOUBLE_EQ(3.0, forceTorqueSensorObj->TorqueXNoise().Mean());
  EXPECT_DOUBLE_EQ(3.1, forceTorqueSensorObj->TorqueXNoise().StdDev());
  EXPECT_DOUBLE_EQ(4.0, forceTorqueSensorObj->TorqueYNoise().Mean());
  EXPECT_DOUBLE_EQ(4.1, forceTorqueSensorObj->TorqueYNoise().StdDev());
  EXPECT_DOUBLE_EQ(5.0, forceTorqueSensorObj->TorqueZNoise().Mean());
  EXPECT_DOUBLE_EQ(5.1, forceTorqueSensorObj->TorqueZNoise().StdDev());
}
