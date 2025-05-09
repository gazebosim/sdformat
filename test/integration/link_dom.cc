/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <sstream>
#include <string>
#include <gtest/gtest.h>

#include <gz/math/Pose3.hh>
#include "sdf/AirPressure.hh"
#include "sdf/AirSpeed.hh"
#include "sdf/Altimeter.hh"
#include "sdf/Camera.hh"
#include "sdf/Collision.hh"
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Imu.hh"
#include "sdf/NavSat.hh"
#include "sdf/Link.hh"
#include "sdf/Magnetometer.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Lidar.hh"
#include "sdf/Root.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.hh"

//////////////////////////////////////////////////
TEST(DOMLink, NotALink)
{
  // Create an Element that is not a link
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Link link;
  sdf::Errors errors = link.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Link") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMLink, NoName)
{
  // Create a "link" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("link");

  sdf::Link link;
  sdf::Errors errors = link.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_TRUE(errors[0].Message().find("link name is required") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMLink, LoadVisualCollision)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "empty.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("default", world->Name());

  // Get the first model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("ground_plane", model->Name());

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("link", link->Name());

  // Get the first visual
  EXPECT_EQ(1u, link->VisualCount());
  EXPECT_TRUE(link->VisualNameExists("visual"));
  EXPECT_FALSE(link->VisualNameExists("visuals"));
  const sdf::Visual *visual = link->VisualByIndex(0);
  ASSERT_NE(nullptr, visual);
  EXPECT_EQ("visual", visual->Name());

  // Get the first collision
  EXPECT_EQ(1u, link->CollisionCount());
  EXPECT_TRUE(link->CollisionNameExists("collision"));
  EXPECT_FALSE(link->CollisionNameExists("collisions"));
  const sdf::Collision *collision = link->CollisionByIndex(0);
  ASSERT_NE(nullptr, collision);
  EXPECT_EQ("collision", collision->Name());
}

//////////////////////////////////////////////////
TEST(DOMLink, InertialDoublePendulum)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  const sdf::Link *baseLink = model->LinkByIndex(0);
  ASSERT_NE(nullptr, baseLink);
  EXPECT_EQ(gz::math::Pose3d::Zero, baseLink->RawPose());
  EXPECT_EQ("", baseLink->PoseRelativeTo());

  const gz::math::Inertiald inertial = baseLink->Inertial();
  EXPECT_DOUBLE_EQ(100.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_FALSE(inertial.FluidAddedMass().has_value());

  const sdf::Link *upperLink = model->LinkByIndex(1);
  ASSERT_NE(nullptr, upperLink);
  EXPECT_EQ(gz::math::Pose3d(0, 0, 2.1, -1.5708, 0, 0),
      upperLink->RawPose());
  EXPECT_EQ("", upperLink->PoseRelativeTo());
  EXPECT_TRUE(upperLink->EnableWind());
  EXPECT_TRUE(upperLink->Kinematic());

  const gz::math::Inertiald inertialUpper = upperLink->Inertial();
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertialUpper.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.Pose().Pos().X());
  EXPECT_DOUBLE_EQ(0.0, inertialUpper.Pose().Pos().Y());
  EXPECT_DOUBLE_EQ(0.5, inertialUpper.Pose().Pos().Z());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());
  EXPECT_FALSE(inertial.FluidAddedMass().has_value());

  const sdf::Link *lowerLink = model->LinkByIndex(2);
  ASSERT_TRUE(lowerLink != nullptr);
  EXPECT_EQ(gz::math::Pose3d(0.25, 1.0, 2.1, -2, 0, 0),
      lowerLink->RawPose());
  EXPECT_EQ("", lowerLink->PoseRelativeTo());
}

//////////////////////////////////////////////////
TEST(DOMLink, InertialComplete)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "inertial_complete.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const gz::math::Inertiald inertial = link->Inertial();
  EXPECT_DOUBLE_EQ(17.982, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(0.125569, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0972062, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.117937, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0008, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(-0.000499757,
      inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(-0.0005, inertial.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.01, inertial.Pose().Pos().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.Pose().Pos().Y());
  EXPECT_DOUBLE_EQ(0.02, inertial.Pose().Pos().Z());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  ASSERT_TRUE(inertial.FluidAddedMass().has_value());
  auto addedMass = inertial.FluidAddedMass().value();
  EXPECT_DOUBLE_EQ(11, addedMass(0, 0));
  EXPECT_DOUBLE_EQ(12, addedMass(0, 1));
  EXPECT_DOUBLE_EQ(13, addedMass(0, 2));
  EXPECT_DOUBLE_EQ(14, addedMass(0, 3));
  EXPECT_DOUBLE_EQ(15, addedMass(0, 4));
  EXPECT_DOUBLE_EQ(16, addedMass(0, 5));
  EXPECT_DOUBLE_EQ(12, addedMass(1, 0));
  EXPECT_DOUBLE_EQ(22, addedMass(1, 1));
  EXPECT_DOUBLE_EQ(23, addedMass(1, 2));
  EXPECT_DOUBLE_EQ(24, addedMass(1, 3));
  EXPECT_DOUBLE_EQ(25, addedMass(1, 4));
  EXPECT_DOUBLE_EQ(26, addedMass(1, 5));
  EXPECT_DOUBLE_EQ(13, addedMass(2, 0));
  EXPECT_DOUBLE_EQ(23, addedMass(2, 1));
  EXPECT_DOUBLE_EQ(33, addedMass(2, 2));
  EXPECT_DOUBLE_EQ(34, addedMass(2, 3));
  EXPECT_DOUBLE_EQ(35, addedMass(2, 4));
  EXPECT_DOUBLE_EQ(36, addedMass(2, 5));
  EXPECT_DOUBLE_EQ(14, addedMass(3, 0));
  EXPECT_DOUBLE_EQ(24, addedMass(3, 1));
  EXPECT_DOUBLE_EQ(34, addedMass(3, 2));
  EXPECT_DOUBLE_EQ(44, addedMass(3, 3));
  EXPECT_DOUBLE_EQ(45, addedMass(3, 4));
  EXPECT_DOUBLE_EQ(46, addedMass(3, 5));
  EXPECT_DOUBLE_EQ(15, addedMass(4, 0));
  EXPECT_DOUBLE_EQ(25, addedMass(4, 1));
  EXPECT_DOUBLE_EQ(35, addedMass(4, 2));
  EXPECT_DOUBLE_EQ(45, addedMass(4, 3));
  EXPECT_DOUBLE_EQ(55, addedMass(4, 4));
  EXPECT_DOUBLE_EQ(56, addedMass(4, 5));
  EXPECT_DOUBLE_EQ(16, addedMass(5, 0));
  EXPECT_DOUBLE_EQ(26, addedMass(5, 1));
  EXPECT_DOUBLE_EQ(36, addedMass(5, 2));
  EXPECT_DOUBLE_EQ(46, addedMass(5, 3));
  EXPECT_DOUBLE_EQ(56, addedMass(5, 4));
  EXPECT_DOUBLE_EQ(66, addedMass(5, 5));
}

//////////////////////////////////////////////////
TEST(DOMLink, InertialInvalid)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "inertial_invalid.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  ASSERT_EQ(1u, errors.size());
  for (auto err : errors)
     std::cout << err.Message() << std::endl;
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::LINK_INERTIA_INVALID);
  EXPECT_EQ(errors[0].Message(), "A link named link has invalid inertia.");

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  ASSERT_EQ(1u, model->LinkCount());
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("link", link->Name());
}

//////////////////////////////////////////////////
TEST(DOMLink, Sensors)
{
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
  EXPECT_EQ("model", model->Name());

  // Get the first link
  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("link", link->Name());
  EXPECT_EQ(27u, link->SensorCount());

  // Get the altimeter sensor
  const sdf::Sensor *altimeterSensor = link->SensorByIndex(0);
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

  // Get the camera sensor
  EXPECT_TRUE(link->SensorNameExists("camera_sensor"));
  EXPECT_FALSE(link->SensorNameExists("bad_camera_sensor"));
  const sdf::Sensor *cameraSensor = link->SensorByName("camera_sensor");
  ASSERT_NE(nullptr, cameraSensor);
  EXPECT_EQ("camera_sensor", cameraSensor->Name());
  EXPECT_EQ(sdf::SensorType::CAMERA, cameraSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), cameraSensor->RawPose());
  EXPECT_FALSE(cameraSensor->EnableMetrics());
  const sdf::Camera *camSensor = cameraSensor->CameraSensor();
  ASSERT_NE(nullptr, camSensor);
  EXPECT_EQ("my_camera", camSensor->Name());
  EXPECT_EQ(gz::math::Pose3d(0.1, 0.2, 0.3, 0, 0, 0),
            camSensor->RawPose());
  EXPECT_DOUBLE_EQ(0.75, camSensor->HorizontalFov().Radian());
  EXPECT_EQ("", cameraSensor->Topic());
  EXPECT_EQ("my_camera/trigger", camSensor->TriggerTopic());
  EXPECT_TRUE(camSensor->Triggered());
  EXPECT_EQ("/camera_sensor/camera_info", camSensor->CameraInfoTopic());
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
  EXPECT_EQ(gz::math::Vector2d(0.2, 0.4), camSensor->DistortionCenter());
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
  EXPECT_DOUBLE_EQ(280, camSensor->LensProjectionFx());
  EXPECT_DOUBLE_EQ(281, camSensor->LensProjectionFy());
  EXPECT_DOUBLE_EQ(162, camSensor->LensProjectionCx());
  EXPECT_DOUBLE_EQ(124, camSensor->LensProjectionCy());
  EXPECT_DOUBLE_EQ(0, camSensor->LensProjectionTx());
  EXPECT_DOUBLE_EQ(0, camSensor->LensProjectionTy());

  gz::math::Pose3d pose;

  // Get the contact sensor
  const sdf::Sensor *contactSensor = link->SensorByName("contact_sensor");
  ASSERT_NE(nullptr, contactSensor);
  EXPECT_EQ("contact_sensor", contactSensor->Name());
  EXPECT_EQ(sdf::SensorType::CONTACT, contactSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 0), contactSensor->RawPose());
  EXPECT_TRUE(contactSensor->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 0), pose);
  EXPECT_TRUE(contactSensor->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(gz::math::Pose3d(4, 5, 3, 0, 0, 0), pose);
  EXPECT_TRUE(contactSensor->EnableMetrics());

  // Get the custom sensor
  const sdf::Sensor *customSensor = link->SensorByName("custom_sensor");
  ASSERT_NE(nullptr, customSensor);
  EXPECT_EQ("custom_sensor", customSensor->Name());
  EXPECT_EQ(sdf::SensorType::CUSTOM, customSensor->Type());
  ASSERT_NE(nullptr, customSensor->Element());
  auto myNamespaceElem = customSensor->Element()->GetElement(
      "my_namespace:custom_sensor");
  ASSERT_NE(nullptr, myNamespaceElem);
  auto someParamElem = myNamespaceElem->GetElement("some_param");
  ASSERT_NE(nullptr, someParamElem);
  EXPECT_EQ(123, someParamElem->Get<int>());

  // Get the depth sensor
  const sdf::Sensor *depthSensor = link->SensorByName("depth_sensor");
  ASSERT_NE(nullptr, depthSensor);
  EXPECT_EQ("depth_sensor", depthSensor->Name());
  EXPECT_EQ(sdf::SensorType::DEPTH_CAMERA, depthSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(7, 8, 9, 0, 0, 0), depthSensor->RawPose());
  EXPECT_TRUE(depthSensor->EnableMetrics());
  const sdf::Camera *depthCamSensor = depthSensor->CameraSensor();
  ASSERT_NE(nullptr, depthCamSensor);
  EXPECT_EQ("my_depth_camera", depthCamSensor->Name());

  // Get the rgbd sensor
  const sdf::Sensor *rgbdSensor = link->SensorByName("rgbd_sensor");
  ASSERT_NE(nullptr, rgbdSensor);
  EXPECT_EQ("rgbd_sensor", rgbdSensor->Name());
  EXPECT_EQ(sdf::SensorType::RGBD_CAMERA, rgbdSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(37, 38, 39, 0, 0, 0), rgbdSensor->RawPose());
  EXPECT_FALSE(rgbdSensor->EnableMetrics());
  const sdf::Camera *rgbdCamSensor = rgbdSensor->CameraSensor();
  ASSERT_NE(nullptr, rgbdCamSensor);
  EXPECT_EQ("my_rgbd_camera", rgbdCamSensor->Name());
  EXPECT_EQ("", rgbdSensor->Topic());
  EXPECT_EQ("", rgbdCamSensor->TriggerTopic());
  EXPECT_TRUE(rgbdCamSensor->Triggered());

  // Get the thermal sensor
  const sdf::Sensor *thermalSensor = link->SensorByName("thermal_sensor");
  ASSERT_NE(nullptr, thermalSensor);
  EXPECT_EQ("thermal_sensor", thermalSensor->Name());
  EXPECT_EQ(sdf::SensorType::THERMAL_CAMERA, thermalSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(37, 38, 39, 0, 0, 0),
            thermalSensor->RawPose());
  EXPECT_FALSE(thermalSensor->EnableMetrics());
  const sdf::Camera *thermalCamSensor = thermalSensor->CameraSensor();
  ASSERT_NE(nullptr, thermalCamSensor);
  EXPECT_EQ("my_thermal_camera", thermalCamSensor->Name());

  // Get the segmentation sensor
  const sdf::Sensor *segmentationSensor =
    link->SensorByName("segmentation_sensor");
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

  // Get the boundingbox sensor
  const sdf::Sensor *boundingboxSensor =
    link->SensorByName("boundingbox_sensor");
  ASSERT_NE(nullptr, boundingboxSensor);
  EXPECT_EQ("boundingbox_sensor", boundingboxSensor->Name());
  EXPECT_EQ(sdf::SensorType::BOUNDINGBOX_CAMERA, boundingboxSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(37, 38, 39, 0, 0, 0),
            boundingboxSensor->RawPose());
  const sdf::Camera *boundingboxCamSensor = boundingboxSensor->CameraSensor();
  ASSERT_NE(nullptr, boundingboxCamSensor);
  EXPECT_EQ("my_boundingbox_camera", boundingboxCamSensor->Name());
  EXPECT_TRUE(boundingboxCamSensor->HasBoundingBoxType());
  EXPECT_EQ("2d", boundingboxCamSensor->BoundingBoxType());

  // Get the force_torque sensor
  const sdf::Sensor *forceTorqueSensor =
    link->SensorByName("force_torque_sensor");
  ASSERT_NE(nullptr, forceTorqueSensor);
  EXPECT_EQ("force_torque_sensor", forceTorqueSensor->Name());
  EXPECT_EQ(sdf::SensorType::FORCE_TORQUE, forceTorqueSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
      forceTorqueSensor->RawPose());
  EXPECT_FALSE(forceTorqueSensor->EnableMetrics());

  // Get the navsat sensor
  const sdf::Sensor *navSatSensor = link->SensorByName("navsat_sensor");
  ASSERT_NE(nullptr, navSatSensor);
  EXPECT_EQ("navsat_sensor", navSatSensor->Name());
  EXPECT_EQ(sdf::SensorType::NAVSAT, navSatSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(13, 14, 15, 0, 0, 0),
      navSatSensor->RawPose());
  EXPECT_FALSE(navSatSensor->EnableMetrics());
  const sdf::NavSat *navSatSensorObj = navSatSensor->NavSatSensor();
  ASSERT_NE(nullptr, navSatSensorObj);

  EXPECT_DOUBLE_EQ(1.2, navSatSensorObj->HorizontalPositionNoise().Mean());
  EXPECT_DOUBLE_EQ(3.4, navSatSensorObj->HorizontalPositionNoise().StdDev());
  EXPECT_DOUBLE_EQ(5.6, navSatSensorObj->VerticalPositionNoise().Mean());
  EXPECT_DOUBLE_EQ(7.8, navSatSensorObj->VerticalPositionNoise().StdDev());
  EXPECT_DOUBLE_EQ(9.1, navSatSensorObj->HorizontalVelocityNoise().Mean());
  EXPECT_DOUBLE_EQ(10.11, navSatSensorObj->HorizontalVelocityNoise().StdDev());
  EXPECT_DOUBLE_EQ(12.13, navSatSensorObj->VerticalVelocityNoise().Mean());
  EXPECT_DOUBLE_EQ(14.15, navSatSensorObj->VerticalVelocityNoise().StdDev());

  // Get the gpu_ray sensor
  const sdf::Sensor *gpuRaySensor = link->SensorByName("gpu_ray_sensor");
  ASSERT_NE(nullptr, gpuRaySensor);
  EXPECT_EQ("gpu_ray_sensor", gpuRaySensor->Name());
  EXPECT_EQ(sdf::SensorType::GPU_LIDAR, gpuRaySensor->Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), gpuRaySensor->RawPose());
  EXPECT_FALSE(gpuRaySensor->EnableMetrics());
  const sdf::Lidar *gpuRay = gpuRaySensor->LidarSensor();
  ASSERT_NE(nullptr, gpuRay);

  // Get the gpu_lidar sensor
  const sdf::Sensor *gpuLidarSensor = link->SensorByName("gpu_lidar_sensor");
  ASSERT_NE(nullptr, gpuLidarSensor);
  EXPECT_EQ("gpu_lidar_sensor", gpuLidarSensor->Name());
  EXPECT_EQ(sdf::SensorType::GPU_LIDAR, gpuLidarSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0),
            gpuLidarSensor->RawPose());
  EXPECT_FALSE(gpuLidarSensor->EnableMetrics());
  const sdf::Lidar *gpuLidar = gpuLidarSensor->LidarSensor();
  ASSERT_NE(nullptr, gpuLidar);

  // Get the imu sensor
  const sdf::Sensor *imuSensor = link->SensorByName("imu_sensor");
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

  // Get the logical camera sensor
  const sdf::Sensor *logicalCameraSensor =
    link->SensorByName("logical_camera_sensor");
  ASSERT_NE(nullptr, logicalCameraSensor);
  EXPECT_EQ("logical_camera_sensor", logicalCameraSensor->Name());
  EXPECT_EQ(sdf::SensorType::LOGICAL_CAMERA, logicalCameraSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(7, 8, 9, 0, 0, 0),
      logicalCameraSensor->RawPose());
  EXPECT_FALSE(logicalCameraSensor->EnableMetrics());

  // Get the magnetometer sensor
  const sdf::Sensor *magnetometerSensor =
    link->SensorByName("magnetometer_sensor");
  ASSERT_NE(nullptr, magnetometerSensor);
  EXPECT_EQ("magnetometer_sensor", magnetometerSensor->Name());
  EXPECT_EQ(sdf::SensorType::MAGNETOMETER, magnetometerSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
      magnetometerSensor->RawPose());
  EXPECT_FALSE(magnetometerSensor->EnableMetrics());
  const sdf::Magnetometer *magSensor = magnetometerSensor->MagnetometerSensor();
  ASSERT_NE(nullptr, magSensor);
  EXPECT_DOUBLE_EQ(0.1, magSensor->XNoise().Mean());
  EXPECT_DOUBLE_EQ(0.2, magSensor->XNoise().StdDev());
  EXPECT_DOUBLE_EQ(1.2, magSensor->YNoise().Mean());
  EXPECT_DOUBLE_EQ(2.3, magSensor->YNoise().StdDev());
  EXPECT_DOUBLE_EQ(3.4, magSensor->ZNoise().Mean());
  EXPECT_DOUBLE_EQ(5.6, magSensor->ZNoise().StdDev());

  // Get the multicamera sensor
  const sdf::Sensor *multicameraSensor =
    link->SensorByName("multicamera_sensor");
  ASSERT_NE(nullptr, multicameraSensor);
  EXPECT_EQ("multicamera_sensor", multicameraSensor->Name());
  EXPECT_EQ(sdf::SensorType::MULTICAMERA, multicameraSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(13, 14, 15, 0, 0, 0),
      multicameraSensor->RawPose());
  EXPECT_FALSE(multicameraSensor->EnableMetrics());

  // Get the ray sensor
  const sdf::Sensor *raySensor = link->SensorByName("ray_sensor");
  ASSERT_NE(nullptr, raySensor);
  EXPECT_EQ("ray_sensor", raySensor->Name());
  EXPECT_EQ(sdf::SensorType::LIDAR, raySensor->Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), raySensor->RawPose());
  EXPECT_FALSE(raySensor->EnableMetrics());
  const sdf::Lidar *ray = raySensor->LidarSensor();
  ASSERT_NE(nullptr, ray);
  EXPECT_EQ(320u, ray->HorizontalScanSamples());
  EXPECT_DOUBLE_EQ(0.9, ray->HorizontalScanResolution());
  EXPECT_DOUBLE_EQ(1.75, *(ray->HorizontalScanMinAngle()));
  EXPECT_DOUBLE_EQ(2.94, *(ray->HorizontalScanMaxAngle()));
  EXPECT_EQ(240u, ray->VerticalScanSamples());
  EXPECT_DOUBLE_EQ(0.8, ray->VerticalScanResolution());
  EXPECT_DOUBLE_EQ(2.75, *(ray->VerticalScanMinAngle()));
  EXPECT_DOUBLE_EQ(3.94, *(ray->VerticalScanMaxAngle()));
  EXPECT_DOUBLE_EQ(1.23, ray->RangeMin());
  EXPECT_DOUBLE_EQ(4.56, ray->RangeMax());
  EXPECT_DOUBLE_EQ(7.89, ray->RangeResolution());
  EXPECT_DOUBLE_EQ(0.98, ray->LidarNoise().Mean());
  EXPECT_DOUBLE_EQ(0.76, ray->LidarNoise().StdDev());

  // Get the lidar sensor
  const sdf::Sensor *lidarSensor = link->SensorByName("lidar_sensor");
  ASSERT_NE(nullptr, lidarSensor);
  EXPECT_EQ("lidar_sensor", lidarSensor->Name());
  EXPECT_EQ(sdf::SensorType::LIDAR, lidarSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), lidarSensor->RawPose());
  EXPECT_TRUE(lidarSensor->EnableMetrics());
  const sdf::Lidar *lidar = lidarSensor->LidarSensor();
  ASSERT_NE(nullptr, lidar);
  EXPECT_EQ(320u, lidar->HorizontalScanSamples());
  EXPECT_DOUBLE_EQ(0.9, lidar->HorizontalScanResolution());
  EXPECT_DOUBLE_EQ(1.75, *(lidar->HorizontalScanMinAngle()));
  EXPECT_DOUBLE_EQ(2.94, *(lidar->HorizontalScanMaxAngle()));
  EXPECT_EQ(240u, lidar->VerticalScanSamples());
  EXPECT_DOUBLE_EQ(0.8, lidar->VerticalScanResolution());
  EXPECT_DOUBLE_EQ(2.75, *(lidar->VerticalScanMinAngle()));
  EXPECT_DOUBLE_EQ(3.94, *(lidar->VerticalScanMaxAngle()));
  EXPECT_DOUBLE_EQ(1.23, lidar->RangeMin());
  EXPECT_DOUBLE_EQ(4.56, lidar->RangeMax());
  EXPECT_DOUBLE_EQ(7.89, lidar->RangeResolution());
  EXPECT_DOUBLE_EQ(0.98, lidar->LidarNoise().Mean());
  EXPECT_DOUBLE_EQ(0.76, lidar->LidarNoise().StdDev());

  // Get the rfid sensor
  const sdf::Sensor *rfidSensor = link->SensorByName("rfid_sensor");
  ASSERT_NE(nullptr, rfidSensor);
  EXPECT_EQ("rfid_sensor", rfidSensor->Name());
  EXPECT_EQ(sdf::SensorType::RFID, rfidSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 0), rfidSensor->RawPose());
  EXPECT_FALSE(rfidSensor->EnableMetrics());

  // Get the rfid tag
  const sdf::Sensor *rfidTag = link->SensorByName("rfid_tag");
  ASSERT_NE(nullptr, rfidTag);
  EXPECT_EQ("rfid_tag", rfidTag->Name());
  EXPECT_EQ(sdf::SensorType::RFIDTAG, rfidTag->Type());
  EXPECT_EQ(gz::math::Pose3d(7, 8, 9, 0, 0, 0), rfidTag->RawPose());
  EXPECT_FALSE(rfidTag->EnableMetrics());

  // Get the sonar sensor
  const sdf::Sensor *sonarSensor = link->SensorByName("sonar_sensor");
  ASSERT_NE(nullptr, sonarSensor);
  EXPECT_EQ("sonar_sensor", sonarSensor->Name());
  EXPECT_EQ(sdf::SensorType::SONAR, sonarSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
            sonarSensor->RawPose());
  EXPECT_FALSE(sonarSensor->EnableMetrics());

  // Get the wireless receiver
  const sdf::Sensor *wirelessReceiver = link->SensorByName("wireless_receiver");
  ASSERT_NE(nullptr, wirelessReceiver);
  EXPECT_EQ("wireless_receiver", wirelessReceiver->Name());
  EXPECT_EQ(sdf::SensorType::WIRELESS_RECEIVER, wirelessReceiver->Type());
  EXPECT_EQ(gz::math::Pose3d(13, 14, 15, 0, 0, 0),
      wirelessReceiver->RawPose());
  EXPECT_FALSE(wirelessReceiver->EnableMetrics());

  // Get the wireless transmitter
  const sdf::Sensor *wirelessTransmitter =
    link->SensorByName("wireless_transmitter");
  ASSERT_NE(nullptr, wirelessTransmitter);
  EXPECT_EQ("wireless_transmitter", wirelessTransmitter->Name());
  EXPECT_EQ(sdf::SensorType::WIRELESS_TRANSMITTER, wirelessTransmitter->Type());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0),
      wirelessTransmitter->RawPose());
  EXPECT_FALSE(wirelessTransmitter->EnableMetrics());

  // Get the air_pressure sensor
  const sdf::Sensor *airPressureSensor = link->SensorByName(
      "air_pressure_sensor");
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

  // Get the air_speed sensor
  const sdf::Sensor *airSpeedSensor = link->SensorByName(
      "air_speed_sensor");
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

  // Get the wide angle camera sensor
  EXPECT_TRUE(link->SensorNameExists("wide_angle_camera_sensor"));
  const sdf::Sensor *wideAngleCameraSensor =
      link->SensorByName("wide_angle_camera_sensor");
  ASSERT_NE(nullptr, wideAngleCameraSensor);
  EXPECT_EQ("wide_angle_camera_sensor", wideAngleCameraSensor->Name());
  EXPECT_EQ(sdf::SensorType::WIDE_ANGLE_CAMERA, wideAngleCameraSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(20, 30, 40, 0, 0, 0),
      wideAngleCameraSensor->RawPose());
  EXPECT_FALSE(wideAngleCameraSensor->EnableMetrics());
  const sdf::Camera *wideAngleCam = wideAngleCameraSensor->CameraSensor();
  ASSERT_NE(nullptr, wideAngleCam);
  EXPECT_EQ("wide_angle_cam", wideAngleCam->Name());
  EXPECT_EQ(gz::math::Pose3d(0.2, 0.3, 0.4, 0, 0, 0),
            wideAngleCam->RawPose());
  EXPECT_DOUBLE_EQ(3.14, wideAngleCam->HorizontalFov().Radian());
  EXPECT_EQ("wideanglecamera", wideAngleCameraSensor->Topic());
  EXPECT_EQ("", wideAngleCam->TriggerTopic());
  EXPECT_FALSE(wideAngleCam->Triggered());
  EXPECT_EQ(320u, wideAngleCam->ImageWidth());
  EXPECT_EQ(240u, wideAngleCam->ImageHeight());
  EXPECT_EQ(sdf::PixelFormatType::RGB_INT8, wideAngleCam->PixelFormat());
  EXPECT_DOUBLE_EQ(0.1, wideAngleCam->NearClip());
  EXPECT_DOUBLE_EQ(100, wideAngleCam->FarClip());
  EXPECT_EQ("custom", wideAngleCam->LensType());
  EXPECT_TRUE(wideAngleCam->LensScaleToHfov());
  EXPECT_DOUBLE_EQ(1.05, wideAngleCam->LensC1());
  EXPECT_DOUBLE_EQ(4.0, wideAngleCam->LensC2());
  EXPECT_DOUBLE_EQ(0.0, wideAngleCam->LensC3());
  EXPECT_DOUBLE_EQ(1.0, wideAngleCam->LensFocalLength());
  EXPECT_EQ("tan", wideAngleCam->LensFunction());
  EXPECT_DOUBLE_EQ(3.1415, wideAngleCam->LensCutoffAngle().Radian());
  EXPECT_EQ(512, wideAngleCam->LensEnvironmentTextureSize());
}

/////////////////////////////////////////////////
TEST(DOMLink, LoadLinkPoseRelativeTo)
{
  const std::string testFile = sdf::testing::TestFile("sdf",
        "model_link_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  using Pose = gz::math::Pose3d;

  // Get the first model
  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_link_relative_to", model->Name());
  EXPECT_EQ(4u, model->LinkCount());
  EXPECT_NE(nullptr, model->LinkByIndex(0));
  EXPECT_NE(nullptr, model->LinkByIndex(1));
  EXPECT_NE(nullptr, model->LinkByIndex(2));
  EXPECT_NE(nullptr, model->LinkByIndex(3));
  EXPECT_EQ(nullptr, model->LinkByIndex(4));
  EXPECT_EQ(Pose(0, 0, 0, 0, 0, 0), model->RawPose());
  EXPECT_EQ("", model->PoseRelativeTo());

  ASSERT_TRUE(model->LinkNameExists("L1"));
  ASSERT_TRUE(model->LinkNameExists("L2"));
  ASSERT_TRUE(model->LinkNameExists("L2a"));
  ASSERT_TRUE(model->LinkNameExists("L3"));
  EXPECT_TRUE(model->LinkByName("L1")->PoseRelativeTo().empty());
  EXPECT_TRUE(model->LinkByName("L2")->PoseRelativeTo().empty());
  EXPECT_EQ("__model__", model->LinkByName("L2a")->PoseRelativeTo());
  EXPECT_EQ("L1", model->LinkByName("L3")->PoseRelativeTo());

  EXPECT_EQ(Pose(1, 0, 0, 0, GZ_PI/2, 0), model->LinkByName("L1")->RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0), model->LinkByName("L2")->RawPose());
  EXPECT_EQ(Pose(12, 0, 0, 0, 0, 0), model->LinkByName("L2a")->RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, 0, 0), model->LinkByName("L3")->RawPose());

  EXPECT_EQ(Pose(1, 0, 0, 0, GZ_PI / 2, 0),
            model->LinkByName("L1")->SemanticPose().RawPose());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0),
            model->LinkByName("L2")->SemanticPose().RawPose());
  EXPECT_EQ(Pose(12, 0, 0, 0, 0, 0),
            model->LinkByName("L2a")->SemanticPose().RawPose());
  EXPECT_EQ(Pose(3, 0, 0, 0, 0, 0),
            model->LinkByName("L3")->SemanticPose().RawPose());

  // Test SemanticPose().Resolve to get each link pose in the model frame
  Pose pose;
  EXPECT_TRUE(
    model->LinkByName("L1")->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, GZ_PI/2, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("L2")->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("L2a")->SemanticPose().Resolve(pose,
                                                     "__model__").empty());
  EXPECT_EQ(Pose(12, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(
    model->LinkByName("L3")->SemanticPose().Resolve(pose, "__model__").empty());
  EXPECT_EQ(Pose(1, 0, -3, 0, GZ_PI/2, 0), pose);
  // test other API too
  EXPECT_TRUE(model->LinkByName("L1")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(1, 0, 0, 0, GZ_PI/2, 0), pose);
  EXPECT_TRUE(model->LinkByName("L2")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(2, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(model->LinkByName("L2a")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(12, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(model->LinkByName("L3")->SemanticPose().Resolve(pose).empty());
  EXPECT_EQ(Pose(1, 0, -3, 0, GZ_PI/2, 0), pose);

  // resolve pose of L1 relative to L3
  // should be inverse of L3's Pose()
  EXPECT_TRUE(
    model->LinkByName("L1")->SemanticPose().Resolve(pose, "L3").empty());
  EXPECT_EQ(Pose(-3, 0, 0, 0, 0, 0), pose);

  EXPECT_TRUE(model->CanonicalLinkName().empty());

  EXPECT_EQ(0u, model->JointCount());
  EXPECT_EQ(nullptr, model->JointByIndex(0));

  EXPECT_EQ(0u, model->FrameCount());
  EXPECT_EQ(nullptr, model->FrameByIndex(0));
}

/////////////////////////////////////////////////
TEST(DOMLink, LoadInvalidLinkPoseRelativeTo)
{
  const std::string testFile = sdf::testing::TestFile("sdf",
        "model_invalid_link_relative_to.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (auto e : errors)
    std::cout << e << std::endl;
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(5u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_INVALID);
  EXPECT_NE(std::string::npos,
    errors[0].Message().find(
      "relative_to name[A] specified by link with name[L] does not match a "
      "nested model, link, joint, or frame name in model"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_CYCLE);
  EXPECT_NE(std::string::npos,
    errors[1].Message().find(
      "relative_to name[self_cycle] is identical to link name[self_cycle], "
      "causing a graph cycle"));
  // errors[2]
  // errors[3]
  // errors[4]
}

/////////////////////////////////////////////////
TEST(DOMLink, ValidInertialPoseRelTo)
{
  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.8'>"
         << "  <model name='A'>"
         << "    <link name='B'>"
         << "      <inertial>"
         << "        <pose relative_to=''>0.1 1 0.2 0 0 -0.52</pose>"
         << "      </inertial>"
         << "    </link>"
         << "  </model>"
         << "</sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(stream.str());
  EXPECT_TRUE(errors.empty());

  const sdf::Model *model = root.Model();
  ASSERT_NE(model, nullptr);

  const sdf::Link *link = model->LinkByName("B");
  ASSERT_NE(link, nullptr);

  EXPECT_EQ(link->Inertial().Pose(),
      gz::math::Pose3d(0.1, 1, 0.2, 0, 0, -0.52));
}

/////////////////////////////////////////////////
TEST(DOMLink, InvalidInertialPoseRelTo)
{
  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.8'>"
         << "  <model name='A'>"
         << "    <frame name='C'>"
         << "      <pose>0 0 1 0 0 0</pose>"
         << "    </frame>"
         << "    <link name='B'>"
         << "      <inertial>"
         << "        <pose relative_to='C'>0.1 1 0.2 0 0 -0.52</pose>"
         << "      </inertial>"
         << "    </link>"
         << "  </model>"
         << "</sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(stream.str());

  // TODO(anyone) add test for warnings once it's implemented

  const sdf::Model *model = root.Model();
  ASSERT_NE(model, nullptr);

  const sdf::Link *link = model->LinkByName("B");
  ASSERT_NE(link, nullptr);

  EXPECT_EQ(link->Inertial().Pose(),
      gz::math::Pose3d(0.1, 1, 0.2, 0, 0, -0.52));
}

/////////////////////////////////////////////////
TEST(DOMLink, InertialAuto)
{
  const std::string testFile = sdf::testing::TestFile("sdf",
        "inertial_stats_auto.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(model, nullptr);

  const sdf::Link *link = model->LinkByName("link_1");
  ASSERT_NE(link, nullptr);

  // Verify inertial values for link_1 match those in inertial_stats.sdf
  gz::math::Inertiald inertial = link->Inertial();
  gz::math::MassMatrix3d massMatrix = inertial.MassMatrix();
  EXPECT_EQ(gz::math::Pose3d::Zero, inertial.Pose());
  EXPECT_DOUBLE_EQ(6.0, massMatrix.Mass());
  EXPECT_DOUBLE_EQ(1.0, massMatrix.Ixx());
  EXPECT_DOUBLE_EQ(1.0, massMatrix.Iyy());
  EXPECT_DOUBLE_EQ(1.0, massMatrix.Izz());
  EXPECT_DOUBLE_EQ(0.0, massMatrix.Ixy());
  EXPECT_DOUBLE_EQ(0.0, massMatrix.Ixz());
  EXPECT_DOUBLE_EQ(0.0, massMatrix.Iyz());

  // Expect //link/inertial element to not contain inertia, mass, or pose
  auto linkElem = link->Element();
  auto inertialElem = linkElem->FindElement("inertial");
  ASSERT_TRUE(inertialElem != nullptr);
  EXPECT_FALSE(inertialElem->HasElement("inertia"));
  EXPECT_FALSE(inertialElem->HasElement("mass"));
  EXPECT_FALSE(inertialElem->HasElement("pose"));
}

/////////////////////////////////////////////////
TEST(DOMLink, InertialAutoExplicitMass)
{
  const std::string testFile = sdf::testing::TestFile("sdf",
        "inertial_stats_auto_mass.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(model, nullptr);

  std::vector<std::string> linkNames{"link_1", "link_2", "link_3", "link_4"};
  for (const std::string &linkName : linkNames)
  {
    const sdf::Link *link = model->LinkByName(linkName);
    ASSERT_NE(link, nullptr);

    // Verify inertial values for link_i match those in inertial_stats.sdf
    gz::math::Inertiald inertial = link->Inertial();
    gz::math::MassMatrix3d massMatrix = inertial.MassMatrix();
    EXPECT_EQ(gz::math::Pose3d::Zero, inertial.Pose());
    EXPECT_DOUBLE_EQ(6.0, massMatrix.Mass());
    EXPECT_DOUBLE_EQ(1.0, massMatrix.Ixx());
    EXPECT_DOUBLE_EQ(1.0, massMatrix.Iyy());
    EXPECT_DOUBLE_EQ(1.0, massMatrix.Izz());
    EXPECT_DOUBLE_EQ(0.0, massMatrix.Ixy());
    EXPECT_DOUBLE_EQ(0.0, massMatrix.Ixz());
    EXPECT_DOUBLE_EQ(0.0, massMatrix.Iyz());
  }
}

/////////////////////////////////////////////////
TEST(DOMLink, InertialAutoSaveInElement)
{
  const std::string testFile = sdf::testing::TestFile("sdf",
        "inertial_stats_auto.sdf");

  // Load the SDF file with SAVE_CALCULATION_IN_ELEMENT
  sdf::ParserConfig config;
  config.SetCalculateInertialConfiguration(
    sdf::ConfigureResolveAutoInertials::SAVE_CALCULATION_IN_ELEMENT);

  sdf::Root root;
  auto errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(model, nullptr);

  const sdf::Link *link = model->LinkByName("link_1");
  ASSERT_NE(link, nullptr);

  // Verify inertial values for link_1 match those in inertial_stats.sdf
  gz::math::Inertiald inertial = link->Inertial();
  gz::math::MassMatrix3d massMatrix = inertial.MassMatrix();
  EXPECT_EQ(gz::math::Pose3d::Zero, inertial.Pose());
  EXPECT_DOUBLE_EQ(6.0, massMatrix.Mass());
  EXPECT_DOUBLE_EQ(1.0, massMatrix.Ixx());
  EXPECT_DOUBLE_EQ(1.0, massMatrix.Iyy());
  EXPECT_DOUBLE_EQ(1.0, massMatrix.Izz());
  EXPECT_DOUBLE_EQ(0.0, massMatrix.Ixy());
  EXPECT_DOUBLE_EQ(0.0, massMatrix.Ixz());
  EXPECT_DOUBLE_EQ(0.0, massMatrix.Iyz());

  // Expect //link/inertial element to contain inertia values
  auto linkElem = link->Element();
  auto inertialElem = linkElem->FindElement("inertial");
  ASSERT_TRUE(inertialElem != nullptr);
  EXPECT_TRUE(inertialElem->HasElement("mass"));
  EXPECT_DOUBLE_EQ(6.0, inertialElem->Get<double>("mass"));
  EXPECT_EQ(gz::math::Pose3d::Zero,
    inertialElem->Get<gz::math::Pose3d>("pose"));
  auto momentOfInertiaElem = inertialElem->FindElement("inertia");
  ASSERT_TRUE(momentOfInertiaElem != nullptr);
  EXPECT_TRUE(momentOfInertiaElem->HasElement("ixx"));
  EXPECT_DOUBLE_EQ(1.0, momentOfInertiaElem->Get<double>("ixx"));
  EXPECT_TRUE(momentOfInertiaElem->HasElement("iyy"));
  EXPECT_DOUBLE_EQ(1.0, momentOfInertiaElem->Get<double>("iyy"));
  EXPECT_TRUE(momentOfInertiaElem->HasElement("izz"));
  EXPECT_DOUBLE_EQ(1.0, momentOfInertiaElem->Get<double>("izz"));
  EXPECT_TRUE(momentOfInertiaElem->HasElement("ixy"));
  EXPECT_DOUBLE_EQ(0.0, momentOfInertiaElem->Get<double>("ixy"));
  EXPECT_TRUE(momentOfInertiaElem->HasElement("ixz"));
  EXPECT_DOUBLE_EQ(0.0, momentOfInertiaElem->Get<double>("ixz"));
  EXPECT_TRUE(momentOfInertiaElem->HasElement("iyz"));
  EXPECT_DOUBLE_EQ(0.0, momentOfInertiaElem->Get<double>("iyz"));
}
