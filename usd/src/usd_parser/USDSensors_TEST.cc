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

#include <gz/math/Pose3.hh>

#include "test_config.h"
#include "test_utils.hh"

#include "USDSensors.hh"

#include "sdf/Sensor.hh"
#include "sdf/Camera.hh"
#include "sdf/Lidar.hh"
#include "sdf/usd/usd_parser/USDData.hh"

/////////////////////////////////////////////////
TEST(USDJointTest, JointTest)
{
  const std::string filename =
    sdf::testing::TestFile("usd", "sensors.usda");
  const auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  sdf::usd::USDData usdData(filename);
  usdData.Init();

  const auto cameraPrim = stage->GetPrimAtPath(pxr::SdfPath(
    "/camera"));
  ASSERT_TRUE(cameraPrim);

  sdf::Sensor sensorCamera = sdf::usd::ParseSensors(
    cameraPrim, usdData);

  EXPECT_EQ(sdf::SensorType::CAMERA, sensorCamera.Type());
  EXPECT_EQ(gz::math::Pose3d(-1.83617, 0, 0.773532, 0, 0.267035, -0),
            sensorCamera.RawPose());
  auto cameraSDF = sensorCamera.CameraSensor();
  ASSERT_TRUE(cameraSDF);
  EXPECT_EQ(640u, cameraSDF->ImageWidth());
  EXPECT_EQ(480u, cameraSDF->ImageHeight());
  EXPECT_EQ(sdf::PixelFormatType::RGB_INT8, cameraSDF->PixelFormat());
  EXPECT_DOUBLE_EQ(1000000, cameraSDF->FarClip());
  EXPECT_DOUBLE_EQ(1, cameraSDF->NearClip());
  EXPECT_DOUBLE_EQ(20.955, cameraSDF->HorizontalFov().Radian());
  EXPECT_DOUBLE_EQ(24.0, cameraSDF->LensFocalLength());

  const auto lidarPrim = stage->GetPrimAtPath(pxr::SdfPath(
    "/lidar"));
  ASSERT_TRUE(lidarPrim);

  sdf::Sensor sensorLidar = sdf::usd::ParseSensors(
    lidarPrim, usdData);

  EXPECT_EQ(sdf::SensorType::GPU_LIDAR, sensorLidar.Type());
  auto lidarSDF = sensorLidar.LidarSensor();
  ASSERT_TRUE(lidarSDF);
  EXPECT_DOUBLE_EQ(-180.00000500895632,
                   lidarSDF->HorizontalScanMinAngle().Degree());
  EXPECT_DOUBLE_EQ(180.00000500895632,
                   lidarSDF->HorizontalScanMaxAngle().Degree());
  EXPECT_DOUBLE_EQ(1, lidarSDF->HorizontalScanResolution());
  EXPECT_DOUBLE_EQ(900, lidarSDF->HorizontalScanSamples());

  EXPECT_DOUBLE_EQ(-15.000000417413029,
                   lidarSDF->VerticalScanMinAngle().Degree());
  EXPECT_DOUBLE_EQ(15.000000417413029,
                   lidarSDF->VerticalScanMaxAngle().Degree());
  EXPECT_DOUBLE_EQ(1, lidarSDF->VerticalScanResolution());
  EXPECT_DOUBLE_EQ(7, lidarSDF->VerticalScanSamples());

  EXPECT_DOUBLE_EQ(0.40000000596046448, lidarSDF->RangeMin());
  EXPECT_DOUBLE_EQ(100, lidarSDF->RangeMax());
  EXPECT_DOUBLE_EQ(0.1, lidarSDF->RangeResolution());
  EXPECT_DOUBLE_EQ(20, sensorLidar.UpdateRate());
}
