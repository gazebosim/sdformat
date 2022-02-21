/*
 * Copyright 2022 Open Source Robotics Foundation
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

#include <string>

#include <gtest/gtest.h>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/cube.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Util.hh>

#include "sdf/usd/sdf_parser/World.hh"
#include "sdf/Root.hh"
#include "test_config.h"
#include "test_utils.hh"
#include "../UsdTestUtils.hh"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
class UsdStageFixture : public::testing::Test
{
  public: UsdStageFixture() = default;

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
    ASSERT_TRUE(this->stage);
  }

  public: pxr::UsdStageRefPtr stage;
};

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Sensors)
{
  sdf::setFindCallback(sdf::usd::testing::findFileCb);
  ignition::common::addFindFileURICallback(
    std::bind(&sdf::usd::testing::FindResourceUri, std::placeholders::_1));

  const auto path = sdf::testing::TestFile("sdf", "camera_lidar_sensor.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);

  std::string cameraPath = worldPath + "/" + "camera";
  std::string cameraLinkPath = cameraPath + "/" + "link";
  std::string cameraSensorPath = cameraLinkPath + "/" + "camera";
  auto cameraSensor = this->stage->GetPrimAtPath(
    pxr::SdfPath(cameraSensorPath));
  ASSERT_TRUE(cameraSensor);
  auto usdCamera = pxr::UsdGeomCamera(cameraSensor);
  ASSERT_TRUE(usdCamera);

  float focalLength;
  pxr::GfVec2f clippingRange;
  float horizontalAperture;
  usdCamera.GetFocalLengthAttr().Get(&focalLength);
  usdCamera.GetClippingRangeAttr().Get(&clippingRange);
  usdCamera.GetHorizontalApertureAttr().Get(&horizontalAperture);
  EXPECT_FLOAT_EQ(40, focalLength);
  EXPECT_EQ(pxr::GfVec2f(0.1, 100), clippingRange);
  EXPECT_FLOAT_EQ(59.98868, horizontalAperture);

  std::string lidarPath = worldPath + "/" + "model_with_lidar";
  std::string lidarLinkPath = lidarPath + "/" + "link";
  std::string lidarSensorPath = lidarLinkPath + "/gpu_lidar/sensor";
  auto lidarSensor = this->stage->GetPrimAtPath(
    pxr::SdfPath(lidarSensorPath));
  ASSERT_TRUE(lidarSensor);
  float hFOV;
  float hResolution;
  float vFOV;
  float vResolution;
  float minRange;
  float maxRange;
  lidarSensor.GetAttribute(pxr::TfToken("minRange")).Get(&minRange);
  lidarSensor.GetAttribute(pxr::TfToken("maxRange")).Get(&maxRange);
  lidarSensor.GetAttribute(pxr::TfToken("horizontalFov")).Get(&hFOV);
  lidarSensor.GetAttribute(pxr::TfToken("horizontalResolution")).Get(&hResolution);
  lidarSensor.GetAttribute(pxr::TfToken("verticalFov")).Get(&vFOV);
  lidarSensor.GetAttribute(pxr::TfToken("verticalResolution")).Get(&vResolution);
  EXPECT_FLOAT_EQ(10, maxRange);
  EXPECT_FLOAT_EQ(0.08, minRange);
  EXPECT_FLOAT_EQ(159.99995, hFOV);
  EXPECT_FLOAT_EQ(1, hResolution);
  EXPECT_FLOAT_EQ(29.999956, vFOV);
  EXPECT_FLOAT_EQ(1, vResolution);

  std::string imuPath = worldPath + "/" + "sensors_box";
  std::string imuLinkPath = imuPath + "/" + "link";
  std::string imuSensorPath = imuLinkPath + "/" + "imu";
  auto imuSensor = this->stage->GetPrimAtPath(
    pxr::SdfPath(imuSensorPath));
  ASSERT_TRUE(imuSensor);
  auto usdIMUCube = pxr::UsdGeomCube(imuSensor);
  ASSERT_TRUE(usdIMUCube);
}
