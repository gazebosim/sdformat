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

#include <string>
#include <unordered_map>

#include <gtest/gtest.h>
#include <ignition/math/Pose3.hh>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdLux/diskLight.h>
#include <pxr/usd/usdLux/distantLight.h>
#include <pxr/usd/usdLux/sphereLight.h>

#include "sdf/Light.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf_usd_parser/light.hh"
#include "sdf_usd_parser/utils.hh"
#include "test_config.h"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
// This fixture also has helper methods to assist with comparing SDF/USD values
class UsdStageFixture : public ::testing::Test
{
  public: UsdStageFixture() = default;

  /// \brief Load an SDF file into the fixture's sdf::Root object
  /// \param[in] _fileName The name of the file to load
  /// \return True if _fileName was successfully loaded. False otherwise
  public: bool LoadSdfFile(const std::string &_fileName)
  {
    auto errors = this->root.Load(_fileName);
    if (!errors.empty())
    {
      std::cerr << "Errors encountered:\n";
      for (const auto &e : errors)
        std::cout << e << "\n";
      return false;
    }

    return true;
  }

  /// \brief Compare the intensity between a USD and SDF light
  /// \param[in] _lightPrim The USD light prim
  /// \param[in] _sdfLight The SDF light
  public: void CheckLightIntensity(const pxr::UsdPrim &_lightPrim,
              const sdf::Light &_sdfLight)
  {
    bool checkedIntensity = false;
    if (auto intensityAttr = _lightPrim.GetAttribute(pxr::TfToken("intensity")))
    {
      float intensityVal = 0.0;
      intensityAttr.Get(&intensityVal);
      EXPECT_FLOAT_EQ(intensityVal, _sdfLight.Intensity() * 10000.0f);
      checkedIntensity = true;
    }
    EXPECT_TRUE(checkedIntensity);
  }

  /// \brief Compare the pose of a USD prim to a desired pose
  /// \param[in] _usdPrim The USD prim
  /// \param[in] _targetPose The pose that _usdPrim should have
  public: void CheckPrimPose(const pxr::UsdPrim &_usdPrim,
              const ignition::math::Pose3d &_targetPose)
  {
    bool checkedTranslate = false;
    if (auto translateAttr = _usdPrim.GetAttribute(pxr::TfToken("xformOp:translate")))
    {
      pxr::GfVec3d usdTranslation;
      translateAttr.Get(&usdTranslation);
      EXPECT_DOUBLE_EQ(usdTranslation[0], _targetPose.Pos().X());
      EXPECT_DOUBLE_EQ(usdTranslation[1], _targetPose.Pos().Y());
      EXPECT_DOUBLE_EQ(usdTranslation[2], _targetPose.Pos().Z());
      checkedTranslate = true;
    }
    EXPECT_TRUE(checkedTranslate);

    bool checkedRotate = false;
    if (auto rotateAttr = _usdPrim.GetAttribute(pxr::TfToken("xformOp:rotateXYZ")))
    {
      pxr::GfVec3f usdRotation;
      rotateAttr.Get(&usdRotation);
      // USD uses degrees, but SDF uses radians. USD also uses floats for angles
      // here, but SDF uses doubles
      const auto sdfRollAngle = static_cast<float>(
          ignition::math::Angle(_targetPose.Rot().Roll()).Degree());
      EXPECT_FLOAT_EQ(usdRotation[0], sdfRollAngle);
      const auto sdfPitchAngle = static_cast<float>(
          ignition::math::Angle(_targetPose.Rot().Pitch()).Degree());
      EXPECT_FLOAT_EQ(usdRotation[1], sdfPitchAngle);
      const auto sdfYawAngle = static_cast<float>(
          ignition::math::Angle(_targetPose.Rot().Yaw()).Degree());
      EXPECT_FLOAT_EQ(usdRotation[2], sdfYawAngle);
      checkedRotate = true;
    }
    EXPECT_TRUE(checkedRotate);

    bool checkedOpOrder = false;
    if (auto opOrderAttr = _usdPrim.GetAttribute(pxr::TfToken("xformOpOrder")))
    {
      pxr::VtArray<pxr::TfToken> opNames;
      opOrderAttr.Get(&opNames);
      // TODO(adlarkin) update this code to handle things like scale in the opOrder
      // (checking for scale should be done elsehwere since prims aren't always
      // scaled, but maybe what I can do here is make sure the opNames size is
      // at least 2 and then make sure translate occurs before rotate)
      ASSERT_EQ(2u, opNames.size());
      EXPECT_EQ(pxr::TfToken("xformOp:translate"), opNames[0]);
      EXPECT_EQ(pxr::TfToken("xformOp:rotateXYZ"), opNames[1]);
      checkedOpOrder = true;
    }
    EXPECT_TRUE(checkedOpOrder);
  }

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
  }

  public: pxr::UsdStageRefPtr stage;
  public: sdf::Root root;
};

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Lights)
{
  const auto path = sdf::testing::TestFile("sdf", "lights.sdf");

  // load the world in the SDF file
  ASSERT_TRUE(this->LoadSdfFile(path));
  ASSERT_EQ(1u, this->root.WorldCount());
  auto world = this->root.WorldByIndex(0u);

  // convert all lights attached directly to the world to USD
  std::unordered_map<std::string, sdf::Light> lightPathToSdf;
  for (unsigned int i = 0; i < world->LightCount(); ++i)
  {
    const auto light = *(world->LightByIndex(i));
    const auto lightPath = std::string("/" + light.Name());
    lightPathToSdf[lightPath] = light;
    EXPECT_TRUE(usd::ParseSdfLight(light, this->stage, lightPath));
  }
  EXPECT_EQ(world->LightCount(), lightPathToSdf.size());

  // check that the lights were parsed correctly
  int numPointLights = 0;
  int numSpotLights = 0;
  int numDirectionalLights = 0;
  for (const auto &prim : this->stage->Traverse())
  {
    auto iter = lightPathToSdf.find(prim.GetPath().GetString());
    ASSERT_NE(lightPathToSdf.end(), iter);
    const auto lightUsd = this->stage->GetPrimAtPath(pxr::SdfPath(iter->first));
    const auto lightSdf = iter->second;

    bool validLight = true;
    if (lightUsd.IsA<pxr::UsdLuxSphereLight>())
    {
      numPointLights++;
      this->CheckLightIntensity(lightUsd, lightSdf);

      bool checkedPointAttr = false;
      if (auto pointAttr = lightUsd.GetAttribute(pxr::TfToken("treatAsPoint")))
      {
        bool isPoint = false;
        pointAttr.Get(&isPoint);
        EXPECT_TRUE(isPoint);
        checkedPointAttr = true;
      }
      EXPECT_TRUE(checkedPointAttr);
    }
    else if (lightUsd.IsA<pxr::UsdLuxDiskLight>())
    {
      numSpotLights++;
      this->CheckLightIntensity(lightUsd, lightSdf);
    }
    else if (lightUsd.IsA<pxr::UsdLuxDistantLight>())
    {
      numDirectionalLights++;
      // the default intensity for pxr::UsdLuxDistantLight is correct,
      // so we don't need to make a call to this->CheckLightIntensity
      // here because no custom intensity is being set for directional
      // lights in USD (see the "increaseIntensity" variable workaround
      // in src/usd/sdf_usd_parser/light.cc for more information)
    }
    else
    {
      validLight = false;
    }

    EXPECT_TRUE(validLight);
    if (validLight)
      this->CheckPrimPose(lightUsd, usd::PoseWrtParent(lightSdf));
  }
  EXPECT_EQ(1, numPointLights);
  EXPECT_EQ(1, numSpotLights);
  EXPECT_EQ(1, numDirectionalLights);
}

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Models)
{
  // TODO(adlarkin) implement this
}
