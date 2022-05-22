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

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

#include <ignition/math/Pose3.hh>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdLux/diskLight.h>
#include <pxr/usd/usdLux/distantLight.h>
#include <pxr/usd/usdLux/sphereLight.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Light.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "test_config.h"
#include "test_utils.hh"
#include "Light.hh"
#include "Model.hh"
#include "../UsdTestUtils.hh"
#include "../UsdUtils.hh"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
// This fixture also has helper methods to assist with comparing SDF/USD light
// values
class UsdLightStageFixture : public ::testing::Test
{
  public: UsdLightStageFixture() = default;

  /// \brief Compare the intensity between a USD and SDF light
  /// \param[in] _lightPrim The USD light prim
  /// \param[in] _sdfLight The SDF light
  public: void CheckLightIntensity(const pxr::UsdPrim &_lightPrim,
              const sdf::Light &_sdfLight)
  {
    const float targetIntensity = _sdfLight.Intensity() * 100.0f;

    bool checkedIntensity = false;
    if (auto intensityAttr = _lightPrim.GetAttribute(pxr::TfToken("intensity")))
    {
      float intensityVal = 0.0;
      intensityAttr.Get(&intensityVal);
      EXPECT_FLOAT_EQ(intensityVal, targetIntensity);
      checkedIntensity = true;
    }
    EXPECT_TRUE(checkedIntensity);

    bool checkedInputIntensity = false;
    if (auto intensityAttr = _lightPrim.GetAttribute(
          pxr::TfToken("inputs:intensity")))
    {
      float intensityVal = 0.0;
      intensityAttr.Get(&intensityVal);
      EXPECT_FLOAT_EQ(intensityVal, targetIntensity);
      checkedInputIntensity = true;
    }
    EXPECT_TRUE(checkedInputIntensity);
  }

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
    ASSERT_TRUE(this->stage);
  }

  public: pxr::UsdStageRefPtr stage;
};

/////////////////////////////////////////////////
TEST_F(UsdLightStageFixture, Lights)
{
  const auto path = sdf::testing::TestFile("sdf", "lights.sdf");
  sdf::Root root;

  // load the world in the SDF file
  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0u);

  // convert all lights attached directly to the world to USD
  std::unordered_map<std::string, sdf::Light> lightPathToSdf;
  for (unsigned int i = 0; i < world->LightCount(); ++i)
  {
    const auto light = *(world->LightByIndex(i));
    const auto lightPath = std::string("/" + light.Name());
    lightPathToSdf[lightPath] = light;
    const auto errors = sdf::usd::ParseSdfLight(light, this->stage, lightPath);
    EXPECT_TRUE(errors.empty());
  }
  EXPECT_EQ(world->LightCount(), lightPathToSdf.size());

  // parse all of the world's models and convert them to USD.
  // Models can have lights attached to their links
  for (uint64_t i = 0; i < world->ModelCount(); ++i)
  {
    // create a dummy world path so that we can call the sdf::usd::ParseSdfModel
    // API
    const auto worldPath = pxr::SdfPath("/" + world->Name());

    const auto model = *(world->ModelByIndex(i));
    const auto modelPath = std::string("/" + model.Name());
    const auto errors =
      sdf::usd::ParseSdfModel(model, this->stage, modelPath, worldPath);
    EXPECT_TRUE(errors.empty());

    // save the model's USD light paths so that they can be verified later
    for (uint64_t j = 0; j < model.LinkCount(); ++j)
    {
      const auto link = *(model.LinkByIndex(j));
      auto lightPathPrefix = modelPath + "/" + link.Name();

      for (uint64_t k = 0; k < link.LightCount(); ++k)
      {
        const auto light = *(link.LightByIndex(k));
        const auto lightPath = lightPathPrefix + "/" + light.Name();
        lightPathToSdf[lightPath] = light;
      }
    }
  }

  // check that the lights were parsed correctly
  int numPointLights = 0;
  int numSpotLights = 0;
  int numDirectionalLights = 0;
  for (const auto &prim : this->stage->Traverse())
  {
    if (!(prim.IsA<pxr::UsdLuxBoundableLightBase>() ||
          prim.IsA<pxr::UsdLuxNonboundableLightBase>()))
    {
      continue;
    }
    auto iter = lightPathToSdf.find(pxr::TfStringify(prim.GetPath()));
    ASSERT_NE(lightPathToSdf.end(), iter);
    const auto lightUsd = this->stage->GetPrimAtPath(pxr::SdfPath(iter->first));
    const auto lightSdf = iter->second;

    bool validLight = true;
    if (lightUsd.IsA<pxr::UsdLuxSphereLight>())
    {
      numPointLights++;

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
    }
    else if (lightUsd.IsA<pxr::UsdLuxDistantLight>())
    {
      numDirectionalLights++;
    }
    else
    {
      validLight = false;
    }

    EXPECT_TRUE(validLight);
    if (validLight)
    {
      this->CheckLightIntensity(lightUsd, lightSdf);
      gz::math::Pose3d pose;
      const auto poseErrors = sdf::usd::PoseWrtParent(lightSdf, pose);
      EXPECT_TRUE(poseErrors.empty());
      sdf::usd::testing::CheckPrimPose(lightUsd, pose);
    }
  }
  EXPECT_EQ(2, numPointLights);
  EXPECT_EQ(2, numSpotLights);
  EXPECT_EQ(2, numDirectionalLights);
}
