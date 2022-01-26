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

#include <ignition/common/Util.hh>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/sdf_parser/World.hh"
#include "sdf/Root.hh"
#include "test_config.h"
#include "test_utils.hh"
#include "UsdTestUtils.hh"

void CheckMaterial(
  const pxr::UsdPrim _prim,
  const pxr::GfVec3f _diffuseColor,
  const pxr::GfVec3f _emissiveColor,
  const std::string &_albedoName,
  const std::string &_normalName,
  const std::string &_roughnessName,
  const std::string &_metallicName,
  float _metallicConstant = 0.5,
  bool _enableEmission = true)
{
  auto variantshader = pxr::UsdShadeShader(_prim);
  EXPECT_TRUE(variantshader);

  std::vector<pxr::UsdShadeInput> inputs = variantshader.GetInputs();
  pxr::GfVec3f diffuseColor {0, 0, 0};
  pxr::GfVec3f emissiveColor {0, 0, 0};

  for (auto &input : inputs)
  {
    if (input.GetBaseName() == "diffuse_texture")
    {
      pxr::SdfAssetPath materialPathUSD;
      pxr::UsdShadeInput diffuseTextureShaderInput =
        variantshader.GetInput(pxr::TfToken("diffuse_texture"));
      auto source = diffuseTextureShaderInput.GetConnectedSources();
      diffuseTextureShaderInput.Get(&materialPathUSD);
      EXPECT_EQ(_albedoName,
        materialPathUSD.GetAssetPath());
    }
    else if (input.GetBaseName() == "metallic_constant")
    {
      pxr::UsdShadeInput metallicConstantShaderInput =
        variantshader.GetInput(pxr::TfToken("metallic_constant"));
      float metallicConstant;
      metallicConstantShaderInput.Get(&metallicConstant);
      EXPECT_FLOAT_EQ(_metallicConstant, metallicConstant);
    }
    else if (input.GetBaseName() == "enable_emission")
    {
      bool enableEmission;
      pxr::UsdShadeInput enableEmissiveShaderInput =
        variantshader.GetInput(pxr::TfToken("enable_emission"));
      enableEmissiveShaderInput.Get(&enableEmission);
      EXPECT_EQ(_enableEmission, enableEmission);
    }
    else if (input.GetBaseName() == "normalmap_texture")
    {
      pxr::SdfAssetPath materialPath;
      pxr::UsdShadeInput normalTextureShaderInput =
        variantshader.GetInput(pxr::TfToken("normalmap_texture"));
      auto source = normalTextureShaderInput.GetConnectedSources();
      normalTextureShaderInput.Get(&materialPath);
      EXPECT_EQ(_normalName, materialPath.GetAssetPath());
    }
    else if (input.GetBaseName() == "reflectionroughness_texture")
    {
      pxr::SdfAssetPath materialPath;
      pxr::UsdShadeInput roughnessTextureShaderInput =
        variantshader.GetInput(pxr::TfToken("reflectionroughness_texture"));
      auto source = roughnessTextureShaderInput.GetConnectedSources();
      roughnessTextureShaderInput.Get(&materialPath);
      EXPECT_EQ(_roughnessName, materialPath.GetAssetPath());
    }
    else if (input.GetBaseName() == "metallic_texture")
    {
      pxr::SdfAssetPath materialPath;
      pxr::UsdShadeInput metallicTextureShaderInput =
        variantshader.GetInput(pxr::TfToken("metallic_texture"));
      auto source = metallicTextureShaderInput.GetConnectedSources();
      metallicTextureShaderInput.Get(&materialPath);
      EXPECT_EQ(_metallicName, materialPath.GetAssetPath());
    }
    else if (input.GetBaseName() == "emissive_color")
    {
      pxr::UsdShadeInput emissiveColorShaderInput =
        variantshader.GetInput(pxr::TfToken("emissive_color"));
      if (emissiveColorShaderInput.Get(&emissiveColor))
      {
        EXPECT_EQ(_emissiveColor, emissiveColor);
      }
    }
    else if (input.GetBaseName() == "diffuse_color_constant")
    {
      auto sourceInfoV = input.GetConnectedSources();
      if (sourceInfoV.size() > 0)
      {
        pxr::UsdShadeInput connectedInput =
          sourceInfoV[0].source.GetInput(sourceInfoV[0].sourceName);

        const pxr::SdfPath& thisAttrPath = connectedInput.GetAttr().GetPath();
        auto connectedPrim = _prim.GetStage()->GetPrimAtPath(
          thisAttrPath.GetPrimPath());
        if(connectedPrim)
          connectedPrim.GetAttribute(
            pxr::TfToken("inputs:diffuse_color_constant")).Get(&diffuseColor);
      }
      {
        pxr::UsdShadeInput diffuseShaderInput =
          variantshader.GetInput(pxr::TfToken("diffuse_color_constant"));
        diffuseShaderInput.Get(&diffuseColor);
      }
      EXPECT_EQ(_diffuseColor, diffuseColor);
    }
  }
}

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
TEST_F(UsdStageFixture, Material)
{
  sdf::setFindCallback(sdf::usd::testing::findFileCb);
  ignition::common::addFindFileURICallback(
    std::bind(&sdf::usd::testing::FindResourceUri, std::placeholders::_1));

  const auto path = sdf::testing::TestFile("sdf", "basic_shapes.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);

  std::string meshPath = worldPath + "/" + "mesh";
  std::string meshLinkPath = meshPath + "/" + "link";
  std::string meshVisualPath = meshLinkPath + "/" + "visual";
  std::string meshGeometryPath =
    meshVisualPath + "/" + "geometry";
  auto meshGeometry = this->stage->GetPrimAtPath(
    pxr::SdfPath(meshGeometryPath));
  ASSERT_TRUE(meshGeometry);

  {
    std::string materialPath = "/Looks/Material_2";
    auto materialPrim = this->stage->GetPrimAtPath(
      pxr::SdfPath(materialPath));
    EXPECT_TRUE(materialPrim);

    std::string materialshaderPath = materialPath + "/Shader";
    auto materialShaderPrim = this->stage->GetPrimAtPath(
      pxr::SdfPath(materialshaderPath));
    EXPECT_TRUE(materialShaderPrim);

    CheckMaterial(materialShaderPrim,
      pxr::GfVec3f(0.2, 0.5, 0.1),
      pxr::GfVec3f(0, 0, 0),
      "materials/textures/albedo_map.png",
      "materials/textures/normal_map.png",
      "materials/textures/roughness_map.png",
      "materials/textures/metallic_map.png"
    );
  }

  {
    std::string materialPath = "/Looks/Material_0";
    auto materialPrim = this->stage->GetPrimAtPath(
      pxr::SdfPath(materialPath));
    EXPECT_TRUE(materialPrim);

    std::string materialshaderPath = materialPath + "/Shader";
    auto materialShaderPrim = this->stage->GetPrimAtPath(
      pxr::SdfPath(materialshaderPath));
    EXPECT_TRUE(materialShaderPrim);

    CheckMaterial(materialShaderPrim,
      pxr::GfVec3f(0, 0.1, 0.2),
      pxr::GfVec3f(0.12, 0.23, 0.34),
      "",
      "",
      "",
      ""
    );
  }
}
