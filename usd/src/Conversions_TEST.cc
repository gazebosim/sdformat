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

#include <memory>

#include <gtest/gtest.h>

#include <gz/common/Material.hh>
#include <gz/common/Pbr.hh>
#include <gz/math/Color.hh>

#include "sdf/Material.hh"
#include "sdf/Pbr.hh"
#include "Conversions.hh"

/////////////////////////////////////////////////
TEST(Conversions, SdfToCommonMaterial)
{
  sdf::Material material;
  material.SetEmissive(gz::math::Color(1, 0.2, 0.2, 0.7));
  material.SetDiffuse(gz::math::Color(0.1, 0.3, 0.4, 0.5));
  material.SetSpecular(gz::math::Color(0.11, 0.22, 0.23, 0.77));
  material.SetAmbient(gz::math::Color(0.25, 0.21, 0.28, 0.5));
  material.SetRenderOrder(5);
  material.SetLighting(true);
  material.SetDoubleSided(false);

  sdf::Pbr pbrSDF;
  sdf::PbrWorkflow pbrWorkflow;

  pbrWorkflow.SetAlbedoMap("AlbedoMap");
  pbrWorkflow.SetMetalnessMap("MetalnessMap");
  pbrWorkflow.SetEmissiveMap("EmissiveMap");
  pbrWorkflow.SetRoughnessMap("RoughnessMap");
  pbrWorkflow.SetSpecularMap("SpecularMap");
  pbrWorkflow.SetEnvironmentMap("EnvironmentMap");
  pbrWorkflow.SetAmbientOcclusionMap("AmbientOcclusionMap");
  pbrWorkflow.SetLightMap("LightMap");
  pbrWorkflow.SetNormalMap(
    "NormalMap", sdf::NormalMapSpace::TANGENT);
  pbrWorkflow.SetRoughness(0.2);
  pbrWorkflow.SetGlossiness(0.3);
  pbrWorkflow.SetMetalness(0.55);

  pbrSDF.SetWorkflow(sdf::PbrWorkflowType::METAL, pbrWorkflow);
  material.SetPbrMaterial(pbrSDF);

  gz::common::Material materialCommon;
  sdf::usd::convert(material, materialCommon);
  const gz::common::Pbr * pbrCommon = materialCommon.PbrMaterial();
  ASSERT_NE(nullptr, pbrCommon);

  EXPECT_EQ(material.Emissive(), materialCommon.Emissive());
  EXPECT_EQ(material.Diffuse(), materialCommon.Diffuse());
  EXPECT_EQ(material.Specular(), materialCommon.Specular());
  EXPECT_EQ(material.Ambient(), materialCommon.Ambient());
  EXPECT_FLOAT_EQ(material.RenderOrder(),
    static_cast<float>(materialCommon.RenderOrder()));
  EXPECT_EQ(material.Lighting(), materialCommon.Lighting());
  EXPECT_EQ(material.DoubleSided(), materialCommon.TwoSidedEnabled());

  EXPECT_EQ(pbrWorkflow.AlbedoMap(), pbrCommon->AlbedoMap());
  EXPECT_EQ(pbrWorkflow.MetalnessMap(), pbrCommon->MetalnessMap());
  EXPECT_EQ(pbrWorkflow.EmissiveMap(), pbrCommon->EmissiveMap());
  EXPECT_EQ(pbrWorkflow.RoughnessMap(), pbrCommon->RoughnessMap());
  EXPECT_EQ(pbrWorkflow.SpecularMap(), pbrCommon->SpecularMap());
  EXPECT_EQ(pbrWorkflow.EnvironmentMap(), pbrCommon->EnvironmentMap());
  EXPECT_EQ(pbrWorkflow.AmbientOcclusionMap(),
    pbrCommon->AmbientOcclusionMap());
  EXPECT_EQ(pbrWorkflow.LightMap(), pbrCommon->LightMap());
  EXPECT_EQ(pbrWorkflow.NormalMap(), pbrCommon->NormalMap());

  EXPECT_EQ(gz::common::NormalMapSpace::TANGENT,
    pbrCommon->NormalMapType());
  EXPECT_EQ(gz::common::PbrType::METAL, pbrCommon->Type());

  EXPECT_DOUBLE_EQ(pbrWorkflow.Roughness(), pbrCommon->Roughness());
  EXPECT_DOUBLE_EQ(pbrWorkflow.Glossiness(), pbrCommon->Glossiness());
  EXPECT_DOUBLE_EQ(pbrWorkflow.Metalness(), pbrCommon->Metalness());
}

TEST(Conversions, CommonToSdfMaterial)
{
  gz::common::Material materialCommon;
  materialCommon.SetEmissive(gz::math::Color(1, 0.2, 0.2, 0.7));
  materialCommon.SetDiffuse(gz::math::Color(0.1, 0.3, 0.4, 0.5));
  materialCommon.SetSpecular(gz::math::Color(0.11, 0.22, 0.23, 0.77));
  materialCommon.SetAmbient(gz::math::Color(0.25, 0.21, 0.28, 0.5));
  materialCommon.SetRenderOrder(5);
  materialCommon.SetLighting(true);
  materialCommon.SetAlphaFromTexture(false, 0.5, true);

  gz::common::Pbr pbrCommon;
  pbrCommon.SetType(gz::common::PbrType::METAL);

  pbrCommon.SetAlbedoMap("AlbedoMap");
  pbrCommon.SetMetalnessMap("MetalnessMap");
  pbrCommon.SetEmissiveMap("EmissiveMap");
  pbrCommon.SetRoughnessMap("RoughnessMap");
  pbrCommon.SetSpecularMap("SpecularMap");
  pbrCommon.SetEnvironmentMap("EnvironmentMap");
  pbrCommon.SetAmbientOcclusionMap("AmbientOcclusionMap");
  pbrCommon.SetLightMap("LightMap");
  pbrCommon.SetNormalMap(
    "NormalMap", gz::common::NormalMapSpace::TANGENT);
  pbrCommon.SetRoughness(0.2);
  pbrCommon.SetGlossiness(0.3);
  pbrCommon.SetMetalness(0.55);

  materialCommon.SetPbrMaterial(pbrCommon);

  const sdf::Material material = sdf::usd::convert(&materialCommon);

  const sdf::Pbr * pbr = material.PbrMaterial();
  ASSERT_NE(nullptr, pbr);
  const sdf::PbrWorkflow * pbrWorkflow =
    pbr->Workflow(sdf::PbrWorkflowType::METAL);
  ASSERT_NE(nullptr, pbrWorkflow);

  EXPECT_EQ(material.Emissive(), materialCommon.Emissive());
  EXPECT_EQ(material.Diffuse(), materialCommon.Diffuse());
  EXPECT_EQ(material.Specular(), materialCommon.Specular());
  EXPECT_EQ(material.Ambient(), materialCommon.Ambient());
  EXPECT_FLOAT_EQ(material.RenderOrder(),
    static_cast<float>(materialCommon.RenderOrder()));
  EXPECT_EQ(material.Lighting(), materialCommon.Lighting());
  EXPECT_EQ(material.DoubleSided(), materialCommon.TwoSidedEnabled());

  EXPECT_EQ(pbrWorkflow->AlbedoMap(), pbrCommon.AlbedoMap());
  EXPECT_EQ(pbrWorkflow->MetalnessMap(), pbrCommon.MetalnessMap());
  EXPECT_EQ(pbrWorkflow->EmissiveMap(), pbrCommon.EmissiveMap());
  EXPECT_EQ(pbrWorkflow->RoughnessMap(), pbrCommon.RoughnessMap());
  EXPECT_EQ(pbrWorkflow->SpecularMap(), pbrCommon.SpecularMap());
  EXPECT_EQ(pbrWorkflow->EnvironmentMap(), pbrCommon.EnvironmentMap());
  EXPECT_EQ(pbrWorkflow->AmbientOcclusionMap(),
    pbrCommon.AmbientOcclusionMap());
  EXPECT_EQ(pbrWorkflow->LightMap(), pbrCommon.LightMap());
  EXPECT_EQ(pbrWorkflow->NormalMap(), pbrCommon.NormalMap());

  EXPECT_EQ(gz::common::NormalMapSpace::TANGENT,
      pbrCommon.NormalMapType());
  EXPECT_EQ(gz::common::PbrType::METAL, pbrCommon.Type());

  EXPECT_DOUBLE_EQ(pbrWorkflow->Roughness(), pbrCommon.Roughness());
  EXPECT_DOUBLE_EQ(pbrWorkflow->Glossiness(), pbrCommon.Glossiness());
  EXPECT_DOUBLE_EQ(pbrWorkflow->Metalness(), pbrCommon.Metalness());
}
