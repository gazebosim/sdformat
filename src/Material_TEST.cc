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

#include <gz/math/Color.hh>
#include "sdf/Material.hh"
#include "sdf/Pbr.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMMaterial, Construction)
{
  sdf::Material material;
  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Ambient());
  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Diffuse());
  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Specular());
  EXPECT_DOUBLE_EQ(0.0, material.Shininess());
  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Emissive());
  EXPECT_TRUE(material.Lighting());
  EXPECT_FLOAT_EQ(0, material.RenderOrder());
  EXPECT_FALSE(material.DoubleSided());
  EXPECT_EQ(nullptr, material.Element());
  EXPECT_EQ("", material.ScriptUri());
  EXPECT_EQ("", material.ScriptName());
  EXPECT_EQ(sdf::ShaderType::PIXEL, material.Shader());
  EXPECT_EQ("", material.NormalMap());
  EXPECT_EQ(nullptr, material.PbrMaterial());
  EXPECT_EQ("", material.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, MoveConstructor)
{
  sdf::Material material;
  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f));
  material.SetDiffuse(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f));
  material.SetSpecular(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f));
  material.SetShininess(5.0);
  material.SetEmissive(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f));
  material.SetLighting(false);
  material.SetRenderOrder(2);
  material.SetDoubleSided(true);
  material.SetScriptUri("banana");
  material.SetScriptName("orange");
  material.SetShader(sdf::ShaderType::VERTEX);
  material.SetNormalMap("blueberry");
  material.SetFilePath("/tmp/path");

  sdf::Material material2(std::move(material));
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f), material2.Ambient());
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f), material2.Diffuse());
  EXPECT_EQ(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f),
      material2.Specular());
  EXPECT_DOUBLE_EQ(5.0, material2.Shininess());
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f),
      material2.Emissive());
  EXPECT_FALSE(material2.Lighting());
  EXPECT_TRUE(material2.DoubleSided());
  EXPECT_FLOAT_EQ(2.0, material2.RenderOrder());
  EXPECT_EQ("banana", material2.ScriptUri());
  EXPECT_EQ("orange", material2.ScriptName());
  EXPECT_EQ(sdf::ShaderType::VERTEX, material2.Shader());
  EXPECT_EQ("blueberry", material2.NormalMap());
  EXPECT_EQ(nullptr, material2.PbrMaterial());
  EXPECT_EQ("/tmp/path", material2.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, CopyConstructor)
{
  sdf::Material material;
  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f));
  material.SetDiffuse(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f));
  material.SetSpecular(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f));
  material.SetShininess(5.0);
  material.SetEmissive(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f));
  material.SetLighting(false);
  material.SetRenderOrder(4);
  material.SetDoubleSided(true);
  material.SetScriptUri("banana");
  material.SetScriptName("orange");
  material.SetShader(sdf::ShaderType::VERTEX);
  material.SetNormalMap("blueberry");
  material.SetFilePath("/tmp/other");

  sdf::Material material2(material);
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f), material2.Ambient());
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f), material2.Diffuse());
  EXPECT_EQ(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f),
      material2.Specular());
  EXPECT_DOUBLE_EQ(5.0, material2.Shininess());
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f),
      material2.Emissive());
  EXPECT_FALSE(material2.Lighting());
  EXPECT_TRUE(material2.DoubleSided());
  EXPECT_FLOAT_EQ(4, material2.RenderOrder());
  EXPECT_EQ("banana", material2.ScriptUri());
  EXPECT_EQ("orange", material2.ScriptName());
  EXPECT_EQ(sdf::ShaderType::VERTEX, material2.Shader());
  EXPECT_EQ("blueberry", material2.NormalMap());
  EXPECT_EQ(nullptr, material2.PbrMaterial());
  EXPECT_EQ("/tmp/other", material2.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, AssignmentOperator)
{
  sdf::Material material;
  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f));
  material.SetDiffuse(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f));
  material.SetSpecular(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f));
  material.SetShininess(5.0);
  material.SetEmissive(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f));
  material.SetLighting(false);
  material.SetRenderOrder(4);
  material.SetDoubleSided(true);
  material.SetScriptUri("banana");
  material.SetScriptName("orange");
  material.SetShader(sdf::ShaderType::VERTEX);
  material.SetNormalMap("blueberry");
  material.SetFilePath("/tmp/another");

  sdf::Material material2;
  material2 = material;
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f), material2.Ambient());
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f), material2.Diffuse());
  EXPECT_EQ(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f),
      material2.Specular());
  EXPECT_DOUBLE_EQ(5.0, material2.Shininess());
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f),
      material2.Emissive());
  EXPECT_FALSE(material2.Lighting());
  EXPECT_TRUE(material2.DoubleSided());
  EXPECT_FLOAT_EQ(4, material2.RenderOrder());
  EXPECT_EQ("banana", material2.ScriptUri());
  EXPECT_EQ("orange", material2.ScriptName());
  EXPECT_EQ(sdf::ShaderType::VERTEX, material2.Shader());
  EXPECT_EQ("blueberry", material2.NormalMap());
  EXPECT_EQ(nullptr, material2.PbrMaterial());
  EXPECT_EQ("/tmp/another", material2.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, MoveAssignmentOperator)
{
  sdf::Material material;
  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f));
  material.SetDiffuse(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f));
  material.SetSpecular(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f));
  material.SetShininess(5.0);
  material.SetEmissive(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f));
  material.SetLighting(false);
  material.SetRenderOrder(4);
  material.SetDoubleSided(true);
  material.SetScriptUri("banana");
  material.SetScriptName("orange");
  material.SetShader(sdf::ShaderType::VERTEX);
  material.SetNormalMap("blueberry");

  sdf::Material material2;
  material2 = std::move(material);
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f), material2.Ambient());
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f), material2.Diffuse());
  EXPECT_EQ(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f),
      material2.Specular());
  EXPECT_DOUBLE_EQ(5.0, material2.Shininess());
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f),
      material2.Emissive());
  EXPECT_FALSE(material2.Lighting());
  EXPECT_TRUE(material2.DoubleSided());
  EXPECT_FLOAT_EQ(4, material2.RenderOrder());
  EXPECT_EQ("banana", material2.ScriptUri());
  EXPECT_EQ("orange", material2.ScriptName());
  EXPECT_EQ(sdf::ShaderType::VERTEX, material2.Shader());
  EXPECT_EQ("blueberry", material2.NormalMap());
  EXPECT_EQ(nullptr, material2.PbrMaterial());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, CopyAssignmentAfterMove)
{
  sdf::Material material1;
  material1.SetScriptUri("material1");

  sdf::Material material2;
  material2.SetScriptUri("material2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Material tmp = std::move(material1);
  material1 = material2;
  material2 = tmp;

  EXPECT_EQ("material2", material1.ScriptUri());
  EXPECT_EQ("material1", material2.ScriptUri());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, Set)
{
  sdf::Material material;
  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Ambient());
  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f));
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f), material.Ambient());

  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Diffuse());
  material.SetDiffuse(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f));
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f), material.Diffuse());

  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Specular());
  material.SetSpecular(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f));
  EXPECT_EQ(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f), material.Specular());

  EXPECT_DOUBLE_EQ(0.0, material.Shininess());
  material.SetShininess(5.0);
  EXPECT_DOUBLE_EQ(5.0, material.Shininess());

  EXPECT_EQ(gz::math::Color(0, 0, 0, 1), material.Emissive());
  material.SetEmissive(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f));
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f), material.Emissive());

  EXPECT_TRUE(material.Lighting());
  material.SetLighting(false);
  EXPECT_FALSE(material.Lighting());

  EXPECT_FLOAT_EQ(0, material.RenderOrder());
  material.SetRenderOrder(5);
  EXPECT_FLOAT_EQ(5, material.RenderOrder());

  EXPECT_FALSE(material.DoubleSided());
  material.SetDoubleSided(true);
  EXPECT_TRUE(material.DoubleSided());

  EXPECT_EQ("", material.ScriptUri());
  material.SetScriptUri("uri");
  EXPECT_EQ("uri", material.ScriptUri());

  EXPECT_EQ("", material.ScriptName());
  material.SetScriptName("name");
  EXPECT_EQ("name", material.ScriptName());

  EXPECT_EQ(sdf::ShaderType::PIXEL, material.Shader());
  material.SetShader(sdf::ShaderType::VERTEX);
  EXPECT_EQ(sdf::ShaderType::VERTEX, material.Shader());

  EXPECT_EQ("", material.NormalMap());
  material.SetNormalMap("map");
  EXPECT_EQ("map", material.NormalMap());

  EXPECT_EQ("", material.FilePath());
  material.SetFilePath("/my/path");
  EXPECT_EQ("/my/path", material.FilePath());

  // set pbr material
  sdf::Pbr pbr;
  sdf::PbrWorkflow workflow;
  workflow.SetType(sdf::PbrWorkflowType::METAL);
  pbr.SetWorkflow(workflow.Type(), workflow);
  material.SetPbrMaterial(pbr);
  EXPECT_NE(material.PbrMaterial(), nullptr);
  EXPECT_EQ(workflow,
      *material.PbrMaterial()->Workflow(sdf::PbrWorkflowType::METAL));

  // Move the material
  sdf::Material moved(std::move(material));
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f, 0.5f), moved.Ambient());
  EXPECT_EQ(gz::math::Color(0.2f, 0.3f, 0.4f, 0.6f), moved.Diffuse());
  EXPECT_EQ(gz::math::Color(0.3f, 0.4f, 0.5f, 0.7f), moved.Specular());
  EXPECT_EQ(gz::math::Color(0.4f, 0.5f, 0.6f, 0.8f), moved.Emissive());
  EXPECT_FALSE(moved.Lighting());
  EXPECT_FLOAT_EQ(5, moved.RenderOrder());
  EXPECT_TRUE(moved.DoubleSided());
  EXPECT_EQ("uri", moved.ScriptUri());
  EXPECT_EQ("name", moved.ScriptName());
  EXPECT_EQ(sdf::ShaderType::VERTEX, moved.Shader());
  EXPECT_EQ("map", moved.NormalMap());
  EXPECT_EQ(workflow,
      *moved.PbrMaterial()->Workflow(sdf::PbrWorkflowType::METAL));
  EXPECT_EQ("/my/path", moved.FilePath());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, InvalidSdf)
{
  sdf::Material material;
  sdf::ElementPtr elem(new sdf::Element());
  elem->SetName("bad");
  sdf::Errors errors = material.Load(elem);
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, ToElement)
{
  sdf::Material material;

  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  material.SetDiffuse(gz::math::Color(0.4f, 0.5f, 0.6f, 0.5f));
  material.SetSpecular(gz::math::Color(0.6f, 0.4f, 0.8f, 0.8f));
  material.SetEmissive(gz::math::Color(0.2f, 0.7f, 0.9f, 0.1f));

  material.SetRenderOrder(0.5);
  material.SetLighting(false);
  material.SetDoubleSided(true);
  material.SetScriptUri("my-uri");
  material.SetScriptName("my-script-name");
  material.SetShader(sdf::ShaderType::VERTEX);
  material.SetNormalMap("my-normal-map");

  sdf::PbrWorkflow workflow;
  workflow.SetAlbedoMap("albedo");
  workflow.SetNormalMap("normal");
  workflow.SetEnvironmentMap("env");
  workflow.SetAmbientOcclusionMap("ambient");
  workflow.SetRoughnessMap("rough");
  workflow.SetMetalnessMap("metalness");
  workflow.SetEmissiveMap("emissive");
  workflow.SetGlossinessMap("gloss");
  workflow.SetSpecularMap("gloss");
  workflow.SetLightMap("light", 1u);
  workflow.SetMetalness(1.0);
  workflow.SetRoughness(0.1);

  // Testing using METAL workflow
  {
    sdf::Pbr pbr;
    workflow.SetType(sdf::PbrWorkflowType::METAL);
    pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, workflow);
    material.SetPbrMaterial(pbr);

    sdf::ElementPtr elem = material.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Material material2;
    material2.Load(elem);

    EXPECT_EQ(material.Ambient(), material2.Ambient());
    EXPECT_EQ(material.Diffuse(), material2.Diffuse());
    EXPECT_EQ(material.Specular(), material2.Specular());
    EXPECT_EQ(material.Emissive(), material2.Emissive());
    EXPECT_DOUBLE_EQ(material.RenderOrder(), material2.RenderOrder());
    EXPECT_EQ(material.Lighting(), material2.Lighting());
    EXPECT_EQ(material.DoubleSided(), material2.DoubleSided());
    EXPECT_EQ(material.ScriptUri(), material2.ScriptUri());
    EXPECT_EQ(material.ScriptName(), material2.ScriptName());
    EXPECT_EQ(material.Shader(), material2.Shader());
    EXPECT_EQ(material.NormalMap(), material2.NormalMap());

    const sdf::Pbr *pbr2 = material2.PbrMaterial();
    const sdf::PbrWorkflow *flow1 = pbr.Workflow(sdf::PbrWorkflowType::METAL);
    const sdf::PbrWorkflow *flow2 = pbr2->Workflow(sdf::PbrWorkflowType::METAL);
    EXPECT_EQ(flow1->AlbedoMap(), flow2->AlbedoMap());
    EXPECT_EQ(flow1->NormalMap(), flow2->NormalMap());
    EXPECT_EQ(flow1->AmbientOcclusionMap(),
              flow2->AmbientOcclusionMap());
    EXPECT_EQ(flow1->RoughnessMap(), flow2->RoughnessMap());
    EXPECT_EQ(flow1->MetalnessMap(), flow2->MetalnessMap());
    EXPECT_EQ(flow1->EmissiveMap(), flow2->EmissiveMap());
    EXPECT_EQ(flow1->LightMap(), flow2->LightMap());
    EXPECT_DOUBLE_EQ(flow1->Metalness(), flow2->Metalness());
    EXPECT_DOUBLE_EQ(flow1->Roughness(), flow2->Roughness());
    EXPECT_EQ(flow1->Type(), flow2->Type());
  }

  // Testing using SPECULAR workflow
  {
    sdf::Pbr pbr;
    workflow.SetType(sdf::PbrWorkflowType::SPECULAR);
    pbr.SetWorkflow(sdf::PbrWorkflowType::SPECULAR, workflow);
    material.SetPbrMaterial(pbr);

    sdf::ElementPtr elem = material.ToElement();
    ASSERT_NE(nullptr, elem);

    sdf::Material material2;
    material2.Load(elem);

    EXPECT_EQ(material.Ambient(), material2.Ambient());
    EXPECT_EQ(material.Diffuse(), material2.Diffuse());
    EXPECT_EQ(material.Specular(), material2.Specular());
    EXPECT_EQ(material.Emissive(), material2.Emissive());
    EXPECT_DOUBLE_EQ(material.RenderOrder(), material2.RenderOrder());
    EXPECT_EQ(material.Lighting(), material2.Lighting());
    EXPECT_EQ(material.DoubleSided(), material2.DoubleSided());
    EXPECT_EQ(material.ScriptUri(), material2.ScriptUri());
    EXPECT_EQ(material.ScriptName(), material2.ScriptName());
    EXPECT_EQ(material.Shader(), material2.Shader());
    EXPECT_EQ(material.NormalMap(), material2.NormalMap());

    const sdf::Pbr *pbr2 = material2.PbrMaterial();
    const sdf::PbrWorkflow *flow1 =
      pbr.Workflow(sdf::PbrWorkflowType::SPECULAR);
    const sdf::PbrWorkflow *flow2 =
      pbr2->Workflow(sdf::PbrWorkflowType::SPECULAR);
    EXPECT_EQ(flow1->AlbedoMap(), flow2->AlbedoMap());
    EXPECT_EQ(flow1->NormalMap(), flow2->NormalMap());
    EXPECT_EQ(flow1->EnvironmentMap(), flow2->EnvironmentMap());
    EXPECT_EQ(flow1->AmbientOcclusionMap(),
              flow2->AmbientOcclusionMap());
    EXPECT_EQ(flow1->EmissiveMap(), flow2->EmissiveMap());
    EXPECT_EQ(flow1->GlossinessMap(), flow2->GlossinessMap());
    EXPECT_EQ(flow1->SpecularMap(), flow2->SpecularMap());
    EXPECT_EQ(flow1->LightMap(), flow2->LightMap());
    EXPECT_EQ(flow1->Type(), flow2->Type());
  }
}

/////////////////////////////////////////////////
TEST(DOMMaterial, ToElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  sdf::Material material;
  sdf::Errors errors;

  material.SetAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  material.SetDiffuse(gz::math::Color(0.4f, 0.5f, 0.6f, 0.5f));
  material.SetSpecular(gz::math::Color(0.6f, 0.4f, 0.8f, 0.8f));
  material.SetEmissive(gz::math::Color(0.2f, 0.7f, 0.9f, 0.1f));

  material.SetRenderOrder(0.5);
  material.SetLighting(false);
  material.SetDoubleSided(true);
  material.SetScriptUri("my-uri");
  material.SetScriptName("my-script-name");
  material.SetShader(sdf::ShaderType::VERTEX);
  material.SetNormalMap("my-normal-map");

  sdf::PbrWorkflow workflow;
  workflow.SetAlbedoMap("albedo");
  workflow.SetNormalMap("normal");
  workflow.SetEnvironmentMap("env");
  workflow.SetAmbientOcclusionMap("ambient");
  workflow.SetRoughnessMap("rough");
  workflow.SetMetalnessMap("metalness");
  workflow.SetEmissiveMap("emissive");
  workflow.SetGlossinessMap("gloss");
  workflow.SetSpecularMap("gloss");
  workflow.SetLightMap("light", 1u);
  workflow.SetMetalness(1.0);
  workflow.SetRoughness(0.1);

  // Testing using METAL workflow
  {
    sdf::Pbr pbr;
    workflow.SetType(sdf::PbrWorkflowType::METAL);
    pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, workflow);
    material.SetPbrMaterial(pbr);

    sdf::ElementPtr elem = material.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Material material2;
    errors = material2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(material.Ambient(), material2.Ambient());
    EXPECT_EQ(material.Diffuse(), material2.Diffuse());
    EXPECT_EQ(material.Specular(), material2.Specular());
    EXPECT_EQ(material.Emissive(), material2.Emissive());
    EXPECT_DOUBLE_EQ(material.RenderOrder(), material2.RenderOrder());
    EXPECT_EQ(material.Lighting(), material2.Lighting());
    EXPECT_EQ(material.DoubleSided(), material2.DoubleSided());
    EXPECT_EQ(material.ScriptUri(), material2.ScriptUri());
    EXPECT_EQ(material.ScriptName(), material2.ScriptName());
    EXPECT_EQ(material.Shader(), material2.Shader());
    EXPECT_EQ(material.NormalMap(), material2.NormalMap());

    const sdf::Pbr *pbr2 = material2.PbrMaterial();
    const sdf::PbrWorkflow *flow1 = pbr.Workflow(sdf::PbrWorkflowType::METAL);
    const sdf::PbrWorkflow *flow2 = pbr2->Workflow(sdf::PbrWorkflowType::METAL);
    EXPECT_EQ(flow1->AlbedoMap(), flow2->AlbedoMap());
    EXPECT_EQ(flow1->NormalMap(), flow2->NormalMap());
    EXPECT_EQ(flow1->AmbientOcclusionMap(),
              flow2->AmbientOcclusionMap());
    EXPECT_EQ(flow1->RoughnessMap(), flow2->RoughnessMap());
    EXPECT_EQ(flow1->MetalnessMap(), flow2->MetalnessMap());
    EXPECT_EQ(flow1->EmissiveMap(), flow2->EmissiveMap());
    EXPECT_EQ(flow1->LightMap(), flow2->LightMap());
    EXPECT_DOUBLE_EQ(flow1->Metalness(), flow2->Metalness());
    EXPECT_DOUBLE_EQ(flow1->Roughness(), flow2->Roughness());
    EXPECT_EQ(flow1->Type(), flow2->Type());
  }

  // Testing using SPECULAR workflow
  {
    sdf::Pbr pbr;
    workflow.SetType(sdf::PbrWorkflowType::SPECULAR);
    pbr.SetWorkflow(sdf::PbrWorkflowType::SPECULAR, workflow);
    material.SetPbrMaterial(pbr);

    sdf::ElementPtr elem = material.ToElement(errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_NE(nullptr, elem);

    sdf::Material material2;
    errors = material2.Load(elem);
    EXPECT_TRUE(errors.empty());

    EXPECT_EQ(material.Ambient(), material2.Ambient());
    EXPECT_EQ(material.Diffuse(), material2.Diffuse());
    EXPECT_EQ(material.Specular(), material2.Specular());
    EXPECT_EQ(material.Emissive(), material2.Emissive());
    EXPECT_DOUBLE_EQ(material.RenderOrder(), material2.RenderOrder());
    EXPECT_EQ(material.Lighting(), material2.Lighting());
    EXPECT_EQ(material.DoubleSided(), material2.DoubleSided());
    EXPECT_EQ(material.ScriptUri(), material2.ScriptUri());
    EXPECT_EQ(material.ScriptName(), material2.ScriptName());
    EXPECT_EQ(material.Shader(), material2.Shader());
    EXPECT_EQ(material.NormalMap(), material2.NormalMap());

    const sdf::Pbr *pbr2 = material2.PbrMaterial();
    const sdf::PbrWorkflow *flow1 =
      pbr.Workflow(sdf::PbrWorkflowType::SPECULAR);
    const sdf::PbrWorkflow *flow2 =
      pbr2->Workflow(sdf::PbrWorkflowType::SPECULAR);
    EXPECT_EQ(flow1->AlbedoMap(), flow2->AlbedoMap());
    EXPECT_EQ(flow1->NormalMap(), flow2->NormalMap());
    EXPECT_EQ(flow1->EnvironmentMap(), flow2->EnvironmentMap());
    EXPECT_EQ(flow1->AmbientOcclusionMap(),
              flow2->AmbientOcclusionMap());
    EXPECT_EQ(flow1->EmissiveMap(), flow2->EmissiveMap());
    EXPECT_EQ(flow1->GlossinessMap(), flow2->GlossinessMap());
    EXPECT_EQ(flow1->SpecularMap(), flow2->SpecularMap());
    EXPECT_EQ(flow1->LightMap(), flow2->LightMap());
    EXPECT_EQ(flow1->Type(), flow2->Type());
  }

 // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
