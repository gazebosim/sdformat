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

#include <ignition/math/Color.hh>
#include "sdf/Material.hh"

/////////////////////////////////////////////////
TEST(DOMMaterial, Construction)
{
  sdf::Material material;
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Ambient());
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Diffuse());
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Specular());
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Emissive());
  EXPECT_TRUE(material.Lighting());
  EXPECT_EQ(nullptr, material.Element());
  EXPECT_EQ("", material.ScriptUri());
  EXPECT_EQ("", material.ScriptName());
  EXPECT_EQ(sdf::ShaderType::PIXEL, material.Shader());
  EXPECT_EQ("", material.NormalMap());
}

/////////////////////////////////////////////////
TEST(DOMMaterial, Set)
{
  sdf::Material material;
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Ambient());
  material.SetAmbient(ignition::math::Color(0.1, 0.2, 0.3, 0.5));
  EXPECT_EQ(ignition::math::Color(0.1, 0.2, 0.3, 0.5), material.Ambient());

  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Diffuse());
  material.SetDiffuse(ignition::math::Color(0.2, 0.3, 0.4, 0.6));
  EXPECT_EQ(ignition::math::Color(0.2, 0.3, 0.4, 0.6), material.Diffuse());

  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Specular());
  material.SetSpecular(ignition::math::Color(0.3, 0.4, 0.5, 0.7));
  EXPECT_EQ(ignition::math::Color(0.3, 0.4, 0.5, 0.7), material.Specular());

  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Emissive());
  material.SetEmissive(ignition::math::Color(0.4, 0.5, 0.6, 0.8));
  EXPECT_EQ(ignition::math::Color(0.4, 0.5, 0.6, 0.8), material.Emissive());

  EXPECT_TRUE(material.Lighting());
  material.SetLighting(false);
  EXPECT_FALSE(material.Lighting());

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

  // Move the material
  sdf::Material material2(std::move(material));
  EXPECT_EQ(ignition::math::Color(0.1, 0.2, 0.3, 0.5), material2.Ambient());
  EXPECT_EQ(ignition::math::Color(0.2, 0.3, 0.4, 0.6), material2.Diffuse());
  EXPECT_EQ(ignition::math::Color(0.3, 0.4, 0.5, 0.7), material2.Specular());
  EXPECT_EQ(ignition::math::Color(0.4, 0.5, 0.6, 0.8), material2.Emissive());
  EXPECT_FALSE(material2.Lighting());
  EXPECT_EQ("uri", material2.ScriptUri());
  EXPECT_EQ("name", material2.ScriptName());
  EXPECT_EQ(sdf::ShaderType::VERTEX, material2.Shader());
  EXPECT_EQ("map", material2.NormalMap());

  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Ambient());
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Diffuse());
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Specular());
  EXPECT_EQ(ignition::math::Color(0, 0, 0, 1), material.Emissive());
  EXPECT_TRUE(material.Lighting());
  EXPECT_EQ(nullptr, material.Element());
  EXPECT_EQ("", material.ScriptUri());
  EXPECT_EQ("", material.ScriptName());
  EXPECT_EQ(sdf::ShaderType::PIXEL, material.Shader());
  EXPECT_EQ("", material.NormalMap());
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
