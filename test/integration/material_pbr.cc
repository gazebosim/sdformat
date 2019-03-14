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

#include <string>
#include <gtest/gtest.h>
#include "sdf/sdf.hh"

#include "test_config.h"

//////////////////////////////////////////////////
TEST(Material, MaterialPBR)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "material_pbr.sdf");

  // Load the SDF file into DOM
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // load material pbr sdf file
  sdf::Errors errors;
  sdf::SDFPtr sdfParsed = sdf::readFile(testFile, errors);
  ASSERT_TRUE(errors.empty());

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "model");

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  EXPECT_TRUE(linkElem->HasElement("visual"));
  sdf::ElementPtr visualElem = linkElem->GetElement("visual");

  // visual metal workflow
  {
    EXPECT_TRUE(visualElem->HasAttribute("name"));
    EXPECT_EQ("visual_metal_workflow", visualElem->Get<std::string>("name"));

    // material
    EXPECT_TRUE(visualElem->HasElement("material"));
    sdf::ElementPtr materialElem = visualElem->GetElement("material");

    // diffuse
    EXPECT_TRUE(materialElem->HasElement("diffuse"));
    sdf::ElementPtr diffuseElem = materialElem->GetElement("diffuse");
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        diffuseElem->Get<ignition::math::Color>());

    // pbr
    EXPECT_TRUE(materialElem->HasElement("pbr"));
    sdf::ElementPtr pbrElem = materialElem->GetElement("pbr");

    // metal
    EXPECT_TRUE(pbrElem->HasElement("metal"));
    sdf::ElementPtr metalElem = pbrElem->GetElement("metal");

    // albedo map
    EXPECT_TRUE(metalElem->HasElement("albedo_map"));
    sdf::ElementPtr albedoMapElem = metalElem->GetElement("albedo_map");
    EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

    // normal map
    EXPECT_TRUE(metalElem->HasElement("normal_map"));
    sdf::ElementPtr normalMapElem = metalElem->GetElement("normal_map");
    EXPECT_TRUE(normalMapElem->HasAttribute("type"));
    EXPECT_EQ("tangent", normalMapElem->Get<std::string>("type"));
    EXPECT_EQ("normal_map.png", normalMapElem->Get<std::string>());

    // metalness map
    EXPECT_TRUE(metalElem->HasElement("metalness_map"));
    sdf::ElementPtr metalnessMapElem = metalElem->GetElement("metalness_map");
    EXPECT_EQ("metalness_map.png", metalnessMapElem->Get<std::string>());

    // metalness
    EXPECT_TRUE(metalElem->HasElement("metalness"));
    sdf::ElementPtr metalnessElem = metalElem->GetElement("metalness");
    EXPECT_DOUBLE_EQ(0.3, metalnessElem->Get<double>());

    // roughness map
    EXPECT_TRUE(metalElem->HasElement("roughness_map"));
    sdf::ElementPtr roughnessMapElem = metalElem->GetElement("roughness_map");
    EXPECT_EQ("roughness_map.png", roughnessMapElem->Get<std::string>());

    // roughness
    EXPECT_TRUE(metalElem->HasElement("roughness"));
    sdf::ElementPtr roughnessElem = metalElem->GetElement("roughness");
    EXPECT_DOUBLE_EQ(0.4, roughnessElem->Get<double>());

    // environment map
    EXPECT_TRUE(metalElem->HasElement("environment_map"));
    sdf::ElementPtr envMapElem = metalElem->GetElement("environment_map");
    EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

    // ambient occlusion map
    EXPECT_TRUE(metalElem->HasElement("ambient_occlusion_map"));
    sdf::ElementPtr aoMapElem = metalElem->GetElement("ambient_occlusion_map");
    EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());
  }

  // visual specular workflow
  {
    visualElem = visualElem->GetNextElement("visual");
    EXPECT_TRUE(visualElem->HasAttribute("name"));
    EXPECT_EQ("visual_specular_workflow", visualElem->Get<std::string>("name"));

    // material
    EXPECT_TRUE(visualElem->HasElement("material"));
    sdf::ElementPtr materialElem = visualElem->GetElement("material");

    // diffuse
    EXPECT_TRUE(materialElem->HasElement("diffuse"));
    sdf::ElementPtr diffuseElem = materialElem->GetElement("diffuse");
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        diffuseElem->Get<ignition::math::Color>());

    // pbr
    EXPECT_TRUE(materialElem->HasElement("pbr"));
    sdf::ElementPtr pbrElem = materialElem->GetElement("pbr");

    // specular
    EXPECT_TRUE(pbrElem->HasElement("specular"));
    sdf::ElementPtr specularElem = pbrElem->GetElement("specular");

    // albedo map
    EXPECT_TRUE(specularElem->HasElement("albedo_map"));
    sdf::ElementPtr albedoMapElem = specularElem->GetElement("albedo_map");
    EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

    // normal map
    EXPECT_TRUE(specularElem->HasElement("normal_map"));
    sdf::ElementPtr normalMapElem = specularElem->GetElement("normal_map");
    EXPECT_TRUE(normalMapElem->HasAttribute("type"));
    EXPECT_EQ("tangent", normalMapElem->Get<std::string>("type"));
    EXPECT_EQ("normal_map.png", normalMapElem->Get<std::string>());

    // specular map
    EXPECT_TRUE(specularElem->HasElement("specular_map"));
    sdf::ElementPtr specularMapElem = specularElem->GetElement("specular_map");
    EXPECT_EQ("specular_map.png", specularMapElem->Get<std::string>());

    // glossiness map
    EXPECT_TRUE(specularElem->HasElement("glossiness_map"));
    sdf::ElementPtr glossinessMapElem =
        specularElem->GetElement("glossiness_map");
    EXPECT_EQ("glossiness_map.png", glossinessMapElem->Get<std::string>());

    // glossiness
    EXPECT_TRUE(specularElem->HasElement("glossiness"));
    sdf::ElementPtr glossinessElem = specularElem->GetElement("glossiness");
    EXPECT_DOUBLE_EQ(0.2, glossinessElem->Get<double>());

    // environment map
    EXPECT_TRUE(specularElem->HasElement("environment_map"));
    sdf::ElementPtr envMapElem = specularElem->GetElement("environment_map");
    EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

    // ambient occlusion map
    EXPECT_TRUE(specularElem->HasElement("ambient_occlusion_map"));
    sdf::ElementPtr aoMapElem =
        specularElem->GetElement("ambient_occlusion_map");
    EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());
  }

  // visual all
  {
    visualElem = visualElem->GetNextElement("visual");
    EXPECT_TRUE(visualElem->HasAttribute("name"));
    EXPECT_EQ("visual_all", visualElem->Get<std::string>("name"));

    // material
    EXPECT_TRUE(visualElem->HasElement("material"));
    sdf::ElementPtr materialElem = visualElem->GetElement("material");

    // diffuse
    EXPECT_TRUE(materialElem->HasElement("diffuse"));
    sdf::ElementPtr diffuseElem = materialElem->GetElement("diffuse");
    EXPECT_EQ(ignition::math::Color(0.2f, 0.5f, 0.1f, 1.0f),
        diffuseElem->Get<ignition::math::Color>());

    // pbr
    EXPECT_TRUE(materialElem->HasElement("pbr"));
    sdf::ElementPtr pbrElem = materialElem->GetElement("pbr");

    {
      // specular
      EXPECT_TRUE(pbrElem->HasElement("specular"));
      sdf::ElementPtr specularElem = pbrElem->GetElement("specular");

      // albedo map
      EXPECT_TRUE(specularElem->HasElement("albedo_map"));
      sdf::ElementPtr albedoMapElem = specularElem->GetElement("albedo_map");
      EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

      // normal map
      EXPECT_TRUE(specularElem->HasElement("normal_map"));
      sdf::ElementPtr normalMapElem = specularElem->GetElement("normal_map");
      EXPECT_TRUE(normalMapElem->HasAttribute("type"));
      EXPECT_EQ("tangent", normalMapElem->Get<std::string>("type"));
      EXPECT_EQ("normal_map_tangent.png", normalMapElem->Get<std::string>());

      // specular map
      EXPECT_TRUE(specularElem->HasElement("specular_map"));
      sdf::ElementPtr specularMapElem =
          specularElem->GetElement("specular_map");
      EXPECT_EQ("specular_map.png", specularMapElem->Get<std::string>());

      // glossiness map
      EXPECT_TRUE(specularElem->HasElement("glossiness_map"));
      sdf::ElementPtr glossinessMapElem =
          specularElem->GetElement("glossiness_map");
      EXPECT_EQ("glossiness_map.png", glossinessMapElem->Get<std::string>());

      // glossiness
      EXPECT_TRUE(specularElem->HasElement("glossiness"));
      sdf::ElementPtr glossinessElem = specularElem->GetElement("glossiness");
      EXPECT_DOUBLE_EQ(0.2, glossinessElem->Get<double>());

      // environment map
      EXPECT_TRUE(specularElem->HasElement("environment_map"));
      sdf::ElementPtr envMapElem = specularElem->GetElement("environment_map");
      EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

      // ambient occlusion map
      EXPECT_TRUE(specularElem->HasElement("ambient_occlusion_map"));
      sdf::ElementPtr aoMapElem =
          specularElem->GetElement("ambient_occlusion_map");
      EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());
    }

    {
      // metal
      EXPECT_TRUE(pbrElem->HasElement("metal"));
      sdf::ElementPtr metalElem = pbrElem->GetElement("metal");

      // albedo map
      EXPECT_TRUE(metalElem->HasElement("albedo_map"));
      sdf::ElementPtr albedoMapElem = metalElem->GetElement("albedo_map");
      EXPECT_EQ("albedo_map.png", albedoMapElem->Get<std::string>());

      // normal map
      EXPECT_TRUE(metalElem->HasElement("normal_map"));
      sdf::ElementPtr normalMapElem = metalElem->GetElement("normal_map");
      EXPECT_TRUE(normalMapElem->HasAttribute("type"));
      EXPECT_EQ("object", normalMapElem->Get<std::string>("type"));
      EXPECT_EQ("normal_map_object.png", normalMapElem->Get<std::string>());

      // metalness map
      EXPECT_TRUE(metalElem->HasElement("metalness_map"));
      sdf::ElementPtr metalnessMapElem = metalElem->GetElement("metalness_map");
      EXPECT_EQ("metalness_map.png", metalnessMapElem->Get<std::string>());

      // metalness
      EXPECT_TRUE(metalElem->HasElement("metalness"));
      sdf::ElementPtr metalnessElem = metalElem->GetElement("metalness");
      EXPECT_DOUBLE_EQ(0.3, metalnessElem->Get<double>());

      // roughness map
      EXPECT_TRUE(metalElem->HasElement("roughness_map"));
      sdf::ElementPtr roughnessMapElem = metalElem->GetElement("roughness_map");
      EXPECT_EQ("roughness_map.png", roughnessMapElem->Get<std::string>());

      // roughness
      EXPECT_TRUE(metalElem->HasElement("roughness"));
      sdf::ElementPtr roughnessElem = metalElem->GetElement("roughness");
      EXPECT_DOUBLE_EQ(0.4, roughnessElem->Get<double>());

      // environment map
      EXPECT_TRUE(metalElem->HasElement("environment_map"));
      sdf::ElementPtr envMapElem = metalElem->GetElement("environment_map");
      EXPECT_EQ("environment_map.png", envMapElem->Get<std::string>());

      // ambient occlusion map
      EXPECT_TRUE(metalElem->HasElement("ambient_occlusion_map"));
      sdf::ElementPtr aoMapElem =
          metalElem->GetElement("ambient_occlusion_map");
      EXPECT_EQ("ambient_occlusion_map.png", aoMapElem->Get<std::string>());
    }
  }
}


