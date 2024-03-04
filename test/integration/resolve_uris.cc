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

#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.hh"

/// When enabled, we expect to see:
/// 1x shapes.sdf (including another SDF file)
/// 1x material script URI callback
/// 2x mesh callback (visual + collision)
/// 2x heightmap callback (visual + collision)
/// 6x texture (diffuse + normal x3)
/// 1x cubemapUri callback
constexpr const int kNumCallbacks = 13;

constexpr const char* kMeshUri =
  "https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/mesh/mesh.dae";

constexpr const char* kHeightmapUri =
  "https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/materials/textures/heightmap.png";  // NOLINT

constexpr const char* kCubemapUri = "dummyUri";

constexpr const char* kMaterialScriptUri =
  "file://media/materials/scripts/gazebo.material";

constexpr const char* kDiffuseUri =
  "https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/materials/textures/diffuse0.png";  // NOLINT

constexpr const char* kNormalUri =
  "https://fuel.ignitionrobotics.org/1.0/an_org/models/a_model/materials/textures/normal0.png";  // NOLINT

/////////////////////////////////////////////////
TEST(ResolveURIs, StoreResolvedDisabled)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll
  // use the source directory
  const std::string testFile =
    sdf::testing::TestFile("sdf", "resolve_uris.sdf");
  const std::string sdfDir = sdf::testing::TestFile("sdf");

  sdf::ParserConfig config;
  config.SetStoreResolvedURIs(false);
  EXPECT_FALSE(config.StoreResolvedURIs());

  size_t callbackCount = 0;
  config.SetFindCallback(
      [=, &callbackCount](const std::string &_file)
      {
        callbackCount++;
        return sdf::filesystem::append(sdfDir, _file);
      });

  sdf::Root root;
  auto errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty());
  // Only the include tag triggers the find callback
  EXPECT_EQ(1u, callbackCount);

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  /// Skybox Cubemap Uri
  auto sky = world->Scene()->Sky();
  auto cubemapUri = sky->CubemapUri();
  auto skyElem = sky->Element();
  auto cubemapElemUri =
    skyElem->GetElement("cubemap_uri")->GetValue()->GetAsString();

  /// Material script uri
  auto groundPlaneModel = world->ModelByName("ground_plane");
  ASSERT_NE(nullptr, groundPlaneModel);
  auto groundPlaneLink = groundPlaneModel->LinkByName("link");
  ASSERT_NE(nullptr, groundPlaneLink);
  auto groundPlaneVisual = groundPlaneLink->VisualByName("visual");
  ASSERT_NE(nullptr, groundPlaneVisual);

  auto scriptUri = groundPlaneVisual->Material()->ScriptUri();
  auto visualElem = groundPlaneVisual->Material()->Element();
  auto scriptElemUri =
    visualElem->
      GetElement("script")->
      GetElement("uri")->
      GetValue()->GetAsString();

  /// Mesh and Heightmap Visual and Collisions
  auto shapesModel = world->ModelByName("shapes");
  ASSERT_NE(nullptr, shapesModel);
  auto shapesLink = shapesModel->LinkByName("link");
  ASSERT_NE(nullptr, shapesLink);

  auto meshVisual = shapesLink->VisualByName("mesh_vis");
  ASSERT_NE(nullptr, meshVisual);
  auto meshVisualUri = meshVisual->Geom()->MeshShape()->Uri();
  auto meshVisualElem = meshVisual->Geom()->MeshShape()->Element();
  auto meshVisualElemUri =
    meshVisualElem->GetElement("uri")->GetValue()->GetAsString();

  auto meshCol = shapesLink->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  auto meshColUri = meshCol->Geom()->MeshShape()->Uri();
  auto meshColElem = meshCol->Geom()->MeshShape()->Element();
  auto meshColElemUri =
    meshColElem->GetElement("uri")->GetValue()->GetAsString();

  auto heightmapVis = shapesLink->VisualByName("heightmap_vis");
  ASSERT_NE(nullptr, heightmapVis);
  auto heightmapVisUri = heightmapVis->Geom()->HeightmapShape()->Uri();
  auto heightmapVisElem = heightmapVis->Geom()->HeightmapShape()->Element();
  auto heightmapVisElemUri =
    heightmapVisElem->GetElement("uri")->GetValue()->GetAsString();

  auto texture = heightmapVis->Geom()->HeightmapShape()->TextureByIndex(0);
  auto diffuseUri = texture->Diffuse();
  auto normalUri = texture->Normal();
  auto diffuseElemUri =
    texture->Element()->GetElement("diffuse")->GetValue()->GetAsString();
  auto normalElemUri =
    texture->Element()->GetElement("normal")->GetValue()->GetAsString();

  auto heightmapCol = shapesLink->CollisionByName("heightmap_col");
  ASSERT_NE(nullptr, heightmapCol);
  auto heightmapColUri = heightmapCol->Geom()->HeightmapShape()->Uri();
  auto heightmapColElem = heightmapCol->Geom()->HeightmapShape()->Element();
  auto heightmapColElemUri =
    heightmapColElem->GetElement("uri")->GetValue()->GetAsString();

  /// DOM URIs are unmodified as requested by configuration
  EXPECT_EQ(kMaterialScriptUri, scriptUri);
  EXPECT_EQ(kCubemapUri, cubemapUri);
  EXPECT_EQ(kMeshUri, meshVisualUri);
  EXPECT_EQ(kMeshUri, meshColUri);
  EXPECT_EQ(kHeightmapUri, heightmapVisUri);
  EXPECT_EQ(kHeightmapUri, heightmapColUri);
  EXPECT_EQ(kDiffuseUri, diffuseUri);
  EXPECT_EQ(kNormalUri, normalUri);

  /// Underlying element URIs are always unmodified
  EXPECT_EQ(kMaterialScriptUri, scriptElemUri);
  EXPECT_EQ(kCubemapUri, cubemapElemUri);
  EXPECT_EQ(kMeshUri, meshVisualElemUri);
  EXPECT_EQ(kMeshUri, meshColElemUri);
  EXPECT_EQ(kHeightmapUri, heightmapVisElemUri);
  EXPECT_EQ(kHeightmapUri, heightmapColElemUri);
  EXPECT_EQ(kDiffuseUri, diffuseElemUri);
  EXPECT_EQ(kNormalUri, normalElemUri);
}

/////////////////////////////////////////////////
TEST(ResolveURIs, StoreResolvedEnabled)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll
  // use the source directory
  const std::string testFile =
    sdf::testing::TestFile("sdf", "resolve_uris.sdf");
  const std::string sdfDir = sdf::testing::TestFile("sdf");

  sdf::ParserConfig config;
  config.SetStoreResolvedURIs(true);
  EXPECT_TRUE(config.StoreResolvedURIs());

  size_t callbackCount = 0;
  config.SetFindCallback(
      [=, &callbackCount](const std::string &_file)
      {
        callbackCount++;

        if (_file.find("https://fuel") != std::string::npos ||
            _file.find("file://") != std::string::npos ||
            _file == "dummyUri")
        {
          return "modified_" + _file;
        }
        else
        {
          return sdf::filesystem::append(sdfDir, _file);
        }
      });

  sdf::Root root;
  auto errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty());
  // Only the include tag triggers the find callback
  EXPECT_EQ(kNumCallbacks, callbackCount);

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  /// Skybox Cubemap Uri
  auto sky = world->Scene()->Sky();
  auto cubemapUri = sky->CubemapUri();
  auto skyElem = sky->Element();
  auto cubemapElemUri =
    skyElem->GetElement("cubemap_uri")->GetValue()->GetAsString();

  /// Material script uri
  auto groundPlaneModel = world->ModelByName("ground_plane");
  ASSERT_NE(nullptr, groundPlaneModel);
  auto groundPlaneLink = groundPlaneModel->LinkByName("link");
  ASSERT_NE(nullptr, groundPlaneLink);
  auto groundPlaneVisual = groundPlaneLink->VisualByName("visual");
  ASSERT_NE(nullptr, groundPlaneVisual);

  auto scriptUri = groundPlaneVisual->Material()->ScriptUri();
  auto visualElem = groundPlaneVisual->Material()->Element();
  auto scriptElemUri =
    visualElem->
      GetElement("script")->
      GetElement("uri")->
      GetValue()->GetAsString();

  /// Mesh and Heightmap Visual and Collisions
  auto shapesModel = world->ModelByName("shapes");
  ASSERT_NE(nullptr, shapesModel);
  auto shapesLink = shapesModel->LinkByName("link");
  ASSERT_NE(nullptr, shapesLink);

  auto meshVisual = shapesLink->VisualByName("mesh_vis");
  ASSERT_NE(nullptr, meshVisual);
  auto meshVisualUri = meshVisual->Geom()->MeshShape()->Uri();
  auto meshVisualElem = meshVisual->Geom()->MeshShape()->Element();
  auto meshVisualElemUri =
    meshVisualElem->GetElement("uri")->GetValue()->GetAsString();

  auto meshCol = shapesLink->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  auto meshColUri = meshCol->Geom()->MeshShape()->Uri();
  auto meshColElem = meshCol->Geom()->MeshShape()->Element();
  auto meshColElemUri =
    meshColElem->GetElement("uri")->GetValue()->GetAsString();

  auto heightmapVis = shapesLink->VisualByName("heightmap_vis");
  ASSERT_NE(nullptr, heightmapVis);
  auto heightmapVisUri = heightmapVis->Geom()->HeightmapShape()->Uri();
  auto heightmapVisElem = heightmapVis->Geom()->HeightmapShape()->Element();
  auto heightmapVisElemUri =
    heightmapVisElem->GetElement("uri")->GetValue()->GetAsString();

  auto texture = heightmapVis->Geom()->HeightmapShape()->TextureByIndex(0);
  auto diffuseUri = texture->Diffuse();
  auto normalUri = texture->Normal();
  auto diffuseElemUri =
    texture->Element()->GetElement("diffuse")->GetValue()->GetAsString();
  auto normalElemUri =
    texture->Element()->GetElement("normal")->GetValue()->GetAsString();

  auto heightmapCol = shapesLink->CollisionByName("heightmap_col");
  ASSERT_NE(nullptr, heightmapCol);
  auto heightmapColUri = heightmapCol->Geom()->HeightmapShape()->Uri();
  auto heightmapColElem = heightmapCol->Geom()->HeightmapShape()->Element();
  auto heightmapColElemUri =
    heightmapColElem->GetElement("uri")->GetValue()->GetAsString();

  /// DOM URIs are modified as user callback provides URIs
  EXPECT_EQ("modified_" + std::string(kMaterialScriptUri), scriptUri);
  EXPECT_EQ("modified_" + std::string(kCubemapUri), cubemapUri);
  EXPECT_EQ("modified_" + std::string(kMeshUri), meshVisualUri);
  EXPECT_EQ("modified_" + std::string(kMeshUri), meshColUri);
  EXPECT_EQ("modified_" + std::string(kHeightmapUri), heightmapVisUri);
  EXPECT_EQ("modified_" + std::string(kHeightmapUri), heightmapColUri);
  EXPECT_EQ("modified_" + std::string(kDiffuseUri), diffuseUri);
  EXPECT_EQ("modified_" + std::string(kNormalUri), normalUri);

  /// Underlying element URIs are always unmodified
  EXPECT_EQ(kMaterialScriptUri, scriptElemUri);
  EXPECT_EQ(kCubemapUri, cubemapElemUri);
  EXPECT_EQ(kMeshUri, meshVisualElemUri);
  EXPECT_EQ(kMeshUri, meshColElemUri);
  EXPECT_EQ(kHeightmapUri, heightmapVisElemUri);
  EXPECT_EQ(kHeightmapUri, heightmapColElemUri);
  EXPECT_EQ(kDiffuseUri, diffuseElemUri);
  EXPECT_EQ(kNormalUri, normalElemUri);
}

/////////////////////////////////////////////////
TEST(ResolveURIs, BadCallback)
{
  // The directory used in AddURIPath must exist in the filesystem, so we'll
  // use the source directory
  const std::string testFile =
    sdf::testing::TestFile("sdf", "resolve_uris.sdf");
  const std::string sdfDir = sdf::testing::TestFile("sdf");

  sdf::ParserConfig config;
  config.SetStoreResolvedURIs(true);
  EXPECT_TRUE(config.StoreResolvedURIs());

  size_t callbackCount = 0;
  config.SetFindCallback(
      [=, &callbackCount](const std::string &_file)
      {
        callbackCount++;

        if (_file.find("https://fuel") != std::string::npos ||
            _file.find("file://") != std::string::npos ||
            _file == "dummyUri")
        {
          return std::string();
        }
        else
        {
          return sdf::filesystem::append(sdfDir, _file);
        }
      });

  sdf::Root root;
  auto errors = root.Load(testFile, config);

  /// There are actually 12 callback URI errors,
  /// but then an additional "failed to load world"
  EXPECT_EQ(kNumCallbacks, errors.size());
  EXPECT_EQ(kNumCallbacks, callbackCount);

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  /// Skybox Cubemap Uri
  auto sky = world->Scene()->Sky();
  auto cubemapUri = sky->CubemapUri();
  auto skyElem = sky->Element();
  auto cubemapElemUri =
    skyElem->GetElement("cubemap_uri")->GetValue()->GetAsString();

  /// Material script uri
  auto groundPlaneModel = world->ModelByName("ground_plane");
  ASSERT_NE(nullptr, groundPlaneModel);
  auto groundPlaneLink = groundPlaneModel->LinkByName("link");
  ASSERT_NE(nullptr, groundPlaneLink);
  auto groundPlaneVisual = groundPlaneLink->VisualByName("visual");
  ASSERT_NE(nullptr, groundPlaneVisual);

  auto scriptUri = groundPlaneVisual->Material()->ScriptUri();
  auto visualElem = groundPlaneVisual->Material()->Element();
  auto scriptElemUri =
    visualElem->
      GetElement("script")->
      GetElement("uri")->
      GetValue()->GetAsString();

  /// Mesh and Heightmap Visual and Collisions
  auto shapesModel = world->ModelByName("shapes");
  ASSERT_NE(nullptr, shapesModel);
  auto shapesLink = shapesModel->LinkByName("link");
  ASSERT_NE(nullptr, shapesLink);

  auto meshVisual = shapesLink->VisualByName("mesh_vis");
  ASSERT_NE(nullptr, meshVisual);
  auto meshVisualUri = meshVisual->Geom()->MeshShape()->Uri();
  auto meshVisualElem = meshVisual->Geom()->MeshShape()->Element();
  auto meshVisualElemUri =
    meshVisualElem->GetElement("uri")->GetValue()->GetAsString();

  auto meshCol = shapesLink->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  auto meshColUri = meshCol->Geom()->MeshShape()->Uri();
  auto meshColElem = meshCol->Geom()->MeshShape()->Element();
  auto meshColElemUri =
    meshColElem->GetElement("uri")->GetValue()->GetAsString();

  auto heightmapVis = shapesLink->VisualByName("heightmap_vis");
  ASSERT_NE(nullptr, heightmapVis);
  auto heightmapVisUri = heightmapVis->Geom()->HeightmapShape()->Uri();
  auto heightmapVisElem = heightmapVis->Geom()->HeightmapShape()->Element();
  auto heightmapVisElemUri =
    heightmapVisElem->GetElement("uri")->GetValue()->GetAsString();

  auto texture = heightmapVis->Geom()->HeightmapShape()->TextureByIndex(0);
  auto diffuseUri = texture->Diffuse();
  auto normalUri = texture->Normal();
  auto diffuseElemUri =
    texture->Element()->GetElement("diffuse")->GetValue()->GetAsString();
  auto normalElemUri =
    texture->Element()->GetElement("normal")->GetValue()->GetAsString();

  auto heightmapCol = shapesLink->CollisionByName("heightmap_col");
  ASSERT_NE(nullptr, heightmapCol);
  auto heightmapColUri = heightmapCol->Geom()->HeightmapShape()->Uri();
  auto heightmapColElem = heightmapCol->Geom()->HeightmapShape()->Element();
  auto heightmapColElemUri =
    heightmapColElem->GetElement("uri")->GetValue()->GetAsString();

  /// DOM URIs are un-modified as the user callback returns empty strings
  EXPECT_EQ(kMaterialScriptUri, scriptUri);
  EXPECT_EQ(kCubemapUri, cubemapUri);
  EXPECT_EQ(kMeshUri, meshVisualUri);
  EXPECT_EQ(kMeshUri, meshColUri);
  EXPECT_EQ(kHeightmapUri, heightmapVisUri);
  EXPECT_EQ(kHeightmapUri, heightmapColUri);
  EXPECT_EQ(kDiffuseUri, diffuseUri);
  EXPECT_EQ(kNormalUri, normalUri);

  /// Underlying element URIs are always unmodified
  EXPECT_EQ(kMaterialScriptUri, scriptElemUri);
  EXPECT_EQ(kCubemapUri, cubemapElemUri);
  EXPECT_EQ(kMeshUri, meshVisualElemUri);
  EXPECT_EQ(kMeshUri, meshColElemUri);
  EXPECT_EQ(kHeightmapUri, heightmapVisElemUri);
  EXPECT_EQ(kHeightmapUri, heightmapColElemUri);
  EXPECT_EQ(kDiffuseUri, diffuseElemUri);
  EXPECT_EQ(kNormalUri, normalElemUri);
}

/////////////////////////////////////////////////
TEST(ResolveURIs, ResolvedRelativeURIs)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "resolve_relative_uris.sdf");
  const std::string sdfDir = sdf::testing::TestFile("sdf");

  sdf::ParserConfig config;
  config.SetStoreResolvedURIs(true);
  EXPECT_TRUE(config.StoreResolvedURIs());

  size_t callbackCount = 0;
  config.SetFindCallback(
      [=, &callbackCount](const std::string &)
      {
        callbackCount++;
        return "";
      });

  sdf::Root root;
  auto errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty());
  // no user callbacks should be invoked
  EXPECT_EQ(0, callbackCount);

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  // Skybox Cubemap Uri
  auto sky = world->Scene()->Sky();
  auto cubemapUri = sky->CubemapUri();

  // Visual mesh and material script uri
  auto model = world->ModelByName("model");
  ASSERT_NE(nullptr, model);
  auto link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);
  auto visual = link->VisualByName("visual");
  ASSERT_NE(nullptr, visual);
  auto meshVisualUri = visual->Geom()->MeshShape()->Uri();
  auto scriptUri = visual->Material()->ScriptUri();

  // Collision mesh uri
  auto meshCol = link->CollisionByName("collision");
  ASSERT_NE(nullptr, meshCol);
  auto meshColUri = meshCol->Geom()->MeshShape()->Uri();

  EXPECT_NE(std::string::npos, cubemapUri.find("dummy_cubemap.dds"));
  EXPECT_NE(std::string::npos, scriptUri.find("dummy_material.material"));
  EXPECT_NE(std::string::npos, meshVisualUri.find("dummy_mesh.dae"));
  EXPECT_NE(std::string::npos, meshColUri.find("dummy_mesh.dae"));
  EXPECT_EQ(meshVisualUri, meshColUri);

  EXPECT_TRUE(std::filesystem::path(cubemapUri).is_absolute());
  EXPECT_TRUE(std::filesystem::path(scriptUri).is_absolute());
  EXPECT_TRUE(std::filesystem::path(meshVisualUri).is_absolute());
  EXPECT_TRUE(std::filesystem::path(meshColUri).is_absolute());

  EXPECT_TRUE(sdf::filesystem::exists(cubemapUri));
  EXPECT_TRUE(sdf::filesystem::exists(scriptUri));
  EXPECT_TRUE(sdf::filesystem::exists(meshVisualUri));
  EXPECT_TRUE(sdf::filesystem::exists(meshColUri));
}

/////////////////////////////////////////////////
TEST(ResolveURIs, ResolvedRelativeURIsSdfString)
{
  const std::string testSdfString = R"(
  <sdf version="1.6">
    <model name="model">
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>media/dummy_mesh.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <mesh>
              <uri>media/dummy_mesh.dae</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>media/dummy_material.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
  </sdf>)";

  sdf::ParserConfig config;
  config.SetStoreResolvedURIs(true);
  EXPECT_TRUE(config.StoreResolvedURIs());

  const std::string kTestMediaPath = sdf::testing::TestFile("sdf", "media");
  size_t callbackCount = 0;
  config.SetFindCallback(
      [=, &callbackCount](const std::string &_file)
      {
        std::string path = _file;
        std::string media = "media";
        path.replace(path.find(media), media.length(), kTestMediaPath);
        callbackCount++;
        return path;
      });

  sdf::Root root;
  auto errors = root.LoadSdfString(testSdfString, config);
  EXPECT_TRUE(errors.empty());
  // callbacks should be invoked for all relative uris
  EXPECT_EQ(3, callbackCount);

  auto model = root.Model();
  ASSERT_NE(nullptr, model);

  // Visual mesh and material script uri
  auto link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);
  auto visual = link->VisualByName("visual");
  ASSERT_NE(nullptr, visual);
  auto meshVisualUri = visual->Geom()->MeshShape()->Uri();
  auto scriptUri = visual->Material()->ScriptUri();

  // Collision mesh uri
  auto meshCol = link->CollisionByName("collision");
  ASSERT_NE(nullptr, meshCol);
  auto meshColUri = meshCol->Geom()->MeshShape()->Uri();

  EXPECT_NE(std::string::npos, scriptUri.find("dummy_material.material"));
  EXPECT_NE(std::string::npos, meshVisualUri.find("dummy_mesh.dae"));
  EXPECT_NE(std::string::npos, meshColUri.find("dummy_mesh.dae"));
  EXPECT_EQ(meshVisualUri, meshColUri);

  EXPECT_TRUE(std::filesystem::path(scriptUri).is_absolute());
  EXPECT_TRUE(std::filesystem::path(meshVisualUri).is_absolute());
  EXPECT_TRUE(std::filesystem::path(meshColUri).is_absolute());

  EXPECT_TRUE(sdf::filesystem::exists(scriptUri));
  EXPECT_TRUE(sdf::filesystem::exists(meshVisualUri));
  EXPECT_TRUE(sdf::filesystem::exists(meshColUri));
}
