/*
 * Copyright 2019 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/Actor.hh"
#include "sdf/Collision.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Geometry.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.h"

const auto g_testPath = sdf::filesystem::append(PROJECT_SOURCE_PATH, "test");
const auto g_modelsPath =
    sdf::filesystem::append(g_testPath, "integration", "model");

/////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::filesystem::append(g_testPath, "integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile =
    sdf::filesystem::append(g_testPath, "sdf", "includes.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  for (auto e : errors)
    std::cout << e.Message() << std::endl;
  EXPECT_TRUE(errors.empty());

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(worldFile, root.Element()->FilePath());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  // Actor
  EXPECT_EQ(1u, world->ActorCount());
  EXPECT_NE(nullptr, world->ActorByIndex(0));
  EXPECT_EQ(nullptr, world->ActorByIndex(1));
  EXPECT_FALSE(world->ActorNameExists(""));
  EXPECT_TRUE(world->ActorNameExists("actor"));

  const auto *actor = world->ActorByIndex(0);

  const auto actorFile = sdf::filesystem::append(g_modelsPath, "test_actor",
      "model.sdf");
  EXPECT_EQ(actorFile, actor->FilePath());

  EXPECT_EQ("actor", actor->Name());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 0), actor->RawPose());
  EXPECT_EQ("", actor->PoseRelativeTo());
  EXPECT_EQ("meshes/skin.dae", actor->SkinFilename());
  EXPECT_DOUBLE_EQ(1.0, actor->SkinScale());
  EXPECT_EQ(1u, actor->AnimationCount());
  EXPECT_NE(nullptr, actor->AnimationByIndex(0));
  EXPECT_EQ(nullptr, actor->AnimationByIndex(1));
  EXPECT_EQ(actorFile, actor->AnimationByIndex(0)->FilePath());
  EXPECT_EQ("meshes/animation.dae", actor->AnimationByIndex(0)->Filename());
  EXPECT_DOUBLE_EQ(1.0, actor->AnimationByIndex(0)->Scale());
  EXPECT_TRUE(actor->AnimationByIndex(0)->InterpolateX());
  EXPECT_FALSE(actor->AnimationNameExists(""));
  EXPECT_TRUE(actor->AnimationNameExists("walking"));

  EXPECT_EQ(1u, actor->TrajectoryCount());
  EXPECT_NE(nullptr, actor->TrajectoryByIndex(0));
  EXPECT_EQ(nullptr, actor->TrajectoryByIndex(1));
  EXPECT_EQ(0u, actor->TrajectoryByIndex(0)->Id());
  EXPECT_EQ("walking", actor->TrajectoryByIndex(0)->Type());
  EXPECT_EQ(4u, actor->TrajectoryByIndex(0)->WaypointCount());
  EXPECT_TRUE(actor->TrajectoryIdExists(0));
  EXPECT_FALSE(actor->TrajectoryIdExists(1));
  EXPECT_TRUE(actor->ScriptLoop());
  EXPECT_DOUBLE_EQ(1.0, actor->ScriptDelayStart());
  EXPECT_TRUE(actor->ScriptAutoStart());

  // Light
  EXPECT_EQ(1u, world->LightCount());
  const auto *pointLight = world->LightByIndex(0);
  ASSERT_NE(nullptr, pointLight);
  EXPECT_EQ("point_light", pointLight->Name());
  EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());
  EXPECT_FALSE(pointLight->CastShadows());
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 10, 0, 0, 0), pointLight->RawPose());
  EXPECT_EQ("world", pointLight->PoseRelativeTo());
  EXPECT_DOUBLE_EQ(123.5, pointLight->AttenuationRange());
  EXPECT_DOUBLE_EQ(1.0, pointLight->LinearAttenuationFactor());
  EXPECT_DOUBLE_EQ(0.0, pointLight->ConstantAttenuationFactor());
  EXPECT_DOUBLE_EQ(20.2, pointLight->QuadraticAttenuationFactor());

  // Model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("test_model", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByIndex(0));
  ASSERT_FALSE(nullptr == model->LinkByName("link"));
  EXPECT_EQ(model->LinkByName("link")->Name(), model->LinkByIndex(0)->Name());
  EXPECT_TRUE(nullptr == model->LinkByIndex(1));
  EXPECT_TRUE(model->LinkNameExists("link"));
  EXPECT_FALSE(model->LinkNameExists("coconut"));

  const auto modelFile = sdf::filesystem::append(g_modelsPath, "test_model",
      "model.sdf");

  const auto *link = model->LinkByName("link");
  const auto *meshCol = link->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  ASSERT_NE(nullptr, meshCol->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshCol->Geom()->Type());
  const auto *meshColGeom = meshCol->Geom()->MeshShape();
  ASSERT_NE(nullptr, meshColGeom);
  EXPECT_EQ("meshes/mesh.dae", meshColGeom->Uri());
  EXPECT_EQ(modelFile, meshColGeom->FilePath());
  EXPECT_TRUE(ignition::math::Vector3d(0.1, 0.2, 0.3) ==
      meshColGeom->Scale());
  EXPECT_EQ("my_submesh", meshColGeom->Submesh());
  EXPECT_TRUE(meshColGeom->CenterSubmesh());

  const auto *meshVis = link->VisualByName("mesh_vis");
  ASSERT_NE(nullptr, meshVis);
  ASSERT_NE(nullptr, meshVis->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshVis->Geom()->Type());
  const auto *meshVisGeom = meshVis->Geom()->MeshShape();
  EXPECT_EQ(modelFile, meshVisGeom->FilePath());
  EXPECT_EQ("meshes/mesh.dae", meshVisGeom->Uri());
  EXPECT_TRUE(ignition::math::Vector3d(1.2, 2.3, 3.4) ==
      meshVisGeom->Scale());
  EXPECT_EQ("another_submesh", meshVisGeom->Submesh());
  EXPECT_FALSE(meshVisGeom->CenterSubmesh());
}

