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

#include <gtest/gtest.h>

#include <gz/math/Pose3.hh>
#include <iostream>
#include <string>

#include "sdf/Actor.hh"
#include "sdf/Collision.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/Geometry.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Mesh.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"
#include "test_config.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
  EXPECT_TRUE(errors.empty()) << errors;

  ASSERT_NE(nullptr, root.Element());
  EXPECT_EQ(worldFile, root.Element()->FilePath());
  EXPECT_EQ("1.8", root.Element()->OriginalVersion());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("1.8", world->Element()->OriginalVersion());

  // Actors
  EXPECT_EQ(2u, world->ActorCount());
  EXPECT_FALSE(world->ActorNameExists(""));

  // Actor without overrides
  EXPECT_NE(nullptr, world->ActorByIndex(0));
  EXPECT_TRUE(world->ActorNameExists("actor"));

  const auto *actor = world->ActorByIndex(0);
  EXPECT_EQ("1.6", actor->Element()->OriginalVersion());

  const auto actorFile = sdf::testing::TestFile(
      "integration", "model", "test_actor", "model.sdf");
  EXPECT_EQ(actorFile, actor->FilePath());

  EXPECT_EQ("actor", actor->Name());
  EXPECT_EQ(gz::math::Pose3d(0, 0, 0, 0, 0, 0), actor->RawPose());
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

  ASSERT_NE(nullptr, actor->Element());
  EXPECT_FALSE(actor->Element()->HasElement("plugin"));

  // Actor with overrides
  EXPECT_NE(nullptr, world->ActorByIndex(1));
  EXPECT_TRUE(world->ActorNameExists("override_actor_name"));

  const auto *actor1 = world->ActorByIndex(1);
  EXPECT_EQ("override_actor_name", actor1->Name());
  EXPECT_EQ(gz::math::Pose3d(7, 8, 9, 0, 0, 0), actor1->RawPose());
  EXPECT_EQ("", actor1->PoseRelativeTo());
  ASSERT_NE(nullptr, actor1->Element());
  EXPECT_TRUE(actor1->Element()->HasElement("plugin"));

  // Lights
  EXPECT_EQ(2u, world->LightCount());
  EXPECT_FALSE(world->LightNameExists(""));

  // Light without overrides
  const auto *pointLight = world->LightByIndex(0);
  ASSERT_NE(nullptr, pointLight);
  EXPECT_EQ("point_light", pointLight->Name());
  EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());
  EXPECT_FALSE(pointLight->CastShadows());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 10, 0, 0, 0), pointLight->RawPose());
  EXPECT_EQ("world", pointLight->PoseRelativeTo());
  EXPECT_DOUBLE_EQ(123.5, pointLight->AttenuationRange());
  EXPECT_DOUBLE_EQ(1.0, pointLight->LinearAttenuationFactor());
  EXPECT_DOUBLE_EQ(0.0, pointLight->ConstantAttenuationFactor());
  EXPECT_DOUBLE_EQ(20.2, pointLight->QuadraticAttenuationFactor());
  EXPECT_EQ("1.6", pointLight->Element()->OriginalVersion());
  ASSERT_NE(nullptr, pointLight->Element());

  // Light with overrides
  const auto *pointLight1 = world->LightByIndex(1);
  ASSERT_NE(nullptr, pointLight1);
  EXPECT_EQ("override_light_name", pointLight1->Name());
  EXPECT_EQ(gz::math::Pose3d(4, 5, 6, 0, 0, 0), pointLight1->RawPose());
  EXPECT_EQ("", pointLight1->PoseRelativeTo());

  // Models
  EXPECT_EQ(3u, world->ModelCount());
  EXPECT_FALSE(world->ModelNameExists(""));

  // Model without overrides
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("test_model", model->Name());
  EXPECT_FALSE(model->Static());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByIndex(0));
  ASSERT_FALSE(nullptr == model->LinkByName("link"));
  EXPECT_EQ(model->LinkByName("link")->Name(), model->LinkByIndex(0)->Name());
  EXPECT_TRUE(nullptr == model->LinkByIndex(1));
  EXPECT_TRUE(model->LinkNameExists("link"));
  EXPECT_FALSE(model->LinkNameExists("coconut"));
  EXPECT_EQ("1.6", model->Element()->OriginalVersion());

  const auto modelFile = sdf::testing::TestFile(
      "integration", "model", "test_model", "model.sdf");

  const auto *link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("1.6", link->Element()->OriginalVersion());
  const auto *meshCol = link->CollisionByName("mesh_col");
  ASSERT_NE(nullptr, meshCol);
  ASSERT_NE(nullptr, meshCol->Geom());
  EXPECT_EQ(sdf::GeometryType::MESH, meshCol->Geom()->Type());
  const auto *meshColGeom = meshCol->Geom()->MeshShape();
  ASSERT_NE(nullptr, meshColGeom);
  EXPECT_EQ("meshes/mesh.dae", meshColGeom->Uri());
  EXPECT_EQ(modelFile, meshColGeom->FilePath());
  EXPECT_TRUE(gz::math::Vector3d(0.1, 0.2, 0.3) ==
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
  EXPECT_TRUE(gz::math::Vector3d(1.2, 2.3, 3.4) ==
      meshVisGeom->Scale());
  EXPECT_EQ("another_submesh", meshVisGeom->Submesh());
  EXPECT_FALSE(meshVisGeom->CenterSubmesh());

  ASSERT_NE(nullptr, model->Element());
  EXPECT_FALSE(model->Element()->HasElement("plugin"));

  // Model with overrides
  const sdf::Model *model1 = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model1);
  EXPECT_EQ("override_model_name", model1->Name());
  EXPECT_TRUE(model1->Static());
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), model1->RawPose());
  EXPECT_EQ("", model1->PoseRelativeTo());
  ASSERT_NE(nullptr, model1->Element());
  EXPECT_TRUE(model1->Element()->HasElement("plugin"));

  const sdf::Model *model2 = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ("test_model_with_file", model2->Name());
  EXPECT_FALSE(model2->Static());
  EXPECT_EQ(1u, model2->LinkCount());
  ASSERT_NE(nullptr, model2->LinkByIndex(0));
  ASSERT_NE(nullptr, model2->LinkByName("link"));
  EXPECT_EQ(model2->LinkByName("link")->Name(), model2->LinkByIndex(0)->Name());
  EXPECT_EQ(nullptr, model2->LinkByIndex(1));
  EXPECT_TRUE(model2->LinkNameExists("link"));
  EXPECT_FALSE(model2->LinkNameExists("coconut"));
  EXPECT_EQ("1.6", model2->Element()->OriginalVersion());
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes_15)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes_1.5.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(worldFile);
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

  // Light
  EXPECT_EQ(1u, world->LightCount());
  const auto *pointLight = world->LightByIndex(0);
  ASSERT_NE(nullptr, pointLight);
  EXPECT_EQ("point_light", pointLight->Name());
  EXPECT_EQ(sdf::LightType::POINT, pointLight->Type());

  // Model
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("test_model", model->Name());
  EXPECT_EQ(1u, model->LinkCount());
  ASSERT_FALSE(nullptr == model->LinkByName("link"));

  const auto *link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);

  // The root and world were version 1.5
  EXPECT_EQ("1.5", root.Element()->OriginalVersion());
  EXPECT_EQ("1.5", world->Element()->OriginalVersion());

  // The included models were version 1.6
  EXPECT_EQ("1.6", actor->Element()->OriginalVersion());
  EXPECT_EQ("1.6", pointLight->Element()->OriginalVersion());
  EXPECT_EQ("1.6", model->Element()->OriginalVersion());
  EXPECT_EQ("1.6", link->Element()->OriginalVersion());
}

//////////////////////////////////////////////////
TEST(IncludesTest, Includes_15_convert)
{
  sdf::setFindCallback(findFileCb);

  const auto worldFile = sdf::testing::TestFile("sdf", "includes_1.5.sdf");

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertFile(worldFile, "1.7", sdf));

  sdf::ElementPtr rootElem = sdf->Root();
  ASSERT_NE(nullptr, rootElem);

  // it is parsed to 1.7
  EXPECT_EQ("1.7", rootElem->Get<std::string>("version"));

  sdf::ElementPtr worldElem = rootElem->GetElement("world");
  ASSERT_NE(nullptr, worldElem);
  EXPECT_EQ(worldElem->Get<std::string>("name"), "default");

  sdf::ElementPtr actorElem = worldElem->GetElement("actor");
  ASSERT_NE(nullptr, actorElem);
  EXPECT_EQ(actorElem->Get<std::string>("name"), "actor");

  sdf::ElementPtr lightElem = worldElem->GetElement("light");
  ASSERT_NE(nullptr, lightElem);
  EXPECT_EQ(lightElem->Get<std::string>("name"), "point_light");

  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ(modelElem->Get<std::string>("name"), "test_model");

  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // The root and world were version 1.5
  EXPECT_EQ("1.5", sdf->OriginalVersion());
  EXPECT_EQ("1.5", rootElem->OriginalVersion());
  EXPECT_EQ("1.5", worldElem->OriginalVersion());

  // The included models were version 1.6
  EXPECT_EQ("1.6", actorElem->OriginalVersion());
  EXPECT_EQ("1.6", lightElem->OriginalVersion());
  EXPECT_EQ("1.6", modelElem->OriginalVersion());
  EXPECT_EQ("1.6", linkElem->OriginalVersion());
}

//////////////////////////////////////////////////
TEST(IncludesTest, IncludeModelMissingConfig)
{
  sdf::setFindCallback(findFileCb);

  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<include>"
    << "  <uri>box_missing_config</uri>"
    << "</include>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::Errors errors;
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed, errors));

  EXPECT_EQ(2u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FILE_READ);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Could not find model.config or manifest.xml in")) << errors[0];
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::URI_LOOKUP);
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "Unable to resolve uri[box_missing_config] to model path")) << errors[1];
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "box_missing_config] since it does not contain a model.config file"))
    << errors[1];

  sdf::Root root;
  errors = root.Load(sdfParsed);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(nullptr, root.Model());
}

//////////////////////////////////////////////////
/// Check that sdformat natively parses URDF files when there are no custom
/// parsers
TEST(IncludesTest, IncludeUrdf)
{
  sdf::setFindCallback(findFileCb);

  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<include>"
    << "  <uri>test_include_urdf</uri>"
    << "</include>"
    << "</sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(stream.str());
  ASSERT_TRUE(errors.empty()) << errors;

  auto model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("test_include_urdf", model->Name());
  EXPECT_EQ(2u, model->LinkCount());
  EXPECT_EQ(1u, model->JointCount());
}

//////////////////////////////////////////////////
TEST(IncludesTest, MergeInclude)
{
  using gz::math::Pose3d;
  using gz::math::Vector3d;
  sdf::ParserConfig config;
  config.SetFindCallback(findFileCb);

  sdf::Root root;
  sdf::Errors errors = root.Load(
      sdf::testing::TestFile("integration", "merge_include_model.sdf"), config);
  EXPECT_TRUE(errors.empty()) << errors;

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  auto model = world->ModelByIndex(0);
  EXPECT_EQ("robot1", model->Name());
  EXPECT_EQ(7u, model->LinkCount());
  EXPECT_EQ(6u, model->JointCount());
  EXPECT_EQ(1u, model->ModelCount());
  ASSERT_NE(nullptr, model->CanonicalLink());
  EXPECT_EQ(model->LinkByIndex(0), model->CanonicalLink());

  std::string prefixedFrameName;
  {
    sdf::Root includedModel;
    errors = includedModel.Load(
      sdf::testing::TestFile("integration", "model",
                             "merge_robot", "model.sdf"));
    EXPECT_TRUE(errors.empty()) << errors;
    ASSERT_NE(nullptr, includedModel.Model());
    prefixedFrameName = "_merged__"
        + includedModel.Model()->Name() + "__model__";
  }

  auto resolvePose = [](const sdf::SemanticPose &_semPose)
  {
    Pose3d result;
    sdf::Errors poseErrors = _semPose.Resolve(result);
    EXPECT_TRUE(poseErrors.empty()) << poseErrors;
    return result;
  };

  // X_PM - Pose of original model (M) in parent model (P) frame. This is the
  // pose override in the //include tag.
  const Pose3d X_PM(100, 0, 0, GZ_PI_4, 0, 0);
  // X_MRw - Pose of the right wheel in the original model (M) as specified in
  // the SDFormat file.
  const Pose3d X_MRw(0.554282, -0.625029, -0.025, -1.5707, 0, 0);
  // X_MLw - Pose of the left wheel in the original model (M) as specified in
  // the SDF file.
  const Pose3d X_MLw(0.554282, 0.625029, -0.025, -1.5707, 0, 0);
  // Check some poses
  {
    // Link "chassis"
    auto testFrame = model->LinkByName("chassis");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_EQ(prefixedFrameName, testFrame->PoseRelativeTo());
    const Pose3d testPose = resolvePose(testFrame->SemanticPose());
    // X_MC - Pose of chassis link(C) in the original model (M) as specified in
    // the SDF file.
    const Pose3d X_MC(-0.151427, 0, 0.175, 0, 0, 0);
    const Pose3d expectedPose = X_PM * X_MC;
    EXPECT_EQ(expectedPose, testPose);
  }
  {
    // Link "top"
    auto testFrame = model->LinkByName("top");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_EQ(prefixedFrameName, testFrame->PoseRelativeTo());
    const Pose3d testPose = resolvePose(testFrame->SemanticPose());
    // From SDFormat file
    // X_MT - Pose of top link(T) in the original model (M) as specified in
    // the SDF file.
    const Pose3d X_MT(0.6, 0, 0.7, 0, 0, 0);
    const Pose3d expectedPose = X_PM * X_MT;
    EXPECT_EQ(expectedPose, testPose);
  }
  {
    // The pose of right_wheel_joint is specified relative to __model__.
    auto testFrame = model->JointByName("right_wheel_joint");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_EQ(prefixedFrameName, testFrame->PoseRelativeTo());
    // Resolve the pose relative to it's child frame (right_wheel)
    const Pose3d testPose = resolvePose(testFrame->SemanticPose());
    // From SDFormat file
    // X_MJr - Pose of right_wheel_joint (Jr) in the original model (M) as
    // specified in the SDF file.
    const Pose3d X_MJr(1, 0, 0, 0, 0, 0);
    const Pose3d expectedPose = X_MRw.Inverse() * X_MJr;
    EXPECT_EQ(expectedPose, testPose);
  }
  {
    // The pose of sensor_frame is specified relative to __model__.
    auto testFrame = model->FrameByName("sensor_frame");
    ASSERT_NE(nullptr, testFrame);
    EXPECT_EQ(prefixedFrameName, testFrame->PoseRelativeTo());
    // Resolve the pose relative to it's child frame (right_wheel)
    const Pose3d testPose = resolvePose(testFrame->SemanticPose());
    // From SDFormat file
    // X_MS - Pose of sensor_frame (S) in the original model (M) as
    // specified in the SDF file.
    const Pose3d X_MS(0, 1, 0, 0, GZ_PI_4, 0);
    const Pose3d expectedPose = X_PM * X_MS;
    EXPECT_EQ(expectedPose, testPose);
  }

  // Check joint axes
  {
    // left_wheel_joint's axis is expressed in __model__.
    auto joint = model->JointByName("left_wheel_joint");
    ASSERT_NE(nullptr, joint);
    auto axis = joint->Axis(0);
    ASSERT_NE(nullptr, axis);
    Vector3d xyz;
    sdf::Errors resolveErrors = axis->ResolveXyz(xyz);
    EXPECT_TRUE(resolveErrors.empty()) << resolveErrors;
    // From SDFormat file
    // R_MJl - Rotation of left_wheel_joint (Jl) in the original model (M) as
    // specified in the SDF file. This is the same as R_MLw since //joint/pose
    // is identity.
    const auto R_MJl = X_MLw.Rot();
    Vector3d xyzInOrigModel(0, 0, 1);
    Vector3d expectedXyz = R_MJl.Inverse() * xyzInOrigModel;
    EXPECT_EQ(expectedXyz, xyz);
  }

  // Check joint parent set as __model__
  {
    // left_wheel_joint's axis is expressed in __model__.
    auto joint = model->JointByName("test_model_parent");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ(prefixedFrameName, joint->ParentName());
  }
  // Check joint child set as __model__
  {
    // left_wheel_joint's axis is expressed in __model__.
    auto joint = model->JointByName("test_model_child");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ(prefixedFrameName, joint->ChildName());
  }

  // Verify that plugins get merged
  auto modelElem = model->Element();
  ASSERT_NE(nullptr, modelElem);
  auto pluginElem = modelElem->FindElement("plugin");
  ASSERT_NE(nullptr, pluginElem);
  EXPECT_EQ("test", pluginElem->Get<std::string>("name"));

  // Verify that custom elements get merged
  auto customFoo = modelElem->FindElement("custom:foo");
  ASSERT_NE(nullptr, customFoo);
  EXPECT_EQ("baz", customFoo->Get<std::string>("name"));

  // Verify that other non-named elements, such as <static> and <enable_wind> do
  // *NOT* get merged. This is also true for unknown elements
  EXPECT_FALSE(modelElem->HasElement("unknown_element"));
  EXPECT_FALSE(modelElem->HasElement("enable_wind"));
  EXPECT_FALSE(modelElem->HasElement("static"));
}

//////////////////////////////////////////////////
TEST(IncludesTest, WorldMergeInclude)
{
  using gz::math::Pose3d;
  sdf::ParserConfig config;
  config.SetFindCallback(findFileCb);

  sdf::Root root;
  sdf::Errors errors = root.Load(
      sdf::testing::TestFile("sdf", "world_merge_include.sdf"),
      config);
  EXPECT_TRUE(errors.empty()) << errors;

  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  std::string proxyFrameName;
    proxyFrameName = "_merged__model_for_world_merge_include__model__";

  auto *proxyFrame = world->FrameByName(proxyFrameName);
  ASSERT_NE(nullptr, proxyFrame);

  EXPECT_EQ(Pose3d(100, 0, 0, 0, 0, 0), proxyFrame->RawPose());

  // Expect that the merged models have poses relative to the proxy frame
  ASSERT_EQ(2, world->ModelCount());
  auto *model1 = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model1);
  EXPECT_EQ(proxyFrameName, model1->PoseRelativeTo());

  auto *model2 = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ(proxyFrameName, model2->PoseRelativeTo());

  // F1, since it had an empty attached_to in the original model, once it's
  // merged, it should be attached to the proxy frame
  auto *frame1 = world->FrameByName("F1");
  ASSERT_NE(nullptr, frame1);
  EXPECT_EQ(proxyFrameName, frame1->AttachedTo());

  // Check that the joint was also merged
  auto *joint1 = world->JointByName("J1");
  ASSERT_NE(nullptr, joint1);
}

//////////////////////////////////////////////////
TEST(IncludesTest, WorldMergeIncludeInvalidElements)
{
  std::string sdfString = R"(
  <sdf version="1.10">
    <world name="invalid_world">
      <include merge="true">
        <uri>test_model</uri>
      </include>
    </world>
  </sdf>)";

  sdf::ParserConfig config;
  config.SetFindCallback(findFileCb);

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdfString, config);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::MERGE_INCLUDE_UNSUPPORTED);
}

//////////////////////////////////////////////////
TEST(IncludesTest, MergeIncludePlacementFrame)
{
  using gz::math::Pose3d;
  sdf::ParserConfig config;
  config.SetFindCallback(findFileCb);

  sdf::Root root;
  sdf::Errors errors = root.Load(
      sdf::testing::TestFile("integration", "merge_include_model.sdf"), config);
  ASSERT_TRUE(errors.empty()) << errors;

  auto world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  auto model = world->ModelByIndex(1);
  EXPECT_EQ("robot2", model->Name());
  EXPECT_EQ(7u, model->LinkCount());
  EXPECT_EQ(6u, model->JointCount());
  auto topLink = model->LinkByName("top");
  ASSERT_NE(nullptr, topLink);
  Pose3d topLinkPose;
  EXPECT_TRUE(topLink->SemanticPose().Resolve(topLinkPose).empty());
  // From SDFormat file
  Pose3d expectedtopLinkPose = Pose3d(0, 0, 2, 0, 0, 0);
  EXPECT_EQ(expectedtopLinkPose, topLinkPose);
}

//////////////////////////////////////////////////
TEST(IncludesTest, InvalidMergeInclude)
{
  // Redirect sdferr output
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

  sdf::ParserConfig config;
  // Set policies to Error to make sure nothing gets printed
  config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::ERR);
  // Using the "file://" URI scheme to allow multiple search paths
  config.AddURIPath("file://", sdf::testing::TestFile("sdf"));
  config.AddURIPath("file://", sdf::testing::TestFile("integration", "model"));

  // Models that are not valid by themselves
  {
    const std::string sdfString = R"(
      <sdf version="1.9">
        <model name="M">
          <link name="L"/>
          <include merge="true">
            <uri>file://model_invalid_frame_only.sdf</uri> <!-- NOLINT -->
          </include>
        </model>
      </sdf>)"; // NOLINT
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sdfString, config);
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::MODEL_WITHOUT_LINK, errors[0].Code());
  }

  {
    const std::string sdfString = R"(
      <sdf version="1.9">
        <model name="M">
          <include merge="true">
            <uri>file://model_invalid_link_relative_to.sdf</uri> <!-- NOLINT -->
          </include>
        </model>
      </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sdfString, config);
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::POSE_RELATIVE_TO_INVALID, errors[0].Code());
  }

  // Actors are not supported for merging
  {
    const std::string sdfString = R"(
    <sdf version="1.9">
      <actor name="A">
        <include merge="true">
          <uri>file://test_actor</uri> <!-- NOLINT -->
        </include>
      </actor>
    </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sdfString, config);
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::MERGE_INCLUDE_UNSUPPORTED, errors[0].Code());
    EXPECT_EQ(4, *errors[0].LineNumber());
  }

  // Lights are not supported for merging
  {
    const std::string sdfString = R"(
    <sdf version="1.9">
      <light name="Lt" type="spot">
        <include merge="true">
          <uri>file://test_light</uri> <!-- NOLINT -->
        </include>
      </light>
    </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sdfString, config);
    ASSERT_FALSE(errors.empty());
    EXPECT_EQ(sdf::ErrorCode::MERGE_INCLUDE_UNSUPPORTED, errors[0].Code());
    EXPECT_EQ("Merge-include is only supported for included models",
              errors[0].Message());
    EXPECT_EQ(4, *errors[0].LineNumber());
  }

  // merge-include can only be used directly under //model or //world
  {
    const std::string sdfString = R"(
    <sdf version="1.9">
      <world name="default">
        <frame name="test_frame">
          <include merge="true">
            <uri>file://merge_robot</uri> <!-- NOLINT -->
          </include>
        </frame>
      </world>
    </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sdfString, config);
    ASSERT_EQ(3, errors.size());
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
    EXPECT_NE(std::string::npos, errors[0].Message().find(
        "child of element[sensor], not defined in SDF."));
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[1].Code());
    EXPECT_NE(std::string::npos, errors[1].Message().find(
        "XML Element[unknown_element], child of element[model], not"
        " defined in SDF. Copying[unknown_element] as children of [model]"));
    EXPECT_EQ(sdf::ErrorCode::MERGE_INCLUDE_UNSUPPORTED, errors[2].Code());
    EXPECT_NE(std::string::npos, errors[2].Message().find(
        "Merge-include does not support parent element of type frame"));
    EXPECT_EQ(5, errors[2].LineNumber());
  }

  // Syntax error in included file
  {
    const std::string sdfString = R"(
    <sdf version="1.9">
      <world name="default">
        <include merge="true">
          <uri>file://invalid_xml_syntax.sdf</uri> <!-- NOLINT -->
        </include>
      </world>
    </sdf>)";
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(sdfString, config);
    ASSERT_EQ(errors.size(), 5u);
    EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FILE_READ);
    EXPECT_NE(std::string::npos,
              errors[0].Message().find("Error parsing XML in file"));
    EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::FILE_READ);
    EXPECT_NE(std::string::npos,
              errors[1].Message().find("Unable to read file"));
    EXPECT_EQ(5, *errors[1].LineNumber());
    EXPECT_EQ(errors[2].Code(), sdf::ErrorCode::ELEMENT_INVALID);
    EXPECT_NE(std::string::npos,
              errors[2].Message().find("Error reading element <world>"));
    EXPECT_EQ(errors[3].Code(), sdf::ErrorCode::ELEMENT_INVALID);
    EXPECT_NE(std::string::npos,
              errors[3].Message().find("Error reading element <sdf>"));
    EXPECT_EQ(errors[4].Code(), sdf::ErrorCode::STRING_READ);
    EXPECT_NE(std::string::npos,
              errors[4].Message().find("Unable to read SDF string"));
  }
  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
