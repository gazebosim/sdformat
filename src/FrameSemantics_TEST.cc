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

#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <ignition/math/Helpers.hh>

#include "sdf/Element.hh"
#include "sdf/Frame.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"

#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "test_config.h"

/////////////////////////////////////////////////
TEST(FrameSemantics, buildFrameAttachedToGraph_Model)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_frame_attached_to.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first model
  const sdf::Model *model = root.Model();

  auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
  errors = sdf::buildFrameAttachedToGraph(graph, model);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_TRUE(sdf::validateFrameAttachedToGraph(graph).empty());
  EXPECT_TRUE(sdf::checkFrameAttachedToGraph(&root));
  EXPECT_TRUE(sdf::checkFrameAttachedToNames(&root));

  graph = graph.ChildModelScope(model->Name());
  EXPECT_EQ(8u, graph.Map().size());
  EXPECT_EQ(8u, graph.Graph().Vertices().size());
  EXPECT_EQ(6u, graph.Graph().Edges().size());

  EXPECT_EQ(1u, graph.Count("__model__"));
  EXPECT_EQ(1u, graph.Count("L"));
  EXPECT_EQ(1u, graph.Count("F00"));
  EXPECT_EQ(1u, graph.Count("F0"));
  EXPECT_EQ(1u, graph.Count("F1"));
  EXPECT_EQ(1u, graph.Count("F2"));
  EXPECT_EQ(0u, graph.Count("invalid"));

  std::string resolvedBody;
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "L").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "__model__").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F00").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F0").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F1").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F2").empty());
  EXPECT_EQ("L", resolvedBody);

  // Try to resolve invalid frame name
  errors = sdf::resolveFrameAttachedToBody(resolvedBody, graph, "invalid");
  for (auto &e : errors)
    std::cerr << e.Message() << std::endl;
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_INVALID);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "FrameAttachedToGraph unable to find unique frame with name ["
        "invalid] in graph."));
}

/////////////////////////////////////////////////
TEST(FrameSemantics, buildFrameAttachedToGraph_World)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "world_frame_attached_to.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(world);

  auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
  errors = sdf::buildFrameAttachedToGraph(graph, world);
  EXPECT_TRUE(errors.empty());
  errors = sdf::validateFrameAttachedToGraph(graph);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_TRUE(sdf::checkFrameAttachedToGraph(&root));
  EXPECT_TRUE(sdf::checkFrameAttachedToNames(&root));

  EXPECT_EQ(9u, graph.Map().size());
  EXPECT_EQ(9u, graph.Graph().Vertices().size());
  EXPECT_EQ(7u, graph.Graph().Edges().size());

  EXPECT_EQ(1u, graph.Count("world"));
  EXPECT_EQ(1u, graph.Count("world_frame"));
  EXPECT_EQ(1u, graph.Count("F0"));
  EXPECT_EQ(1u, graph.Count("F1"));
  EXPECT_EQ(1u, graph.Count("F2"));
  EXPECT_EQ(1u, graph.Count("M1"));
  EXPECT_EQ(0u, graph.Count("invalid"));

  std::string resolvedBody;
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "world").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(
      resolvedBody, graph, "world_frame").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F0").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F1").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F2").empty());
  EXPECT_EQ("M1::L", resolvedBody);

  // Try to resolve invalid frame name
  errors = sdf::resolveFrameAttachedToBody(resolvedBody, graph, "invalid");
  for (auto &e : errors)
    std::cerr << e.Message() << std::endl;
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_INVALID);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "FrameAttachedToGraph unable to find unique frame with name ["
        "invalid] in graph."));

  // Create graph from embedded model
  ASSERT_NE(nullptr, world);
  const sdf::Model *model = world->ModelByIndex(0);

  auto ownedModelGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> modelGraph(ownedModelGraph);
  errors = sdf::buildFrameAttachedToGraph(modelGraph, model);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(sdf::validateFrameAttachedToGraph(modelGraph).empty());

  modelGraph = modelGraph.ChildModelScope(model->Name());

  EXPECT_EQ(5u, modelGraph.Map().size());
  EXPECT_EQ(5u, modelGraph.Graph().Vertices().size());
  EXPECT_EQ(3u, modelGraph.Graph().Edges().size());

  EXPECT_EQ(1u, modelGraph.Count("L"));
  EXPECT_EQ(1u, modelGraph.Count("__model__"));
  EXPECT_EQ(1u, modelGraph.Count("F0"));
  EXPECT_EQ(0u, modelGraph.Count("invalid"));

  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "L").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(
      resolvedBody, modelGraph, "__model__").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F0").empty());
  EXPECT_EQ("L", resolvedBody);
}

/////////////////////////////////////////////////
TEST(FrameSemantics, buildPoseRelativeToGraph)
{
  const std::string testFile =
    sdf::testing::TestFile("sdf", "model_frame_relative_to_joint.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.Model();

  auto ownedGraph = std::make_shared<sdf::PoseRelativeToGraph>();
  sdf::ScopedGraph<sdf::PoseRelativeToGraph> graph(ownedGraph);
  auto errors = sdf::buildPoseRelativeToGraph(graph, model);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(sdf::validatePoseRelativeToGraph(graph).empty());
  graph = graph.ChildModelScope(model->Name());

  EXPECT_EQ(10u, graph.Map().size());
  EXPECT_EQ(10u, graph.Graph().Vertices().size());
  EXPECT_EQ(9u, graph.Graph().Edges().size());

  EXPECT_EQ(1u, graph.Count("__model__"));
  EXPECT_EQ(1u, graph.Count("P"));
  EXPECT_EQ(1u, graph.Count("C"));
  EXPECT_EQ(1u, graph.Count("J"));
  EXPECT_EQ(1u, graph.Count("F1"));
  EXPECT_EQ(1u, graph.Count("F2"));
  EXPECT_EQ(1u, graph.Count("F3"));
  EXPECT_EQ(1u, graph.Count("F4"));

  // Test resolvePoseRelativeToRoot for each frame.
  ignition::math::Pose3d pose;
  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "__model__").empty());
  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "P").empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "F1").empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 1, 0, 0, 0), pose);

  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "C").empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "F2").empty());
  EXPECT_EQ(ignition::math::Pose3d(4, 0, 0, 0, IGN_PI/2, 0), pose);

  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "J").empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 3, 0, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "F3").empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 3, 3, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(sdf::resolvePoseRelativeToRoot(pose, graph, "F4").empty());
  EXPECT_EQ(ignition::math::Pose3d(6, 3, 3, 0, 0, 0), pose);

  // Test resolvePose for each frame with its relative_to value.
  // Numbers should match the raw pose value in the model file.
  EXPECT_TRUE(sdf::resolvePose(pose, graph, "P", "__model__").empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::resolvePose(pose, graph, "C", "__model__").empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(sdf::resolvePose(pose, graph, "J", "C").empty());
  EXPECT_EQ(ignition::math::Pose3d(0, 3, 0, 0, -IGN_PI/2, 0), pose);

  EXPECT_TRUE(sdf::resolvePose(pose, graph, "F1", "P").empty());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 1, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::resolvePose(pose, graph, "F2", "C").empty());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 2, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::resolvePose(pose, graph, "F3", "J").empty());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 3, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(sdf::resolvePose(pose, graph, "F4", "F3").empty());
  EXPECT_EQ(ignition::math::Pose3d(0, 0, 4, 0, -IGN_PI/2, 0), pose);

  // Try to resolve invalid frame names
  errors = sdf::resolvePose(pose, graph, "invalid", "__model__");
  for (auto &e : errors)
    std::cerr << e.Message() << std::endl;
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_INVALID);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "PoseRelativeToGraph unable to find unique frame with name ["
        "invalid] in graph."));

  errors = sdf::resolvePose(pose, graph, "__model__", "invalid");
  for (auto &e : errors)
    std::cerr << e.Message() << std::endl;
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::POSE_RELATIVE_TO_INVALID);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "PoseRelativeToGraph unable to find unique frame with name ["
        "invalid] in graph."));
}

/////////////////////////////////////////////////
TEST(NestedFrameSemantics, buildFrameAttachedToGraph_Model)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_nested_frame_attached_to.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first model
  const sdf::Model *model = root.Model();

  auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
  errors = sdf::buildFrameAttachedToGraph(graph, model);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_TRUE(sdf::validateFrameAttachedToGraph(graph).empty());
  EXPECT_TRUE(sdf::checkFrameAttachedToGraph(&root));
  EXPECT_TRUE(sdf::checkFrameAttachedToNames(&root));

  graph = graph.ChildModelScope(model->Name());
  EXPECT_EQ(1u, graph.Count("__model__"));
  EXPECT_EQ(1u, graph.Count("L"));
  EXPECT_EQ(1u, graph.Count("M1"));
  EXPECT_EQ(1u, graph.Count("M1::__model__"));
  EXPECT_EQ(1u, graph.Count("M1::L"));
  EXPECT_EQ(1u, graph.Count("M1::M2"));
  EXPECT_EQ(1u, graph.Count("M1::M2::L"));
  EXPECT_EQ(1u, graph.Count("M1::F"));
  EXPECT_EQ(1u, graph.Count("F0"));
  EXPECT_EQ(1u, graph.Count("F1"));
  EXPECT_EQ(1u, graph.Count("F2"));
  EXPECT_EQ(1u, graph.Count("F3"));
  EXPECT_EQ(1u, graph.Count("F4"));
  EXPECT_EQ(0u, graph.Count("invalid"));

  std::string resolvedBody;
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "__model__").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "L").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
      sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::__model__")
          .empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::L").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::M2").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::M2::L").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::F").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F0").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F1").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F2").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F3").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F4").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);

  // Try to resolve invalid frame name
  errors = sdf::resolveFrameAttachedToBody(resolvedBody, graph, "invalid");
  for (auto &e : errors)
    std::cerr << e.Message() << std::endl;
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_INVALID);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "FrameAttachedToGraph unable to find unique frame with name ["
        "invalid] in graph."));
}

/////////////////////////////////////////////////
TEST(NestedFrameSemantics, buildFrameAttachedToGraph_World)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_nested_frame_attached_to.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  // Get the first world
  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_TRUE(world);

  auto ownedGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> graph(ownedGraph);
  errors = sdf::buildFrameAttachedToGraph(graph, world);
  EXPECT_TRUE(errors.empty());
  errors = sdf::validateFrameAttachedToGraph(graph);
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_TRUE(sdf::checkFrameAttachedToGraph(&root));
  EXPECT_TRUE(sdf::checkFrameAttachedToNames(&root));

  EXPECT_EQ(1u, graph.Count("world"));
  EXPECT_EQ(1u, graph.Count("world_frame"));
  EXPECT_EQ(1u, graph.Count("M1"));
  EXPECT_EQ(1u, graph.Count("M1::__model__"));
  EXPECT_EQ(1u, graph.Count("M1::L"));
  EXPECT_EQ(1u, graph.Count("M1::M2"));
  EXPECT_EQ(1u, graph.Count("M1::M2::__model__"));
  EXPECT_EQ(1u, graph.Count("M1::M2::L"));
  EXPECT_EQ(1u, graph.Count("M1::F0"));
  EXPECT_EQ(1u, graph.Count("F0"));
  EXPECT_EQ(1u, graph.Count("F1"));
  EXPECT_EQ(1u, graph.Count("F2"));
  EXPECT_EQ(1u, graph.Count("F3"));
  EXPECT_EQ(1u, graph.Count("F4"));
  EXPECT_EQ(1u, graph.Count("F5"));
  EXPECT_EQ(1u, graph.Count("F6"));
  EXPECT_EQ(0u, graph.Count("invalid"));

  std::string resolvedBody;
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "world").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(
      resolvedBody, graph, "world_frame").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
      sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::__model__")
          .empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::L").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::M2").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
      sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::M2::__model__")
          .empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::M2::L").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "M1::F0").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F0").empty());
  EXPECT_EQ("world", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F1").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F2").empty());
  EXPECT_EQ("M1::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F3").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F4").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F5").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, graph, "F6").empty());
  EXPECT_EQ("M1::M2::L", resolvedBody);

  // Try to resolve invalid frame name
  errors = sdf::resolveFrameAttachedToBody(resolvedBody, graph, "invalid");
  for (auto &e : errors)
    std::cerr << e.Message() << std::endl;
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FRAME_ATTACHED_TO_INVALID);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "FrameAttachedToGraph unable to find unique frame with name ["
        "invalid] in graph."));

  // Create graph from embedded model
  ASSERT_NE(nullptr, world);
  const sdf::Model *model = world->ModelByIndex(0);

  auto ownedModelGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> modelGraph(ownedModelGraph);
  errors = sdf::buildFrameAttachedToGraph(modelGraph, model);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(sdf::validateFrameAttachedToGraph(modelGraph).empty());

  modelGraph = modelGraph.ChildModelScope(model->Name());

  EXPECT_EQ(1u, modelGraph.Count("__model__"));
  EXPECT_EQ(1u, modelGraph.Count("L"));
  EXPECT_EQ(1u, modelGraph.Count("M2"));
  EXPECT_EQ(1u, modelGraph.Count("M2::__model__"));
  EXPECT_EQ(1u, modelGraph.Count("M2::L"));
  EXPECT_EQ(1u, modelGraph.Count("F0"));
  EXPECT_EQ(0u, modelGraph.Count("invalid"));
  // The following Count expectations are 0 because the frames are out of scope.
  EXPECT_EQ(0u, modelGraph.Count("world"));
  EXPECT_EQ(0u, modelGraph.Count("world_frame"));
  EXPECT_EQ(0u, modelGraph.Count("F1"));
  EXPECT_EQ(0u, modelGraph.Count("F2"));
  EXPECT_EQ(0u, modelGraph.Count("F3"));
  EXPECT_EQ(0u, modelGraph.Count("F4"));
  EXPECT_EQ(0u, modelGraph.Count("F5"));
  EXPECT_EQ(0u, modelGraph.Count("F6"));

  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(
      resolvedBody, modelGraph, "__model__").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "L").empty());
  EXPECT_EQ("L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "M2").empty());
  EXPECT_EQ("M2::L", resolvedBody);
  EXPECT_TRUE(
      sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "M2::__model__")
          .empty());
  EXPECT_EQ("M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "M2::L").empty());
  EXPECT_EQ("M2::L", resolvedBody);
  EXPECT_TRUE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F0").empty());
  EXPECT_EQ("M2::L", resolvedBody);

  errors = sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "world");
  ASSERT_FALSE(errors.empty());

  EXPECT_EQ(
      "FrameAttachedToGraph unable to find unique frame with name "
      "[world] in graph.",
      errors[0].Message());

  errors =
      sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "world_frame");
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(
      "FrameAttachedToGraph unable to find unique frame with name "
      "[world_frame] in graph.",
      errors[0].Message());

  EXPECT_FALSE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F1").empty());
  EXPECT_FALSE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F2").empty());
  EXPECT_FALSE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F3").empty());
  EXPECT_FALSE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F4").empty());
  EXPECT_FALSE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F5").empty());
  EXPECT_FALSE(
    sdf::resolveFrameAttachedToBody(resolvedBody, modelGraph, "F6").empty());
}

/////////////////////////////////////////////////
TEST(NestedFrameSemantics, ModelWithoutLinksWithNestedStaticModel)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_nested_static_model.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);

  auto ownedModelGraph = std::make_shared<sdf::FrameAttachedToGraph>();
  sdf::ScopedGraph<sdf::FrameAttachedToGraph> modelGraph(ownedModelGraph);
  errors = sdf::buildFrameAttachedToGraph(modelGraph, model);
  EXPECT_TRUE(errors.empty()) << errors;
  errors = sdf::validateFrameAttachedToGraph(modelGraph);
  EXPECT_TRUE(errors.empty()) << errors;

  std::string resolvedBody;
  errors = sdf::resolveFrameAttachedToBody(
      resolvedBody, modelGraph, "model_nested_static_model");
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_EQ("model_nested_static_model::child_model::static_model::__model__",
      resolvedBody);
}

/////////////////////////////////////////////////
TEST(NestedFrameSemantics, InvalidAttachedToScope)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "world_frame_invalid_attached_to_scope.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  ASSERT_FALSE(errors.empty());
  const std::string expectedMsg =
      "attached_to name[M2::M1::L1] specified by frame with name[F] "
      "does not match a model or frame name in world with "
      "name[world_frame_invalid_attached_to_scope].";
  EXPECT_EQ(expectedMsg, errors[0].Message());

  EXPECT_FALSE(sdf::checkFrameAttachedToGraph(&root));
  EXPECT_FALSE(sdf::checkFrameAttachedToNames(&root));
}
