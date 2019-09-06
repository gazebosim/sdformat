/*
 * Copyright 2015 Open Source Robotics Foundation
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
#include "sdf/FrameSemantics.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"

#include "test_config.h"

/////////////////////////////////////////////////
TEST(FrameSemantics, buildKinematicGraph)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "double_pendulum.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);

  sdf::KinematicGraph graph;
  auto errors = sdf::buildKinematicGraph(graph, model);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(3u, graph.map.size());
  EXPECT_EQ(3u, graph.graph.Vertices().size());
  EXPECT_EQ(2u, graph.graph.Edges().size());

  EXPECT_EQ(1u, graph.map.count("base"));
  EXPECT_EQ(1u, graph.map.count("lower_link"));
  EXPECT_EQ(1u, graph.map.count("upper_link"));

  auto baseId = graph.map["base"];
  auto lowerLinkId = graph.map["lower_link"];

  for (auto const &nameId : graph.map)
  {
    EXPECT_EQ(nameId.first, graph.graph.VertexFromId(nameId.second).Name());
    auto sourceVertexPair =
      ignition::math::graph::FindSourceVertex(graph.graph, nameId.second);
    EXPECT_EQ(baseId, sourceVertexPair.first.Id());
    auto sinkVertexPair =
      ignition::math::graph::FindSinkVertex(graph.graph, nameId.second);
    EXPECT_EQ(lowerLinkId, sinkVertexPair.first.Id());
  }
}

/////////////////////////////////////////////////
TEST(FrameSemantics, buildFrameAttachedToGraph)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_frame_attached_to.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);

  sdf::FrameAttachedToGraph graph;
  auto errors = sdf::buildFrameAttachedToGraph(graph, model);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(sdf::validateFrameAttachedToGraph(graph).empty());

  EXPECT_EQ(6u, graph.map.size());
  EXPECT_EQ(6u, graph.graph.Vertices().size());
  EXPECT_EQ(5u, graph.graph.Edges().size());

  EXPECT_EQ(1u, graph.map.count(""));
  EXPECT_EQ(1u, graph.map.count("L"));
  EXPECT_EQ(1u, graph.map.count("F00"));
  EXPECT_EQ(1u, graph.map.count("F0"));
  EXPECT_EQ(1u, graph.map.count("F1"));
  EXPECT_EQ(1u, graph.map.count("F2"));

  auto linkId = graph.map["L"];

  for (auto const &nameId : graph.map)
  {
    EXPECT_EQ(nameId.first, graph.graph.VertexFromId(nameId.second).Name());
    auto sinkVertexPair =
      ignition::math::graph::FindSinkVertex(graph.graph, nameId.second);
    EXPECT_EQ(linkId, sinkVertexPair.first.Id());
  }
}

/////////////////////////////////////////////////
TEST(FrameSemantics, buildPoseRelativeToGraph)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "model_frame_relative_to_joint.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);

  sdf::PoseRelativeToGraph graph;
  auto errors = sdf::buildPoseRelativeToGraph(graph, model);
  EXPECT_TRUE(errors.empty());
  EXPECT_TRUE(sdf::validatePoseRelativeToGraph(graph).empty());

  EXPECT_EQ(8u, graph.map.size());
  EXPECT_EQ(8u, graph.graph.Vertices().size());
  EXPECT_EQ(7u, graph.graph.Edges().size());

  EXPECT_EQ(1u, graph.map.count(""));
  EXPECT_EQ(1u, graph.map.count("P"));
  EXPECT_EQ(1u, graph.map.count("C"));
  EXPECT_EQ(1u, graph.map.count("J"));
  EXPECT_EQ(1u, graph.map.count("F1"));
  EXPECT_EQ(1u, graph.map.count("F2"));
  EXPECT_EQ(1u, graph.map.count("F3"));
  EXPECT_EQ(1u, graph.map.count("F4"));

  auto sinkId = graph.map[""];

  for (auto const &nameId : graph.map)
  {
    EXPECT_EQ(nameId.first, graph.graph.VertexFromId(nameId.second).Name());
    auto sinkVertexPair =
      ignition::math::graph::FindSinkVertex(graph.graph, nameId.second);
    EXPECT_EQ(sinkId, sinkVertexPair.first.Id());
  }

  ignition::math::Pose3d pose;
  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d::Zero, pose);
  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "P", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 0, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "F1", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(1, 0, 1, 0, 0, 0), pose);

  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "C", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 0, 0, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "F2", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(4, 0, 0, 0, IGN_PI/2, 0), pose);

  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "J", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 3, 0, 0, 0, 0), pose);
  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "F3", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(2, 3, 3, 0, IGN_PI/2, 0), pose);
  EXPECT_TRUE(sdf::computePoseRelativeToRoot(graph, "F4", pose).empty());
  EXPECT_EQ(ignition::math::Pose3d(6, 3, 3, 0, 0, 0), pose);
}
