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
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"

#include "test_config.h"

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

  EXPECT_EQ(1u, graph.map.count("__model__"));
  EXPECT_EQ(1u, graph.map.count("L"));
  EXPECT_EQ(1u, graph.map.count("F00"));
  EXPECT_EQ(1u, graph.map.count("F0"));
  EXPECT_EQ(1u, graph.map.count("F1"));
  EXPECT_EQ(1u, graph.map.count("F2"));

  // Disable this part of test since FindSinkVertex isn't part of public API.
  // auto linkId = graph.map["L"];

  // for (auto const &nameId : graph.map)
  // {
  //   EXPECT_EQ(nameId.first, graph.graph.VertexFromId(nameId.second).Name());
  //   auto sinkVertexPair =
  //     ignition::math::graph::FindSinkVertex(graph.graph, nameId.second);
  //   EXPECT_EQ(linkId, sinkVertexPair.first.Id());
  // }
}
