/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include <sstream>
#include <string>

#include "sdf/sdf.hh"

#include "test_config.h"

/////////////////////////////////////////////////
TEST(PrintConfig, PreserveIncludes)
{
  const std::string modelPath = sdf::testing::TestFile("integration", "model");

  sdf::ParserConfig parserConfig;
  parserConfig.SetFindCallback(
    [&](const std::string &_file)
    {
      return sdf::filesystem::append(modelPath, _file);
    });

  const std::string includeStr =
R"(<include>
  <uri>box</uri>
  <name>test_box</name>
  <pose>1 2 3 0 0 0</pose>
  <placement_frame>link</placement_frame>
  <plugin name='test_plugin' filename='test_plugin_file'/>
</include>
)";

  const std::string modelWithIncludeStr =
R"(<model name='test2'>
  <include>
    <uri>test_model</uri>
    <name>test_model</name>
    <pose>1 2 3 0 0 0</pose>
    <placement_frame>link</placement_frame>
    <plugin name='test_plugin' filename='test_plugin_file'/>
  </include>
</model>
)";

  const std::string sdfStr =
    "<sdf version='1.9'>"
    "  <world name='default'>"
    + includeStr +
    modelWithIncludeStr +
    "  </world>"
    "</sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdfStr, parserConfig);
  EXPECT_TRUE(errors.empty()) << errors;

  auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  auto *includedModel = world->ModelByIndex(0);
  ASSERT_NE(includedModel, nullptr);
  auto *modelWithInclude = world->ModelByIndex(1);
  ASSERT_NE(modelWithInclude, nullptr);

  const std::string expandedIncludeStr =
R"(<model name='test_box' placement_frame='link'>
  <pose>1 2 3 0 0 0</pose>
  <link name='link'>
    <collision name='collision'>
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name='visual'>
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
  <plugin name='test_plugin' filename='test_plugin_file'/>
</model>
)";

  const std::string expandedModelWithIncludeStr =
R"(<model name='test2'>
  <model name='test_model' placement_frame='link'>
    <link name='link'>
      <collision name='mesh_col'>
        <geometry>
          <mesh>
            <uri>meshes/mesh.dae</uri>
            <submesh>
              <name>my_submesh</name>
              <center>true</center>
            </submesh>
            <scale>0.1 0.2 0.3</scale>
          </mesh>
        </geometry>
      </collision>
      <visual name='mesh_vis'>
        <geometry>
          <mesh>
            <uri>meshes/mesh.dae</uri>
            <submesh>
              <name>another_submesh</name>
              <center>false</center>
            </submesh>
            <scale>1.2 2.3 3.4</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
    <pose>1 2 3 0 0 0</pose>
    <plugin name='test_plugin' filename='test_plugin_file'/>
  </model>
</model>
)";

  sdf::PrintConfig config;
  // by default, included model should be expanded
  EXPECT_EQ(includedModel->Element()->ToString("", config), expandedIncludeStr);
  EXPECT_EQ(modelWithInclude->Element()->ToString("", config),
            expandedModelWithIncludeStr);

  config.SetPreserveIncludes(true);
  EXPECT_EQ(includedModel->Element()->ToString("", config), includeStr);
  EXPECT_EQ(modelWithInclude->Element()->ToString("", config),
            modelWithIncludeStr);
}

/////////////////////////////////////////////////
// Test verifies preserving includes does not work for merge-includes.
// Need to update test if issue is addressed.
// https://github.com/ignitionrobotics/sdformat/issues/769
TEST(PrintConfig, PreserveIncludesWithMerge)
{
  const std::string modelPath = sdf::testing::TestFile("integration", "model");

  sdf::ParserConfig parserConfig;
  parserConfig.SetFindCallback(
    [&](const std::string &_file)
    {
      return sdf::filesystem::append(modelPath, _file);
    });

  const std::string includeMergeStr =
R"(<model name="m2">
  <include merge="true">
    <uri>box</uri>
    <name>test_box2</name>
  </include>
</model>
)";

  const std::string sdfStr =
    "<sdf version='1.9'>"
    "  <world name='default'>"
    + includeMergeStr +
    "  </world>"
    "</sdf>";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdfStr, parserConfig);
  EXPECT_TRUE(errors.empty()) << errors;

  auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  auto *includeMergedModel = world->ModelByIndex(0);
  ASSERT_NE(includeMergedModel, nullptr);

  // The expected output pose string here still contains a -0 on the pitch value
  // as it was set using ignition::math::Pose3d::operator<<, this test will have
  // to be modified when we start using ignitionrobotics/ign-math#206.
  const std::string expectedIncludeMerge =
R"(<model name='m2'>
  <frame name='_merged__test_box2__model__' attached_to='link'>
    <pose relative_to='__model__'>0 0 0.5 0 0 0</pose>
  </frame>
  <link name='link'>
    <collision name='collision'>
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name='visual'>
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
    <pose relative_to='_merged__test_box2__model__'>0 0 0 0 0 0</pose>
  </link>
</model>
)";

  sdf::PrintConfig config;
  EXPECT_EQ(includeMergedModel->Element()->ToString("", config),
            expectedIncludeMerge);
  config.SetPreserveIncludes(true);
  EXPECT_NE(includeMergedModel->Element()->ToString("", config),
            includeMergeStr);
  EXPECT_EQ(includeMergedModel->Element()->ToString("", config),
            expectedIncludeMerge);
}
