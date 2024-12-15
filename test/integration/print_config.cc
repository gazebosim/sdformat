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

#include <limits>
#include <map>
#include <sstream>
#include <string>

#include "sdf/Camera.hh"
#include "sdf/Geometry.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/Root.hh"
#include "sdf/Sensor.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"

#include "test_config.hh"

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
  config.SetOutPrecision(6);
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
// https://github.com/gazebosim/sdformat/issues/769
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
  // as it was set using gz::math::Pose3d::operator<<, this test will have
  // to be modified when we start using gazebosim/gz-math#206.
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

void PrecisionTest(const sdf::ElementPtr _elem,
    const std::map<std::string, std::string> &_expected)
{
  ASSERT_NE(_elem, nullptr);

  EXPECT_EQ(_elem->GetValue()->GetAsString(), _expected.at("default"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  EXPECT_EQ(_elem->GetValue()->GetAsString(config), _expected.at("6"));

  config.SetOutPrecision(2);
  EXPECT_EQ(_elem->GetValue()->GetAsString(config), _expected.at("2"));

  config.SetOutPrecision(std::numeric_limits<int>::max());
  EXPECT_EQ(_elem->GetValue()->GetAsString(config), _expected.at("default"));
}

/////////////////////////////////////////////////
TEST(PrintConfig, OutPrecision)
{
  const std::string sdfStr =
R"(<?xml version='1.0' ?>
<sdf version="1.9">
  <world name="default">
    <model name="M1">
      <link name="L1">
        <visual name="v1">
          <pose rotation_format='quat_xyzw'>
            1 1.0 -1.5707963267948966
            0.707106781 0 0 0.707106781
          </pose>
          <geometry>
            <sphere>
              <radius>1.23456789123456789</radius>
            </sphere>
          </geometry>
          <material>
            <diffuse>0.15 0.6 0.87525741223854215 1</diffuse>
            <render_order>0.16</render_order>
          </material>
          <transparency>0.15</transparency>
        </visual>
        <sensor name='camera' type='camera'>
          <camera>
            <horizontal_fov>1.4</horizontal_fov>
            <clip>
              <near>0.15</near>
            </clip>
            <distortion>
              <center>0.1512345678912345678 1.4</center>
            </distortion>
          </camera>
        </sensor>
      </link>
      <joint name="j1" type="revolute">
        <parent>world</parent>
        <child>L1</child>
        <axis>
          <xyz>1.5707963267948966 0.34 0.56</xyz>
        </axis>
      </joint>
    </model>
  </world>
</sdf>
)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(sdfStr);
  EXPECT_TRUE(errors.empty()) << errors;

  auto *world = root.WorldByIndex(0);
  ASSERT_NE(world, nullptr);
  auto *model = world->ModelByIndex(0);
  ASSERT_NE(model, nullptr);
  auto *link = model->LinkByIndex(0);
  ASSERT_NE(link, nullptr);
  auto *visual = link->VisualByIndex(0);
  ASSERT_NE(visual, nullptr);


  // Below we are comparing default (maximum) then setting the precision
  // to 6 (std::ostream's default), 2, and then back to max.
  // Used this IEEE 754 floating point converter to determine expected strings:
  // https://baseconvert.com/ieee-754-floating-point
  // double (64 bit) has max of 17 digits
  // float (32 bit) has max of 9 digits

  // key: precision, value: expected string
  std::map<std::string, std::string> expected = {
      {"default", ""},
      {"6", ""},
      {"2", ""}
  };
  sdf::ElementPtr elem;

  // //pose -> type: gz::math::Pose3d
  {
    SCOPED_TRACE("PrecisionTest: Pose3d");
    elem = visual->Element()->FindElement("pose");
    expected["default"] =
        "1 1 -1.5707963267948966   "
        "0.70710678100000002 0 0 0.70710678100000002";
    expected["6"] = "1 1 -1.5708   0.707107 0 0 0.707107";
    expected["2"] = "1 1 -1.6   0.71 0 0 0.71";

    PrecisionTest(elem, expected);
  }

  // //radius -> type: double
  {
    auto *geom = visual->Geom();
    ASSERT_NE(geom, nullptr);
    auto *sphere = geom->SphereShape();
    ASSERT_NE(sphere, nullptr);

    SCOPED_TRACE("PrecisionTest: double");
    elem = sphere->Element()->FindElement("radius");
    expected["default"] = "1.2345678912345679";
    expected["6"] = "1.23457";
    expected["2"] = "1.2";

    PrecisionTest(elem, expected);
  }

  auto *material = visual->Material();
  ASSERT_NE(material, nullptr);

  // //material/diffuse -> type: gz::math::Color (float values)
  {
    SCOPED_TRACE("PrecisionTest: Color");
    elem = material->Element()->FindElement("diffuse");
    expected["default"] =
        "0.150000006 0.600000024 0.875257432 1";
    expected["6"] = "0.15 0.6 0.875257 1";
    expected["2"] = "0.15 0.6 0.88 1";

    PrecisionTest(elem, expected);
  }

  // //material/render_order -> type: float
  {
    SCOPED_TRACE("PrecisionTest: float");
    elem = material->Element()->FindElement("render_order");
    expected["default"] = "0.159999996";
    expected["6"] = "0.16";
    expected["2"] = "0.16";

    PrecisionTest(elem, expected);
  }

  // //camera/distortion/center -> type: gz::math::Vector2d
  {
    auto *sensor = link->SensorByIndex(0);
    ASSERT_NE(sensor, nullptr);
    auto *camera = sensor->CameraSensor();
    ASSERT_NE(camera, nullptr);
    elem = camera->Element()->FindElement("distortion");
    ASSERT_NE(elem, nullptr);

    SCOPED_TRACE("PrecisionTest: Vector2d");
    elem = elem->FindElement("center");
    expected["default"] = "0.15123456789123457 1.3999999999999999";
    expected["6"] = "0.151235 1.4";
    expected["2"] = "0.15 1.4";

    PrecisionTest(elem, expected);
  }

  // //joint/axis/xyz -> type: gz::math::Vector3d
  {
    auto *joint = model->JointByIndex(0);
    ASSERT_NE(joint, nullptr);
    auto *axis = joint->Axis(0);
    ASSERT_NE(axis, nullptr);

    SCOPED_TRACE("PrecisionTest: Vector3d");
    elem = axis->Element()->FindElement("xyz");
    expected["default"] =
        "1.5707963267948966 0.34000000000000002 0.56000000000000005";
    expected["6"] = "1.5708 0.34 0.56";
    expected["2"] = "1.6 0.34 0.56";

    PrecisionTest(elem, expected);
  }
}
