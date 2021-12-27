/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include "sdf/parser.hh"
#include "sdf/Plugin.hh"
#include "sdf/Element.hh"

/////////////////////////////////////////////////
TEST(DOMPlugin, Construction)
{
  sdf::Plugin plugin;
  EXPECT_EQ(nullptr, plugin.Element());

  EXPECT_TRUE(plugin.Name().empty());
  EXPECT_TRUE(plugin.Filename().empty());
  EXPECT_TRUE(plugin.Contents().empty());

  plugin.SetName("my-plugin");
  EXPECT_EQ("my-plugin", plugin.Name());

  plugin.SetFilename("filename.so");
  EXPECT_EQ("filename.so", plugin.Filename());

  sdf::ElementPtr content(new sdf::Element);
  content->SetName("an-element");
  plugin.InsertContent(content);
  EXPECT_EQ(1u, plugin.Contents().size());

  sdf::ElementPtr elem = plugin.ToElement();
  ASSERT_NE(nullptr, elem);

  ASSERT_NE(nullptr, elem->GetAttribute("name"));
  EXPECT_EQ("my-plugin", elem->GetAttribute("name")->GetAsString());

  ASSERT_NE(nullptr, elem->GetAttribute("filename"));
  EXPECT_EQ("filename.so", elem->GetAttribute("filename")->GetAsString());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, MoveConstructor)
{
  sdf::Plugin plugin;
  plugin.SetName("pluginname");
  plugin.SetFilename("filename");

  sdf::ElementPtr content(new sdf::Element);
  content->SetName("an-element");
  plugin.InsertContent(content);

  sdf::Plugin plugin2(std::move(plugin));
  EXPECT_EQ("pluginname", plugin2.Name());
  EXPECT_EQ("filename", plugin2.Filename());
  ASSERT_EQ(1u, plugin2.Contents().size());
  EXPECT_EQ("an-element", plugin2.Contents()[0]->GetName());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, CopyConstructor)
{
  sdf::Plugin plugin;
  plugin.SetName("pluginname");
  plugin.SetFilename("filename");

  sdf::Plugin plugin2(plugin);
  EXPECT_EQ("pluginname", plugin2.Name());
  EXPECT_EQ("filename", plugin2.Filename());

  EXPECT_EQ("pluginname", plugin.Name());
  EXPECT_EQ("filename", plugin.Filename());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, CopyAssigmentOperator)
{
  sdf::Plugin plugin;
  plugin.SetName("pluginname");
  plugin.SetFilename("filename");

  sdf::Plugin plugin2;
  plugin2 = plugin;
  EXPECT_EQ("pluginname", plugin2.Name());
  EXPECT_EQ("filename", plugin2.Filename());

  EXPECT_EQ("pluginname", plugin.Name());
  EXPECT_EQ("filename", plugin.Filename());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, MoveAssignmentConstructor)
{
  sdf::Plugin plugin;
  plugin.SetName("pluginname");
  plugin.SetFilename("filename");

  sdf::Plugin plugin2;
  plugin2 = std::move(plugin);
  EXPECT_EQ("pluginname", plugin2.Name());
  EXPECT_EQ("filename", plugin2.Filename());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, CopyAssignmentAfterMove)
{
  sdf::Plugin plugin;
  plugin.SetName("pluginname");
  plugin.SetFilename("filename");

  sdf::Plugin plugin2;
  plugin2.SetName("pluginname2");
  plugin2.SetFilename("filename2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Plugin tmp = std::move(plugin);
  plugin = plugin2;
  plugin2 = tmp;

  EXPECT_EQ("pluginname", plugin2.Name());
  EXPECT_EQ("filename", plugin2.Filename());

  EXPECT_EQ("pluginname2", plugin.Name());
  EXPECT_EQ("filename2", plugin.Filename());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, Load)
{
  sdf::Plugin plugin;
  sdf::Errors errors;

  // Null sdf
  errors = plugin.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = plugin.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, plugin.Element());

  sdf->SetName("plugin");

  // Missing name and filename attribute
  errors = plugin.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[1].Code());

  sdf->AddAttribute("name", "string", "__default__", true);
  sdf->GetAttribute("name")->Set<std::string>("my-plugin-name");

  // Now just missing filename
  errors = plugin.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());

  sdf->AddAttribute("filename", "string", "__default__", true);
  sdf->GetAttribute("filename")->Set<std::string>("filename.so");

  // No errors.
  errors = plugin.Load(sdf);
  ASSERT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, LoadWithChildren)
{
  std::string pluginStr = R"(<plugin name='3D View' filename='MinimalScene'>
  <ignition-gui>
    <title>3D View</title>
    <property type='bool' key='showTitleBar'>false</property>
    <property type='string' key='state'>docked</property>
  </ignition-gui>
  <engine>ogre</engine>
  <scene>scene</scene>
  <ambient_light>0.4 0.4 0.4</ambient_light>
  <background_color>0.8 0.8 0.8</background_color>
  <camera_pose>-6 0 6 0 0.5 0</camera_pose>
</plugin>
)";

  std::string pluginStrWithSdf = std::string("<sdf version='1.9'>") +
    pluginStr + "</sdf>";
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("plugin.sdf", elem);
  ASSERT_TRUE(sdf::readString(pluginStrWithSdf, elem));

  sdf::Plugin plugin;
  sdf::Errors errors;
  errors = plugin.Load(elem);
  ASSERT_EQ(0u, errors.size());

  EXPECT_EQ("3D View", plugin.Name());
  EXPECT_EQ("MinimalScene", plugin.Filename());

  // The elements should be the same
  EXPECT_EQ(elem->ToString(""), plugin.Element()->ToString(""));

  sdf::ElementPtr toElem = plugin.ToElement();

  // The elements should be the same
  EXPECT_EQ(elem->ToString(""), toElem->ToString(""));
  EXPECT_EQ(pluginStr, toElem->ToString(""));

  // Test plugin copy
  sdf::Plugin plugin3;
  plugin3 = plugin;
  plugin.ClearContents();
  sdf::Plugin plugin4(plugin3);

  toElem = plugin3.ToElement();
  EXPECT_EQ(6u, plugin3.Contents().size());
  EXPECT_EQ(pluginStr, toElem->ToString(""));

  toElem = plugin4.ToElement();
  EXPECT_EQ(6u, plugin4.Contents().size());
  EXPECT_EQ(pluginStr, toElem->ToString(""));
}

/////////////////////////////////////////////////
TEST(DOMPlugin, ToElement)
{
  sdf::Plugin plugin;
  plugin.SetName("my-plugin");
  EXPECT_EQ("my-plugin", plugin.Name());

  plugin.SetFilename("filename.so");
  EXPECT_EQ("filename.so", plugin.Filename());

  sdf::ElementPtr content(new sdf::Element);
  content->SetName("an-element");
  plugin.InsertContent(content);
  EXPECT_EQ(1u, plugin.Contents().size());

  sdf::ElementPtr elem = plugin.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Plugin plugin2;
  plugin2.Load(elem);

  EXPECT_EQ(plugin.Name(), plugin2.Name());
  EXPECT_EQ(plugin.Filename(), plugin2.Filename());
  EXPECT_EQ(1u, plugin2.Contents().size());
  EXPECT_EQ("an-element", plugin2.Contents()[0]->GetName());
}
