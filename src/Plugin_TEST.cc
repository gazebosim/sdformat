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
#include "test_utils.hh"

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
  ASSERT_EQ(1u, errors.size()) << errors;
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());

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
TEST(DOMPlugin, LoadWithoutName)
{
  std::string pluginStr = R"(<plugin filename='MinimalScene'>
  <gz-gui>
    <title>3D View</title>
    <property type='Gz.Msgs.Boolean'>false</property>
    <property type='bool' key='showTitleBar'>0</property>
    <property type='string' key='state'>docked</property>
  </gz-gui>
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

  EXPECT_TRUE(plugin.Name().empty());
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
TEST(DOMPlugin, LoadWithChildren)
{
  std::string pluginStr = R"(<plugin name='3D View' filename='MinimalScene'>
  <gz-gui>
    <title>3D View</title>
    <property type='Gz.Msgs.Boolean'>false</property>
    <property type='bool' key='showTitleBar'>0</property>
    <property type='string' key='state'>docked</property>
  </gz-gui>
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

/////////////////////////////////////////////////
TEST(DOMPlugin, OutputStreamOperator)
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
  std::string elemString = elem->ToString("");

  std::ostringstream stream;
  stream << plugin;

  EXPECT_EQ(elemString, stream.str());

  // The expected plugin output string.
  std::string expected = R"foo(<plugin name='my-plugin' filename='filename.so'>
  <an-element/>
</plugin>
)foo";

  EXPECT_EQ(expected, stream.str());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, InputStreamOperator)
{
  // The provided plugin input string.
  std::string input = R"foo(<plugin name='my-plugin' filename='filename.so'>
  <an-element/>
</plugin>
)foo";
  std::istringstream stream(input);

  sdf::Plugin plugin;
  stream >> plugin;

  EXPECT_EQ("my-plugin", plugin.Name());
  EXPECT_EQ("filename.so", plugin.Filename());
  EXPECT_EQ(1u, plugin.Contents().size());

  sdf::ElementPtr elem = plugin.ToElement();
  ASSERT_NE(nullptr, elem);
  std::string elemString = elem->ToString("");
  EXPECT_EQ(input, elemString);
}

/////////////////////////////////////////////////
TEST(DOMPlugin, InsertStringContent)
{
  sdf::Plugin plugin("my-filename", "my-name",
      "<render_engine>ogre2</render_engine>");
  EXPECT_EQ("my-filename", plugin.Filename());
  EXPECT_EQ("my-name", plugin.Name());
  ASSERT_EQ(1u, plugin.Contents().size());
  EXPECT_EQ("render_engine", plugin.Contents()[0]->GetName());
  EXPECT_EQ("ogre2", plugin.Contents()[0]->Get<std::string>());

  std::string extraContent = R"foo(
  <with_attribute value='bar'>1.234</with_attribute>
  <sibling>hello</sibling>
  <with_children>
    <child1>goodbye</child1>
    <child2>goodbye</child2>
  </with_children>
)foo";

  // Insert more content using a string
  EXPECT_TRUE(plugin.InsertContent(extraContent));

  std::ostringstream completeContent;
  completeContent << "  <render_engine>ogre2</render_engine>" << extraContent;

  std::ostringstream completePlugin;
  completePlugin << "<plugin name='my-name' filename='my-filename'>\n"
    << completeContent.str()
    << "</plugin>\n";
  EXPECT_EQ(completePlugin.str(), plugin.ToElement()->ToString(""));

  // Try out curly braces initialization
  sdf::Plugin plugin2{plugin.Filename(), plugin.Name(), completeContent.str()};
  EXPECT_EQ(plugin.ToElement()->ToString(""),
            plugin2.ToElement()->ToString(""));

  // Try to insert poorly formed XML, which should fail and not modify the
  // content.
  EXPECT_FALSE(plugin2.InsertContent("<a></b>"));
  EXPECT_EQ(plugin.ToElement()->ToString(""),
            plugin2.ToElement()->ToString(""));

  // An empty string will also fail and not modify the content
  EXPECT_FALSE(plugin2.InsertContent(""));
  EXPECT_EQ(plugin.ToElement()->ToString(""),
            plugin2.ToElement()->ToString(""));

  // Constructing a new plugin with no content
  sdf::Plugin plugin3{"a filename", "a name"};
  EXPECT_EQ("a filename", plugin3.Filename());
  EXPECT_EQ("a name", plugin3.Name());
  EXPECT_TRUE(plugin3.Contents().empty());

  // Constructing a new plugin with bad XML content
  sdf::Plugin plugin4{"filename", "name", "<garbage>"};
  EXPECT_EQ("filename", plugin4.Filename());
  EXPECT_EQ("name", plugin4.Name());
  EXPECT_TRUE(plugin4.Contents().empty());

  // Constructing a new plugin with bad XML content
  sdf::Plugin plugin5{"filename", "name", "    "};
  EXPECT_EQ("filename", plugin5.Filename());
  EXPECT_EQ("name", plugin5.Name());
  EXPECT_TRUE(plugin5.Contents().empty());
}

/////////////////////////////////////////////////
TEST(DOMPlugin, EqualityOperators)
{
  sdf::Plugin plugin("my-filename", "my-name",
      "<render_engine>ogre2</render_engine>");
  sdf::Plugin plugin2(plugin);
  sdf::Plugin plugin3;

  EXPECT_EQ(plugin, plugin2);
  EXPECT_NE(plugin, plugin3);
  EXPECT_NE(plugin2, plugin3);

  // Test contents
  plugin2.ClearContents();
  EXPECT_NE(plugin, plugin2);
  plugin.ClearContents();
  EXPECT_EQ(plugin, plugin2);

  // test name
  plugin2.SetName("new-name");
  EXPECT_NE(plugin, plugin2);
  plugin.SetName("new-name");
  EXPECT_EQ(plugin, plugin2);

  // test filename
  plugin2.SetFilename("new-filename");
  EXPECT_NE(plugin, plugin2);
  plugin.SetFilename("new-filename");
  EXPECT_EQ(plugin, plugin2);
}

///////////////////////////////////////////////
TEST(DOMPlugin, ErrorOutput)
{
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

  sdf::Errors errors;
  sdf::Plugin Plugin(errors, "", "", "Not valid xml");
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARSING_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Error parsing XML from string: "));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
