/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"
#include "sdf/Error.hh"

#include "test_config.h"

/////////////////////////////////////////////////
/// Test the copy of XML elements that are not part of the specification.
TEST(Unknown, CopyUnknownElement)
{
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <gui/>
    <custom>
      <custom_child>ThisIsCustom</custom_child>
      <another test="yes">1.23</another>
      <nested attr="my nested xml">
        <string>AString</string>
      </nested>
    </custom>
    <unknown/>
  </world>
</sdf>)";

  sdf::Errors errors;

  // Read an SDF file, and store the result in sdfParsed.
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::readString(xmlString, sdfParsed, errors);

  sdf::ElementPtr elem = sdfParsed->Root();
  ASSERT_NE(nullptr, elem);

  sdf::ElementPtr worldElem = elem->GetElement("world");
  ASSERT_NE(nullptr, worldElem);

  // Expect exactly one copy of gui element
  sdf::ElementPtr guiElem = worldElem->GetElement("gui");
  ASSERT_NE(nullptr, guiElem);
  EXPECT_EQ(nullptr, guiElem->GetNextElement("gui"));

  // Expect exactly one copy of unknown element
  sdf::ElementPtr unknownElem = worldElem->GetElement("unknown");
  ASSERT_NE(nullptr, unknownElem);
  EXPECT_EQ(nullptr, unknownElem->GetNextElement("unknown"));

  // Expect exactly one copy of custom element
  sdf::ElementPtr customElem = worldElem->GetElement("custom");
  ASSERT_NE(nullptr, customElem);
  EXPECT_EQ(nullptr, customElem->GetNextElement("custom"));

  std::string customChild = customElem->Get<std::string>("custom_child");
  EXPECT_EQ("ThisIsCustom", customChild);

  sdf::ElementPtr anotherElem = customElem->GetElement("another");
  EXPECT_EQ("yes", anotherElem->Get<std::string>("test"));
  EXPECT_DOUBLE_EQ(1.23, anotherElem->Get<double>());

  sdf::ElementPtr nestedElem = customElem->GetElement("nested");
  EXPECT_EQ("my nested xml", nestedElem->Get<std::string>("attr"));
  EXPECT_EQ("AString", nestedElem->Get<std::string>("string"));
}

/////////////////////////////////////////////////
/// Test that elements that aren't part of the spec are flagged with when
/// UnrecognizedElementsPolicy is set to err
TEST(UnrecognizedElements, UnrecognizedElementsWithWarningsPolicies)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "unrecognized_elements.sdf");

  sdf::ParserConfig config;

  {
    config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::ERR);
    sdf::Root root;
    const auto errors = root.Load(testFile, config);
    ASSERT_FALSE(errors.empty());
    constexpr char expectedMessage[] =
      "XML Element[not_a_link_element], child of element[link], not defined in"
      " SDF. Copying[not_a_link_element] as children of [link].\n";
    EXPECT_EQ(errors[0].Message(), expectedMessage);
  }
  {
    config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::WARN);
    sdf::Root root;
    const auto errors = root.Load(testFile, config);
    EXPECT_TRUE(errors.empty());
  }
  {
    config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::LOG);
    sdf::Root root;
    const auto errors = root.Load(testFile, config);
    EXPECT_TRUE(errors.empty());
  }
}

/////////////////////////////////////////////////
/// Test that elements that aren't part of the spec but have the namespace
/// separator ":" don't cause errors even with EnforcementPolicy::ERR
TEST(UnrecognizedElements, UnrecognizedElementsWithNamespaces)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "unrecognized_elements_with_namespace.sdf");

  sdf::ParserConfig config;
  config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::ERR);

  sdf::Root root;
  const auto errors = root.Load(testFile, config);
  EXPECT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
/// Test that elements that aren't part of the version of the spec
/// cause an error (#327) - e.g. something that was valid in SDFormat 1.6 but
/// not in SDFormat>=1.7 (https://git.io/JtKKb).
TEST(UnrecognizedElements, OldElementsInNewSchemas)
{
  // This should be valid in 1.6, but not in 1.8 (do to usage of
  // `use_parent_model_frame`).
  auto make_model_xml_string = [](std::string version)
  {
    return R"(
<?xml version="1.0" ?>
<sdf version=")" + version + R"(">
  <model name="default">
    <joint name="joint_WA" type="revolute">
      <parent>world</parent>
      <child>A</child>
      <axis>
        <xyz>0 1 0</xyz>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
    </joint>
    <link name="A"/>
  </model>
</sdf>)";
  };

  sdf::ParserConfig config;
  config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::ERR);

  // Valid in 1.6.
  {
    sdf::Root root;
    const auto errors = root.LoadSdfString(
        make_model_xml_string("1.6"), config);
    ASSERT_TRUE(errors.empty());
  }

  // Invalid in >=1.7.
  {
    sdf::Root root;
    const auto errors = root.LoadSdfString(
        make_model_xml_string("1.8"), config);
    ASSERT_EQ(errors.size(), 1u);
    constexpr char expectedMessage[] =
        "XML Element[use_parent_model_frame], child of element[axis], not "
        "defined in SDF. Copying[use_parent_model_frame] as children of "
        "[axis].\n";
    EXPECT_EQ(errors[0].Message(), expectedMessage);
  }
}
