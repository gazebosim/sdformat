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
    <custom>
      <custom_child>ThisIsCustom</custom_child>
      <another test="yes">1.23</another>
      <nested attr="my nested xml">
        <string>AString</string>
      </nested>
    </custom>
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

  sdf::ElementPtr customElem = worldElem->GetElement("custom");
  ASSERT_NE(nullptr, customElem);

  std::string customChild = customElem->Get<std::string>("custom_child");
  EXPECT_EQ("ThisIsCustom", customChild);

  sdf::ElementPtr anotherElem = customElem->GetElement("another");
  EXPECT_EQ("yes", anotherElem->Get<std::string>("test"));
  EXPECT_DOUBLE_EQ(1.23, anotherElem->Get<double>());

  sdf::ElementPtr nestedElem = customElem->GetElement("nested");
  EXPECT_EQ("my nested xml", nestedElem->Get<std::string>("attr"));
  EXPECT_EQ("AString", nestedElem->Get<std::string>("string"));
}
