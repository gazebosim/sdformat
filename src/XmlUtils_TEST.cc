/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <string>
#include <tinyxml2.h>

#include "XmlUtils.hh"

/////////////////////////////////////////////////
TEST(XMLUtils, DeepClone)
{
  tinyxml2::XMLDocument oldDoc;
  tinyxml2::XMLDocument newDoc;

  std::string docXml = R"(<document>
    <nodeA>
      <nodeB attr="true">Hello World</nodeB>
    </nodeA>
  </document>)";

  auto ret = oldDoc.Parse(docXml.c_str());
  ASSERT_EQ(tinyxml2::XML_SUCCESS, ret);

  auto root = oldDoc.FirstChild();
  sdf::Errors errors;
  auto newRoot = sdf::DeepClone(errors, &newDoc, root);
  EXPECT_TRUE(errors.empty()) << errors;

  EXPECT_STREQ("document", newRoot->ToElement()->Name());

  auto newChild = newRoot->FirstChild();
  EXPECT_STREQ("nodeA", newChild->ToElement()->Name());

  auto newChildB = newChild->FirstChild();
  EXPECT_STREQ("nodeB", newChildB->ToElement()->Name());

  auto childB_attr = newChildB->ToElement()->Attribute("attr");
  EXPECT_STREQ("true", childB_attr);

  auto childB_text = newChildB->ToElement()->GetText();
  EXPECT_STREQ("Hello World", childB_text);
}

/////////////////////////////////////////////////
TEST(XMLUtils, InvalidDeepClone)
{
  sdf::Errors errors;
  auto newRoot = sdf::DeepClone(errors, nullptr, nullptr);
  EXPECT_EQ(1u, errors.size()) << errors;
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::XML_ERROR);
}

/////////////////////////////////////////////////
TEST(XMLUtils, ElementToString)
{
  tinyxml2::XMLDocument doc;

  std::string docXml =
R"(<document>
    <nodeA>
        <nodeB attr="true">Hello World</nodeB>
    </nodeA>
</document>
)";

  auto ret = doc.Parse(docXml.c_str());
  ASSERT_EQ(tinyxml2::XML_SUCCESS, ret);

  auto root = doc.FirstChild();
  sdf::Errors errors;
  std::string docString = sdf::ElementToString(errors, root->ToElement());
  EXPECT_TRUE(errors.empty()) << errors;
  EXPECT_EQ(docXml, docString);
}

/////////////////////////////////////////////////
TEST(XMLUtils, InvalidElementToString)
{
  sdf::Errors errors;
  std::string docString = sdf::ElementToString(errors, nullptr);
  EXPECT_EQ(1u, errors.size()) << errors;
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::XML_ERROR);
}
