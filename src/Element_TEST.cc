/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include "sdf/Element.hh"
#include "sdf/Param.hh"

/////////////////////////////////////////////////
TEST(Element, New)
{
  sdf::Element elem;

  ASSERT_FALSE(elem.GetCopyChildren());
  ASSERT_EQ(elem.ReferenceSDF(), "");
  ASSERT_EQ(elem.GetParent(), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, Child)
{
  sdf::Element child;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();

  ASSERT_EQ(child.GetParent(), nullptr);

  child.SetParent(parent);

  ASSERT_NE(child.GetParent(), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, Name)
{
  sdf::Element elem;

  ASSERT_EQ(elem.GetName(), "");

  elem.SetName("test");

  ASSERT_EQ(elem.GetName(), "test");
}

/////////////////////////////////////////////////
TEST(Element, Required)
{
  sdf::Element elem;

  ASSERT_EQ(elem.GetRequired(), "");

  elem.SetRequired("1");

  ASSERT_EQ(elem.GetRequired(), "1");
}

/////////////////////////////////////////////////
TEST(Element, CopyChildren)
{
  sdf::Element elem;

  ASSERT_FALSE(elem.GetCopyChildren());

  elem.SetCopyChildren(true);

  ASSERT_TRUE(elem.GetCopyChildren());
}

/////////////////////////////////////////////////
TEST(Element, ReferenceSDF)
{
  sdf::Element elem;

  ASSERT_EQ(elem.ReferenceSDF(), "");

  elem.SetReferenceSDF("test");

  ASSERT_EQ(elem.ReferenceSDF(), "test");
}

/////////////////////////////////////////////////
TEST(Element, AddValue)
{
  sdf::Element elem;

  elem.SetName("test");
  elem.AddValue("string", "foo", false, "foo description");

  sdf::ParamPtr param = elem.GetValue();
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");
}

/////////////////////////////////////////////////
TEST(Element, AddAttribute)
{
  sdf::Element elem;

  ASSERT_EQ(elem.GetAttributeCount(), 0UL);

  elem.AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(elem.GetAttributeCount(), 1UL);

  elem.AddAttribute("attr", "float", "0.0", false, "float description");
  ASSERT_EQ(elem.GetAttributeCount(), 2UL);

  sdf::ParamPtr param = elem.GetAttribute("test");
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");

  param = elem.GetAttribute("attr");
  ASSERT_TRUE(param->IsType<float>());
  ASSERT_EQ(param->GetKey(), "attr");
  ASSERT_EQ(param->GetTypeName(), "float");
  ASSERT_EQ(param->GetDefaultAsString(), "0");
  ASSERT_EQ(param->GetDescription(), "float description");
}

/////////////////////////////////////////////////
TEST(Element, GetAttributeSet)
{
  sdf::Element elem;
  ASSERT_EQ(elem.GetAttributeCount(), 0UL);
  elem.AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(elem.GetAttributeCount(), 1UL);

  EXPECT_FALSE(elem.GetAttributeSet("test"));
  elem.GetAttribute("test")->Set("asdf");
  EXPECT_TRUE(elem.GetAttributeSet("test"));
}

/////////////////////////////////////////////////
TEST(Element, Include)
{
  sdf::Element elem;

  ASSERT_EQ(elem.GetInclude(), "");

  elem.SetInclude("foo.txt");

  ASSERT_EQ(elem.GetInclude(), "foo.txt");
}

/////////////////////////////////////////////////
TEST(Element, Description)
{
  sdf::Element elem;

  ASSERT_EQ(elem.GetDescription(), "");

  elem.SetDescription("Element description");

  ASSERT_EQ(elem.GetDescription(), "Element description");
}

/////////////////////////////////////////////////
TEST(Element, AddElementNoDescription)
{
  sdf::Element elem;

  sdf::ElementPtr ptr = elem.AddElement("foo");

  ASSERT_EQ(ptr, sdf::ElementPtr());
}

/////////////////////////////////////////////////
TEST(Element, AddElementReferenceSDF)
{
  // TODO(clalancette): The Element class uses shared_from_this(), and one of
  // the prerequisites of it is that the object must *already* have a
  // shared_ptr to it before calling shared_from_this().  The fallout from that
  // is that you cannot instantiate an object using the normal constructor,
  // and then call the .AddElement() method.  You must make a shared_ptr,
  // and then you can call ->AddElement(), which works.  This is kind of a
  // weird restriction on a class, so we might want to make this better.
  sdf::ElementPtr child = std::make_shared<sdf::Element>();
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr desc = std::make_shared<sdf::Element>();
  desc->SetName("parent");
  parent->SetName("parent");
  parent->AddElementDescription(desc);

  child->SetParent(parent);

  child->SetReferenceSDF("refsdf");
  child->SetName("parent");

  ASSERT_EQ(child->GetElementDescriptionCount(), 0UL);
  sdf::ElementPtr ptr = child->AddElement("parent");

  ASSERT_EQ(child->GetElementDescriptionCount(), 1UL);

  sdf::ElementPtr check = child->GetElementDescription(4);
  ASSERT_EQ(check, sdf::ElementPtr());

  check = child->GetElementDescription(0);
  ASSERT_NE(check, sdf::ElementPtr());
  ASSERT_EQ(check->GetName(), "parent");

  check = child->GetElementDescription("bad");
  ASSERT_EQ(check, sdf::ElementPtr());

  check = child->GetElementDescription("parent");
  ASSERT_NE(check, sdf::ElementPtr());
  ASSERT_EQ(check->GetName(), "parent");

  ASSERT_FALSE(child->HasElement("foo"));
  ASSERT_TRUE(child->HasElement("parent"));

  child->Reset();
  ASSERT_EQ(child->GetElementDescriptionCount(), 0UL);
  ASSERT_EQ(child->GetAttributeCount(), 0UL);
}

/////////////////////////////////////////////////
TEST(Element, GetTemplates)
{
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();

  elem->AddAttribute("test", "string", "foo", false, "foo description");

  std::string out = elem->Get<std::string>("test");
  ASSERT_EQ(out, "foo");

  std::pair<std::string, bool> pairout = elem->Get<std::string>("test", "def");
  ASSERT_EQ(pairout.first, "foo");
  ASSERT_EQ(pairout.second, true);

  bool found = elem->Get<std::string>("test", out, "def");
  ASSERT_EQ(out, "foo");
  ASSERT_EQ(found, true);
}

/////////////////////////////////////////////////
TEST(Element, Clone)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr child = std::make_shared<sdf::Element>();
  sdf::ElementPtr desc = std::make_shared<sdf::Element>();

  parent->InsertElement(child);
  ASSERT_NE(parent->GetFirstElement(), nullptr);

  parent->AddElementDescription(desc);
  ASSERT_EQ(parent->GetElementDescriptionCount(), 1UL);

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  parent->AddValue("string", "foo", false, "foo description");

  sdf::ElementPtr newelem = parent->Clone();

  ASSERT_NE(newelem->GetFirstElement(), nullptr);
  ASSERT_EQ(newelem->GetElementDescriptionCount(), 1UL);
  ASSERT_EQ(newelem->GetAttributeCount(), 1UL);
}

/////////////////////////////////////////////////
TEST(Element, ClearElements)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr child = std::make_shared<sdf::Element>();

  parent->InsertElement(child);

  ASSERT_NE(parent->GetFirstElement(), nullptr);

  parent->ClearElements();

  ASSERT_EQ(parent->GetFirstElement(), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, ToStringElements)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr child = std::make_shared<sdf::Element>();

  parent->InsertElement(child);
  ASSERT_NE(parent->GetFirstElement(), nullptr);

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  std::string stringval = parent->ToString("myprefix");
  ASSERT_EQ(stringval, "myprefix< test='foo'>\nmyprefix  </>\nmyprefix</>\n");
}

/////////////////////////////////////////////////
TEST(Element, ToStringValue)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  parent->AddValue("string", "val", false, "val description");

  std::string stringval = parent->ToString("myprefix");
  ASSERT_EQ(stringval, "myprefix< test='foo'>val</>\n");
}

/////////////////////////////////////////////////
TEST(Element, ToStringInclude)
{
  sdf::Element elem;

  ASSERT_EQ(elem.GetInclude(), "");

  elem.SetInclude("foo.txt");

  ASSERT_EQ(elem.GetInclude(), "foo.txt");

  std::string stringval = elem.ToString("myprefix");
  ASSERT_EQ(stringval, "myprefix<include filename='foo.txt'/>\n");
}

/////////////////////////////////////////////////
TEST(Element, DocLeftPane)
{
  sdf::Element elem;

  elem.SetDescription("Element description");
  elem.AddElementDescription(std::make_shared<sdf::Element>());

  std::string html;
  int index = 1;
  elem.PrintDocLeftPane(html, 0, index);
  ASSERT_EQ(html,
            "<a id='1' onclick='highlight(1);' href=\"#1\">&lt&gt</a>"
            "<div style='padding-left:0px;'>\n"
            "<a id='2' onclick='highlight(2);' href=\"#2\">&lt&gt</a>"
            "<div style='padding-left:4px;'>\n</div>\n</div>\n");
}

/////////////////////////////////////////////////
TEST(Element, DocRightPane)
{
  sdf::Element elem;

  elem.SetDescription("Element description");
  elem.AddAttribute("test", "string", "foo", false, "foo description");
  elem.AddValue("string", "val", false, "val description");
  elem.AddElementDescription(std::make_shared<sdf::Element>());

  std::string html;
  int index = 1;
  elem.PrintDocRightPane(html, 0, index);
  ASSERT_EQ(html,
            "<a name=\"1\">&lt&gt</a><div style='padding-left:0px;'>\n"
            "<div style='background-color: #ffffff'>\n"
            "<font style='font-weight:bold'>Description: </font>"
            "Element description<br>\n"
            "<font style='font-weight:bold'>Required: </font>"
            "&nbsp;&nbsp;&nbsp;\n<font style='font-weight:bold'>Type: </font>"
            "string&nbsp;&nbsp;&nbsp;\n"
            "<font style='font-weight:bold'>Default: </font>val\n"
            "</div>"
            "<div style='background-color: #dedede; padding-left:10px; "
            "display:inline-block;'>\n"
            "<font style='font-weight:bold'>Attributes</font><br>"
            "<div style='display: inline-block;padding-bottom: 4px;'>\n"
            "<div style='float:left; width: 80px;'>\n"
            "<font style='font-style: italic;'>test</font>: "
            "</div>\n"
            "<div style='float:left; padding-left: 4px; width: 300px;'>\n"
            "foo description<br>\n<font style='font-weight:bold'>Type: </font>"
            "string&nbsp;&nbsp;&nbsp;"
            "<font style='font-weight:bold'>Default: </font>"
            "foo<br>"
            "</div>\n"
            "</div>\n"
            "</div>\n"
            "<br>\n<a name=\"2\">&lt&gt</a>"
            "<div style='padding-left:4px;'>\n"
            "<div style='background-color: #ffffff'>\n"
            "<font style='font-weight:bold'>Description: </font>"
            "none<br>\n<font style='font-weight:bold'>Required: </font>"
            "&nbsp;&nbsp;&nbsp;\n"
            "<font style='font-weight:bold'>Type: </font>n/a\n"
            "</div>"
            "</div>\n"
            "</div>\n");
}

/////////////////////////////////////////////////
TEST(Element, SetEmpty)
{
  sdf::Element elem;

  ASSERT_FALSE(elem.Set<std::string>(""));
}

/////////////////////////////////////////////////
TEST(Element, Set)
{
  sdf::Element elem;

  elem.AddValue("string", "val", false, "val description");

  ASSERT_TRUE(elem.Set<std::string>("hello"));
}

/////////////////////////////////////////////////
TEST(Element, Copy)
{
  sdf::ElementPtr src = std::make_shared<sdf::Element>();
  sdf::ElementPtr dest = std::make_shared<sdf::Element>();

  src->SetName("test");
  src->AddValue("string", "val", false, "val description");
  src->AddAttribute("test", "string", "foo", false, "foo description");
  src->InsertElement(std::make_shared<sdf::Element>());

  dest->Copy(src);

  sdf::ParamPtr param = dest->GetValue();
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "val");
  ASSERT_EQ(param->GetDescription(), "val description");

  ASSERT_EQ(dest->GetAttributeCount(), 1UL);
  param = dest->GetAttribute("test");
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");

  ASSERT_NE(dest->GetFirstElement(), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, CopyDestValue)
{
  sdf::ElementPtr src = std::make_shared<sdf::Element>();
  sdf::ElementPtr dest = std::make_shared<sdf::Element>();

  src->AddValue("string", "val", false, "val description");
  src->AddAttribute("test", "string", "foo", false, "foo description");
  src->InsertElement(std::make_shared<sdf::Element>());

  dest->AddValue("string", "val", false, "val description");
  dest->Copy(src);

  sdf::ParamPtr param = dest->GetValue();
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "val");
  ASSERT_EQ(param->GetDescription(), "val description");

  ASSERT_EQ(dest->GetAttributeCount(), 1UL);
  param = dest->GetAttribute("test");
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");

  ASSERT_NE(dest->GetFirstElement(), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, GetNextElement)
{
  sdf::ElementPtr child = std::make_shared<sdf::Element>();
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();

  child->SetParent(parent);

  ASSERT_EQ(child->GetNextElement("foo"), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, GetNextElementMultiple)
{
  sdf::ElementPtr child1 = std::make_shared<sdf::Element>();
  sdf::ElementPtr child2 = std::make_shared<sdf::Element>();
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();

  child1->SetParent(parent);
  child2->SetParent(parent);

  parent->InsertElement(child1);
  parent->InsertElement(child2);

  ASSERT_NE(child1->GetNextElement(""), nullptr);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
