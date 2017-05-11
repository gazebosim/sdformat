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
  fprintf(stderr, "type: %s\n", param->GetTypeName().c_str());
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
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
