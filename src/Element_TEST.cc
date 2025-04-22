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
#include "sdf/Filesystem.hh"
#include "sdf/Param.hh"
#include "test_utils.hh"

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
  parent->SetFilePath("/parent/path/model.sdf");
  parent->SetOriginalVersion("1.5");

  ASSERT_EQ(child.GetParent(), nullptr);

  child.SetParent(parent);

  ASSERT_NE(child.GetParent(), nullptr);
  EXPECT_EQ("/parent/path/model.sdf", child.FilePath());
  EXPECT_EQ("1.5", child.OriginalVersion());
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
TEST(Element, SetExplicitlySetInFile)
{
  // The hierarchy in xml:
  // <parent>
  //   <elem>
  //     <child>
  //       <child2>
  //         <grandChild/>
  //       <child2>
  //     </child>
  //     <sibling/>
  //     <sibling2/>
  //   </elem>
  //   <elem2/>
  // </parent>

  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  parent->InsertElement(elem, true);
  sdf::ElementPtr elem2 = std::make_shared<sdf::Element>();
  parent->InsertElement(elem2, true);

  EXPECT_TRUE(elem->GetExplicitlySetInFile());

  elem->SetExplicitlySetInFile(false);

  EXPECT_FALSE(elem->GetExplicitlySetInFile());

  elem->SetExplicitlySetInFile(true);

  EXPECT_TRUE(elem->GetExplicitlySetInFile());

  // the children and siblings of the element should all be
  // set to the same value when using this function
  sdf::ElementPtr child = std::make_shared<sdf::Element>();
  child->SetParent(elem);
  elem->InsertElement(child, false);

  sdf::ElementPtr sibling = std::make_shared<sdf::Element>();
  sibling->SetParent(elem);
  elem->InsertElement(sibling);

  sdf::ElementPtr sibling2 = std::make_shared<sdf::Element>();
  sibling2->SetParent(elem);
  elem->InsertElement(sibling2);

  sdf::ElementPtr child2 = std::make_shared<sdf::Element>();
  child2->SetParent(child);
  child->InsertElement(child2);

  sdf::ElementPtr grandChild = std::make_shared<sdf::Element>();
  grandChild->SetParent(child);
  child->InsertElement(grandChild);

  EXPECT_TRUE(elem->GetExplicitlySetInFile());
  EXPECT_TRUE(child->GetExplicitlySetInFile());
  EXPECT_TRUE(sibling->GetExplicitlySetInFile());
  EXPECT_TRUE(sibling2->GetExplicitlySetInFile());
  EXPECT_TRUE(child2->GetExplicitlySetInFile());
  EXPECT_TRUE(grandChild->GetExplicitlySetInFile());
  EXPECT_TRUE(elem2->GetExplicitlySetInFile());

  elem->SetExplicitlySetInFile(false);
  EXPECT_FALSE(elem->GetExplicitlySetInFile());
  EXPECT_FALSE(child->GetExplicitlySetInFile());
  EXPECT_FALSE(sibling->GetExplicitlySetInFile());
  EXPECT_FALSE(sibling2->GetExplicitlySetInFile());
  EXPECT_FALSE(child2->GetExplicitlySetInFile());
  EXPECT_FALSE(grandChild->GetExplicitlySetInFile());

  // SetExplicitlySetInFile(false) is be called only on `elem`. We expect
  // GetExplicitlySetInFile() to be false for all children and grandchildren of
  // `elem`, but true for `elem2`, which is a sibling of `elem`.
  EXPECT_TRUE(elem2->GetExplicitlySetInFile());
}

/////////////////////////////////////////////////
TEST(Element, SetExplicitlySetInFileWithInsert)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  parent->SetExplicitlySetInFile(false);
  sdf::ElementPtr child = std::make_shared<sdf::Element>();
  child->SetParent(parent);
  parent->InsertElement(child);

  EXPECT_FALSE(parent->GetExplicitlySetInFile());
  EXPECT_TRUE(child->GetExplicitlySetInFile());
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
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();

  elem->SetName("test");
  elem->AddValue("string", "foo", false, "foo description");

  sdf::ParamPtr param = elem->GetValue();
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");
  ASSERT_NE(param->GetParentElement(), nullptr);
  EXPECT_EQ(param->GetParentElement(), elem);
}

/////////////////////////////////////////////////
TEST(Element, AddAttribute)
{
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();

  ASSERT_EQ(elem->GetAttributeCount(), 0UL);

  elem->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(elem->GetAttributeCount(), 1UL);

  elem->AddAttribute("attr", "float", "0.0", false, "float description");
  ASSERT_EQ(elem->GetAttributeCount(), 2UL);

  sdf::ParamPtr param = elem->GetAttribute("test");
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");

  param = elem->GetAttribute("attr");
  ASSERT_TRUE(param->IsType<float>());
  ASSERT_EQ(param->GetKey(), "attr");
  ASSERT_EQ(param->GetTypeName(), "float");
  ASSERT_EQ(param->GetDefaultAsString(), "0");
  ASSERT_EQ(param->GetDescription(), "float description");
}

/////////////////////////////////////////////////
TEST(Element, GetAttributeSet)
{
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  ASSERT_EQ(elem->GetAttributeCount(), 0UL);
  elem->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(elem->GetAttributeCount(), 1UL);

  EXPECT_FALSE(elem->GetAttributeSet("test"));
  elem->GetAttribute("test")->Set("asdf");
  EXPECT_TRUE(elem->GetAttributeSet("test"));
}

/////////////////////////////////////////////////
TEST(Element, RemoveAttribute)
{
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  ASSERT_EQ(elem->GetAttributeCount(), 0UL);

  elem->AddAttribute("test", "string", "foo", false, "foo description");
  elem->AddAttribute("attr", "float", "0.0", false, "float description");
  ASSERT_EQ(elem->GetAttributeCount(), 2UL);

  elem->RemoveAttribute("test");
  EXPECT_EQ(elem->GetAttributeCount(), 1UL);
  EXPECT_EQ(elem->GetAttribute("test"), nullptr);
  EXPECT_NE(elem->GetAttribute("attr"), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, RemoveAllAttributes)
{
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  ASSERT_EQ(elem->GetAttributeCount(), 0UL);

  elem->AddAttribute("test", "string", "foo", false, "foo description");
  elem->AddAttribute("test2", "string", "foo", false, "foo description");
  elem->AddAttribute("attr", "float", "0.0", false, "float description");
  ASSERT_EQ(elem->GetAttributeCount(), 3UL);

  elem->RemoveAllAttributes();
  EXPECT_EQ(elem->GetAttributeCount(), 0UL);
  EXPECT_EQ(elem->GetAttribute("test"), nullptr);
  EXPECT_EQ(elem->GetAttribute("test2"), nullptr);
  EXPECT_EQ(elem->GetAttribute("attr"), nullptr);
}

/////////////////////////////////////////////////
TEST(Element, Include)
{
  sdf::Element elem;

  auto includeElemToStore = std::make_shared<sdf::Element>();
  includeElemToStore->SetName("include");
  auto uriDesc = std::make_shared<sdf::Element>();
  uriDesc->SetName("uri");
  uriDesc->AddValue("string", "", true);
  includeElemToStore->AddElementDescription(uriDesc);

  includeElemToStore->GetElement("uri")->Set("foo.txt");
  elem.SetIncludeElement(includeElemToStore);

  auto includeElem = elem.GetIncludeElement();
  ASSERT_NE(nullptr, includeElem);
  ASSERT_TRUE(includeElem->HasElement("uri"));
  EXPECT_EQ("foo.txt", includeElem->GetElement("uri")->Get<std::string>());
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

  parent->SetName("parent");
  child->SetName("child");

  parent->InsertElement(child);
  ASSERT_NE(parent->GetFirstElement(), nullptr);

  parent->AddElementDescription(desc);
  ASSERT_EQ(parent->GetElementDescriptionCount(), 1UL);

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  parent->AddValue("string", "foo", false, "foo description");

  parent->SetFilePath("/path/to/file.sdf");
  parent->SetLineNumber(12);
  parent->SetXmlPath("/sdf/world[@name=\"default\"]");
  parent->SetOriginalVersion("1.5");

  auto includeElemToStore = std::make_shared<sdf::Element>();
  includeElemToStore->SetName("include");
  parent->SetIncludeElement(includeElemToStore);

  sdf::ElementPtr newelem = parent->Clone();

  EXPECT_EQ("/path/to/file.sdf", newelem->FilePath());
  ASSERT_TRUE(newelem->LineNumber().has_value());
  EXPECT_EQ(12, newelem->LineNumber().value());
  EXPECT_EQ("/sdf/world[@name=\"default\"]", newelem->XmlPath());
  EXPECT_EQ("1.5", newelem->OriginalVersion());
  ASSERT_NE(newelem->GetFirstElement(), nullptr);
  ASSERT_EQ(newelem->GetElementDescriptionCount(), 1UL);
  ASSERT_EQ(newelem->GetAttributeCount(), 1UL);
  ASSERT_NE(newelem->GetIncludeElement(), nullptr);
  EXPECT_EQ("include", newelem->GetIncludeElement()->GetName());
  ASSERT_TRUE(newelem->GetExplicitlySetInFile());

  ASSERT_NE(nullptr, parent->GetValue()->GetParentElement());
  EXPECT_EQ(parent, parent->GetValue()->GetParentElement());

  ASSERT_NE(nullptr, newelem->GetValue()->GetParentElement());
  EXPECT_EQ(newelem, newelem->GetValue()->GetParentElement());

  auto clonedAttribs = newelem->GetAttributes();
  EXPECT_EQ(newelem, clonedAttribs[0]->GetParentElement());
}

/////////////////////////////////////////////////
TEST(Element, ClearElements)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr child = std::make_shared<sdf::Element>();

  parent->SetFilePath("/path/to/file.sdf");
  parent->SetLineNumber(12);
  parent->SetXmlPath("/sdf/world[@name=\"default\"]");
  parent->SetOriginalVersion("1.5");
  child->SetParent(parent);
  parent->InsertElement(child);

  EXPECT_EQ("/path/to/file.sdf", parent->FilePath());
  ASSERT_TRUE(parent->LineNumber().has_value());
  EXPECT_EQ(12, parent->LineNumber().value());
  EXPECT_EQ("/sdf/world[@name=\"default\"]", parent->XmlPath());
  EXPECT_EQ("1.5", parent->OriginalVersion());
  ASSERT_NE(parent->GetFirstElement(), nullptr);
  EXPECT_EQ("/path/to/file.sdf", parent->GetFirstElement()->FilePath());
  EXPECT_EQ("1.5", parent->GetFirstElement()->OriginalVersion());

  parent->ClearElements();

  ASSERT_EQ(parent->GetFirstElement(), nullptr);
  EXPECT_EQ("/path/to/file.sdf", parent->FilePath());
  ASSERT_TRUE(parent->LineNumber().has_value());
  EXPECT_EQ(12, parent->LineNumber().value());
  EXPECT_EQ("1.5", parent->OriginalVersion());
}

/////////////////////////////////////////////////
TEST(Element, Clear)
{
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr child = std::make_shared<sdf::Element>();

  parent->SetFilePath("/path/to/file.sdf");
  parent->SetLineNumber(12);
  parent->SetXmlPath("/sdf/world[@name=\"default\"]");
  parent->SetOriginalVersion("1.5");
  child->SetParent(parent);
  parent->InsertElement(child);

  EXPECT_EQ("/path/to/file.sdf", parent->FilePath());
  ASSERT_TRUE(parent->LineNumber().has_value());
  EXPECT_EQ(12, parent->LineNumber().value());
  EXPECT_EQ("/sdf/world[@name=\"default\"]", parent->XmlPath());
  EXPECT_EQ("1.5", parent->OriginalVersion());
  ASSERT_NE(parent->GetFirstElement(), nullptr);
  EXPECT_EQ("/path/to/file.sdf", parent->GetFirstElement()->FilePath());
  EXPECT_EQ("1.5", parent->GetFirstElement()->OriginalVersion());

  parent->Clear();

  ASSERT_EQ(parent->GetFirstElement(), nullptr);
  EXPECT_TRUE(parent->FilePath().empty());
  EXPECT_FALSE(parent->LineNumber().has_value());
  EXPECT_TRUE(parent->XmlPath().empty());
  EXPECT_TRUE(parent->OriginalVersion().empty());
}

/////////////////////////////////////////////////
TEST(Element, ToStringElements)
{
  sdf::Errors errors;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  sdf::ElementPtr child = std::make_shared<sdf::Element>();

  parent->SetName("parent");
  child->SetName("child");

  parent->InsertElement(child);
  ASSERT_NE(parent->GetFirstElement(), nullptr);

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  // attribute won't print unless it has been set
  EXPECT_FALSE(parent->GetAttributeSet("test"));
  EXPECT_EQ(parent->ToString(errors, "<!-- prefix -->"),
    "<!-- prefix --><parent>\n"
    "<!-- prefix -->  <child/>\n"
    "<!-- prefix --></parent>\n");
  EXPECT_TRUE(errors.empty());

  sdf::ParamPtr test = parent->GetAttribute("test");
  ASSERT_NE(nullptr, test);
  EXPECT_FALSE(test->GetSet());
  EXPECT_TRUE(test->SetFromString("foo"));
  EXPECT_TRUE(test->GetSet());
  EXPECT_TRUE(parent->GetAttributeSet("test"));

  EXPECT_EQ(parent->ToString(errors, "<!-- prefix -->"),
    "<!-- prefix --><parent test='foo'>\n"
    "<!-- prefix -->  <child/>\n"
    "<!-- prefix --></parent>\n");
  EXPECT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
TEST(Element, ToStringRequiredAttributes)
{
  sdf::Errors errors;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  parent->AddAttribute("test", "string", "foo", true, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  // A required attribute should print even if it has not been set
  EXPECT_FALSE(parent->GetAttributeSet("test"));
  EXPECT_EQ(parent->ToString(errors, "myprefix"), "myprefix< test='foo'/>\n");
  EXPECT_TRUE(errors.empty());

  sdf::ParamPtr test = parent->GetAttribute("test");
  ASSERT_NE(nullptr, test);
  EXPECT_FALSE(test->GetSet());
  EXPECT_TRUE(test->SetFromString("bar"));
  EXPECT_TRUE(test->GetSet());
  EXPECT_TRUE(parent->GetAttributeSet("test"));

  EXPECT_EQ(parent->ToString(errors, "myprefix"), "myprefix< test='bar'/>\n");
  EXPECT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
TEST(Element, ToStringValue)
{
  sdf::Errors errors;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  parent->AddValue("string", "val", false, "val description");

  EXPECT_FALSE(parent->GetAttributeSet("test"));
  EXPECT_EQ(parent->ToString(errors, "myprefix"),
            "myprefix<>val</>\n");
  EXPECT_TRUE(errors.empty());

  sdf::ParamPtr test = parent->GetAttribute("test");
  ASSERT_NE(nullptr, test);
  EXPECT_FALSE(test->GetSet());
  EXPECT_TRUE(test->SetFromString("foo"));
  EXPECT_TRUE(test->GetSet());
  EXPECT_TRUE(parent->GetAttributeSet("test"));
  EXPECT_EQ(parent->ToString(errors, "myprefix"),
            "myprefix< test='foo'>val</>\n");
  EXPECT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
TEST(Element, ToStringClonedElement)
{
  sdf::Errors errors;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();

  parent->AddAttribute("test", "string", "foo", false, "foo description");
  ASSERT_EQ(parent->GetAttributeCount(), 1UL);

  sdf::ParamPtr test = parent->GetAttribute("test");
  ASSERT_NE(nullptr, test);
  EXPECT_FALSE(test->GetSet());
  EXPECT_TRUE(test->SetFromString("foo"));
  EXPECT_TRUE(test->GetSet());

  sdf::ElementPtr parentClone = parent->Clone();
  EXPECT_TRUE(parentClone->GetAttributeSet("test"));
  EXPECT_EQ(parent->ToString(errors, "myprefix"),
            parentClone->ToString(errors, "myprefix"));
  EXPECT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
TEST(Element, ToStringDefaultElements)
{
  sdf::Errors errors;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  parent->SetName("parent");
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  elem->SetName("elem");
  elem->SetParent(parent);
  parent->InsertElement(elem);
  sdf::ElementPtr elem2 = std::make_shared<sdf::Element>();
  elem2->SetName("elem2");
  elem2->SetParent(parent);
  parent->InsertElement(elem2);

  std::ostringstream stream;
  stream
    << "<parent>\n"
    << "  <elem/>\n"
    << "  <elem2/>\n"
    << "</parent>\n";

  EXPECT_EQ(parent->ToString(errors, "", false, false), stream.str());
  EXPECT_EQ(parent->ToString(errors, ""), stream.str());
  EXPECT_EQ(parent->ToString(errors, "", true, false), stream.str());

  elem->SetExplicitlySetInFile(false);

  std::ostringstream stream2;
  stream2
    << "<parent>\n"
    << "  <elem2/>\n"
    << "</parent>\n";

  EXPECT_EQ(parent->ToString(errors, "", false, false), stream2.str());
  EXPECT_EQ(parent->ToString(errors, ""), stream.str());
  EXPECT_EQ(parent->ToString(errors, "", true, false), stream.str());

  parent->SetExplicitlySetInFile(false);

  EXPECT_EQ(parent->ToString(errors, "", false, false), "");
  EXPECT_EQ(parent->ToString(errors, ""), stream.str());
  EXPECT_EQ(parent->ToString(errors, "", true, false), stream.str());
  EXPECT_TRUE(errors.empty());
}

/////////////////////////////////////////////////
TEST(Element, ToStringDefaultAttributes)
{
  sdf::Errors errors;
  sdf::ElementPtr element = std::make_shared<sdf::Element>();
  element->SetName("foo");
  element->AddAttribute("test", "string", "foo", false, "foo description");
  element->AddAttribute("test2", "string", "bar", true, "bar description");

  EXPECT_EQ(element->ToString(errors, ""),
            element->ToString(errors, "", true, false));
  EXPECT_EQ(element->ToString(errors, ""),
            element->ToString(errors, "", false, false));

  std::ostringstream stream;
  stream << "<foo test2='bar'/>\n";

  EXPECT_EQ(element->ToString(errors, ""), stream.str());
  EXPECT_EQ(element->ToString(errors, "", true, false), stream.str());
  EXPECT_EQ(element->ToString(errors, "", false, false), stream.str());

  std::ostringstream stream2;
  stream2 << "<foo test='foo' test2='bar'/>\n";

  EXPECT_EQ(element->ToString(errors, "", true, true), stream2.str());
  EXPECT_EQ(element->ToString(errors, "", false, true), stream2.str());
  EXPECT_TRUE(errors.empty());
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
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();

  elem->SetDescription("Element description");
  elem->AddAttribute("test", "string", "foo", false, "foo description");
  elem->AddValue("string", "val", false, "val description");
  elem->AddElementDescription(std::make_shared<sdf::Element>());

  std::string html;
  int index = 1;
  sdf::Errors errors;
  elem->PrintDocRightPane(html, 0, index);
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

  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  sdf::Errors errors;

  ASSERT_FALSE(elem->Set<std::string>(errors, ""));
  ASSERT_EQ(errors.size(), 0u);

  elem->AddValue("int", "0", true, errors, "value");
  ASSERT_EQ(errors.size(), 0u);
  ASSERT_FALSE(elem->Set<std::string>(errors, ""));
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Empty string used when setting a required parameter. Key[]"));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(Element, Set)
{
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  sdf::Errors errors;

  elem->AddValue("string", "val", false, "val description");

  ASSERT_TRUE(elem->Set<std::string>(errors, "hello"));
  ASSERT_EQ(errors.size(), 0u);
}

/////////////////////////////////////////////////
TEST(Element, Copy)
{
  sdf::ElementPtr src = std::make_shared<sdf::Element>();
  sdf::ElementPtr dest = std::make_shared<sdf::Element>();

  src->SetName("test");
  src->SetFilePath("/path/to/file.sdf");
  src->SetLineNumber(12);
  src->SetXmlPath("/sdf/world[@name=\"default\"]");
  src->SetOriginalVersion("1.5");
  src->AddValue("string", "val", false, "val description");
  src->AddAttribute("test", "string", "foo", false, "foo description");
  src->InsertElement(std::make_shared<sdf::Element>());

  auto includeElemToStore = std::make_shared<sdf::Element>();
  includeElemToStore->SetName("include");
  src->SetIncludeElement(includeElemToStore);

  dest->Copy(src);

  EXPECT_EQ("/path/to/file.sdf", dest->FilePath());
  ASSERT_TRUE(dest->LineNumber().has_value());
  EXPECT_EQ(12, dest->LineNumber().value());
  EXPECT_EQ("/sdf/world[@name=\"default\"]", dest->XmlPath());
  EXPECT_EQ("1.5", dest->OriginalVersion());

  sdf::ParamPtr param = dest->GetValue();
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "val");
  ASSERT_EQ(param->GetDescription(), "val description");
  ASSERT_NE(param->GetParentElement(), nullptr);
  EXPECT_EQ(param->GetParentElement(), dest);

  ASSERT_EQ(dest->GetAttributeCount(), 1UL);
  ASSERT_TRUE(dest->GetExplicitlySetInFile());
  param = dest->GetAttribute("test");
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");
  ASSERT_NE(param->GetParentElement(), nullptr);
  EXPECT_EQ(param->GetParentElement(), dest);

  ASSERT_NE(dest->GetFirstElement(), nullptr);
  ASSERT_NE(dest->GetIncludeElement(), nullptr);
  EXPECT_EQ("include", dest->GetIncludeElement()->GetName());
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
  ASSERT_NE(param->GetParentElement(), nullptr);
  EXPECT_EQ(param->GetParentElement(), dest);

  ASSERT_EQ(dest->GetAttributeCount(), 1UL);
  param = dest->GetAttribute("test");
  ASSERT_TRUE(param->IsType<std::string>());
  ASSERT_EQ(param->GetKey(), "test");
  ASSERT_EQ(param->GetTypeName(), "string");
  ASSERT_EQ(param->GetDefaultAsString(), "foo");
  ASSERT_EQ(param->GetDescription(), "foo description");
  ASSERT_NE(param->GetParentElement(), nullptr);
  EXPECT_EQ(param->GetParentElement(), dest);

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
  ASSERT_EQ(child2->GetNextElement(""), nullptr);
}

/////////////////////////////////////////////////
/// Helper function to add child elements without having to create descriptions
sdf::ElementPtr addChildElement(sdf::ElementPtr _parent,
                                const std::string &_elementName,
                                const bool _addNameAttribute,
                                const std::string &_childName)
{
  sdf::ElementPtr child = std::make_shared<sdf::Element>();
  child->SetParent(_parent);
  child->SetName(_elementName);
  _parent->InsertElement(child);

  if (_addNameAttribute)
  {
    child->AddAttribute("name", "string", _childName, false, "description");
  }
  return child;
}

/////////////////////////////////////////////////
TEST(Element, CountNamedElements)
{
  sdf::Errors errors;
  sdf::ElementPtr parent = std::make_shared<sdf::Element>();
  // expect empty map element with no children
  EXPECT_TRUE(parent->CountNamedElements(errors).empty());
  EXPECT_TRUE(parent->CountNamedElements(errors, "child").empty());
  // expect empty set of element type names
  EXPECT_TRUE(parent->GetElementTypeNames().empty());
  // since there are no child names, they must be unique
  EXPECT_TRUE(parent->HasUniqueChildNames(errors));
  EXPECT_TRUE(parent->HasUniqueChildNames(errors, "child"));


  // The following calls should make the following child elements:
  // <child name="child1"/>
  // <child name="child2"/>
  // <element name="child2"/>
  // <element name="child3"/>
  // <element />
  addChildElement(parent, "child", true, "child1");
  addChildElement(parent, "child", true, "child2");
  addChildElement(parent, "element", true, "child2");
  addChildElement(parent, "element", true, "child3");
  addChildElement(parent, "element", false, "unset");

  // test GetElementTypeNames
  auto typeNames = parent->GetElementTypeNames();
  EXPECT_EQ(typeNames.size(), 2u);
  EXPECT_EQ(typeNames.count("child"), 1u);
  EXPECT_EQ(typeNames.count("element"), 1u);

  // test HasUniqueChildNames
  EXPECT_TRUE(parent->HasUniqueChildNames(errors, "empty"));
  EXPECT_TRUE(parent->HasUniqueChildNames(errors, "child"));
  EXPECT_TRUE(parent->HasUniqueChildNames(errors, "element"));
  // The following have matching names that are detected when passing
  // default "" to HasUniqueChildNames().
  // <child name="child2"/>
  // <element name="child2"/>
  EXPECT_FALSE(parent->HasUniqueChildNames(errors));
  EXPECT_FALSE(parent->HasUniqueChildNames(errors, ""));

  EXPECT_TRUE(parent->CountNamedElements(errors, "empty").empty());

  auto childMap = parent->CountNamedElements(errors, "child");
  EXPECT_FALSE(childMap.empty());
  EXPECT_EQ(childMap.size(), 2u);
  EXPECT_EQ(childMap.count("child1"), 1u);
  EXPECT_EQ(childMap.count("child2"), 1u);
  EXPECT_EQ(childMap.at("child1"), 1u);
  EXPECT_EQ(childMap.at("child2"), 1u);

  auto elementMap = parent->CountNamedElements(errors, "element");
  EXPECT_FALSE(elementMap.empty());
  EXPECT_EQ(elementMap.size(), 2u);
  EXPECT_EQ(elementMap.count("child2"), 1u);
  EXPECT_EQ(elementMap.count("child3"), 1u);
  EXPECT_EQ(elementMap.at("child2"), 1u);
  EXPECT_EQ(elementMap.at("child3"), 1u);

  auto allMap = parent->CountNamedElements(errors, "");
  EXPECT_FALSE(allMap.empty());
  EXPECT_EQ(allMap.size(), 3u);
  EXPECT_EQ(allMap.count("child1"), 1u);
  EXPECT_EQ(allMap.count("child2"), 1u);
  EXPECT_EQ(allMap.count("child3"), 1u);
  EXPECT_EQ(allMap.count("unset"), 0u);
  EXPECT_EQ(allMap.at("child1"), 1u);
  EXPECT_EQ(allMap.at("child2"), 2u);
  EXPECT_EQ(allMap.at("child3"), 1u);
  EXPECT_TRUE(errors.empty());
}

TEST(Element, FindElement)
{
  // <root>
  //   <elem_A>
  //     <child_elem_A />
  //   </elem_A>
  //
  //   <elem_B>
  //     <child_elem_B name="first_child"/>
  //     <child_elem_B/>
  //   </elem_B>
  // </root>
  sdf::ElementPtr root = std::make_shared<sdf::Element>();
  root->SetName("root");

  // Create elements
  {
    auto elemA = addChildElement(root, "elem_A", false, "");
    addChildElement(elemA, "child_elem_A", false, "");

    auto elemB = addChildElement(root, "elem_B", false, "");
    auto firstChildElemB =
        addChildElement(elemB, "child_elem_B", true, "first_child");

    addChildElement(elemB, "child_elem_B", false, "");
  }

  {
    auto elemA = root->FindElement("elem_A");
    ASSERT_NE(nullptr, elemA);
    EXPECT_NE(nullptr, elemA->FindElement("child_elem_A"));
    EXPECT_EQ(nullptr, elemA->FindElement("non_existent_elem"));

    auto elemB = root->FindElement("elem_B");
    ASSERT_NE(nullptr, elemB);
    // This should find the first "child_elem_B" element, which has the name
    // attribute
    auto childElemB = elemB->FindElement("child_elem_B");
    ASSERT_TRUE(childElemB->HasAttribute("name"));
    EXPECT_EQ("first_child", childElemB->GetAttribute("name")->GetAsString());
  }

  // Check that it works with const pointers
  {
    sdf::ElementConstPtr rootConst = root;
    sdf::ElementConstPtr elemA = rootConst->FindElement("elem_A");
    ASSERT_NE(nullptr, elemA);
    EXPECT_NE(nullptr, elemA->FindElement("child_elem_A"));
    EXPECT_EQ(nullptr, elemA->FindElement("non_existent_elem"));

    sdf::ElementConstPtr elemB = root->FindElement("elem_B");
    ASSERT_NE(nullptr, elemB);
    // This should find the first "child_elem_B" element, which has the name
    // attribute
    sdf::ElementConstPtr childElemB = elemB->FindElement("child_elem_B");
    ASSERT_TRUE(childElemB->HasAttribute("name"));
    EXPECT_EQ("first_child", childElemB->GetAttribute("name")->GetAsString());
  }
  {
    sdf::ElementConstPtr rootConst = root;
    sdf::ElementConstPtr elemA = rootConst->FindElement("elem_A");
    ASSERT_NE(nullptr, elemA);
    EXPECT_NE(nullptr, elemA->FindElement("child_elem_A"));
    EXPECT_EQ(nullptr, elemA->FindElement("non_existent_elem"));

    sdf::ElementConstPtr elemB = root->FindElement("elem_B");
    ASSERT_NE(nullptr, elemB);
    // This should find the first "child_elem_B" element, which has the name
    // attribute
    sdf::ElementConstPtr childElemB = elemB->FindElement("child_elem_B");
    ASSERT_TRUE(childElemB->HasAttribute("name"));
    EXPECT_EQ("first_child", childElemB->GetAttribute("name")->GetAsString());
  }
}
