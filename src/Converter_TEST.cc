/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "sdf/Converter.hh"

////////////////////////////////////////////////////
/// Set up an xml string for testing
std::string getXmlString()
{
  std::stringstream stream;
  stream << "<elemA attrA='A'>"
         << "  <elemB attrB='B'>"
         << "    <elemC attrC='C'>"
         << "      <elemD>D</elemD>"
         << "    </elemC>"
         << "  </elemB>"
         << "</elemA>";
  return stream.str();
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from elem to elem
TEST(Converter, MoveElemElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test moving from elem to elem
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from element='elemC::elemD'/>"
                << "     <to element='elemE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemC"));
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "D");
  convertedElem =  convertedElem->FirstChildElement("elemC");
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from elem to attr
TEST(Converter, MoveElemAttr)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from elem to attr
  TiXmlDocument xmlDoc2;
  xmlDoc2.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from element='elemC::elemD'/>"
                << "     <to attribute='attrE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc2;
  convertXmlDoc2.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc2, &convertXmlDoc2);

  TiXmlElement *convertedElem =  xmlDoc2.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "D");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from attr to attr
TEST(Converter, MoveAttrAttr)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to attr
  TiXmlDocument xmlDoc3;
  xmlDoc3.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from attribute='elemC::attrC'/>"
                << "     <to attribute='attrE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  TiXmlElement *convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from attr to elem
TEST(Converter, MoveAttrElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to elem
  TiXmlDocument xmlDoc4;
  xmlDoc4.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from attribute='elemC::attrC'/>"
                << "     <to element='elemE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc4;
  convertXmlDoc4.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc4, &convertXmlDoc4);

  TiXmlElement *convertedElem =  xmlDoc4.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "C");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemC"));
  convertedElem =  convertedElem->FirstChildElement("elemC");
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from elem to elem across multiple levels
TEST(Converter, MoveElemElemMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from elem to elem across multiple levels
  TiXmlDocument xmlDoc5;
  xmlDoc5.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <move>"
                << "   <from element='elemB::elemC::elemD'/>"
                << "   <to element='elemE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc5;
  convertXmlDoc5.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc5, &convertXmlDoc5);

  TiXmlElement *convertedElem =  xmlDoc5.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "D");
  convertedElem =  convertedElem->FirstChildElement("elemB");
  ASSERT_TRUE(convertedElem != NULL);
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from attr to attr across multiple levels
TEST(Converter, MoveAttrAttrMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to attr across multiple levels
  TiXmlDocument xmlDoc6;
  xmlDoc6.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <move>"
                << "   <from attribute='elemB::elemC::attrC'/>"
                << "   <to attribute='attrE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc6;
  convertXmlDoc6.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc6, &convertXmlDoc6);

  TiXmlElement *convertedElem =  xmlDoc6.FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_TRUE(convertedElem != NULL);
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from elem to attr across multiple levels
TEST(Converter, MoveElemAttrMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from elem to attr across multiple levels
  TiXmlDocument xmlDoc7;
  xmlDoc7.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <move>"
                << "   <from element='elemB::elemC::elemD'/>"
                << "   <to attribute='attrE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc7;
  convertXmlDoc7.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc7, &convertXmlDoc7);

  TiXmlElement *convertedElem =  xmlDoc7.FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "D");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_TRUE(convertedElem != NULL);
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test moving from attr to elem across multiple levels
TEST(Converter, MoveAttrElemMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to elem across multiple levels
  TiXmlDocument xmlDoc8;
  xmlDoc8.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <move>"
                << "   <from attribute='elemB::elemC::attrC'/>"
                << "   <to element='elemE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc8;
  convertXmlDoc8.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc8, &convertXmlDoc8);

  TiXmlElement *convertedElem =  xmlDoc8.FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "C");
  convertedElem =  convertedElem->FirstChildElement("elemB");
  ASSERT_TRUE(convertedElem != NULL);
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Add function is working
/// Test adding element and attribute
TEST(Converter, Add)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test adding element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <add element='elemBB' value='BB'/>"
                << "    <convert name='elemC'>"
                << "      <convert name='elemD'>"
                << "        <add attribute='attrDD' value='DD'/>"
                << "      </convert>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_TRUE(convertedElem != NULL);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemC"));
  EXPECT_TRUE(convertedElem->FirstChildElement("elemBB"));
  std::string elemValue = convertedElem->FirstChildElement("elemBB")->GetText();
  EXPECT_EQ(elemValue, "BB");
  convertedElem = convertedElem->FirstChildElement("elemC");
  ASSERT_TRUE(convertedElem != NULL);
  convertedElem = convertedElem->FirstChildElement("elemD");
  ASSERT_TRUE(convertedElem != NULL);
  std::string attrValue = convertedElem->Attribute("attrDD");
  EXPECT_EQ(attrValue, "DD");
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
