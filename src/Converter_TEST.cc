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
#include <array>
#include <sstream>
#include "sdf/Exception.hh"
#include "sdf/Filesystem.hh"

#include "Converter.hh"
#include "test_config.h"

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
/// Set up an xml string with a repeated element for testing
std::string getRepeatedXmlString()
{
  std::stringstream stream;
  stream << "<elemA attrA='A'>"
         << "  <elemB attrB='B'>"
         << "    <elemC attrC='C'>"
         << "      <elemD>D</elemD>"
         << "      <elemD>D</elemD>"
         << "      <elemD>D</elemD>"
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
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test moving from elem to elem
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "      <from element='elemC::elemD'/>"
                << "      <to element='elemE'/>"
                << "    </move>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "D");
  convertedElem =  convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
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
                << "      <from element='elemC::elemD'/>"
                << "      <to attribute='attrE'/>"
                << "    </move>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc2;
  convertXmlDoc2.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc2, &convertXmlDoc2);

  TiXmlElement *convertedElem =  xmlDoc2.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "D");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
                << "      <from attribute='elemC::attrC'/>"
                << "      <to attribute='attrE'/>"
                << "    </move>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  TiXmlElement *convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
                << "      <from attribute='elemC::attrC'/>"
                << "      <to element='elemE'/>"
                << "    </move>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc4;
  convertXmlDoc4.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc4, &convertXmlDoc4);

  TiXmlElement *convertedElem =  xmlDoc4.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "C");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  convertedElem =  convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
                << "    <from element='elemB::elemC::elemD'/>"
                << "    <to element='elemE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc5;
  convertXmlDoc5.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc5, &convertXmlDoc5);

  TiXmlElement *convertedElem =  xmlDoc5.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "D");
  convertedElem =  convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
                << "    <from attribute='elemB::elemC::attrC'/>"
                << "    <to attribute='attrE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc6;
  convertXmlDoc6.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc6, &convertXmlDoc6);

  TiXmlElement *convertedElem =  xmlDoc6.FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
                << "    <from element='elemB::elemC::elemD'/>"
                << "    <to attribute='attrE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc7;
  convertXmlDoc7.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc7, &convertXmlDoc7);

  TiXmlElement *convertedElem =  xmlDoc7.FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "D");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
                << "    <from attribute='elemB::elemC::attrC'/>"
                << "    <to element='elemE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc8;
  convertXmlDoc8.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc8, &convertXmlDoc8);

  TiXmlElement *convertedElem =  xmlDoc8.FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "C");
  convertedElem =  convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
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
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
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
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  ASSERT_NE(nullptr, convertedElem->FirstChildElement("elemBB"));
  std::string elemValue = convertedElem->FirstChildElement("elemBB")->GetText();
  EXPECT_EQ(elemValue, "BB");
  convertedElem = convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement("elemD");
  ASSERT_NE(nullptr, convertedElem);
  std::string attrValue = convertedElem->Attribute("attrDD");
  EXPECT_EQ(attrValue, "DD");
}

////////////////////////////////////////////////////
TEST(Converter, AddNoElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Test adding element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <add element='elemBB' value='BB'/>"
                << "    <convert name='elemC'>"
                << "      <convert name='elemD'>"
                << "        <add/>"
                << "      </convert>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Verify the xml
  TiXmlElement *childElem = xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
TEST(Converter, AddNoValue)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Test adding element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <add element='elemBB' value='BB'/>"
                << "    <convert name='elemC'>"
                << "      <convert name='elemD'>"
                << "        <add attribute='attrDD'/>"
                << "      </convert>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *childElem = xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Remove function is working
/// Test removing element
TEST(Converter, RemoveElement)
{
  // Set up an xml string for testing
  std::string xmlString = getRepeatedXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test removing element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <remove element='elemD'/>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  convertedElem = convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement("elemD");
  ASSERT_TRUE(convertedElem == nullptr);
}

////////////////////////////////////////////////////
/// Ensure that Converter::Remove function is working with descendant_name
/// Test removing element
TEST(Converter, RemoveDescendantElement)
{
  // Set up an xml string for testing
  std::string xmlString = getRepeatedXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test removing element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert descendant_name='elemC'>"
                << "    <remove element='elemD'/>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  convertedElem = convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement("elemD");
  ASSERT_TRUE(convertedElem == nullptr);
}

TEST(Converter, RemoveDescendantNestedElement)
{
  // Set up an xml string for testing
  std::string xmlString = R"(
  <sdf>
    <model name="M">
      <frame name="F1"/>
      <link name="L1"/>
      <link name="L2"/>
      <model name="NM1">
        <frame name="L1"/>
      </model>
      <model name="NM2">
        <frame name="L2"/>
      </model>
      <link name="L3"/>
    </model>
  </sdf>)";

  std::string convertString = R"(
    <convert name="sdf">
      <convert descendant_name="model">
        <remove element="frame"/>
      </convert>
    </convert>)";

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertString.c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  std::string expectedString = R"(
  <sdf>
    <model name="M">
      <link name="L1"/>
      <link name="L2"/>
      <model name="NM1"/>
      <model name="NM2"/>
      <link name="L3"/>
    </model>
  </sdf>)";
  TiXmlDocument expectedXmlDoc;
  expectedXmlDoc.Parse(expectedString.c_str());

  std::stringstream xmlDocOut;
  xmlDocOut << xmlDoc;

  std::stringstream expectedXmlDocOut;
  expectedXmlDocOut << expectedXmlDoc;
  EXPECT_EQ(xmlDocOut.str(), expectedXmlDocOut.str());
}
////////////////////////////////////////////////////
/// Ensure that Converter ignores descendants of <plugin> or namespaced elements
TEST(Converter, DescendantIgnorePluginOrNamespacedElements)
{
  // Set up an xml string for testing
  std::string xmlString = R"(
  <sdf>
      <elemA>
        <elemB />
      </elemA>
      <plugin name="P">
        <elemA>
          <elemB />
        </elemA>
      </plugin>
      <nsA:elemC>
        <elemA>
          <elemB />
        </elemA>
      </nsA:elemC>
  </sdf>)";

  std::string convertString = R"(
    <convert name="sdf">
      <convert descendant_name="elemA">
        <remove element="elemB"/>
      </convert>
    </convert>)";

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertString.c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Only the elemB directly under the first elemA should be removed
  std::string expectedString = R"(
  <sdf>
      <elemA/>
      <plugin name="P">
        <elemA>
          <elemB />
        </elemA>
      </plugin>
      <nsA:elemC>
        <elemA>
          <elemB />
        </elemA>
      </nsA:elemC>
  </sdf>)";
  TiXmlDocument expectedXmlDoc;
  expectedXmlDoc.Parse(expectedString.c_str());

  std::stringstream xmlDocOut;
  xmlDocOut << xmlDoc;

  std::stringstream expectedXmlDocOut;
  expectedXmlDocOut << expectedXmlDoc;
  EXPECT_EQ(xmlDocOut.str(), expectedXmlDocOut.str());
}

////////////////////////////////////////////////////
/// Ensure that Converter::Remove function is working
/// Test removing element and sub-elements
TEST(Converter, RemoveElementSubElement)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test adding element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <remove element='elemC'/>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  ASSERT_TRUE(convertedElem->FirstChildElement("elemC") == nullptr);
}

////////////////////////////////////////////////////
/// Ensure that Converter::Remove function is working
/// Test removing attribute
TEST(Converter, RemoveAttr)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test adding element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <remove attribute='attrC'/>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  convertedElem = convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_TRUE(convertedElem->Attribute("attrC") == nullptr);
  convertedElem = convertedElem->FirstChildElement("elemD");
  ASSERT_NE(nullptr, convertedElem);
}

////////////////////////////////////////////////////
TEST(Converter, RemoveNoElement)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Test adding element
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <remove/>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test an invalid move
TEST(Converter, MoveInvalid)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "      <from element='elemC::'/>"
                << "      <to element='elemE'/>"
                << "    </move>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // In this case, we had an invalid elemC:: in the conversion, which
  // means that the conversion quietly failed.  Make sure the new
  // document is the same as the original.
  // Verify the xml
  TiXmlElement *convertElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemA");
  convertElem = convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemB");
  convertElem = convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemC");
  convertElem = convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Move function is working
/// Test an invalid move
TEST(Converter, MoveInvalidPrefix)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "      <from element='::elemC'/>"
                << "      <to element='elemE'/>"
                << "    </move>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // In this case, we had an invalid ::elemC in the conversion, which
  // means that the conversion quietly failed.  Make sure the new
  // document is the same as the original.
  // Verify the xml
  TiXmlElement *convertElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemA");
  convertElem =  convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemB");
  convertElem =  convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemC");
  convertElem =  convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Copy function is working
/// Test moving from elem to elem
TEST(Converter, CopyElemElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test moving from elem to elem
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <copy>"
                << "      <from element='elemC::elemD'/>"
                << "      <to element='elemE'/>"
                << "    </copy>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  TiXmlElement *elemB = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, elemB);
  EXPECT_EQ(elemB->ValueStr(), "elemB");
  TiXmlElement *elemC = elemB->FirstChild("elemC")->ToElement();
  ASSERT_NE(nullptr, elemC);
  TiXmlElement *elemD = elemC->FirstChildElement();
  ASSERT_NE(nullptr, elemD);
  std::string elemValue = elemD->GetText();
  EXPECT_EQ(elemValue, "D");
  TiXmlElement *elemE = elemB->FirstChild("elemE")->ToElement();
  ASSERT_NE(nullptr, elemE);
  elemValue = elemE->GetText();
  EXPECT_EQ(elemValue, "D");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test an invalid map
TEST(Converter, MapInvalid)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Set up an invalid convert file that should do nothing
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                // missing from and to
                << "  <map/>"
                << "  <map>"
                << "    <from/>"
                << "  </map>"
                // missing/empty name attributes
                << "  <map>"
                << "    <from/>"
                << "    <to/>"
                << "  </map>"
                << "  <map>"
                << "    <from name=''/>"
                << "    <to/>"
                << "  </map>"
                << "  <map>"
                << "    <from name='attrA'/>"
                << "    <to name=''/>"
                << "  </map>"
                // missing value elements
                << "  <map>"
                << "    <from name='attrA'/>"
                << "    <to name='attrB'/>"
                << "  </map>"
                << "  <map>"
                << "    <from name='attrA'>"
                << "      <value/>"
                << "    </from>"
                << "    <to name='attrB'/>"
                << "  </map>"
                // empty first value elements
                << "  <map>"
                << "    <from name='attrA'>"
                << "      <value/>"
                << "    </from>"
                << "    <to name='attrB'>"
                << "      <value/>"
                << "    </to>"
                << "  </map>"
                << "  <map>"
                << "    <from name='attrA'>"
                << "      <value>A</value>"
                << "    </from>"
                << "    <to name='attrB'>"
                << "      <value/>"
                << "    </to>"
                << "  </map>"
                // empty subsequent value elements
                << "  <map>"
                << "    <from name='attrA'>"
                << "      <value>A</value>"
                << "      <value/>"
                << "    </from>"
                << "    <to name='attrB'>"
                << "      <value>A1</value>"
                << "    </to>"
                << "  </map>"
                << "  <map>"
                << "    <from name='attrA'>"
                << "      <value>A</value>"
                << "      <value>B</value>"
                << "    </from>"
                << "    <to name='attrB'>"
                << "      <value>A1</value>"
                << "      <value/>"
                << "    </to>"
                << "  </map>"
                << "  <convert name='elemB'>"
                // invalid elemC/
                << "    <map>"
                << "      <from name='elemC/'>"
                << "        <value>C</value>"
                << "      </from>"
                << "      <to name='elemE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                // invalid elemC/
                << "    <map>"
                << "      <from name='elemC/@'>"
                << "        <value>C</value>"
                << "      </from>"
                << "      <to name='elemE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                // invalid /elemC, errors without message
                << "    <map>"
                << "      <from name='/elemC'>"
                << "        <value>C</value>"
                << "      </from>"
                << "      <to name='elemE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                // invalid elemE/, errors without message
                << "    <map>"
                << "      <from name='elemC/elemD'>"
                << "        <value>D</value>"
                << "      </from>"
                << "      <to name='elemE/'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                // invalid elemE/, errors without message
                << "    <map>"
                << "      <from name='elemC/elemD'>"
                << "        <value>D</value>"
                << "      </from>"
                << "      <to name='elemE/@'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                // invalid /elemE, errors without message
                << "    <map>"
                << "      <from name='elemC/elemD'>"
                << "        <value>D</value>"
                << "      </from>"
                << "      <to name='/elemE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  std::ostringstream xmlDocBefore;
  xmlDocBefore << xmlDoc;
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Only invalid conversion statements.
  // Make sure the new document is the same as the original.
  std::ostringstream xmlDocAfter;
  xmlDocAfter << xmlDoc;
  EXPECT_EQ(xmlDocBefore.str(), xmlDocAfter.str());
  // Verify the xml
  TiXmlElement *convertElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemA");
  ASSERT_NE(nullptr, convertElem->Attribute("attrA"));
  std::string attrValue = convertElem->Attribute("attrA");
  EXPECT_EQ("A", attrValue);
  convertElem = convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemB");
  ASSERT_NE(nullptr, convertElem->Attribute("attrB"));
  attrValue = convertElem->Attribute("attrB");
  EXPECT_EQ("B", attrValue);
  convertElem = convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemC");
  ASSERT_NE(nullptr, convertElem->Attribute("attrC"));
  attrValue = convertElem->Attribute("attrC");
  EXPECT_EQ("C", attrValue);
  convertElem = convertElem->FirstChildElement();
  ASSERT_NE(nullptr, convertElem);
  EXPECT_EQ(convertElem->ValueStr(), "elemD");
  ASSERT_NE(nullptr, convertElem->GetText());
  std::string textValue = convertElem->GetText();
  EXPECT_EQ("D", textValue);
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from elem to elem
TEST(Converter, MapElemElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem = childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test moving from elem to elem
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <map>"
                << "        <from name='elemD'>"
                << "          <value>D</value>"
                << "        </from>"
                << "        <to name='elemE'>"
                << "          <value>E</value>"
                << "        </to>"
                << "      </map>"
                // test with multiple from/to values
                << "      <map>"
                << "        <from name='elemD'>"
                << "          <value>d</value>"
                << "          <value>D</value>"
                << "        </from>"
                << "        <to name='elemF'>"
                << "          <value>f</value>"
                << "          <value>F</value>"
                << "        </to>"
                << "      </map>"
                // test with multiple 'from' values, and 1 'to'
                << "      <map>"
                << "        <from name='elemD'>"
                << "          <value>d</value>"
                << "          <value>D</value>"
                << "        </from>"
                << "        <to name='elemG'>"
                << "          <value>g</value>"
                << "        </to>"
                << "      </map>"
                // test with 'from' values that don't match
                << "      <map>"
                << "        <from name='elemD'>"
                << "          <value>e</value>"
                << "          <value>E</value>"
                << "        </from>"
                << "        <to name='elemH'>"
                << "          <value>h</value>"
                << "          <value>H</value>"
                << "        </to>"
                << "      </map>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  ASSERT_NE(nullptr, convertedElem->FirstChildElement("elemD"));
  std::string elemValue = convertedElem->FirstChildElement("elemD")->GetText();
  EXPECT_EQ(elemValue, "D");
  ASSERT_NE(nullptr, convertedElem->FirstChildElement("elemE"));
  elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "E");
  ASSERT_NE(nullptr, convertedElem->FirstChildElement("elemF"));
  elemValue = convertedElem->FirstChildElement("elemF")->GetText();
  EXPECT_EQ(elemValue, "F");
  ASSERT_NE(nullptr, convertedElem->FirstChildElement("elemG"));
  elemValue = convertedElem->FirstChildElement("elemG")->GetText();
  EXPECT_EQ(elemValue, "g");
  EXPECT_EQ(nullptr, convertedElem->FirstChildElement("elemH"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from elem to attr
TEST(Converter, MapElemAttr)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from elem to attr
  TiXmlDocument xmlDoc2;
  xmlDoc2.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <map>"
                << "      <from name='elemC/elemD'>"
                << "        <value>D</value>"
                << "      </from>"
                << "      <to name='@attrE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc2;
  convertXmlDoc2.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc2, &convertXmlDoc2);

  TiXmlElement *convertedElem =  xmlDoc2.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  // check for new attribute
  ASSERT_NE(nullptr, convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "E");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from attr to attr
TEST(Converter, MapAttrAttr)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to attr
  TiXmlDocument xmlDoc3;
  xmlDoc3.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <map>"
                << "      <from name='@attrB'>"
                << "        <value>B</value>"
                << "      </from>"
                << "      <to name='@attrE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  TiXmlElement *convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  // check for new attribute
  ASSERT_NE(nullptr, convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "E");
  // check that original attribute still exists
  ASSERT_NE(nullptr, convertedElem->Attribute("attrB"));
  attrValue = convertedElem->Attribute("attrB");
  EXPECT_EQ(attrValue, "B");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_TRUE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from attr to elem
TEST(Converter, MapAttrElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to elem
  TiXmlDocument xmlDoc4;
  xmlDoc4.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <map>"
                << "      <from name='@attrB'>"
                << "        <value>B</value>"
                << "      </from>"
                << "      <to name='elemE'>"
                << "        <value>E</value>"
                << "      </to>"
                << "    </map>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc4;
  convertXmlDoc4.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc4, &convertXmlDoc4);

  TiXmlElement *convertedElem =  xmlDoc4.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  ASSERT_NE(nullptr, convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "E");
  // check that original attribute still exists
  ASSERT_NE(nullptr, convertedElem->Attribute("attrB"));
  std::string attrValue = convertedElem->Attribute("attrB");
  EXPECT_EQ(attrValue, "B");
  EXPECT_NE(nullptr, convertedElem->FirstChildElement("elemC"));
  convertedElem = convertedElem->FirstChildElement("elemC");
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_TRUE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from elem to elem across multiple levels
TEST(Converter, MapElemElemMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from elem to elem across multiple levels
  TiXmlDocument xmlDoc5;
  xmlDoc5.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <map>"
                << "    <from name='elemB/elemC/elemD'>"
                << "      <value>D</value>"
                << "    </from>"
                << "    <to name='elemCC/elemDD/elemE'>"
                << "      <value>E</value>"
                << "    </to>"
                << "  </map>"
                << "</convert>";
  TiXmlDocument convertXmlDoc5;
  convertXmlDoc5.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc5, &convertXmlDoc5);

  TiXmlElement *convertedElem =  xmlDoc5.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  TiXmlElement *convertedElem2 = convertedElem->FirstChildElement("elemCC");
  ASSERT_NE(nullptr, convertedElem2);
  convertedElem2 = convertedElem2->FirstChildElement("elemDD");
  ASSERT_NE(nullptr, convertedElem2);
  ASSERT_NE(nullptr, convertedElem2->FirstChildElement("elemE"));
  std::string elemValue = convertedElem2->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "E");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from attr to attr across multiple levels
TEST(Converter, MapAttrAttrMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to attr across multiple levels
  TiXmlDocument xmlDoc6;
  xmlDoc6.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <map>"
                << "    <from name='elemB/elemC/@attrC'>"
                << "      <value>C</value>"
                << "    </from>"
                << "    <to name='elemCC/elemDD/@attrDD'>"
                << "      <value>DD</value>"
                << "    </to>"
                << "  </map>"
                << "</convert>";
  TiXmlDocument convertXmlDoc6;
  convertXmlDoc6.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc6, &convertXmlDoc6);

  TiXmlElement *convertedElem =  xmlDoc6.FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  TiXmlElement *convertedElem2 = convertedElem->FirstChildElement("elemCC");
  ASSERT_NE(nullptr, convertedElem2);
  convertedElem2 = convertedElem2->FirstChildElement("elemDD");
  ASSERT_NE(nullptr, convertedElem2);
  std::string attrValue = convertedElem2->Attribute("attrDD");
  EXPECT_EQ(attrValue, "DD");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_TRUE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from elem to attr across multiple levels
TEST(Converter, MapElemAttrMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from elem to attr across multiple levels
  TiXmlDocument xmlDoc7;
  xmlDoc7.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <map>"
                << "    <from name='elemB/elemC/elemD'>"
                << "      <value>D</value>"
                << "    </from>"
                << "    <to name='elemCC/elemDD/@attrDD'>"
                << "      <value>DD</value>"
                << "    </to>"
                << "  </map>"
                << "</convert>";
  TiXmlDocument convertXmlDoc7;
  convertXmlDoc7.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc7, &convertXmlDoc7);

  TiXmlElement *convertedElem =  xmlDoc7.FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  TiXmlElement *convertedElem2 = convertedElem->FirstChildElement("elemCC");
  ASSERT_NE(nullptr, convertedElem2);
  convertedElem2 = convertedElem2->FirstChildElement("elemDD");
  ASSERT_NE(nullptr, convertedElem2);
  std::string attrValue = convertedElem2->Attribute("attrDD");
  EXPECT_EQ(attrValue, "DD");
  convertedElem = convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemD"));
}

////////////////////////////////////////////////////
/// Ensure that Converter::Map function is working
/// Test moving from attr to elem across multiple levels
TEST(Converter, MapAttrElemMultipleLevels)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to elem across multiple levels
  TiXmlDocument xmlDoc8;
  xmlDoc8.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <map>"
                << "    <from name='elemB/elemC/@attrC'>"
                << "      <value>C</value>"
                << "    </from>"
                << "    <to name='elemCC/elemDD/elemE'>"
                << "      <value>E</value>"
                << "    </to>"
                << "  </map>"
                << "</convert>";
  TiXmlDocument convertXmlDoc8;
  convertXmlDoc8.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc8, &convertXmlDoc8);

  TiXmlElement *convertedElem =  xmlDoc8.FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  TiXmlElement *convertedElem2 = convertedElem->FirstChildElement("elemCC");
  ASSERT_NE(nullptr, convertedElem2);
  convertedElem2 = convertedElem2->FirstChildElement("elemDD");
  ASSERT_NE(nullptr, convertedElem2);
  EXPECT_NE(nullptr, convertedElem2->FirstChildElement("elemE"));
  std::string elemValue = convertedElem2->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "E");
  convertedElem =  convertedElem->FirstChildElement("elemB");
  ASSERT_NE(nullptr, convertedElem);
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_TRUE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
TEST(Converter, RenameElemElem)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  ASSERT_NE(nullptr, childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test moving from elem to elem
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <rename>"
                << "        <from element='elemD'/>"
                << "        <to element='elemE'/>"
                << "      </rename>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  TiXmlElement *elemB = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, elemB);
  EXPECT_EQ(elemB->ValueStr(), "elemB");
  TiXmlElement *elemC = elemB->FirstChild("elemC")->ToElement();
  ASSERT_NE(nullptr, elemC);
  TiXmlElement *elemE = elemC->FirstChildElement();
  ASSERT_NE(nullptr, elemE);
  EXPECT_EQ(elemE->ValueStr(), "elemE");
  std::string elemValue = elemE->GetText();
  ASSERT_EQ(elemValue, "D");
}

////////////////////////////////////////////////////
TEST(Converter, RenameAttrAttr)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test moving from attr to attr
  TiXmlDocument xmlDoc3;
  xmlDoc3.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <rename>"
                << "        <from attribute='attrC'/>"
                << "        <to element='elemE' attribute='attrE'/>"
                << "      </rename>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  TiXmlElement *convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  convertedElem = convertedElem->FirstChild("elemE")->ToElement();
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
}

////////////////////////////////////////////////////
TEST(Converter, RenameNoFrom)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test failing to move since there is nothing specified in the "from" element
  TiXmlDocument xmlDoc3;
  xmlDoc3.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <rename>"
                << "        <from/>"
                << "        <to element='elemE' attribute='attrE'/>"
                << "      </rename>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  TiXmlElement *convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
TEST(Converter, RenameNoTo)
{
  // Set up an xml string for testing
  std::string xmlString = getXmlString();

  // Test failing to move since there is nothing specified in the "to" element
  TiXmlDocument xmlDoc3;
  xmlDoc3.Parse(xmlString.c_str());
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <convert name='elemC'>"
                << "      <rename>"
                << "        <from attribute='attrC'/>"
                << "        <to attribute='attrE'/>"
                << "      </rename>"
                << "    </convert>"
                << "  </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  TiXmlElement *convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  convertedElem =  convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  convertedElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

////////////////////////////////////////////////////
TEST(Converter, GazeboToSDF)
{
  std::stringstream stream;
  stream << "<gazebo version='1.2'>"
         << "</gazebo>";
  std::string xmlString = stream.str();

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());
  EXPECT_FALSE(sdf::Converter::Convert(&xmlDoc, "1.3"));
}

////////////////////////////////////////////////////
TEST(Converter, NullDoc)
{
  TiXmlDocument xmlDoc;
  TiXmlDocument convertXmlDoc;

  ASSERT_THROW(sdf::Converter::Convert(nullptr, &convertXmlDoc),
               sdf::AssertionInternalError);
  ASSERT_THROW(sdf::Converter::Convert(&xmlDoc, nullptr),
               sdf::AssertionInternalError);
  ASSERT_THROW(sdf::Converter::Convert(nullptr, "1.4"),
               sdf::AssertionInternalError);
}

////////////////////////////////////////////////////
TEST(Converter, NoVersion)
{
  std::string xmlString("<sdf></sdf>");

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  ASSERT_FALSE(sdf::Converter::Convert(&xmlDoc, "1.3"));
}

////////////////////////////////////////////////////
TEST(Converter, SameVersion)
{
  std::string xmlString("<sdf version='1.3'></sdf>");

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  std::ostringstream xmlDocBefore;
  xmlDocBefore << xmlDoc;

  ASSERT_TRUE(sdf::Converter::Convert(&xmlDoc, "1.3"));
  std::ostringstream xmlDocAfter;
  xmlDocAfter << xmlDoc;

  // Expect xmlDoc to be unchanged after conversion
  EXPECT_EQ(xmlDocBefore.str(), xmlDocAfter.str());
}

////////////////////////////////////////////////////
TEST(Converter, NewerVersion)
{
  std::string xmlString("<sdf version='1.5'></sdf>");

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  ASSERT_TRUE(sdf::Converter::Convert(&xmlDoc, "1.6"));
}

////////////////////////////////////////////////////
TEST(Converter, MuchNewerVersion)
{
  std::string xmlString("<sdf version='1.3'></sdf>");

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  ASSERT_TRUE(sdf::Converter::Convert(&xmlDoc, "1.6"));
}

static std::string ConvertDoc_15_16()
{
  return sdf::testing::SourceFile("sdf", "1.6", "1_5.convert");
}
static std::string ConvertDoc_16_17()
{
  return sdf::testing::SourceFile("sdf", "1.7", "1_6.convert");
}

/////////////////////////////////////////////////
/// Test conversion of imu in 1.5 to 1.6
TEST(Converter, IMU_15_to_16)
{
  // The imu noise in 1.5 format
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <model name="box_old_imu_noise">
      <link name="link">
        <sensor name='imu_sensor' type='imu'>
          <imu>
            <noise>
              <type>gaussian</type>
              <rate>
                <mean>0</mean>
                <stddev>0.0002</stddev>
                <bias_mean>7.5e-06</bias_mean>
                <bias_stddev>8e-07</bias_stddev>
              </rate>
              <accel>
                <mean>0</mean>
                <stddev>0.017</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </accel>
            </noise>
          </imu>
        </sensor>
      </link>
    </model>
  </world>
</sdf>)";

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Convert
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.LoadFile(ConvertDoc_15_16());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Check some basic elements
  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "sdf");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "world");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "model");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "link");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "sensor");

  // Get the imu
  TiXmlElement *imuElem = convertedElem->FirstChildElement();
  EXPECT_EQ(imuElem->ValueStr(), "imu");

  // Get the angular_velocity
  TiXmlElement *angVelElem = imuElem->FirstChildElement();
  EXPECT_EQ(angVelElem->ValueStr(), "angular_velocity");

  // Get the linear_acceleration
  TiXmlElement *linAccElem = angVelElem->NextSiblingElement();
  EXPECT_EQ(linAccElem->ValueStr(), "linear_acceleration");

  std::array<char, 3> axis = {'x', 'y', 'z'};

  TiXmlElement *angVelAxisElem = angVelElem->FirstChildElement();
  TiXmlElement *linAccAxisElem = linAccElem->FirstChildElement();

  // Iterate over <x>, <y>, and <z> elements under <angular_velocity> and
  // <linear_acceleration>
  for (auto const &a : axis)
  {
    EXPECT_EQ(angVelAxisElem->Value()[0], a);
    EXPECT_EQ(linAccAxisElem->Value()[0], a);

    TiXmlElement *angVelAxisNoiseElem = angVelAxisElem->FirstChildElement();
    TiXmlElement *linAccAxisNoiseElem = linAccAxisElem->FirstChildElement();

    EXPECT_EQ(angVelAxisNoiseElem->ValueStr(), "noise");
    EXPECT_EQ(linAccAxisNoiseElem->ValueStr(), "noise");

    EXPECT_STREQ(angVelAxisNoiseElem->Attribute("type"), "gaussian");
    EXPECT_STREQ(linAccAxisNoiseElem->Attribute("type"), "gaussian");

    EXPECT_STREQ(angVelAxisNoiseElem->FirstChildElement("mean")->GetText(),
                 "0");
    EXPECT_STREQ(linAccAxisNoiseElem->FirstChildElement("mean")->GetText(),
                 "0");

    EXPECT_STREQ(angVelAxisNoiseElem->FirstChildElement("stddev")->GetText(),
                 "0.0002");
    EXPECT_STREQ(linAccAxisNoiseElem->FirstChildElement("stddev")->GetText(),
                 "0.017");

    EXPECT_STREQ(angVelAxisNoiseElem->FirstChildElement("bias_mean")->GetText(),
                 "7.5e-06");
    EXPECT_STREQ(linAccAxisNoiseElem->FirstChildElement("bias_mean")->GetText(),
                 "0.1");

    EXPECT_STREQ(angVelAxisNoiseElem->FirstChildElement(
          "bias_stddev")->GetText(), "8e-07");
    EXPECT_STREQ(linAccAxisNoiseElem->FirstChildElement(
          "bias_stddev")->GetText(), "0.001");

    angVelAxisElem = angVelAxisElem->NextSiblingElement();
    linAccAxisElem = linAccAxisElem->NextSiblingElement();
  }
}

/////////////////////////////////////////////////
/// Test conversion of gravity, magnetic_field in 1.5 to 1.6
TEST(Converter, World_15_to_16)
{
  // The gravity and magnetic_field in 1.5 format
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      <gravity>1 0 -9.8</gravity>
      <magnetic_field>1 2 3</magnetic_field>
    </physics>
  </world>
</sdf>)";

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Convert
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.LoadFile(ConvertDoc_15_16());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Check some basic elements
  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "sdf");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "world");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "physics");

  // gravity and magnetic_field should have been moved from physics to world
  EXPECT_EQ(nullptr, convertedElem->FirstChildElement("gravity"));
  EXPECT_EQ(nullptr, convertedElem->FirstChildElement("magnetic_field"));

  // Get the gravity
  TiXmlElement *gravityElem = convertedElem->NextSiblingElement("gravity");
  ASSERT_NE(nullptr, gravityElem);
  EXPECT_STREQ(gravityElem->GetText(), "1 0 -9.8");

  // Get the magnetic_field
  TiXmlElement *magneticFieldElem =
    convertedElem->NextSiblingElement("magnetic_field");
  ASSERT_NE(nullptr, magneticFieldElem);
  EXPECT_STREQ(magneticFieldElem->GetText(), "1 2 3");
}

/////////////////////////////////////////////////
/// Test conversion of pose attributes in 1.6 to 1.7
TEST(Converter, Pose_16_to_17)
{
  // A world file with pose elements in 1.5 format
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <model name="model">
      <pose frame="world">0 0 0 0 0 0</pose>
      <link name="parent"/>
      <link name="child">
        <pose frame="joint">0 0 0 0 0 0</pose>
      </link>
      <joint name="joint" type="fixed">
        <parent>parent</parent>
        <child>child</child>
        <pose frame="parent">0 0 0 0 0 0</pose>
      </joint>
    </model>
  </world>
</sdf>)";

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Convert
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.LoadFile(ConvertDoc_16_17());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Check some basic elements
  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "sdf");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "world");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "model");

  TiXmlElement *modelPoseElem = convertedElem->FirstChildElement();
  ASSERT_NE(nullptr, modelPoseElem);
  EXPECT_EQ("pose", modelPoseElem->ValueStr());
  // frame attribute should have been moved to relative_to
  EXPECT_EQ(nullptr, modelPoseElem->Attribute("frame"));
  EXPECT_NE(nullptr, modelPoseElem->Attribute("relative_to"));
  EXPECT_STREQ("world", modelPoseElem->Attribute("relative_to"));

  TiXmlElement *parentLinkElem = modelPoseElem->NextSiblingElement();
  ASSERT_NE(nullptr, parentLinkElem);
  EXPECT_EQ("link", parentLinkElem->ValueStr());
  EXPECT_EQ(nullptr, parentLinkElem->FirstChildElement());

  TiXmlElement *childLinkElem = parentLinkElem->NextSiblingElement();
  ASSERT_NE(nullptr, childLinkElem);
  EXPECT_EQ("link", childLinkElem->ValueStr());
  TiXmlElement *childLinkPoseElem = childLinkElem->FirstChildElement();
  ASSERT_NE(nullptr, childLinkPoseElem);
  EXPECT_EQ("pose", childLinkPoseElem->ValueStr());
  // frame attribute should have been moved to relative_to
  EXPECT_EQ(nullptr, childLinkPoseElem->Attribute("frame"));
  EXPECT_NE(nullptr, childLinkPoseElem->Attribute("relative_to"));
  EXPECT_STREQ("joint", childLinkPoseElem->Attribute("relative_to"));

  TiXmlElement *jointLinkElem = childLinkElem->NextSiblingElement();
  ASSERT_NE(nullptr, jointLinkElem);
  EXPECT_EQ("joint", jointLinkElem->ValueStr());
  TiXmlElement *jointLinkPoseElem = jointLinkElem->FirstChildElement("pose");
  ASSERT_NE(nullptr, jointLinkPoseElem);
  EXPECT_EQ("pose", jointLinkPoseElem->ValueStr());
  // frame attribute should have been moved to relative_to
  EXPECT_EQ(nullptr, jointLinkPoseElem->Attribute("frame"));
  EXPECT_NE(nullptr, jointLinkPoseElem->Attribute("relative_to"));
  EXPECT_STREQ("parent", jointLinkPoseElem->Attribute("relative_to"));
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
