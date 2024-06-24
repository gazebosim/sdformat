/*
 * Copyright 2020 Open Source Robotics Foundation
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
#include <sstream>
#include <gtest/gtest.h>

#include "ParamPassing.hh"
#include "sdf/Element.hh"
#include "sdf/parser.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(ParamPassing, GetElement)
{
  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.7'>"
         << "  <model name='test'>"
         << "    <model name='test_model'>"
         << "      <link name='test_link'>"
         << "        <collision name='test_visual'>"
         << "          <geometry>"
         << "            <box>"
         << "              <size>0.1 0.1 0.1</size>"
         << "            </box>"
         << "          </geometry>"
         << "        </collision>"
         << "      </link>"
         << "      <link name='test_link2'>"
         << "        <visual name='test_visual'>"
         << "          <geometry>"
         << "            <box>"
         << "              <size>0.5 0.5 0.5</size>"
         << "            </box>"
         << "          </geometry>"
         << "        </visual>"
         << "      </link>"
         << "    </model>"
         << "  </model>"
         << "</sdf>";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  ASSERT_TRUE(sdf::readString(stream.str(), sdf));

  // checking element ptrs to <link name='test_link'> are equal
  sdf::ElementPtr elem = sdf->Root()->GetFirstElement()
                                        ->GetElement("model")
                                        ->GetElement("link");
  sdf::ElementPtr paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                                 "link",
                                                 "test_model::test_link");
  EXPECT_NE(nullptr, elem);
  EXPECT_NE(nullptr, paramPassElem);
  EXPECT_EQ(elem, paramPassElem);

  // checking element ptrs to <visual name='test_visual'> are equal
  elem = sdf->Root()->GetFirstElement()
                        ->GetElement("model")
                        ->GetElement("link")
                        ->GetNextElement()
                        ->GetElement("visual");
  paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                 "visual",
                                 "test_model::test_link2::test_visual");
  EXPECT_NE(nullptr, elem);
  EXPECT_NE(nullptr, paramPassElem);
  EXPECT_EQ(elem, paramPassElem);

  // No element <visual name='test_visual'> (element is a collision)
  paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                 "visual",
                                 "test_model::test_link::test_visual");
  EXPECT_EQ(nullptr, paramPassElem);

  // incorrect element identifier (model::test_link::test_visual)
  paramPassElem = sdf::ParamPassing::getElementById(sdf->Root(),
                                 "collision",
                                 "model::test_link::test_visual");
  EXPECT_EQ(nullptr, paramPassElem);
}

////////////////////////////////////////
// Test warnings outputs for GetElementByName
TEST(ParamPassing, GetElementByNameWarningOutput)
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

  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.8'>"
         << "  <model name='test_model'>"
         << "    <link name='link_name'/>"
         << "  </model>"
         << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  sdf::Errors errors;
  sdf::ParserConfig parserConfig;
  parserConfig.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  sdf::readString(stream.str(), parserConfig, sdfParsed, errors);
  EXPECT_TRUE(errors.empty());

  std::ostringstream stream2;
  stream2 << "  <model>"
          << "  </model>";
  tinyxml2::XMLDocument doc;
  doc.Parse(stream2.str().c_str());

  sdf::ParamPassing::getElementByName(sdfParsed->Root(),
                        doc.FirstChildElement("model"),
                        parserConfig,
                        errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::WARNING);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "The original element [model] contains the attribute 'name' but none was"
      " provided in the element modifier. The assumed element to be modified "
      "is: <model name='test_model'>"));
  errors.clear();

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();

  parserConfig.SetWarningsPolicy(sdf::EnforcementPolicy::WARN);
  sdf::ParamPassing::getElementByName(sdfParsed->Root(),
                        doc.FirstChildElement("model"),
                        parserConfig,
                        errors);
  EXPECT_TRUE(errors.empty());
  // Check the warning has been printed
  EXPECT_NE(std::string::npos, buffer.str().find(
      "The original element [model] contains the attribute 'name' but none "
      "was provided in the element modifier. The assumed element to be "
      "modified is: <model name='test_model'>")) << buffer.str();
}

////////////////////////////////////////
// Test warnings outputs for GetElementByName
TEST(ParamPassing, ModifyChildrenNameWarningOutput)
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

  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.8'>"
         << "  <model name='test_model'>"
         << "    <link name='link_name'/>"
         << "  </model>"
         << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  sdf::Errors errors;
  sdf::ParserConfig parserConfig;
  parserConfig.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  sdf::readString(stream.str(), parserConfig, sdfParsed, errors);
  EXPECT_TRUE(errors.empty());

  std::ostringstream stream2;
  stream2 << "<sdf version='1.8'>"
          << "  <model name='test'>"
          << "  </model>"
          << "</sdf>";
  tinyxml2::XMLDocument doc;
  doc.Parse(stream2.str().c_str());

  sdf::ParamPassing::modifyChildren(
                        doc.FirstChildElement("sdf"),
                        parserConfig,
                        sdfParsed->Root(),
                        errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::WARNING);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "No modifications for element <model name=\"test\"/>\n provided, "
      "skipping modification for:\n<sdf version=\"1.8\">\n"
      "    <model name=\"test\"/>\n</sdf>"));
  errors.clear();

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();

  parserConfig.SetWarningsPolicy(sdf::EnforcementPolicy::WARN);
  sdf::ParamPassing::modifyChildren(
                        doc.FirstChildElement("sdf"),
                        parserConfig,
                        sdfParsed->Root(),
                        errors);
  EXPECT_TRUE(errors.empty());
  // Check the warning has been printed
  EXPECT_NE(std::string::npos, buffer.str().find(
      "No modifications for element <model name=\"test\"/>\n provided, "
      "skipping modification for:\n<sdf version=\"1.8\">\n"
      "    <model name=\"test\"/>\n</sdf>")) << buffer.str();
}
