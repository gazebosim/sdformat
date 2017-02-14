/*
 * Copyright 2012-2017 Open Source Robotics Foundation
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

#include "sdf/sdf.hh"
#include "sdf/parser_urdf.hh"

std::string get_minimal_urdf_txt()
{
  std::ostringstream stream;
  stream << "<robot name='test_robot'>"
         << "  <link name='link1' />"
         << "</robot>";
  return stream.str();
}

/* By design, errors are only reported in std output */
TEST(URDFParser, InitModelDoc_EmptyDoc_NoThrow)
{
   ASSERT_NO_THROW(
     TiXmlDocument doc = TiXmlDocument();
     sdf::URDF2SDF parser_;
     TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
   );
}

TEST(URDFParser, InitModelDoc_BasicModel_NoThrow)
{
    ASSERT_NO_THROW(
      TiXmlDocument doc;
      doc.Parse(get_minimal_urdf_txt().c_str());
      sdf::URDF2SDF parser_;
      TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
    );
}

TEST(URDFParser, ParseResults_BasicModel_ParseEqualToModel)
{
   // URDF -> SDF
   TiXmlDocument doc;
   doc.Parse(get_minimal_urdf_txt().c_str());
   sdf::URDF2SDF parser_;
   TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
   std::string sdf_result_str;
   sdf_result_str << sdf_result;

   // SDF -> SDF
   std::ostringstream stream;
   // Use hard-coded "1.4" for version string
   // until parser_urdf.cc exports version "1.5"
   // see `sdf->SetAttribute("version", "1.4");`
   // in URDF2SDF::InitModelString()
   stream << "<sdf version='" << "1.4" << "'>"
          << "  <model name='test_robot' />"
          << "</sdf>";
   TiXmlDocument sdf_doc;
   sdf_doc.Parse(stream.str().c_str());
   std::string sdf_same_result_str;
   sdf_same_result_str << sdf_doc;

   ASSERT_EQ(sdf_same_result_str, sdf_result_str);
}

TEST(URDFParser, ParseRobotOriginXYZBlank)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin />"
         << "  <link name=\"link\" />"
         << "</robot>";
  TiXmlDocument doc;
  doc.Parse(stream.str().c_str());
  sdf::URDF2SDF parser_;
  TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
  TiXmlElement* sdf = sdf_result.FirstChildElement("sdf");
  EXPECT_TRUE(sdf != NULL);
  TiXmlElement* model = sdf->FirstChildElement("model");
  EXPECT_TRUE(model != NULL);
  TiXmlElement* pose = model->FirstChildElement("pose");
  ASSERT_TRUE(pose != NULL);
}

TEST(URDFParser, ParseRobotOriginRPYBlank)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin xyz=\"0 0 0\"/>"
         << "  <link name=\"link\" />"
         << "</robot>";
  TiXmlDocument doc;
  sdf::URDF2SDF parser_;
  doc.Parse(stream.str().c_str());
  TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
  TiXmlElement* sdf = sdf_result.FirstChildElement("sdf");
  EXPECT_TRUE(sdf != NULL);
  TiXmlElement* model = sdf->FirstChildElement("model");
  EXPECT_TRUE(model != NULL);
  TiXmlElement* pose = model->FirstChildElement("pose");
  ASSERT_TRUE(pose != NULL);
}

TEST(URDFParser, ParseRobotOriginInvalidXYZ)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin xyz=\"0 foo 0\" rpy=\"0 0 0\"/>"
         << "  <link name=\"link\" />"
         << "</robot>";
  TiXmlDocument doc;
  sdf::URDF2SDF parser_;
  doc.Parse(stream.str().c_str());
  TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
  TiXmlElement* sdf = sdf_result.FirstChildElement("sdf");
  EXPECT_TRUE(sdf != NULL);
  TiXmlElement* model = sdf->FirstChildElement("model");
  EXPECT_TRUE(model != NULL);
  TiXmlElement* pose = model->FirstChildElement("pose");
  ASSERT_TRUE(pose != NULL);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
