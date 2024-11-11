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

#include <list>

#include "parser_urdf.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
std::string getMinimalUrdfTxt()
{
  std::ostringstream stream;
  stream << "<robot name='test_robot'>"
         << "  <link name='link1' />"
         << "</robot>";
  return stream.str();
}

/////////////////////////////////////////////////
std::string convertUrdfStrToSdfStr(
    const std::string &_urdf,
    const sdf::ParserConfig &_config = sdf::ParserConfig::GlobalConfig())
{
  sdf::URDF2SDF parser_;
  tinyxml2::XMLDocument sdf_result;
  parser_.InitModelString(_urdf, _config, &sdf_result);
  tinyxml2::XMLPrinter printer;
  sdf_result.Accept(&printer);
  return printer.CStr();
}

/////////////////////////////////////////////////
void convertUrdfStrToSdf(
    const std::string &_urdf, sdf::SDF &_sdf,
    const sdf::ParserConfig &_config = sdf::ParserConfig::GlobalConfig())
{
  _sdf.SetFromString(convertUrdfStrToSdfStr(_urdf, _config));
}

/////////////////////////////////////////////////
/* By design, errors are only reported in std output */
TEST(URDFParser, InitModelDoc_EmptyDoc_NoThrow)
{
  // Suppress deprecation for sdf::URDF2SDF
  ASSERT_NO_THROW(
    tinyxml2::XMLDocument doc = tinyxml2::XMLDocument();
    sdf::URDF2SDF parser_;
    sdf::ParserConfig config_;
    tinyxml2::XMLDocument sdf_result;
    parser_.InitModelDoc(&doc, config_, &sdf_result);
  );    // NOLINT(whitespace/parens)
}

/////////////////////////////////////////////////
TEST(URDFParser, InitModelDoc_BasicModel_NoThrow)
{
  // Suppress deprecation for sdf::URDF2SDF
  ASSERT_NO_THROW(
    tinyxml2::XMLDocument doc;
    doc.Parse(getMinimalUrdfTxt().c_str());
    sdf::URDF2SDF parser_;
    sdf::ParserConfig config_;
    tinyxml2::XMLDocument sdf_result;
    parser_.InitModelDoc(&doc, config_, &sdf_result);
  );    // NOLINT(whitespace/parens)
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseResults_BasicModel_ParseEqualToModel)
{
  // URDF -> SDF
  tinyxml2::XMLDocument doc;
  doc.Parse(getMinimalUrdfTxt().c_str());
  sdf::URDF2SDF parser_;
  sdf::ParserConfig config_;

  tinyxml2::XMLDocument sdf_result;
  parser_.InitModelDoc(&doc, config_, &sdf_result);

  tinyxml2::XMLPrinter printer;
  sdf_result.Print(&printer);
  std::string sdf_result_str = printer.CStr();

  // SDF -> SDF
  std::ostringstream stream;
  // parser_urdf.cc exports version "1.11"
  stream << "<sdf version='" << "1.11" << "'>"
         << "  <model name='test_robot' />"
         << "</sdf>";
  tinyxml2::XMLDocument sdf_doc;
  sdf_doc.Parse(stream.str().c_str());

  tinyxml2::XMLPrinter printer2;
  sdf_doc.Print(&printer2);
  std::string sdf_same_result_str = printer2.CStr();

  ASSERT_EQ(sdf_same_result_str, sdf_result_str);
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseRobotOriginXYZBlank)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin />"
         << "  <link name=\"link\" />"
         << "</robot>";
  tinyxml2::XMLDocument doc;
  doc.Parse(stream.str().c_str());
  sdf::URDF2SDF parser_;
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdf_result;
  parser_.InitModelDoc(&doc, config_, &sdf_result);
  tinyxml2::XMLElement *sdf = sdf_result.FirstChildElement("sdf");
  ASSERT_NE(nullptr, sdf);
  tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
  ASSERT_NE(nullptr, model);
  tinyxml2::XMLElement *pose = model->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseRobotOriginRPYBlank)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin xyz=\"0 0 0\"/>"
         << "  <link name=\"link\" />"
         << "</robot>";
  tinyxml2::XMLDocument doc;
  sdf::URDF2SDF parser_;
  doc.Parse(stream.str().c_str());
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdf_result;
  parser_.InitModelDoc(&doc, config_, &sdf_result);
  tinyxml2::XMLElement *sdf = sdf_result.FirstChildElement("sdf");
  ASSERT_NE(nullptr, sdf);
  tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
  ASSERT_NE(nullptr, model);
  tinyxml2::XMLElement *pose = model->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseRobotMaterialBlank)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <link name=\"link\">"
         << "    <inertial>"
         << "      <mass value=\"1\"/>"
         << "      <inertia ixx=\"1\" ixy=\"0.0\" ixz=\"0.0\""
         << "               iyy=\"1\" iyz=\"0.0\" izz=\"1\"/>"
         << "    </inertial>"
         << "    <visual>"
         << "      <geometry>"
         << "        <sphere radius=\"1.0\"/>"
         << "      </geometry>"
         << "    </visual>"
         << "  </link>"
         << "  <gazebo reference=\"link\">"
         << "    <mu1>0.2</mu1>"
         << "  </gazebo>"
         << "</robot>";
  tinyxml2::XMLDocument doc;
  doc.Parse(stream.str().c_str());
  sdf::URDF2SDF parser;
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdfXml;
  parser.InitModelDoc(&doc, config_, &sdfXml);
  auto sdfElem = sdfXml.FirstChildElement("sdf");
  ASSERT_NE(nullptr, sdfElem);
  auto modelElem = sdfElem->FirstChildElement("model");
  ASSERT_NE(nullptr, modelElem);
  auto linkElem = modelElem->FirstChildElement("link");
  ASSERT_NE(nullptr, linkElem);
  auto visualElem = linkElem->FirstChildElement("visual");
  ASSERT_NE(nullptr, visualElem);

  auto materialElem = visualElem->FirstChildElement("material");
  ASSERT_EQ(nullptr, materialElem);

  parser.ListSDFExtensions();
  parser.ListSDFExtensions("link");
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseRobotMaterialName)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <link name=\"link\">"
         << "    <inertial>"
         << "      <mass value=\"1\"/>"
         << "      <inertia ixx=\"1\" ixy=\"0.0\" ixz=\"0.0\""
         << "               iyy=\"1\" iyz=\"0.0\" izz=\"1\"/>"
         << "    </inertial>"
         << "    <visual>"
         << "      <geometry>"
         << "        <sphere radius=\"1.0\"/>"
         << "      </geometry>"
         << "    </visual>"
         << "  </link>"
         << "  <gazebo reference=\"link\">"
         << "    <material>Gazebo/Orange</material>"
         << "  </gazebo>"
         << "</robot>";
  tinyxml2::XMLDocument doc;
  doc.Parse(stream.str().c_str());
  sdf::URDF2SDF parser;
  sdf::ParserConfig config_;

  tinyxml2::XMLDocument sdfXml;
  parser.InitModelDoc(&doc, config_, &sdfXml);

  auto sdfElem = sdfXml.FirstChildElement("sdf");
  ASSERT_NE(nullptr, sdfElem);
  auto modelElem = sdfElem->FirstChildElement("model");
  ASSERT_NE(nullptr, modelElem);
  auto linkElem = modelElem->FirstChildElement("link");
  ASSERT_NE(nullptr, linkElem);
  auto visualElem = linkElem->FirstChildElement("visual");
  ASSERT_NE(nullptr, visualElem);

  auto materialElem = visualElem->FirstChildElement("material");
  ASSERT_NE(nullptr, materialElem);
  auto scriptElem = materialElem->FirstChildElement("script");
  ASSERT_NE(nullptr, scriptElem);
  auto nameElem = scriptElem->FirstChildElement("name");
  ASSERT_NE(nullptr, nameElem);
  EXPECT_EQ("Gazebo/Orange", std::string(nameElem->GetText()));
  auto uriElem = scriptElem->FirstChildElement("uri");
  ASSERT_NE(nullptr, uriElem);
  EXPECT_EQ("file://media/materials/scripts/gazebo.material",
      std::string(uriElem->GetText()));

  parser.ListSDFExtensions();
  parser.ListSDFExtensions("link");
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseRobotOriginInvalidXYZ)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin xyz=\"0 foo 0\" rpy=\"0 0 0\"/>"
         << "  <link name=\"link\" />"
         << "</robot>";
  tinyxml2::XMLDocument doc;
  sdf::URDF2SDF parser_;
  doc.Parse(stream.str().c_str());
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdf_result;
  parser_.InitModelDoc(&doc, config_, &sdf_result);
  tinyxml2::XMLElement *sdf = sdf_result.FirstChildElement("sdf");
  ASSERT_NE(nullptr, sdf);
  tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
  ASSERT_NE(nullptr, model);
  tinyxml2::XMLElement *pose = model->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseGazeboLinkFactors)
{
  std::multimap<std::string, std::vector<std::string>> elements
  {
    {"dampingFactor", {"model", "link", "velocity_decay", "linear", "25.2"}},
    {"dampingFactor", {"model", "link", "velocity_decay", "angular", "25.2"}},
    {"maxVel", {"model", "link", "collision", "surface", "contact", "ode",
                "max_vel", "1.7"}},
    {"minDepth", {"model", "link", "collision", "surface", "contact", "ode",
                  "min_depth", "0.001"}},
    {"mu1", {"model", "link", "collision", "surface", "friction", "ode",
             "mu", "8.9"}},
    {"mu2", {"model", "link", "collision", "surface", "friction", "ode",
             "mu2", "12.5"}},
    {"kp", {"model", "link", "collision", "surface", "contact", "ode",
            "kp", "1000.2"}},
    {"kd", {"model", "link", "collision", "surface", "contact", "ode",
            "kd", "100.1"}},
    {"maxContacts", {"model", "link", "collision", "max_contacts", "99"}},
    {"laserRetro", {"model", "link", "collision", "laser_retro", "72.8"}},
  };

  for (std::map<std::string, std::vector<std::string> >::iterator it =
         elements.begin(); it != elements.end(); ++it)
  {
    std::string value = it->second[it->second.size() - 1];
    std::ostringstream stream;
    stream << "<robot name=\"test\">"
           << "  <link name=\"wheel_left_link\">"
           << "    <collision>"
           << "      <geometry>"
           << "        <cylinder length=\"0.0206\" radius=\"0.0352\"/>"
           << "      </geometry>"
           << "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
           << "    </collision>"
           << "    <inertial>"
           << "      <mass value=\"0.01\" />"
           << "      <origin xyz=\"0 0 0\" />"
           << "      <inertia ixx=\"0.001\" ixy=\"0.0\" ixz=\"0.0\""
           << "               iyy=\"0.001\" iyz=\"0.0\""
           << "               izz=\"0.001\" />"
           << "    </inertial>"
           << "  </link>"
           << "  <gazebo reference=\"wheel_left_link\">"
           << "    <" << it->first << ">" << value << "</" << it->first << ">"
           << "  </gazebo>"
           << "</robot>";

    tinyxml2::XMLDocument doc;
    sdf::URDF2SDF parser_;
    doc.Parse(stream.str().c_str());
    sdf::ParserConfig config_;
    tinyxml2::XMLDocument sdf_result;
    parser_.InitModelDoc(&doc, config_, &sdf_result);

    tinyxml2::XMLElement *tmp = sdf_result.FirstChildElement("sdf");
    ASSERT_NE(nullptr, tmp);

    unsigned int i;

    for (i = 0; i < it->second.size() - 1; ++i)
    {
      tmp = tmp->FirstChildElement(it->second[i].c_str());
      ASSERT_NE(nullptr, tmp);
    }

    // For the last element, check that it is exactly what we expect
    EXPECT_EQ(tmp->FirstChild()->Value(), it->second[i]);
    parser_.ListSDFExtensions();
    parser_.ListSDFExtensions("wheel_left_link");
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseGazeboInvalidDampingFactor)
{
  std::ostringstream stream;
  stream << "<robot name=\"test\">"
         << "  <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
         << "  <link name=\"link\">"
         << "    <inertial>"
         << "      <mass value=\"0.5\" />"
         << "    </inertial>"
         << "  </link>"
         << "  <gazebo reference=\"link\">"
         << "    <dampingFactor>foo</dampingFactor>"
         << "  </gazebo>"
         << "</robot>";
  tinyxml2::XMLDocument doc;
  sdf::URDF2SDF parser_;
  doc.Parse(stream.str().c_str());
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdf_result;
  ASSERT_THROW(parser_.InitModelDoc(&doc, config_, &sdf_result),
               std::invalid_argument);

  parser_.ListSDFExtensions();
  parser_.ListSDFExtensions("link");
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseGazeboJointElements)
{
  std::map<std::string, std::vector<std::string>> elements
  {
    {"stopCfm", {"model", "joint", "physics", "ode", "limit", "cfm", "0.8"}},
    {"stopErp", {"model", "joint", "physics", "ode", "limit", "erp", "0.8"}},
    {"fudgeFactor", {"model", "joint", "physics", "ode", "fudge_factor",
        "11.1"}},
  };

  for (std::map<std::string, std::vector<std::string> >::iterator it =
         elements.begin(); it != elements.end(); ++it)
  {
    std::string value = it->second[it->second.size() - 1];
    std::ostringstream stream;
    stream << "<robot name=\"test\">"
           << "  <link name=\"chest_link\">"
           << "    <inertial>"
           << "      <mass value=\"10\" />"
           << "      <origin xyz=\"0 0 -0.1\" />"
           << "      <inertia ixx=\"1E-4\""
           << "               iyy=\"1E-4\""
           << "               izz=\"1E-4\""
           << "               ixy=\"1E-7\""
           << "               ixz=\"1E-7\""
           << "               iyz=\"1E-7\"/>"
           << "    </inertial>"
           << "    <collision>"
           << "      <origin xyz=\"0 0 -0.2337\" rpy=\"0.0 0.0 0.0\" />"
           << "    </collision>"
           << "  </link>"
           << "  <link name=\"neck_tilt\">"
           << "    <inertial>"
           << "      <mass value=\"0.940\" />"
           << "      <origin xyz=\"0.000061 0.003310 0.028798\"/>"
           << "      <inertia ixx=\"0.001395\""
           << "               iyy=\"0.001345\""
           << "               izz=\"0.000392\""
           << "               ixy=\"-0.000000\""
           << "               ixz=\"-0.000000\""
           << "               iyz=\"-0.000085\" />"
           << "    </inertial>"
           << "    <collision>"
           << "      <origin xyz=\"0 0 0\" rpy=\"0.0 0.0 0.0\" />"
           << "    </collision>"
           << "  </link>"
           << "  <joint name=\"head_j0\" type=\"revolute\">"
           << "    <axis xyz=\"0 -1 0\" />"
           << "    <origin xyz=\"0.0 0.0 0.07785\" rpy=\"0 0 0\" />"
           << "    <parent link=\"chest_link\"/>"
           << "    <child link=\"neck_tilt\"/>"
           << "    <limit effort=\"100\""
           << "           velocity=\"0.349\""
           << "           lower=\"-0.314\""
           << "           upper=\"0.017\"/>"
           << "  </joint>"
           << "  <gazebo reference=\"head_j0\">"
           << "    <" << it->first << ">" << value << "</" << it->first << ">"
           << "  </gazebo>"
           << "</robot>";

    tinyxml2::XMLDocument doc;
    sdf::URDF2SDF parser_;
    doc.Parse(stream.str().c_str());
    sdf::ParserConfig config_;
    tinyxml2::XMLDocument sdf_result;
    parser_.InitModelDoc(&doc, config_, &sdf_result);

    tinyxml2::XMLElement *tmp = sdf_result.FirstChildElement("sdf");
    ASSERT_NE(nullptr, tmp);

    unsigned int i;

    for (i = 0; i < it->second.size() - 1; ++i)
    {
      tmp = tmp->FirstChildElement(it->second[i].c_str());
      ASSERT_NE(nullptr, tmp);
    }

    // For the last element, check that it is exactly what we expect
    EXPECT_STREQ(tmp->FirstChild()->Value(), it->second[i].c_str());

    parser_.ListSDFExtensions();
    parser_.ListSDFExtensions("head_j0");
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, CheckFixedJointOptions_NoOption)
{
  // Convert a fixed joint with no options (i.e. it will be lumped)
  std::ostringstream fixedJointNoOptions;
  fixedJointNoOptions << "<robot name='test_robot'>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='fixed'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "</robot>";

  // Check that there are no joints in the converted SDF
  sdf::SDF fixedJointNoOptionsSDF;
  convertUrdfStrToSdf(fixedJointNoOptions.str(), fixedJointNoOptionsSDF);
  sdf::ElementPtr elem = fixedJointNoOptionsSDF.Root();
  ASSERT_NE(nullptr, elem);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_FALSE(elem->HasElement("joint"));
}

/////////////////////////////////////////////////
TEST(URDFParser, CheckFixedJointOptions_disableJointLumping)
{
  // Convert a fixed joint with disableJointLumping
  // (i.e. converted to fake revolute joint)
  std::ostringstream fixedJointDisableJointLumping;
  fixedJointDisableJointLumping << "<robot name='test_robot'>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='fixed'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "  <gazebo reference='joint1_2'>"
    << "    <disableFixedJointLumping>true</disableFixedJointLumping>"
    << "  </gazebo>"
    << "</robot>";

  // Check that there is a revolute joint in the converted SDF
  sdf::SDF fixedJointDisableJointLumpingSDF;
  convertUrdfStrToSdf(fixedJointDisableJointLumping.str(),
      fixedJointDisableJointLumpingSDF);
  sdf::ElementPtr elem = fixedJointDisableJointLumpingSDF.Root();
  ASSERT_NE(nullptr, elem);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem->HasElement("joint"));
  elem = elem->GetElement("joint");
  std::string jointType = elem->Get<std::string>("type");
  ASSERT_EQ(jointType, "revolute");
}

/////////////////////////////////////////////////
TEST(URDFParser, CheckFixedJointOptions_preserveFixedJoint)
{
  // Convert a fixed joint with only preserveFixedJoint
  // (i.e. converted to fixed joint)
  std::ostringstream fixedJointPreserveFixedJoint;
  fixedJointPreserveFixedJoint << "<robot name='test_robot'>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='fixed'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "  <gazebo reference='joint1_2'>"
    << "    <preserveFixedJoint>true</preserveFixedJoint>"
    << "  </gazebo>"
    << "</robot>";

  // Check that there is a fixed joint in the converted SDF
  sdf::SDF fixedJointPreserveFixedJointSDF;
  convertUrdfStrToSdf(fixedJointPreserveFixedJoint.str(),
      fixedJointPreserveFixedJointSDF);
  sdf::ElementPtr elem = fixedJointPreserveFixedJointSDF.Root();
  ASSERT_NE(nullptr, elem);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem->HasElement("joint"));
  elem = elem->GetElement("joint");
  std::string jointType = elem->Get<std::string>("type");
  ASSERT_EQ(jointType, "fixed");
}

/////////////////////////////////////////////////
TEST(URDFParser, CheckParserConfig_preserveFixedJoint)
{
  // Convert a fixed joint with only preserveFixedJoint
  // (i.e. converted to fixed joint)
  std::ostringstream fixedJointPreserveFixedJoint;
  fixedJointPreserveFixedJoint << "<robot name='test_robot'>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='fixed'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "</robot>";

  // Check that there is a fixed joint in the converted SDF
  sdf::SDF fixedJointPreserveFixedJointSDF;

  // Set the config option to preserve fixed joints.
  sdf::ParserConfig config;
  config.URDFSetPreserveFixedJoint(true);

  convertUrdfStrToSdf(fixedJointPreserveFixedJoint.str(),
      fixedJointPreserveFixedJointSDF, config);
  sdf::ElementPtr elem = fixedJointPreserveFixedJointSDF.Root();
  ASSERT_NE(nullptr, elem);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem->HasElement("joint"));
  elem = elem->GetElement("joint");
  std::string jointType = elem->Get<std::string>("type");
  ASSERT_EQ(jointType, "fixed");
}

/////////////////////////////////////////////////
TEST(URDFParser,
    CheckFixedJointOptions_preserveFixedJoint_and_disableJointLumping)
{
  // Convert a fixed joint with disableJointLumping and preserveFixedJoint
  // (i.e. converted to fixed joint)
  std::ostringstream fixedJointPreserveFixedJoint;
  fixedJointPreserveFixedJoint << "<robot name='test_robot'>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='fixed'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "  <gazebo reference='joint1_2'>"
    << "    <disableFixedJointLumping>true</disableFixedJointLumping>"
    << "  </gazebo>"
    << "  <gazebo reference='joint1_2'>"
    << "    <preserveFixedJoint>true</preserveFixedJoint>"
    << "  </gazebo>"
    << "</robot>";

  // Check that there is a fixed joint in the converted SDF
  sdf::SDF fixedJointPreserveFixedJointSDF;
  convertUrdfStrToSdf(fixedJointPreserveFixedJoint.str(),
      fixedJointPreserveFixedJointSDF);
  sdf::ElementPtr elem = fixedJointPreserveFixedJointSDF.Root();
  ASSERT_NE(nullptr, elem);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem->HasElement("joint"));
  elem = elem->GetElement("joint");
  std::string jointType = elem->Get<std::string>("type");
  ASSERT_EQ(jointType, "fixed");
}

/////////////////////////////////////////////////
TEST(URDFParser, ParserConfigSetPreserveFixedJointTrueOverridesGazeboTags)
{
  {
    std::ostringstream fixedJointShouldBeLumped;
    fixedJointShouldBeLumped << "<robot name='test_robot'>"
      << "  <link name='link1'>"
      << "    <inertial>"
      << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
      << "      <mass value='1.0'/>"
      << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
      << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
      << "    </inertial>"
      << "  </link>"
      << "  <link name='link2'>"
      << "    <inertial>"
      << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
      << "      <mass value='1.0'/>"
      << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
      << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
      << "    </inertial>"
      << "  </link>"
      << "  <joint name='joint1_2' type='fixed'>"
      << "    <parent link='link1' />"
      << "    <child  link='link2' />"
      << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
      << "  </joint>"
      << "  <gazebo reference='joint1_2'>"
      << "    <disableFixedJointLumping>false</disableFixedJointLumping>"
      << "  </gazebo>"
      << "</robot>";

    // Set the config option to preserve fixed joints, this should override the
    // gazebo tag <disableFixedJointLumping>false<disableFixedJointLumping/>
    // and disable lumping.
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);

    // Check that there is a fixed joint in the converted SDF
    sdf::SDF fixedJointShouldBeLumpedSDF;
    convertUrdfStrToSdf(fixedJointShouldBeLumped.str(),
        fixedJointShouldBeLumpedSDF, config);
    sdf::ElementPtr elem = fixedJointShouldBeLumpedSDF.Root();
    ASSERT_NE(nullptr, elem);
    ASSERT_TRUE(elem->HasElement("model"));
    elem = elem->GetElement("model");
    ASSERT_TRUE(elem->HasElement("joint"));
    elem = elem->GetElement("joint");
    std::string jointType = elem->Get<std::string>("type");
    ASSERT_EQ(jointType, "fixed");
  }

  {
    std::ostringstream fixedJointPreserveFixedJoint;
    fixedJointPreserveFixedJoint << "<robot name='test_robot'>"
      << "  <link name='link1'>"
      << "    <inertial>"
      << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
      << "      <mass value='1.0'/>"
      << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
      << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
      << "    </inertial>"
      << "  </link>"
      << "  <link name='link2'>"
      << "    <inertial>"
      << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
      << "      <mass value='1.0'/>"
      << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
      << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
      << "    </inertial>"
      << "  </link>"
      << "  <joint name='joint1_2' type='fixed'>"
      << "    <parent link='link1' />"
      << "    <child  link='link2' />"
      << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
      << "  </joint>"
      << "  <gazebo reference='joint1_2'>"
      << "    <preserveFixedJoint>false</preserveFixedJoint>"
      << "  </gazebo>"
      << "</robot>";

    // Set the config option to preserve fixed joints, this should override the
    // gazebo tag <preserveFixedJoint>false<preserveFixedJoint/> and disable
    // lumping.
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);

    // Check that there is a fixed joint in the converted SDF
    sdf::SDF fixedJointPreserveFixedJointSDF;
    convertUrdfStrToSdf(fixedJointPreserveFixedJoint.str(),
        fixedJointPreserveFixedJointSDF, config);
    sdf::ElementPtr elem = fixedJointPreserveFixedJointSDF.Root();
    ASSERT_NE(nullptr, elem);
    ASSERT_TRUE(elem->HasElement("model"));
    elem = elem->GetElement("model");
    ASSERT_TRUE(elem->HasElement("joint"));
    elem = elem->GetElement("joint");
    std::string jointType = elem->Get<std::string>("type");
    ASSERT_EQ(jointType, "fixed");
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, CheckFixedJointOptions_NoOption_Repeated)
{
  // Convert a fixed joint with no options (i.e. it will be lumped)
  // Converting again to make sure that the map
  // joint names ---> lumped options has been correctly reset
  std::ostringstream fixedJointNoOptions;
  fixedJointNoOptions << "<robot name='test_robot'>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='fixed'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "</robot>";

  // Check that there are no joints in the converted SDF
  sdf::SDF fixedJointNoOptionsSDF;
  convertUrdfStrToSdf(fixedJointNoOptions.str(), fixedJointNoOptionsSDF);
  sdf::ElementPtr elem = fixedJointNoOptionsSDF.Root();
  ASSERT_NE(nullptr, elem);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_FALSE(elem->HasElement("joint"));
}

/////////////////////////////////////////////////
TEST(URDFParser, CheckJointTransform)
{
  std::stringstream str;
  str.precision(16);
  str << "<robot name='test_robot'>"
    << "  <link name='world'/>"
    << "  <joint name='jointw_1' type='fixed'>"
    << "    <parent link='world' />"
    << "    <child  link='link1' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "  <link name='link1'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint1_2' type='continuous'>"
    << "    <parent link='link1' />"
    << "    <child  link='link2' />"
    << "    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 " << GZ_PI*0.5 << "' />"
    << "  </joint>"
    << "  <link name='link2'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "  <joint name='joint2_3' type='continuous'>"
    << "    <parent link='link2' />"
    << "    <child  link='link3' />"
    << "    <origin xyz='1.0 0.0 0.0' rpy='0.0 0.0 0.0' />"
    << "  </joint>"
    << "  <link name='link3'>"
    << "    <inertial>"
    << "      <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 0.0'/>"
    << "      <mass value='1.0'/>"
    << "      <inertia ixx='1.0' ixy='0.0' ixz='0.0'"
    << "               iyy='1.0' iyz='0.0' izz='1.0'/>"
    << "    </inertial>"
    << "  </link>"
    << "</robot>";

  std::string expectedSdf = R"(<sdf version="1.11">
    <model name="test_robot">
        <joint type="fixed" name="jointw_1">
            <pose relative_to="__model__">0 0 0 0 0 0</pose>
            <parent>world</parent>
            <child>link1</child>
        </joint>
        <link name="link1">
            <pose relative_to="jointw_1"/>
            <inertial>
                <pose>0 0 0 0 0 0</pose>
                <mass>1</mass>
                <inertia>
                    <ixx>1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>1</iyy>
                    <iyz>0</iyz>
                    <izz>1</izz>
                </inertia>
            </inertial>
        </link>
        <joint type="revolute" name="joint1_2">
            <pose relative_to="link1">0 0 0 0 0 1.570796326794897</pose>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit/>
                <dynamics/>
            </axis>
        </joint>
        <link name="link2">
            <pose relative_to="joint1_2"/>
            <inertial>
                <pose>0 0 0 0 0 0</pose>
                <mass>1</mass>
                <inertia>
                    <ixx>1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>1</iyy>
                    <iyz>0</iyz>
                    <izz>1</izz>
                </inertia>
            </inertial>
        </link>
        <joint type="revolute" name="joint2_3">
            <pose relative_to="link2">1 0 0 0 0 0</pose>
            <parent>link2</parent>
            <child>link3</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit/>
                <dynamics/>
            </axis>
        </joint>
        <link name="link3">
            <pose relative_to="joint2_3"/>
            <inertial>
                <pose>0 0 0 0 0 0</pose>
                <mass>1</mass>
                <inertia>
                    <ixx>1</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>1</iyy>
                    <iyz>0</iyz>
                    <izz>1</izz>
                </inertia>
            </inertial>
        </link>
    </model>
</sdf>
)";

  std::string sdfStr = convertUrdfStrToSdfStr(str.str());
  EXPECT_EQ(expectedSdf, sdfStr);
}
/////////////////////////////////////////////////
TEST(URDFParser, OutputPrecision)
{
  std::string str = R"(
    <robot name='test_robot'>
      <link name='link1'>
          <inertial>
            <mass value="0.1" />
            <origin rpy="1.570796326794895 0 0" xyz="0.123456789123456 0 0.0" />
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
          </inertial>
        </link>
    </robot>)";

  sdf::URDF2SDF parser;
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdfResult;
  parser.InitModelString(str, config_, &sdfResult);

  auto root = sdfResult.RootElement();
  auto model = root->FirstChildElement("model");
  ASSERT_NE(nullptr, model);
  auto link = model->FirstChildElement("link");
  ASSERT_NE(nullptr, link);
  auto inertial = link->FirstChildElement("inertial");
  ASSERT_NE(nullptr, inertial);
  auto pose = inertial->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->FirstChild());
  std::string poseTxt = pose->FirstChild()->Value();
  EXPECT_FALSE(poseTxt.empty());

  std::string poseValues[6];
  std::istringstream ss(poseTxt);

  for (int i = 0; i < 6; ++i)
  {
    ss >> poseValues[i];
  }

  // Check output precision
  EXPECT_EQ("0.123456789123456", poseValues[0]);
  EXPECT_EQ("1.570796326794895", poseValues[3]);

  // Check that 0 doesn't get printed as -0
  EXPECT_EQ("0", poseValues[1]);
  EXPECT_EQ("0", poseValues[2]);
  EXPECT_EQ("0", poseValues[4]);
  EXPECT_EQ("0", poseValues[5]);
}

/////////////////////////////////////////////////
TEST(URDFParser, ParseWhitespace)
{
  std::string str = R"(<robot name="test">
  <link name="link">
    <inertial>
      <mass value="0.1" />
      <origin rpy="1.570796326794895 0 0" xyz="0.123456789123456 0 0.0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <geometry>
        <sphere radius="1.0"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="1.0"/>
      </geometry>
    </collision>
  </link>
  <gazebo reference="link">
    <material>
      Gazebo/Orange
    </material>
    <mu1>
      100
    </mu1>
    <mu2>

      1000

    </mu2>
  </gazebo>
</robot>)";
  tinyxml2::XMLDocument doc;
  doc.Parse(str.c_str());

  sdf::URDF2SDF parser;
  sdf::ParserConfig config_;
  tinyxml2::XMLDocument sdfXml;
  parser.InitModelDoc(&doc, config_, &sdfXml);

  auto root = sdfXml.RootElement();
  ASSERT_NE(nullptr, root);
  auto modelElem = root->FirstChildElement("model");
  ASSERT_NE(nullptr, modelElem);
  auto linkElem = modelElem->FirstChildElement("link");
  ASSERT_NE(nullptr, linkElem);
  auto visualElem = linkElem->FirstChildElement("visual");
  ASSERT_NE(nullptr, visualElem);
  auto collisionElem = linkElem->FirstChildElement("collision");
  ASSERT_NE(nullptr, collisionElem);

  auto materialElem = visualElem->FirstChildElement("material");
  ASSERT_NE(nullptr, materialElem);
  auto scriptElem = materialElem->FirstChildElement("script");
  ASSERT_NE(nullptr, scriptElem);
  auto nameElem = scriptElem->FirstChildElement("name");
  ASSERT_NE(nullptr, nameElem);
  EXPECT_EQ("Gazebo/Orange", std::string(nameElem->GetText()));

  auto surfaceElem = collisionElem->FirstChildElement("surface");
  ASSERT_NE(nullptr, surfaceElem);
  auto frictionElem = surfaceElem->FirstChildElement("friction");
  ASSERT_NE(nullptr, frictionElem);
  auto odeElem = frictionElem->FirstChildElement("ode");
  ASSERT_NE(nullptr, odeElem);
  auto muElem = odeElem->FirstChildElement("mu");
  ASSERT_NE(nullptr, muElem);
  auto mu2Elem = odeElem->FirstChildElement("mu2");
  ASSERT_NE(nullptr, mu2Elem);

  EXPECT_EQ("100", std::string(muElem->GetText()));
  EXPECT_EQ("1000", std::string(mu2Elem->GetText()));
}

/////////////////////////////////////////////////
TEST(URDFParser, LinksWithMassIssuesIdentified)
{
  // Redirect sdfwarn output
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

  // explicitly zero mass
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link1] has a mass value of less than or equal to zero. Please "
        "ensure this link has a valid mass to prevent any missing links or "
        "joints in the resulting SDF model");
  }

  // no <inertial> block defined
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'/>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link1] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
  }

  // negative mass
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='-0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link1] has a mass value of less than or equal to zero. Please "
        "ensure this link has a valid mass to prevent any missing links or "
        "joints in the resulting SDF model");
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, ZeroMassIntermediateLinkWithNoFixedJoints)
{
  // Redirect sdfwarn output
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

  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='continuous'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    // conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, ZeroMassIntermediateLinkWithFixedParentJoint)
{
  // Redirect sdfwarn output
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

  // joint lumping and reduction occurs
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='continuous'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    // lumping and reduction occurs, no warnings should be present
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));

    // link2 visual lumps into link1
    tinyxml2::XMLElement *lumpedVisual = link->FirstChildElement("visual");
    ASSERT_NE(nullptr, lumpedVisual);
    ASSERT_NE(nullptr, lumpedVisual->Attribute("name"));
    EXPECT_EQ("link1_fixed_joint_lump__link2_visual",
        std::string(lumpedVisual->Attribute("name")));

    // link2 collision lumps into link1
    tinyxml2::XMLElement *lumpedCollision =
        link->FirstChildElement("collision");
    ASSERT_NE(nullptr, lumpedCollision);
    ASSERT_NE(nullptr, lumpedCollision->Attribute("name"));
    EXPECT_EQ("link1_fixed_joint_lump__link2_collision",
        std::string(lumpedCollision->Attribute("name")));

    // joint1_2 converted into frame, attached to link1
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frame->Attribute("name"));
    EXPECT_EQ("joint1_2", std::string(frame->Attribute("name")));
    ASSERT_NE(nullptr, frame->Attribute("attached_to"));
    EXPECT_EQ("link1", std::string(frame->Attribute("attached_to")));

    // link2 converted into frame, attached to joint1_2
    frame = frame->NextSiblingElement("frame");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frame->Attribute("name"));
    EXPECT_EQ("link2", std::string(frame->Attribute("name")));
    ASSERT_NE(nullptr, frame->Attribute("attached_to"));
    EXPECT_EQ("joint1_2", std::string(frame->Attribute("attached_to")));

    // joint2_3 will change to be relative to link1
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, joint->Attribute("name"));
    EXPECT_EQ("joint2_3", std::string(joint->Attribute("name")));
    tinyxml2::XMLElement *jointPose = joint->FirstChildElement("pose");
    ASSERT_NE(nullptr, jointPose);
    ASSERT_NE(nullptr, jointPose->Attribute("relative_to"));
    EXPECT_EQ("link1", std::string(jointPose->Attribute("relative_to")));

    // link3
    link = link->NextSiblingElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link3", std::string(link->Attribute("name")));
    tinyxml2::XMLElement *linkPose = link->FirstChildElement("pose");
    ASSERT_NE(nullptr, linkPose);
    ASSERT_NE(nullptr, linkPose->Attribute("relative_to"));
    EXPECT_EQ("joint2_3", std::string(linkPose->Attribute("relative_to")));
  }

  // Disabling lumping using gazebo tags, fails with warnings suggesting to
  // remove gazebo tags
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='continuous'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <gazebo reference='joint1_2'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    // conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint1_2], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }

  // Disabling lumping using ParserConfig::URDFPreserveFixedJoint, fails with
  // warnings suggesting to set to false
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='continuous'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    parser.InitModelString(str, config, &sdfResult);

    // conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint1_2], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, ZeroMassIntermediateLinkWithFixedChildJoint)
{
  // Redirect sdfwarn output
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

  // lumping and reduction occurs, mass of link3 lumped into link2
  // link3 and joint2_3 converted into frames
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    // lumping and reduction occurs, no warnings should be present
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));

    // mass of link3 lumped into link2, link2 not converted to frame
    link = link->NextSiblingElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link2", std::string(link->Attribute("name")));

    // link2 visual and collision remain
    tinyxml2::XMLElement *visual = link->FirstChildElement("visual");
    ASSERT_NE(nullptr, visual);
    ASSERT_NE(nullptr, visual->Attribute("name"));
    EXPECT_EQ("link2_visual", std::string(visual->Attribute("name")));
    tinyxml2::XMLElement *collision = link->FirstChildElement("collision");
    ASSERT_NE(nullptr, collision);
    ASSERT_NE(nullptr, collision->Attribute("name"));
    EXPECT_EQ("link2_collision", std::string(collision->Attribute("name")));

    // joint1_2
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, joint->Attribute("name"));
    EXPECT_EQ("joint1_2", std::string(joint->Attribute("name")));

    // joint2_3 converted into a frame
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frame->Attribute("name"));
    EXPECT_EQ("joint2_3", std::string(frame->Attribute("name")));
    ASSERT_NE(nullptr, frame->Attribute("attached_to"));
    EXPECT_EQ("link2", std::string(frame->Attribute("attached_to")));

    // link3 converted into a frame
    frame = frame->NextSiblingElement("frame");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frame->Attribute("name"));
    EXPECT_EQ("link3", std::string(frame->Attribute("name")));
    ASSERT_NE(nullptr, frame->Attribute("attached_to"));
    EXPECT_EQ("joint2_3", std::string(frame->Attribute("attached_to")));
  }

  // turn off lumping with gazebo tag, conversion fails with warnings to suggest
  // removing gazebo tags
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <gazebo reference='joint2_3'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed child joint[joint2_3], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }

  // Disabling lumping using ParserConfig::URDFPreserveFixedJoint, fails with
  // warnings suggesting to set to false
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    parser.InitModelString(str, config, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed child joint[joint2_3], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, ZeroMassIntermediateLinkWithAllFixedJointsButLumpingOff)
{
  // Redirect sdfwarn output
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

  // turn off all lumping with gazebo tags, conversion fails with suggestions to
  // remove gazebo tags
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <gazebo reference='joint1_2'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
        <gazebo reference='joint2_3'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint1_2], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed child joint[joint2_3], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }

  // Disabling lumping using ParserConfig::URDFPreserveFixedJoint, fails with
  // warnings suggesting to set to false
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    parser.InitModelString(str, config, &sdfResult);

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint1_2], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed child joint[joint2_3], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);

    // no other frames and joints
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    EXPECT_EQ(nullptr, joint);
  }
}

/////////////////////////////////////////////////
TEST(URDFParser, ZeroMassLeafLink)
{
  // Redirect sdfwarn output
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

  // lumping and reduction occurs
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    // lumping and reduction occurs, no warnings should be present
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link3] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link3] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));

    // link2
    link = link->NextSiblingElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link2", std::string(link->Attribute("name")));

    // joint1_2
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, joint->Attribute("name"));
    EXPECT_EQ("joint1_2", std::string(joint->Attribute("name")));

    // link3 visual and collision lumped into link2
    tinyxml2::XMLElement *visual = link->FirstChildElement("visual");
    ASSERT_NE(nullptr, visual);
    ASSERT_NE(nullptr, visual->Attribute("name"));
    EXPECT_EQ("link2_fixed_joint_lump__link3_visual",
        std::string(visual->Attribute("name")));
    tinyxml2::XMLElement *collision = link->FirstChildElement("collision");
    ASSERT_NE(nullptr, collision);
    ASSERT_NE(nullptr, collision->Attribute("name"));
    EXPECT_EQ("link2_fixed_joint_lump__link3_collision",
        std::string(collision->Attribute("name")));

    // joint2_3 converted into a frame
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frame->Attribute("name"));
    EXPECT_EQ("joint2_3", std::string(frame->Attribute("name")));
    ASSERT_NE(nullptr, frame->Attribute("attached_to"));
    EXPECT_EQ("link2", std::string(frame->Attribute("attached_to")));

    // link3 converted into a frame
    frame = frame->NextSiblingElement("frame");
    ASSERT_NE(nullptr, frame);
    ASSERT_NE(nullptr, frame->Attribute("name"));
    EXPECT_EQ("link3", std::string(frame->Attribute("name")));
    ASSERT_NE(nullptr, frame->Attribute("attached_to"));
    EXPECT_EQ("joint2_3", std::string(frame->Attribute("attached_to")));
  }

  // lumping and reduction turned off with gazebo tag, expect link3 and joint2_3
  // to be ignored and warnings suggesting to remove gazebo tag
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <gazebo reference='joint2_3'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig defaultConfig;
    parser.InitModelString(str, defaultConfig, &sdfResult);

    // joint2_3 and link3 will be ignored, and warnings suggesting to remove
    // gazebo tags
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link3] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint2_3], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint2_3] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link3] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));

    // link2
    link = link->NextSiblingElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link2", std::string(link->Attribute("name")));

    // joint1_2
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, joint->Attribute("name"));
    EXPECT_EQ("joint1_2", std::string(joint->Attribute("name")));

    // link3 visual and collision not lumped into link2
    tinyxml2::XMLElement *visual = link->FirstChildElement("visual");
    EXPECT_EQ(nullptr, visual);
    tinyxml2::XMLElement *collision = link->FirstChildElement("collision");
    EXPECT_EQ(nullptr, collision);

    // joint2_3 ignored
    joint = joint->NextSiblingElement("joint");
    EXPECT_EQ(nullptr, joint);
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
  }

  // ParserConfig::URDFSetPreserveFixedJoint set to true, expect link3 and
  // joint2_3 to be ignored and warnings suggesting to remove gazebo tag
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string str = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <link name='link3'> <!-- zero mass link -->
          <visual>
            <origin rpy="3 -0 0" xyz="0 0 0"/>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </visual>
          <collision>
            <geometry>
              <sphere radius="2.0"/>
            </geometry>
          </collision>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::URDF2SDF parser;
    tinyxml2::XMLDocument sdfResult;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    parser.InitModelString(str, config, &sdfResult);

    // joint2_3 and link3 will be ignored, and warnings suggesting to remove
    // gazebo tags
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link3] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint2_3], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint2_3] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link3] is not modeled in sdf");

    tinyxml2::XMLElement *sdf = sdfResult.FirstChildElement("sdf");
    ASSERT_NE(nullptr, sdf);
    tinyxml2::XMLElement *model = sdf->FirstChildElement("model");
    ASSERT_NE(nullptr, model);
    tinyxml2::XMLElement *link = model->FirstChildElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link1", std::string(link->Attribute("name")));

    // link2
    link = link->NextSiblingElement("link");
    ASSERT_NE(nullptr, link);
    ASSERT_NE(nullptr, link->Attribute("name"));
    EXPECT_EQ("link2", std::string(link->Attribute("name")));

    // joint1_2
    tinyxml2::XMLElement *joint = model->FirstChildElement("joint");
    ASSERT_NE(nullptr, joint);
    ASSERT_NE(nullptr, joint->Attribute("name"));
    EXPECT_EQ("joint1_2", std::string(joint->Attribute("name")));

    // link3 visual and collision not lumped into link2
    tinyxml2::XMLElement *visual = link->FirstChildElement("visual");
    EXPECT_EQ(nullptr, visual);
    tinyxml2::XMLElement *collision = link->FirstChildElement("collision");
    EXPECT_EQ(nullptr, collision);

    // joint2_3 ignored
    joint = joint->NextSiblingElement("joint");
    EXPECT_EQ(nullptr, joint);
    link = link->NextSiblingElement("link");
    EXPECT_EQ(nullptr, link);
    tinyxml2::XMLElement *frame = model->FirstChildElement("frame");
    EXPECT_EQ(nullptr, frame);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
