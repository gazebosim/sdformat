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

#include "sdf/sdf.hh"
#include "sdf/parser_urdf.hh"

class URDFParser : public ::testing::Test
{
  public:
      TiXmlDocument get_empty()
      {
          return TiXmlDocument();
      }

      std::string get_minimal_urdf_txt()
      {
          std::ostringstream stream;
          stream << "<robot name='test_robot'>"
                 << "<link name='link1' />"
                 << "</robot>";
          return stream.str();
      }

      std::string get_minimal_sdf_txt()
      {
          std::ostringstream stream;
          // Use hard-coded "1.4" for version string
          // until parser_urdf.cc exports version "1.5"
          // see `sdf->SetAttribute("version", "1.4");`
          // in URDF2SDF::InitModelString()
          stream << "<sdf version='" << "1.4" << "'>"
                 << "<model name='test_robot' />"
                 << "</sdf>";
          return stream.str();
      }

      TiXmlDocument convert_str_to_xml(const std::string& urdf)
      {
          TiXmlDocument tmp;
          tmp.Parse(urdf.c_str());
          return tmp;
      }

      void convert_urdf_str_to_sdf(const std::string& urdf, sdf::SDF& _sdf)
      {
          TiXmlDocument sdf_result = parser_.InitModelString(urdf);
          std::string sdf_result_string;
          sdf_result_string << sdf_result;
          _sdf.SetFromString(sdf_result_string);
          return;
      }

    protected:
        sdf::URDF2SDF parser_;
};

/* By design, errors are only reported in std output */
TEST_F(URDFParser, InitModelDoc_EmptyDoc_NoThrow)
{
   ASSERT_NO_THROW(
     TiXmlDocument doc = get_empty();
     TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
   );
}

TEST_F(URDFParser, InitModelDoc_BasicModel_NoThrow)
{
    ASSERT_NO_THROW(
      std::string urdf = get_minimal_urdf_txt();
      TiXmlDocument doc = convert_str_to_xml(urdf);
      TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
    );
}

TEST_F(URDFParser, ParseResults_BasicModel_ParseEqualToModel)
{
   // URDF -> SDF
   std::string urdf = get_minimal_urdf_txt();
   TiXmlDocument doc = convert_str_to_xml(urdf);
   TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
   std::string sdf_result_str;
   sdf_result_str << sdf_result;

   // SDF -> SDF
   std::string sdf = get_minimal_sdf_txt();
   TiXmlDocument sdf_doc = convert_str_to_xml(sdf);
   std::string sdf_same_result_str;
   sdf_same_result_str << sdf_doc;

   ASSERT_EQ(sdf_same_result_str, sdf_result_str);
}

TEST_F(URDFParser, CheckFixedJointOptions_NoOption)
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
  convert_urdf_str_to_sdf(fixedJointNoOptions.str(), fixedJointNoOptionsSDF);
  sdf::ElementPtr elem = fixedJointNoOptionsSDF.Root();
  ASSERT_TRUE(elem != nullptr);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_FALSE(elem->HasElement("joint"));
}

TEST_F(URDFParser, CheckFixedJointOptions_disableJointLumping)
{
  // Convert a fixed joint with disableJointLumping (i.e. converted to fake revolute joint)
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
  convert_urdf_str_to_sdf(fixedJointDisableJointLumping.str(), fixedJointDisableJointLumpingSDF);
  sdf::ElementPtr elem = fixedJointDisableJointLumpingSDF.Root();
  ASSERT_TRUE(elem != nullptr);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem->HasElement("joint"));
  elem = elem->GetElement("joint");
  std::string jointType = elem->Get<std::string>("type");
  ASSERT_EQ(jointType, "revolute");
}

TEST_F(URDFParser, CheckFixedJointOptions_preserveFixedJoint)
{
  // Convert a fixed joint with disableJointLumping and preserveFixedJoint (i.e. converted to fixed joint)
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
  convert_urdf_str_to_sdf(fixedJointPreserveFixedJoint.str(), fixedJointPreserveFixedJointSDF);
  sdf::ElementPtr elem = fixedJointPreserveFixedJointSDF.Root();
  ASSERT_TRUE(elem != nullptr);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem->HasElement("joint"));
  elem = elem->GetElement("joint");
  std::string jointType = elem->Get<std::string>("type");
  ASSERT_EQ(jointType, "fixed");
}

TEST_F(URDFParser, CheckFixedJointOptions_NoOption_Repeated)
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
  convert_urdf_str_to_sdf(fixedJointNoOptions.str(), fixedJointNoOptionsSDF);
  sdf::ElementPtr elem = fixedJointNoOptionsSDF.Root();
  ASSERT_TRUE(elem != nullptr);
  ASSERT_TRUE(elem->HasElement("model"));
  elem = elem->GetElement("model");
  ASSERT_FALSE(elem->HasElement("joint"));
}
