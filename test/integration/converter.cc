/*
 * Copyright 2015 Open Source Robotics Foundation
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
#include "sdf/Converter.hh"

#include "test_config.h"

const std::string CONVERT_DOC =
  std::string(PROJECT_SOURCE_PATH) + "/sdf/1.6/1_5.convert";

/////////////////////////////////////////////////
/// Test conversion of imu in 1.5 to 1.6
TEST(ConverterIntegration, IMU_15_to_16)
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
  convertXmlDoc.LoadFile(CONVERT_DOC);
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
/// Test conversion using the parser sdf file converter interface.
TEST(ConverterIntegration, ParserFileConverter)
{
  std::string filename = std::string(PROJECT_SOURCE_PATH) +
    "/test/integration/audio.sdf";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertFile(filename, "1.6", sdf));

  sdf::ElementPtr rootElem = sdf->Root();
  ASSERT_TRUE(rootElem != NULL);
  EXPECT_EQ(rootElem->Get<std::string>("version"), "1.6");

  sdf::ElementPtr modelElem = rootElem->GetElement("model");
  ASSERT_TRUE(modelElem != NULL);
  EXPECT_EQ(modelElem->Get<std::string>("name"), "full_audio_parameters");

  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_TRUE(linkElem != NULL);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  sdf::ElementPtr collElem = linkElem->GetElement("collision");
  ASSERT_TRUE(collElem != NULL);
  EXPECT_EQ(collElem->Get<std::string>("name"), "collision");

  sdf::ElementPtr sinkElem = linkElem->GetElement("audio_sink");
  ASSERT_TRUE(sinkElem != NULL);

  sdf::ElementPtr sourceElem = linkElem->GetElement("audio_source");
  ASSERT_TRUE(sourceElem != NULL);
}

/////////////////////////////////////////////////
/// Test conversion using the parser sdf string converter interface.
TEST(ConverterIntegration, ParserStringConverter)
{
  // The gravity and magnetic_field in 1.5 format
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <magnetic_field>1 2 3</magnetic_field>
    </physics>
  </world>
</sdf>)";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertString(xmlString, "1.6", sdf));
  ASSERT_TRUE(sdf->Root() != NULL);
  EXPECT_EQ(sdf->Root()->GetName(), "sdf");

  sdf::ElementPtr worldElem = sdf->Root()->GetElement("world");
  ASSERT_TRUE(worldElem != NULL);
  EXPECT_EQ(worldElem->Get<std::string>("name"), "default");

  sdf::ElementPtr physicsElem = worldElem->GetElement("physics");
  ASSERT_TRUE(physicsElem != NULL);
  EXPECT_EQ(physicsElem->Get<std::string>("name"), "default_physics");
  EXPECT_EQ(physicsElem->Get<std::string>("type"), "ode");

  sdf::ElementPtr gravityElem = worldElem->GetElement("gravity");
  ASSERT_TRUE(gravityElem != NULL);
  EXPECT_EQ(gravityElem->Get<ignition::math::Vector3d>(),
      ignition::math::Vector3d(0, 0, -9.8));

  sdf::ElementPtr magElem = worldElem->GetElement("magnetic_field");
  ASSERT_TRUE(magElem != NULL);
  EXPECT_EQ(magElem->Get<ignition::math::Vector3d>(),
      ignition::math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
/// Test conversion of gravity, magnetic_field in 1.5 to 1.6
TEST(ConverterIntegration, World_15_to_16)
{
  // The gravity and magnetic_field in 1.5 format
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <magnetic_field>1 2 3</magnetic_field>
    </physics>
  </world>
</sdf>)";

  TiXmlDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  // Convert
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.LoadFile(CONVERT_DOC);
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  // Check some basic elements
  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "sdf");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "world");
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "physics");

  // Get the gravity
  TiXmlElement *gravityElem = convertedElem->NextSiblingElement("gravity");
  ASSERT_TRUE(gravityElem != NULL);
  EXPECT_STREQ(gravityElem->GetText(), "0 0 -9.8");

  // Get the magnetic_field
  TiXmlElement *magneticFieldElem =
    convertedElem->NextSiblingElement("magnetic_field");
  ASSERT_TRUE(magneticFieldElem != NULL);
  EXPECT_STREQ(magneticFieldElem->GetText(), "1 2 3");
}
