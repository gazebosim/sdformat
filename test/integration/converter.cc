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

#include <array>
#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

/// \brief Use different sdf versions for ParserStringConverter Test.
void ParserStringConverter(const std::string &_version);

/////////////////////////////////////////////////
/// Test conversion using the parser sdf file converter interface.
TEST(ConverterIntegration, ParserFileConverter)
{
  const auto filename = sdf::testing::TestFile("integration", "audio.sdf");

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertFile(filename, "1.6", sdf));

  sdf::ElementPtr rootElem = sdf->Root();
  ASSERT_NE(nullptr, rootElem);
  EXPECT_EQ("1.6", rootElem->Get<std::string>("version"));
  EXPECT_EQ("1.4", sdf->OriginalVersion());
  EXPECT_EQ("1.4", rootElem->OriginalVersion());

  sdf::ElementPtr modelElem = rootElem->GetElement("model");
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ(modelElem->Get<std::string>("name"), "full_audio_parameters");
  EXPECT_EQ("1.4", modelElem->OriginalVersion());

  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");
  EXPECT_EQ("1.4", linkElem->OriginalVersion());

  sdf::ElementPtr collElem = linkElem->GetElement("collision");
  ASSERT_NE(nullptr, collElem);
  EXPECT_EQ(collElem->Get<std::string>("name"), "collision");
  EXPECT_EQ("1.4", collElem->OriginalVersion());

  sdf::ElementPtr sinkElem = linkElem->GetElement("audio_sink");
  ASSERT_NE(nullptr, sinkElem);
  EXPECT_EQ("1.4", sinkElem->OriginalVersion());

  sdf::ElementPtr sourceElem = linkElem->GetElement("audio_source");
  ASSERT_NE(nullptr, sourceElem);
  EXPECT_EQ("1.4", sourceElem->OriginalVersion());
}

/////////////////////////////////////////////////
/// Convert to a previous SDF version
TEST(ConverterIntegration, convertFileToNotLatestVersion)
{
  const auto filename = sdf::testing::TestFile(
      "integration", "audio.sdf");

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertFile(filename, "1.5", sdf));

  sdf::ElementPtr rootElem = sdf->Root();
  ASSERT_NE(nullptr, rootElem);
  EXPECT_EQ("1.5", rootElem->Get<std::string>("version"));
  EXPECT_EQ("1.4", sdf->OriginalVersion());
  EXPECT_EQ("1.4", rootElem->OriginalVersion());
}

/////////////////////////////////////////////////
/// Test conversion using the parser sdf string converter interface.
TEST(ConverterIntegration, ParserStringConverter)
{
  ParserStringConverter("1.5");
}

TEST(ConverterIntegration, ParserStringConverterFrom14)
{
  ParserStringConverter("1.4");
}

void ParserStringConverter(const std::string &_version)
{
  // The gravity and magnetic_field in 1.5 format
  std::string xmlString = R"(
<?xml version="1.0" ?>
<sdf version=")" + _version + R"(">
  <world name="default">
    <physics type="ode">
      <gravity>1 0 -9.8</gravity>
      <magnetic_field>1 2 3</magnetic_field>
    </physics>
  </world>
</sdf>)";

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  EXPECT_TRUE(sdf::convertString(xmlString, "1.6", sdf));
  ASSERT_NE(nullptr, sdf->Root());
  EXPECT_EQ(sdf->Root()->GetName(), "sdf");
  EXPECT_EQ("1.6", sdf->Root()->Get<std::string>("version"));
  EXPECT_EQ(_version, sdf->OriginalVersion());
  EXPECT_EQ(_version, sdf->Root()->OriginalVersion());

  sdf::ElementPtr worldElem = sdf->Root()->GetElement("world");
  ASSERT_NE(nullptr, worldElem);
  EXPECT_EQ(worldElem->Get<std::string>("name"), "default");
  EXPECT_EQ(_version, worldElem->OriginalVersion());

  sdf::ElementPtr physicsElem = worldElem->GetElement("physics");
  ASSERT_NE(nullptr, physicsElem);
  EXPECT_EQ(physicsElem->Get<std::string>("name"), "default_physics");
  EXPECT_EQ(physicsElem->Get<std::string>("type"), "ode");
  EXPECT_EQ(_version, physicsElem->OriginalVersion());

  // gravity and magnetic_field should have been moved from physics to world
  EXPECT_FALSE(physicsElem->HasElement("gravity"));
  EXPECT_FALSE(physicsElem->HasElement("magnetic_field"));

  sdf::ElementPtr gravityElem = worldElem->GetElement("gravity");
  ASSERT_NE(nullptr, gravityElem);
  EXPECT_EQ(gravityElem->Get<ignition::math::Vector3d>(),
            ignition::math::Vector3d(1, 0, -9.8));
  EXPECT_EQ(_version, gravityElem->OriginalVersion());

  sdf::ElementPtr magElem = worldElem->GetElement("magnetic_field");
  ASSERT_NE(nullptr, magElem);
  EXPECT_EQ(magElem->Get<ignition::math::Vector3d>(),
            ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(_version, magElem->OriginalVersion());
}

/////////////////////////////////////////////////
/// Test conversion unflattened world in 1.7 to 1.8
TEST(ConverterIntegration, UnflattenConversion)
{
  const std::string filename =
  sdf::testing::SourceFile("test", "sdf",
                          "flattened_test_nested_model_with_frames.sdf");

  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);

  ASSERT_TRUE(sdf::convertFile(filename, "1.8", sdf));

  ASSERT_NE(nullptr, sdf->Root());
  EXPECT_EQ(sdf->Root()->GetName(), "sdf");
  EXPECT_EQ("1.8", sdf->Root()->Get<std::string>("version"));
  EXPECT_EQ("1.7", sdf->OriginalVersion());
  EXPECT_EQ("1.7", sdf->Root()->OriginalVersion());

  sdf::ElementPtr worldElem = sdf->Root()->GetElement("world");
  ASSERT_NE(nullptr, worldElem);
  EXPECT_EQ(worldElem->Get<std::string>("name"), "default");

  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  ASSERT_NE(nullptr, modelElem);
  EXPECT_EQ(modelElem->Get<std::string>("name"),
            "test_nested_model_with_frames");

  sdf::ElementPtr nestModelElem = modelElem->GetElement("model");
  ASSERT_NE(nullptr, nestModelElem);
  EXPECT_EQ(nestModelElem->Get<std::string>("name"), "test_model_with_frames");
  EXPECT_EQ(nestModelElem->Get<std::string>("canonical_link"), "L1");

  sdf::ElementPtr poseElem = nestModelElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "__model__");

  sdf::ElementPtr frameElem = nestModelElem->GetElement("frame");
  ASSERT_NE(nullptr, frameElem);
  EXPECT_EQ(frameElem->Get<std::string>("name"), "F1");
  EXPECT_EQ(frameElem->Get<std::string>("attached_to"), "__model__");

  frameElem = frameElem->GetNextElement("frame");
  ASSERT_NE(nullptr, frameElem);
  EXPECT_EQ(frameElem->Get<std::string>("name"), "F2");
  EXPECT_EQ(frameElem->Get<std::string>("attached_to"), "__model__");
  poseElem = frameElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "F1");

  sdf::ElementPtr linkElem = frameElem->GetNextElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "L1");
  poseElem = linkElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "F1");
  poseElem = linkElem->GetElement("visual")->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "F2");
  poseElem = linkElem->GetElement("collision")->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "__model__");

  linkElem = linkElem->GetNextElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "L2");
  poseElem = linkElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "F1");

  linkElem = linkElem->GetNextElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "L3");
  poseElem = linkElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "L2");

  linkElem = linkElem->GetNextElement("link");
  ASSERT_NE(nullptr, linkElem);
  EXPECT_EQ(linkElem->Get<std::string>("name"), "L4");
  poseElem = linkElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "__model__");

  sdf::ElementPtr jointElem = linkElem->GetNextElement("joint");
  ASSERT_NE(nullptr, jointElem);
  EXPECT_EQ(jointElem->Get<std::string>("name"), "J1");
  poseElem = jointElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "L1");
  EXPECT_EQ(jointElem->Get<std::string>("parent"), "L1");
  EXPECT_EQ(jointElem->Get<std::string>("child"), "L2");
  sdf::ElementPtr xyzElem = jointElem->GetElement("axis")->GetElement("xyz");
  ASSERT_NE(nullptr, xyzElem);
  EXPECT_EQ(xyzElem->Get<std::string>("expressed_in"), "F2");
  xyzElem = jointElem->GetElement("axis2")->GetElement("xyz");
  ASSERT_NE(nullptr, xyzElem);
  EXPECT_EQ(xyzElem->Get<std::string>("expressed_in"), "F2");

  jointElem = jointElem->GetNextElement("joint");
  ASSERT_NE(nullptr, jointElem);
  EXPECT_EQ(jointElem->Get<std::string>("name"), "J2");
  EXPECT_EQ(jointElem->Get<std::string>("parent"), "L2");
  EXPECT_EQ(jointElem->Get<std::string>("child"), "L3");
  xyzElem = jointElem->GetElement("axis")->GetElement("xyz");
  ASSERT_NE(nullptr, xyzElem);
  EXPECT_EQ(xyzElem->Get<std::string>("expressed_in"), "__model__");

  jointElem = jointElem->GetNextElement("joint");
  ASSERT_NE(nullptr, jointElem);
  EXPECT_EQ(jointElem->Get<std::string>("name"), "J3");
  poseElem = jointElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "__model__");
  EXPECT_EQ(jointElem->Get<std::string>("parent"), "L3");
  EXPECT_EQ(jointElem->Get<std::string>("child"), "L4");

  nestModelElem = jointElem->GetNextElement("model");
  ASSERT_NE(nullptr, nestModelElem);
  EXPECT_EQ(nestModelElem->Get<std::string>("name"), "M2");
  poseElem = nestModelElem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "F1");
}
