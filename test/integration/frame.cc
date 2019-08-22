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

#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

////////////////////////////////////////
// Test parsing a model element that has a frame element
TEST(Frame, ModelFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <frame name='mframe'>"
    << "    <pose relative_to='/world'>1 1 0 0 0 0</pose>"
    << "  </frame>"
    << "  <pose relative_to='mframe'>1 0 0 0 0 0</pose>"
    << "  <link name='link'/>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "my_model");

  // model frame
  EXPECT_TRUE(modelElem->HasElement("frame"));
  sdf::ElementPtr frameElem = modelElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "mframe");

  // model frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("relative_to"));
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "/world");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
            ignition::math::Pose3d(1, 1, 0, 0, 0, 0));

  // model pose
  EXPECT_TRUE(modelElem->HasElement("pose"));
  sdf::ElementPtr modelPoseElem = modelElem->GetElement("pose");
  EXPECT_TRUE(modelPoseElem->HasAttribute("relative_to"));
  EXPECT_EQ(modelPoseElem->Get<std::string>("relative_to"), "mframe");
  EXPECT_EQ(modelPoseElem->Get<ignition::math::Pose3d>(),
            ignition::math::Pose3d(1, 0, 0, 0, 0, 0));

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");
}

////////////////////////////////////////
// Test parsing a model element with an empty frame element
TEST(Frame, FrameDefaultPose)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <frame name='mframe'/>"
    << "  <link name='link'/>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "my_model");

  // model frame
  EXPECT_TRUE(modelElem->HasElement("frame"));
  sdf::ElementPtr frameElem = modelElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "mframe");

  // model frame pose
  EXPECT_TRUE(!frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("relative_to"));
  EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
            ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");
}

////////////////////////////////////////
// Test parsing a model element with no frames - for backward compatibility
TEST(Frame, NoFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='link'/>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed->Root()->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "my_model");

  {
    // model frame
    EXPECT_TRUE(!modelElem->HasElement("frame"));
    sdf::ElementPtr frameElem = modelElem->GetElement("frame");
    EXPECT_TRUE(frameElem->HasAttribute("name"));
    EXPECT_EQ(frameElem->Get<std::string>("name"), "");

    // model frame pose
    EXPECT_TRUE(!frameElem->HasElement("pose"));
    sdf::ElementPtr poseElem = frameElem->GetElement("pose");
    EXPECT_TRUE(poseElem->HasAttribute("relative_to"));
    EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
              ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  }

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  {
    // link pose
    EXPECT_TRUE(!linkElem->HasElement("pose"));
    sdf::ElementPtr poseElem = linkElem->GetElement("pose");
    EXPECT_TRUE(poseElem->HasAttribute("relative_to"));
    EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
              ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  }
}

////////////////////////////////////////
// Test parsing nested model states
TEST(Frame, StateFrame)
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='my_model'>"
    << "  <frame name='mframe'>"
    << "    <pose relative_to='/world'>1 0 2 0 0 0</pose>"
    << "  </frame>"
    << "  <pose relative_to='mframe'>3 3 9 0 0 0</pose>"
    << "  <link name='my_link'>"
    << "    <pose relative_to='lframe'>111 3 0 0 0 0</pose>"
    << "  </link>"
    << "</model>"
    << "<light name='my_light'>"
    << "    <pose relative_to='lframe'>99 0 22 0 0 0</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(sdfStr.str(), sdfParsed));

  // load the state sdf
  EXPECT_TRUE(sdfParsed->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");

  EXPECT_TRUE(stateElem->HasElement("model"));
  sdf::ElementPtr modelStateElem = stateElem->GetElement("model");

  // model
  EXPECT_TRUE(modelStateElem->HasAttribute("name"));
  EXPECT_EQ(modelStateElem->Get<std::string>("name"), "my_model");

  {
    // model frame
    EXPECT_TRUE(modelStateElem->HasElement("frame"));
    sdf::ElementPtr frameElem = modelStateElem->GetElement("frame");
    EXPECT_TRUE(frameElem->HasAttribute("name"));
    EXPECT_EQ(frameElem->Get<std::string>("name"), "mframe");

    // model frame pose
    EXPECT_TRUE(frameElem->HasElement("pose"));
    sdf::ElementPtr poseElem = frameElem->GetElement("pose");
    EXPECT_TRUE(poseElem->HasAttribute("relative_to"));
    EXPECT_EQ(poseElem->Get<std::string>("relative_to"), "/world");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
              ignition::math::Pose3d(1, 0, 2, 0, 0, 0));

    // model pose
    EXPECT_TRUE(modelStateElem->HasElement("pose"));
    sdf::ElementPtr modelPoseElem = modelStateElem->GetElement("pose");
    EXPECT_TRUE(modelPoseElem->HasAttribute("relative_to"));
    EXPECT_EQ(modelPoseElem->Get<std::string>("relative_to"), "mframe");
    EXPECT_EQ(modelPoseElem->Get<ignition::math::Pose3d>(),
              ignition::math::Pose3d(3, 3, 9, 0, 0, 0));
  }

  // link
  EXPECT_TRUE(modelStateElem->HasElement("link"));
  sdf::ElementPtr linkStateElem = modelStateElem->GetElement("link");
  EXPECT_TRUE(linkStateElem->HasAttribute("name"));
  EXPECT_EQ(linkStateElem->Get<std::string>("name"), "my_link");

  {
    // link pose
    EXPECT_TRUE(linkStateElem->HasElement("pose"));
    sdf::ElementPtr linkPoseElem = linkStateElem->GetElement("pose");
    EXPECT_TRUE(linkPoseElem->HasAttribute("relative_to"));
    EXPECT_EQ(linkPoseElem->Get<std::string>("relative_to"), "lframe");
    EXPECT_EQ(linkPoseElem->Get<ignition::math::Pose3d>(),
              ignition::math::Pose3d(111, 3, 0, 0, 0, 0));
  }

  EXPECT_TRUE(stateElem->HasElement("light"));
  sdf::ElementPtr lightStateElem = stateElem->GetElement("light");

  // light
  EXPECT_TRUE(lightStateElem->HasAttribute("name"));
  EXPECT_EQ(lightStateElem->Get<std::string>("name"), "my_light");

  {
    // light pose
    EXPECT_TRUE(lightStateElem->HasElement("pose"));
    sdf::ElementPtr lightPoseElem = lightStateElem->GetElement("pose");
    EXPECT_TRUE(lightPoseElem->HasAttribute("relative_to"));
    EXPECT_EQ(lightPoseElem->Get<std::string>("relative_to"), "lframe");
    EXPECT_EQ(lightPoseElem->Get<ignition::math::Pose3d>(),
              ignition::math::Pose3d(99, 0, 22, 0, 0, 0));
  }
}

////////////////////////////////////////
// Test parsing a include element that has a pose element
TEST(Frame, IncludeRelativeTo)
{
  const std::string MODEL_PATH =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "integration",
                            "model", "box");

  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "<include>"
    << "  <name>my_model</name>"
    << "  <pose relative_to='/world'>5 -2 1 0 0 0</pose>"
    << "  <uri>" + MODEL_PATH +"</uri>"
    << "</include>"
    << "</world>"
    << "</sdf>";


  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // model
  EXPECT_TRUE(sdfParsed->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed->Root()->GetElement("world");

  EXPECT_TRUE(worldElem->HasElement("model"));
  sdf::ElementPtr modelElem = worldElem->GetElement("model");
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  EXPECT_EQ(modelElem->Get<std::string>("name"), "my_model");

  // model pose
  EXPECT_TRUE(modelElem->HasElement("pose"));
  sdf::ElementPtr modelPoseElem = modelElem->GetElement("pose");
  EXPECT_TRUE(modelPoseElem->HasAttribute("relative_to"));
  EXPECT_EQ(modelPoseElem->Get<std::string>("relative_to"), "/world");
  EXPECT_EQ(modelPoseElem->Get<ignition::math::Pose3d>(),
            ignition::math::Pose3d(5, -2, 1, 0, 0, 0));
}
