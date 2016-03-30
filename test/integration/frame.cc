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
#include <string>
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
    << "    <pose frame='/world'>1 1 0 0 0 0</pose>"
    << "  </frame>"
    << "  <pose frame='mframe'>1 0 0 0 0 0</pose>"
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
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 1, 0, 0, 0, 0));

  // model pose
  EXPECT_TRUE(modelElem->HasElement("pose"));
  sdf::ElementPtr modelPoseElem = modelElem->GetElement("pose");
  EXPECT_TRUE(modelPoseElem->HasAttribute("frame"));
  EXPECT_EQ(modelPoseElem->Get<std::string>("frame"), "mframe");
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
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "");
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
    EXPECT_TRUE(poseElem->HasAttribute("frame"));
    EXPECT_EQ(poseElem->Get<std::string>("frame"), "");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  }

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  {
    // link frame
    EXPECT_TRUE(!linkElem->HasElement("frame"));
    sdf::ElementPtr frameElem = linkElem->GetElement("frame");
    EXPECT_TRUE(frameElem->HasAttribute("name"));
    EXPECT_EQ(frameElem->Get<std::string>("name"), "");

    // link frame pose
    EXPECT_TRUE(!frameElem->HasElement("pose"));
    sdf::ElementPtr poseElem = frameElem->GetElement("pose");
    EXPECT_TRUE(poseElem->HasAttribute("frame"));
    EXPECT_EQ(poseElem->Get<std::string>("frame"), "");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  }
}

////////////////////////////////////////
// Test parsing a link element that has a frame element
TEST(Frame, LinkFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='link'>"
    << "    <frame name='lframe'>"
    << "      <pose frame='model'>1 0 2 0 0 0</pose>"
    << "    </frame>"
    << "    <pose frame='lframe'>0 5 0 0 0 0</pose>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // link frame
  EXPECT_TRUE(linkElem->HasElement("frame"));
  sdf::ElementPtr frameElem = linkElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "lframe");

  // link frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "model");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 0, 2, 0, 0, 0));

  // link pose
  EXPECT_TRUE(linkElem->HasElement("pose"));
  sdf::ElementPtr linkPoseElem = linkElem->GetElement("pose");
  EXPECT_TRUE(linkPoseElem->HasAttribute("frame"));
  EXPECT_EQ(linkPoseElem->Get<std::string>("frame"), "lframe");
  EXPECT_EQ(linkPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 5, 0, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a joint element that has a frame element
TEST(Frame, JointFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='parent'/>"
    << "  <link name='child'/>"
    << "  <joint name='revjoint' type='revolute'>"
    << "    <parent>parent</parent>"
    << "    <child>child</child>"
    << "    <axis>"
    << "      <xyz>1 0 0</xyz>"
    << "    </axis>"
    << "    <frame name='jframe'>"
    << "      <pose frame='child'>0 0 1 0 0 0</pose>"
    << "    </frame>"
    << "    <pose frame='jframe'>0 2 1 0 0 0</pose>"
    << "  </joint>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "parent");
  linkElem = linkElem->GetNextElement("link");
  EXPECT_TRUE(linkElem != NULL);
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "child");

  // joint
  EXPECT_TRUE(modelElem->HasElement("joint"));
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem->HasAttribute("name"));
  EXPECT_EQ(jointElem->Get<std::string>("name"), "revjoint");
  EXPECT_TRUE(jointElem->HasAttribute("type"));
  EXPECT_EQ(jointElem->Get<std::string>("type"), "revolute");

  // joint links
  EXPECT_TRUE(jointElem->HasElement("parent"));
  EXPECT_EQ(jointElem->Get<std::string>("parent"), "parent");
  EXPECT_TRUE(jointElem->HasElement("child"));
  EXPECT_EQ(jointElem->Get<std::string>("child"), "child");

  // joint axis
  EXPECT_TRUE(jointElem->HasElement("axis"));
  sdf::ElementPtr axisElem = jointElem->GetElement("axis");

  EXPECT_TRUE(axisElem->HasElement("xyz"));
  EXPECT_EQ(axisElem->Get<ignition::math::Vector3d>("xyz"),
      ignition::math::Vector3d(1, 0, 0));

  // joint frame
  EXPECT_TRUE(jointElem->HasElement("frame"));
  sdf::ElementPtr frameElem = jointElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "jframe");

  // joint frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "child");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));

  // joint pose
  EXPECT_TRUE(jointElem->HasElement("pose"));
  sdf::ElementPtr jointPoseElem = jointElem->GetElement("pose");
  EXPECT_TRUE(jointPoseElem->HasAttribute("frame"));
  EXPECT_EQ(jointPoseElem->Get<std::string>("frame"), "jframe");
  EXPECT_EQ(jointPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 2, 1, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a collision element that has a frame element
TEST(Frame, CollisionFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='link'>"
    << "    <collision name='collision'>"
    << "      <frame name='cframe'>"
    << "        <pose frame='link'>1 3 1 0 0 0</pose>"
    << "      </frame>"
    << "      <pose frame='cframe'>0 2 0 0 0 0</pose>"
    << "    </collision>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // collision
  EXPECT_TRUE(linkElem->HasElement("collision"));
  sdf::ElementPtr collisionElem = linkElem->GetElement("collision");
  EXPECT_TRUE(collisionElem->HasAttribute("name"));
  EXPECT_EQ(collisionElem->Get<std::string>("name"), "collision");

  // collision frame
  EXPECT_TRUE(collisionElem->HasElement("frame"));
  sdf::ElementPtr frameElem = collisionElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "cframe");

  // collision frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 3, 1, 0, 0, 0));

  // collision pose
  EXPECT_TRUE(collisionElem->HasElement("pose"));
  sdf::ElementPtr collisionPoseElem = collisionElem->GetElement("pose");
  EXPECT_TRUE(collisionPoseElem->HasAttribute("frame"));
  EXPECT_EQ(collisionPoseElem->Get<std::string>("frame"), "cframe");
  EXPECT_EQ(collisionPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 2, 0, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a visual element that has a frame element
TEST(Frame, VisualFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='link'>"
    << "    <visual name='visual'>"
    << "      <frame name='vframe'>"
    << "        <pose frame='link'>1 1 1 0 0 0</pose>"
    << "      </frame>"
    << "      <pose frame='vframe'>2 2 2 0 0 0</pose>"
    << "    </visual>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // visual
  EXPECT_TRUE(linkElem->HasElement("visual"));
  sdf::ElementPtr visualElem = linkElem->GetElement("visual");
  EXPECT_TRUE(visualElem->HasAttribute("name"));
  EXPECT_EQ(visualElem->Get<std::string>("name"), "visual");

  // visual frame
  EXPECT_TRUE(visualElem->HasElement("frame"));
  sdf::ElementPtr frameElem = visualElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "vframe");

  // visual frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 1, 1, 0, 0, 0));

  // visual pose
  EXPECT_TRUE(visualElem->HasElement("pose"));
  sdf::ElementPtr visualPoseElem = visualElem->GetElement("pose");
  EXPECT_TRUE(visualPoseElem->HasAttribute("frame"));
  EXPECT_EQ(visualPoseElem->Get<std::string>("frame"), "vframe");
  EXPECT_EQ(visualPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(2, 2, 2, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing an inertial element that has a frame element
TEST(Frame, InertialFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='link'>"
    << "    <inertial>"
    << "      <frame name='iframe'>"
    << "        <pose frame='link'>1 2 3 0 0 0</pose>"
    << "      </frame>"
    << "      <pose frame='iframe'>3 2 1 0 0 0</pose>"
    << "    </inertial>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "link");

  // inertial
  EXPECT_TRUE(linkElem->HasElement("inertial"));
  sdf::ElementPtr inertialElem = linkElem->GetElement("inertial");

  // inertial frame
  EXPECT_TRUE(inertialElem->HasElement("frame"));
  sdf::ElementPtr frameElem = inertialElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "iframe");

  // inertial frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0));

  // inertial pose
  EXPECT_TRUE(inertialElem->HasElement("pose"));
  sdf::ElementPtr inertialPoseElem = inertialElem->GetElement("pose");
  EXPECT_TRUE(inertialPoseElem->HasAttribute("frame"));
  EXPECT_EQ(inertialPoseElem->Get<std::string>("frame"), "iframe");
  EXPECT_EQ(inertialPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(3, 2, 1, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a light element that has a frame element
TEST(Frame, LightFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<light type= 'directional' name='my_light'>"
    << "  <frame name='lframe'>"
    << "    <pose frame='/world'>0.1 10 0 0 0 0</pose>"
    << "  </frame>"
    << "  <pose frame='lframe'>0.1 0 0 0 0 0</pose>"
    << "  <diffuse>0.2 0.3 0.4 1</diffuse>"
    << "  <specular>0.3 0.4 0.5 1</specular>"
    << "</light>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // light
  EXPECT_TRUE(sdfParsed->Root()->HasElement("light"));
  sdf::ElementPtr lightElem = sdfParsed->Root()->GetElement("light");
  EXPECT_TRUE(lightElem->HasAttribute("name"));
  EXPECT_EQ(lightElem->Get<std::string>("name"), "my_light");
  EXPECT_TRUE(lightElem->HasAttribute("type"));
  EXPECT_EQ(lightElem->Get<std::string>("type"), "directional");

  // light frame
  EXPECT_TRUE(lightElem->HasElement("frame"));
  sdf::ElementPtr frameElem = lightElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "lframe");

  // light frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0.1, 10, 0, 0, 0, 0));

  // light pose
  EXPECT_TRUE(lightElem->HasElement("pose"));
  sdf::ElementPtr lightPoseElem = lightElem->GetElement("pose");
  EXPECT_TRUE(lightPoseElem->HasAttribute("frame"));
  EXPECT_EQ(lightPoseElem->Get<std::string>("frame"), "lframe");
  EXPECT_EQ(lightPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0.1, 0, 0, 0, 0, 0));

  // diffuse
  EXPECT_TRUE(lightElem->HasElement("diffuse"));
  sdf::ElementPtr diffuseElem = lightElem->GetElement("diffuse");
  EXPECT_EQ(diffuseElem->Get<sdf::Color>(), sdf::Color(0.2f, 0.3f, 0.4f, 1.0f));

  // specular
  EXPECT_TRUE(lightElem->HasElement("specular"));
  sdf::ElementPtr specularElem = lightElem->GetElement("specular");
  EXPECT_EQ(specularElem->Get<sdf::Color>(),
      sdf::Color(0.3f, 0.4f, 0.5f, 1.0f));
}

////////////////////////////////////////
// Test parsing an actor element that has a frame element
TEST(Frame, ActorFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<actor name='my_actor'>"
    << "  <frame name='aframe'>"
    << "    <pose frame='/world'>1 5 0 0 0 0</pose>"
    << "  </frame>"
    << "  <pose frame='aframe'>0.1 3 0 0 0 0</pose>"
    << "  <skin>"
    << "    <filename>moonwalk.dae</filename>"
    << "  </skin>"
    << "  <animation name='walking'>"
    << "    <filename>walk.dae</filename>"
    << "  </animation>"
    << "  <script/>"
    << "</actor>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // actor
  EXPECT_TRUE(sdfParsed->Root()->HasElement("actor"));
  sdf::ElementPtr actorElem = sdfParsed->Root()->GetElement("actor");
  EXPECT_TRUE(actorElem->HasAttribute("name"));
  EXPECT_EQ(actorElem->Get<std::string>("name"), "my_actor");

  // actor frame
  EXPECT_TRUE(actorElem->HasElement("frame"));
  sdf::ElementPtr frameElem = actorElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "aframe");

  // actor frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 5, 0, 0, 0, 0));

  // actor pose
  EXPECT_TRUE(actorElem->HasElement("pose"));
  sdf::ElementPtr actorPoseElem = actorElem->GetElement("pose");
  EXPECT_TRUE(actorPoseElem->HasAttribute("frame"));
  EXPECT_EQ(actorPoseElem->Get<std::string>("frame"), "aframe");
  EXPECT_EQ(actorPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0.1, 3, 0, 0, 0, 0));

  // skin
  EXPECT_TRUE(actorElem->HasElement("skin"));
  sdf::ElementPtr skinElem = actorElem->GetElement("skin");
  EXPECT_TRUE(skinElem->HasElement("filename"));
  sdf::ElementPtr filenameElem = skinElem->GetElement("filename");
  EXPECT_EQ(filenameElem->Get<std::string>(), "moonwalk.dae");

  // animation
  EXPECT_TRUE(actorElem->HasElement("animation"));
  sdf::ElementPtr animationElem = actorElem->GetElement("animation");
  EXPECT_TRUE(animationElem->HasAttribute("name"));
  EXPECT_EQ(animationElem->Get<std::string>("name"), "walking");
  EXPECT_TRUE(animationElem->HasElement("filename"));
  filenameElem = animationElem->GetElement("filename");
  EXPECT_EQ(filenameElem->Get<std::string>(), "walk.dae");
}


////////////////////////////////////////
// Test parsing nested model states
TEST(NestedModel, StateFrame)
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='my_model'>"
    << "  <frame name='mframe'>"
    << "    <pose frame='/world'>1 0 2 0 0 0</pose>"
    << "  </frame>"
    << "  <pose frame='mframe'>3 3 9 0 0 0</pose>"
    << "  <link name='my_link'>"
    << "    <frame name='lframe'>"
    << "      <pose frame='mframe'>8 5 2 0 0 0</pose>"
    << "    </frame>"
    << "    <pose frame='lframe'>111 3 0 0 0 0</pose>"
    << "  </link>"
    << "</model>"
    << "<light name='my_light'>"
    << "  <frame name='lframe'>"
    << "    <pose frame='/world'>1 0 1 0 0 0</pose>"
    << "  </frame>"
    << "    <pose frame='lframe'>99 0 22 0 0 0</pose>"
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
    EXPECT_TRUE(poseElem->HasAttribute("frame"));
    EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(1, 0, 2, 0, 0, 0));

    // model pose
    EXPECT_TRUE(modelStateElem->HasElement("pose"));
    sdf::ElementPtr modelPoseElem = modelStateElem->GetElement("pose");
    EXPECT_TRUE(modelPoseElem->HasAttribute("frame"));
    EXPECT_EQ(modelPoseElem->Get<std::string>("frame"), "mframe");
    EXPECT_EQ(modelPoseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(3, 3, 9, 0, 0, 0));
  }

  // link
  EXPECT_TRUE(modelStateElem->HasElement("link"));
  sdf::ElementPtr linkStateElem = modelStateElem->GetElement("link");
  EXPECT_TRUE(linkStateElem->HasAttribute("name"));
  EXPECT_EQ(linkStateElem->Get<std::string>("name"), "my_link");

  {
    // link frame
    EXPECT_TRUE(linkStateElem->HasElement("frame"));
    sdf::ElementPtr frameElem = linkStateElem->GetElement("frame");
    EXPECT_TRUE(frameElem->HasAttribute("name"));
    EXPECT_EQ(frameElem->Get<std::string>("name"), "lframe");

    // link frame pose
    EXPECT_TRUE(frameElem->HasElement("pose"));
    sdf::ElementPtr poseElem = frameElem->GetElement("pose");
    EXPECT_TRUE(poseElem->HasAttribute("frame"));
    EXPECT_EQ(poseElem->Get<std::string>("frame"), "mframe");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(8, 5, 2, 0, 0, 0));

    // link pose
    EXPECT_TRUE(linkStateElem->HasElement("pose"));
    sdf::ElementPtr linkPoseElem = linkStateElem->GetElement("pose");
    EXPECT_TRUE(linkPoseElem->HasAttribute("frame"));
    EXPECT_EQ(linkPoseElem->Get<std::string>("frame"), "lframe");
    EXPECT_EQ(linkPoseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(111, 3, 0, 0, 0, 0));
  }

  EXPECT_TRUE(stateElem->HasElement("light"));
  sdf::ElementPtr lightStateElem = stateElem->GetElement("light");

  // light
  EXPECT_TRUE(lightStateElem->HasAttribute("name"));
  EXPECT_EQ(lightStateElem->Get<std::string>("name"), "my_light");

  {
    // light frame
    EXPECT_TRUE(lightStateElem->HasElement("frame"));
    sdf::ElementPtr frameElem = lightStateElem->GetElement("frame");
    EXPECT_TRUE(frameElem->HasAttribute("name"));
    EXPECT_EQ(frameElem->Get<std::string>("name"), "lframe");

    // light frame pose
    EXPECT_TRUE(frameElem->HasElement("pose"));
    sdf::ElementPtr poseElem = frameElem->GetElement("pose");
    EXPECT_TRUE(poseElem->HasAttribute("frame"));
    EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
    EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(1, 0, 1, 0, 0, 0));

    // light pose
    EXPECT_TRUE(lightStateElem->HasElement("pose"));
    sdf::ElementPtr lightPoseElem = lightStateElem->GetElement("pose");
    EXPECT_TRUE(lightPoseElem->HasAttribute("frame"));
    EXPECT_EQ(lightPoseElem->Get<std::string>("frame"), "lframe");
    EXPECT_EQ(lightPoseElem->Get<ignition::math::Pose3d>(),
        ignition::math::Pose3d(99, 0, 22, 0, 0, 0));
  }
}

////////////////////////////////////////
// Test parsing a projector element that has a frame element
TEST(Frame, ProjectorFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='my_link'>"
    << "    <projector name='my_projector'>"
    << "      <frame name='pframe'>"
    << "        <pose frame='link'>1 -1 0 0 0 0</pose>"
    << "      </frame>"
    << "      <pose frame='pframe'>-1 0 0 0 0 0</pose>"
    << "    </projector>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "my_link");

  // projector
  EXPECT_TRUE(linkElem->HasElement("projector"));
  sdf::ElementPtr projectorElem = linkElem->GetElement("projector");
  EXPECT_TRUE(projectorElem->HasAttribute("name"));
  EXPECT_EQ(projectorElem->Get<std::string>("name"), "my_projector");

  // projector frame
  EXPECT_TRUE(projectorElem->HasElement("frame"));
  sdf::ElementPtr frameElem = projectorElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "pframe");

  // projector frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, -1, 0, 0, 0, 0));

  // projector pose
  EXPECT_TRUE(projectorElem->HasElement("pose"));
  sdf::ElementPtr projectorPoseElem = projectorElem->GetElement("pose");
  EXPECT_TRUE(projectorPoseElem->HasAttribute("frame"));
  EXPECT_EQ(projectorPoseElem->Get<std::string>("frame"), "pframe");
  EXPECT_EQ(projectorPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(-1, 0, 0, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a sensor element that has a frame element
TEST(Frame, SensorFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='my_link'>"
    << "    <sensor name='my_sensor' type='ray'>"
    << "      <frame name='sframe'>"
    << "        <pose frame='link'>1 -1 1 0 0 0</pose>"
    << "      </frame>"
    << "      <pose frame='sframe'>1 2 2 0 0 0</pose>"
    << "    </sensor>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "my_link");

  // sensor
  EXPECT_TRUE(linkElem->HasElement("sensor"));
  sdf::ElementPtr sensorElem = linkElem->GetElement("sensor");
  EXPECT_TRUE(sensorElem->HasAttribute("name"));
  EXPECT_EQ(sensorElem->Get<std::string>("name"), "my_sensor");
  EXPECT_TRUE(sensorElem->HasAttribute("type"));
  EXPECT_EQ(sensorElem->Get<std::string>("type"), "ray");

  // sensor frame
  EXPECT_TRUE(sensorElem->HasElement("frame"));
  sdf::ElementPtr frameElem = sensorElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "sframe");

  // sensor frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, -1, 1, 0, 0, 0));

  // sensor pose
  EXPECT_TRUE(sensorElem->HasElement("pose"));
  sdf::ElementPtr sensorPoseElem = sensorElem->GetElement("pose");
  EXPECT_TRUE(sensorPoseElem->HasAttribute("frame"));
  EXPECT_EQ(sensorPoseElem->Get<std::string>("frame"), "sframe");
  EXPECT_EQ(sensorPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1, 2, 2, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a camera sensor element that has a frame element
TEST(Frame, CameraFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='my_link'>"
    << "    <sensor name='my_sensor' type='camera'>"
    << "      <camera>"
    << "        <frame name='cframe'>"
    << "          <pose frame='link'>-1.3 0 1 0 0 0</pose>"
    << "        </frame>"
    << "        <pose frame='cframe'>4 2 2 0 0 0</pose>"
    << "      </camera>"
    << "    </sensor>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "my_link");

  // sensor
  EXPECT_TRUE(linkElem->HasElement("sensor"));
  sdf::ElementPtr sensorElem = linkElem->GetElement("sensor");
  EXPECT_TRUE(sensorElem->HasAttribute("name"));
  EXPECT_EQ(sensorElem->Get<std::string>("name"), "my_sensor");
  EXPECT_TRUE(sensorElem->HasAttribute("type"));
  EXPECT_EQ(sensorElem->Get<std::string>("type"), "camera");

  // sensor
  EXPECT_TRUE(sensorElem->HasElement("camera"));
  sdf::ElementPtr cameraElem = sensorElem->GetElement("camera");

  // camera frame
  EXPECT_TRUE(cameraElem->HasElement("frame"));
  sdf::ElementPtr frameElem = cameraElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "cframe");

  // camera frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(-1.3, 0, 1, 0, 0, 0));

  // camera pose
  EXPECT_TRUE(cameraElem->HasElement("pose"));
  sdf::ElementPtr cameraPoseElem = cameraElem->GetElement("pose");
  EXPECT_TRUE(cameraPoseElem->HasAttribute("frame"));
  EXPECT_EQ(cameraPoseElem->Get<std::string>("frame"), "cframe");
  EXPECT_EQ(cameraPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(4, 2, 2, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a audio source element that has a frame element
TEST(Frame, AudioSourceFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<model name='my_model'>"
    << "  <link name='my_link'>"
    << "    <audio_source>"
    << "      <frame name='asframe'>"
    << "        <pose frame='link'>0 -1 0 0 0 0</pose>"
    << "      </frame>"
    << "      <pose frame='asframe'>0 0 2 0 0 0</pose>"
    << "    </audio_source>"
    << "  </link>"
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

  // link
  EXPECT_TRUE(modelElem->HasElement("link"));
  sdf::ElementPtr linkElem = modelElem->GetElement("link");
  EXPECT_TRUE(linkElem->HasAttribute("name"));
  EXPECT_EQ(linkElem->Get<std::string>("name"), "my_link");

  // audio source
  EXPECT_TRUE(linkElem->HasElement("audio_source"));
  sdf::ElementPtr audioSourceElem = linkElem->GetElement("audio_source");

  // audio source frame
  EXPECT_TRUE(audioSourceElem->HasElement("frame"));
  sdf::ElementPtr frameElem = audioSourceElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "asframe");

  // audio source frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "link");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, -1, 0, 0, 0, 0));

  // audio source pose
  EXPECT_TRUE(audioSourceElem->HasElement("pose"));
  sdf::ElementPtr audioSourcePoseElem = audioSourceElem->GetElement("pose");
  EXPECT_TRUE(audioSourcePoseElem->HasAttribute("frame"));
  EXPECT_EQ(audioSourcePoseElem->Get<std::string>("frame"), "asframe");
  EXPECT_EQ(audioSourcePoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a population element that has a frame element
TEST(Frame, PopulationFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "<population name='my_population'>"
    << "  <frame name='pframe'>"
    << "    <pose frame='/world'>0 1 0 0 0 0</pose>"
    << "  </frame>"
    << "  <pose frame='pframe'>0 0 0.2 0 0 0</pose>"
    << "</population>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // population
  EXPECT_TRUE(sdfParsed->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("population"));
  sdf::ElementPtr populationElem = worldElem->GetElement("population");
  EXPECT_TRUE(populationElem->HasAttribute("name"));
  EXPECT_EQ(populationElem->Get<std::string>("name"), "my_population");

  // population frame
  EXPECT_TRUE(populationElem->HasElement("frame"));
  sdf::ElementPtr frameElem = populationElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "pframe");

  // population frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 1, 0, 0, 0, 0));

  // population pose
  EXPECT_TRUE(populationElem->HasElement("pose"));
  sdf::ElementPtr populationPoseElem = populationElem->GetElement("pose");
  EXPECT_TRUE(populationPoseElem->HasAttribute("frame"));
  EXPECT_EQ(populationPoseElem->Get<std::string>("frame"), "pframe");
  EXPECT_EQ(populationPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(0, 0, 0.2, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a gui camera element that has a frame element
TEST(Frame, GuiCameraFrame)
{
  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "<gui>"
    << "  <camera name='my_camera'>"
    << "    <frame name='cframe'>"
    << "      <pose frame='/world'>1.2 1 0 0 0 0</pose>"
    << "    </frame>"
    << "    <pose frame='cframe'>3.1 22 0 0 0 0</pose>"
    << "  </camera>"
    << "</gui>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Verify correct parsing

  // gui
  EXPECT_TRUE(sdfParsed->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = sdfParsed->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("gui"));
  sdf::ElementPtr guiElem = worldElem->GetElement("gui");

  // camera
  EXPECT_TRUE(guiElem->HasElement("camera"));
  sdf::ElementPtr cameraElem = guiElem->GetElement("camera");
  EXPECT_TRUE(cameraElem->HasAttribute("name"));
  EXPECT_EQ(cameraElem->Get<std::string>("name"), "my_camera");

  // camera frame
  EXPECT_TRUE(cameraElem->HasElement("frame"));
  sdf::ElementPtr frameElem = cameraElem->GetElement("frame");
  EXPECT_TRUE(frameElem->HasAttribute("name"));
  EXPECT_EQ(frameElem->Get<std::string>("name"), "cframe");

  // camera frame pose
  EXPECT_TRUE(frameElem->HasElement("pose"));
  sdf::ElementPtr poseElem = frameElem->GetElement("pose");
  EXPECT_TRUE(poseElem->HasAttribute("frame"));
  EXPECT_EQ(poseElem->Get<std::string>("frame"), "/world");
  EXPECT_EQ(poseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(1.2, 1, 0, 0, 0, 0));

  // camera pose
  EXPECT_TRUE(cameraElem->HasElement("pose"));
  sdf::ElementPtr cameraPoseElem = cameraElem->GetElement("pose");
  EXPECT_TRUE(cameraPoseElem->HasAttribute("frame"));
  EXPECT_EQ(cameraPoseElem->Get<std::string>("frame"), "cframe");
  EXPECT_EQ(cameraPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(3.1, 22, 0, 0, 0, 0));
}

////////////////////////////////////////
// Test parsing a include element that has a pose element
TEST(Frame, IncludeFrame)
{
  const std::string MODEL_PATH = std::string(PROJECT_SOURCE_PATH)
      + "/test/integration/model/box";

  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>"
    << "<world name='default'>"
    << "<include>"
    << "  <name>my_model</name>"
    << "  <pose frame='/world'>5 -2 1 0 0 0</pose>"
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
  EXPECT_TRUE(modelPoseElem->HasAttribute("frame"));
  EXPECT_EQ(modelPoseElem->Get<std::string>("frame"), "/world");
  EXPECT_EQ(modelPoseElem->Get<ignition::math::Pose3d>(),
      ignition::math::Pose3d(5, -2, 1, 0, 0, 0));
}
