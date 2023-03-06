/*
 * Copyright 2023 Open Source Robotics Foundation
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

#include <string>
#include <gtest/gtest.h>

#include "sdf/sdf.hh"
#include "sdf/parser.hh"
#include "sdf/ParserConfig.hh"
#include "test_config.h"
#include "test_utils.hh"

/////////////////////////////////////////////////
sdf::SDFPtr InitSDF()
{
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  return sdf;
}

//////////////////////////////////////////////////
TEST(URDF2SDF, ValidBasicURDF)
{
  std::string urdfXml = R"(
    <robot name='test_robot'>
      <link name='link1'>
        <inertial>
          <mass value='0.1' />
          <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
          <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
        </inertial>
      </link>
    </robot>)";

  sdf::ParserConfig config;

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(urdfXml, config);
  ASSERT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->LinkNameExists("link1"));
}

//////////////////////////////////////////////////
TEST(URDF2SDF, ValidContinuousJointedURDF)
{
  std::string urdfXml = R"(
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
        <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
      </joint>
    </robot>)";

  sdf::ParserConfig config;

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(urdfXml, config);
  ASSERT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->LinkNameExists("link1"));
  EXPECT_TRUE(model->LinkNameExists("link2"));
  EXPECT_TRUE(model->JointNameExists("joint1_2"));
}

//////////////////////////////////////////////////
TEST(URDF2SDF, ZeroMassIntermediateLinkWithFixedParentJoint)
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
    std::string urdfXml = R"(
        <robot name='test_robot'>
          <link name='link1'>
            <inertial>
              <mass value='0.1' />
              <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
              <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
            </inertial>
          </link>
          <link name='link2'/>
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // joint1_2 converted into frame, attached to link1
    EXPECT_TRUE(model->FrameNameExists("joint1_2"));
    const sdf::Frame *frame = model->FrameByName("joint1_2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link1"), frame->AttachedTo());

    // link2 converted into frame, attached to joint1_2
    EXPECT_TRUE(model->FrameNameExists("link2"));
    frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("joint1_2"), frame->AttachedTo());

    // joint2_3 will change to be relative to link1
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
    const sdf::Joint *joint = model->JointByName("joint2_3");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ(std::string("link1"), joint->ParentLinkName());
    EXPECT_EQ(std::string("link3"), joint->ChildLinkName());

    // link3
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }

  // conversion to frame, by disabling lumping and preserving fixed joints using
  // gazebo tags
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
        <robot name='test_robot'>
          <link name='link1'>
            <inertial>
              <mass value='0.1' />
              <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
              <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
            </inertial>
          </link>
          <link name='link2'/>
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
          <gazebo reference='joint1_2'>
            <preserveFixedJoint>true</preserveFixedJoint>
          </gazebo>
        </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // link2 converted into frame, attached to link1
    EXPECT_TRUE(model->FrameNameExists("link2"));
    const sdf::Frame *frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link1"), frame->AttachedTo());

    // joint1_2 converted into frame, attached to link2
    EXPECT_TRUE(model->FrameNameExists("joint1_2"));
    frame = model->FrameByName("joint1_2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link2"), frame->AttachedTo());

    // joint2_3 will still have parent link as link2
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
    const sdf::Joint *joint = model->JointByName("joint2_3");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ(std::string("link2"), joint->ParentLinkName());
    EXPECT_EQ(std::string("link3"), joint->ChildLinkName());

    // link3
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }

  // conversion to frame, by preserving fixed joints in ParserConfig
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
        <robot name='test_robot'>
          <link name='link1'>
            <inertial>
              <mass value='0.1' />
              <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
              <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
            </inertial>
          </link>
          <link name='link2'/>
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

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // link2 converted into frame, attached to link1
    EXPECT_TRUE(model->FrameNameExists("link2"));
    const sdf::Frame *frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link1"), frame->AttachedTo());

    // joint1_2 converted into frame, attached to link2
    EXPECT_TRUE(model->FrameNameExists("joint1_2"));
    frame = model->FrameByName("joint1_2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link2"), frame->AttachedTo());

    // joint2_3 will still have parent link as link2
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
    const sdf::Joint *joint = model->JointByName("joint2_3");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ(std::string("link2"), joint->ParentLinkName());
    EXPECT_EQ(std::string("link3"), joint->ChildLinkName());

    // link3
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }
}

//////////////////////////////////////////////////
TEST(URDF2SDF, ZeroMassIntermediateLinkWithFixedChildJoint)
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

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'/>
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // lumping and reduction occurs, explicit conversion to frame does not
    // happen
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] has a mass value of less than or equal to zero");
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // mass of link3 lumped into link2, link2 not converted to frame
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // joint2_3 converted into a frame
    EXPECT_TRUE(model->FrameNameExists("joint2_3"));
    const sdf::Frame *frame = model->FrameByName("joint2_3");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link2"), frame->AttachedTo());

    // link3 converted into a frame
    EXPECT_TRUE(model->FrameNameExists("link3"));
    frame = model->FrameByName("link3");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("joint2_3"), frame->AttachedTo());
  }

  // turn off lumping, preserve fixed child joint, conversion fails
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'/>
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
        <gazebo reference='joint2_3'>
          <preserveFixedJoint>true</preserveFixedJoint>
        </gazebo>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // Everything beneath link2 is ignored
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing the <disableFixedJointLumping> tag "
        "or setting it to false on fixed child joint[joint2_3], or setting "
        "ParserConfig::URDFPreserveFixedJoint to true, could help resolve this "
        "error");
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->FrameNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
    EXPECT_FALSE(model->FrameNameExists("joint2_3"));
  }

  // preserving fixed joints using config, conversion fails
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <link name='link2'/>
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

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // Everything beneath link2 is ignored
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing the <disableFixedJointLumping> tag "
        "or setting it to false on fixed child joint[joint2_3], or setting "
        "ParserConfig::URDFPreserveFixedJoint to true, could help resolve this "
        "error");
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->FrameNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
    EXPECT_FALSE(model->FrameNameExists("joint2_3"));
  }
}

//////////////////////////////////////////////////
TEST(URDF2SDF, URDFConvertForceTorqueSensorModels)
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

  // valid force torque sensor
  {
    // clear the contents of the buffer
    buffer.str("");

    const std::string sdfTestFile =
        sdf::testing::TestFile("integration", "force_torque_sensor.urdf");
    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.Load(sdfTestFile, defaultConfig);
    EXPECT_TRUE(errors.empty()) << errors;
  }

  // invalid force torque sensor, with massless child link, revolute joint
  {
    // clear the contents of the buffer
    buffer.str("");

    const std::string sdfTestFile =
        sdf::testing::TestFile(
            "integration",
            "invalid_force_torque_sensor_massless_child_link.urdf");
    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.Load(sdfTestFile, defaultConfig);

    // parent joint is not fixed, conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link_1] has no <inertial> block defined");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint_1] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link_1] is not modeled in sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("base_link"));
    EXPECT_TRUE(model->LinkNameExists("link_2"));
    EXPECT_TRUE(model->JointNameExists("joint_2"));
    EXPECT_FALSE(model->FrameNameExists("link_1"));
    EXPECT_FALSE(model->LinkNameExists("link_1"));
    EXPECT_FALSE(model->FrameNameExists("joint_1"));
    EXPECT_FALSE(model->JointNameExists("joint_1"));
  }

  // invalid force torque sensor, with massless child link, fixed parent joint
  {
    // clear the contents of the buffer
    buffer.str("");

    const std::string sdfTestFile =
        sdf::testing::TestFile(
            "integration",
            "invalid_force_torque_sensor_massless_child_link_converted_to_frame"
            ".urdf");
    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.Load(sdfTestFile, config);

    // link_1 converted to frame, attached to base_link
    // joint_1 converted to frame, attached to link_1
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link_1] has no <inertial> block defined");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("base_link"));
    EXPECT_TRUE(model->LinkNameExists("link_2"));
    EXPECT_TRUE(model->JointNameExists("joint_2"));
    const sdf::Frame *frame = model->FrameByName("link_1");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("base_link"), frame->AttachedTo());
    frame = model->FrameByName("joint_1");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("link_1"), frame->AttachedTo());
  }
}
