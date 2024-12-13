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

#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/parser.hh"
#include "sdf/ParserConfig.hh"
#include "test_config.hh"
#include "test_utils.hh"

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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);
    ASSERT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    const sdf::Link *link = model->LinkByName("link1");
    ASSERT_NE(nullptr, link);

    // link2 visual and collision lumps into link1
    EXPECT_TRUE(link->VisualNameExists("link1_fixed_joint_lump__link2_visual"));
    EXPECT_TRUE(
        link->CollisionNameExists("link1_fixed_joint_lump__link2_collision"));

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
    EXPECT_EQ(std::string("link1"), joint->ParentName());
    EXPECT_EQ(std::string("link3"), joint->ChildName());

    // link3
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }

  // Disabling lumping using gazebo tags, fails with warnings suggesting to
  // remove gazebo tags
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // no sdf errors, however we expect warnings and ignored joints and links
    ASSERT_TRUE(errors.empty()) << errors;
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "allowing joint lumping by removing any <disableFixedJointLumping> or "
        "<preserveFixedJoint> gazebo tag on fixed parent joint[joint1_2], as "
        "well as ensuring that ParserConfig::URDFPreserveFixedJoint is false, "
        "could help resolve this warning.");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    const sdf::Link *link = model->LinkByName("link1");
    ASSERT_NE(nullptr, link);

    // expect everything below joint1_2 to be ignored
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));

    // expect no visual or collision from link2 lumped into link1
    EXPECT_FALSE(
        link->VisualNameExists("link1_fixed_joint_lump__link2_visual"));
    EXPECT_FALSE(
        link->CollisionNameExists("link1_fixed_joint_lump__link2_collision"));
  }

  // Disabling lumping using ParserConfig::URDFPreserveFixedJoint, fails with
  // warnings suggesting to set to false
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

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // no sdf errors, however we expect warnings and ignored joints and links
    ASSERT_TRUE(errors.empty()) << errors;
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

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    const sdf::Link *link = model->LinkByName("link1");
    ASSERT_NE(nullptr, link);

    // expect everything below joint1_2 to be ignored
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));

    // expect no visual or collision from link2 lumped into link1
    EXPECT_FALSE(
        link->VisualNameExists("link1_fixed_joint_lump__link2_visual"));
    EXPECT_FALSE(
        link->CollisionNameExists("link1_fixed_joint_lump__link2_collision"));
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // lumping and reduction occurs, no warnings or errors should be present
    ASSERT_TRUE(errors.empty()) << errors;
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link2] is not modeled in sdf");

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // mass of link3 lumped into link2, link2 not converted to frame
    const sdf::Link *link = model->LinkByName("link2");
    ASSERT_NE(nullptr, link);

    // joint1_2
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // link2 visual and collision remain
    EXPECT_TRUE(link->VisualNameExists("link2_visual"));
    EXPECT_TRUE(link->CollisionNameExists("link2_collision"));

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

  // turn off lumping with gazebo tag, conversion fails with dropped links and
  // joints, and warnings to remove gazebo tags
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // Everything beneath joint1_2 is ignored, no sdf errors, but warnings
    // expected with suggestion to remove gazebo tag
    ASSERT_TRUE(errors.empty()) << errors;
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
        "could help resolve this warning.");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "[1] child links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link2] is not modeled in sdf");

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

  // Disabling lumping using ParserConfig::URDFPreserveFixedJoint, fails with
  // warnings suggesting to set to false
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

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // Everything beneath joint1_2 is ignored, no sdf errors, but warnings
    // expected with suggestion to remove gazebo tag
    ASSERT_TRUE(errors.empty()) << errors;
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // lumping and reduction occurs, no warnings should be present
    ASSERT_TRUE(errors.empty()) << errors;
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link3] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link3] is not modeled in sdf");

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // link2
    const sdf::Link *link = model->LinkByName("link2");
    ASSERT_NE(nullptr, link);
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // joint1_2
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // link3 visual and collision lumped into link2
    EXPECT_TRUE(
        link->VisualNameExists("link2_fixed_joint_lump__link3_visual"));
    EXPECT_TRUE(
        link->CollisionNameExists("link2_fixed_joint_lump__link3_collision"));

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

  // lumping and reduction turned off with gazebo tag, expect link3 and joint2_3
  // to be ignored and warnings suggesting to remove gazebo tag
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

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // joint2_3 and link3 will be ignored, and warnings suggesting to remove
    // gazebo tags
    ASSERT_TRUE(errors.empty()) << errors;
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

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // link2
    const sdf::Link *link = model->LinkByName("link2");
    ASSERT_NE(nullptr, link);
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // joint1_2
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // link3 visual and collision not lumped into link2
    EXPECT_FALSE(
        link->VisualNameExists("link2_fixed_joint_lump__link3_visual"));
    EXPECT_FALSE(
        link->CollisionNameExists("link2_fixed_joint_lump__link3_collision"));

    // joint2_3 and link3 ignored
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->FrameNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
    EXPECT_FALSE(model->FrameNameExists("joint2_3"));
  }

  // ParserConfig::URDFSetPreserveFixedJoint set to true, expect link3 and
  // joint2_3 to be ignored and warnings suggesting to remove gazebo tag
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

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // joint2_3 and link3 will be ignored, and warnings suggesting to remove
    // gazebo tags
    ASSERT_TRUE(errors.empty()) << errors;
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

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));

    // link2
    const sdf::Link *link = model->LinkByName("link2");
    ASSERT_NE(nullptr, link);
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // joint1_2
    EXPECT_TRUE(model->JointNameExists("joint1_2"));

    // link3 visual and collision not lumped into link2
    EXPECT_FALSE(
        link->VisualNameExists("link2_fixed_joint_lump__link3_visual"));
    EXPECT_FALSE(
        link->CollisionNameExists("link2_fixed_joint_lump__link3_collision"));

    // joint2_3 and link3 ignored
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

  // invalid force torque sensor, with massless child link, fixed parent joint,
  // lumping and reduction occurs, no errors or warning will be emitted, but
  // force torque sensor won't be able to attach to joint-converted frame, this
  // is the case where users need to be wary about
  // TODO(aaronchongth): To provide warning when joint is dropped/converted
  // during lumping and reduction
  {
    // clear the contents of the buffer
    buffer.str("");

    const std::string sdfTestFile =
        sdf::testing::TestFile(
            "integration",
            "invalid_force_torque_sensor_lumped_and_reduced.urdf");
    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.Load(sdfTestFile, defaultConfig);

    // lumping and reduction occurs, we expect no warnings or errors
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link_1] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "link[link_1] is not modeled in sdf");

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("base_link"));
    EXPECT_TRUE(model->FrameNameExists("link_1"));
    EXPECT_TRUE(model->FrameNameExists("joint_1"));
    EXPECT_TRUE(model->LinkNameExists("link_2"));
    EXPECT_TRUE(model->JointNameExists("joint_2"));
  }

  // invalid force torque sensor, with massless child link, revolute parent
  // joint, there will be warnings with joint_1 and link_1 ignored
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

    // lumping and reduction does not occur, joint_1 and link_1 ignored
    EXPECT_TRUE(errors.empty()) << errors;
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link_1] has no <inertial> block defined. Please ensure this link "
        "has a valid mass to prevent any missing links or joints in the "
        "resulting SDF model");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "parent joint[joint_1] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "link[link_1] is not modeled in sdf");

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
}
