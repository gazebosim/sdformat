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
TEST(URDF2SDF, ValidFixedJointedURDFWhereLumpingOccurs)
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
      <joint name='joint1_2' type='fixed'>
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
  EXPECT_FALSE(model->LinkNameExists("link2"));
  EXPECT_TRUE(model->FrameNameExists("link2"));
  EXPECT_FALSE(model->JointNameExists("joint1_2"));
  EXPECT_TRUE(model->FrameNameExists("joint1_2"));
}

//////////////////////////////////////////////////
TEST(URDF2SDF, ValidFixedJointedURDFWhereLumpingDoesNotOccurs)
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
      <joint name='joint1_2' type='fixed'>
        <parent link='link1' />
        <child  link='link2' />
        <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
      </joint>
      <gazebo reference='joint1_2'>
        <disableFixedJointLumping>true</disableFixedJointLumping>
      </gazebo>
      <gazebo reference='joint1_2'>
        <preserveFixedJoint>true</preserveFixedJoint>
      </gazebo>
    </robot>)";

  sdf::ParserConfig config;
  config.URDFSetPreserveFixedJoint(true);

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
TEST(URDF2SDF, NotConvertingLinksWithNoInertialBlockOrZeroMass)
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

    // link2 has no inertia, it will not be modeled into sdf, it's parent joint
    // will not be modeled too
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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetConvertLinkWithNoMassToFrame(false);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, parent joint ["
        "joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, not modeled "
        "in sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    // link1 has no inertia, will not be modeled into sdf, as well as its
    // child joint and child link, link2
    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
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

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetConvertLinkWithNoMassToFrame(false);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, not modeled in "
        "sdf");
    ASSERT_FALSE(errors.empty()) << errors;

    bool foundMissingLinkError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::MODEL_WITHOUT_LINK)
      {
        foundMissingLinkError = true;
        break;
      }
    }
    EXPECT_TRUE(foundMissingLinkError);
  }
}

//////////////////////////////////////////////////
TEST(URDF2SDF, URDFConvertRootLinkWithZeroMassToFrame)
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

  // root link no fixed joint
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
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
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, not modeled in "
        "sdf");
    ASSERT_FALSE(errors.empty());
    bool foundMissingLinkError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::MODEL_WITHOUT_LINK)
      {
        foundMissingLinkError = true;
        break;
      }
    }
    EXPECT_TRUE(foundMissingLinkError);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_FALSE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
  }

  // root link with fixed joint
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);
    // lumping occurs therefore conversion of link1 to a frame was not
    // considered
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->FrameNameExists("joint1_2"));
  }

  // root link child fixed joint with lumping turned off, however will still be
  // converted into a revolute joint
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
        <gazebo reference='joint1_2'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);
    // child joint gets converted to a revolute joint with min 0, max 0, but no
    // fixed parent joint available, conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, not modeled in "
        "sdf");
    ASSERT_FALSE(errors.empty());
    bool foundMissingLinkError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::MODEL_WITHOUT_LINK)
      {
        foundMissingLinkError = true;
        break;
      }
    }
    EXPECT_TRUE(foundMissingLinkError);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_FALSE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
  }

  // root link child fixed joint with lumping turned off, don't convert into
  // revolute joints, however fixed joints will be reduced
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
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
    // fixed joints to be reduced, but still no fixed parent joints,
    // conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, not modeled in "
        "sdf");
    ASSERT_FALSE(errors.empty());
    bool foundMissingLinkError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::MODEL_WITHOUT_LINK)
      {
        foundMissingLinkError = true;
        break;
      }
    }
    EXPECT_TRUE(foundMissingLinkError);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_FALSE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
  }

  // root link child fixed joint with lumping turned off, don't convert into
  // revolute joints, preserve fixed joints
  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
        <link name='link2'>
          <inertial>
            <mass value='0.1' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
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
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // child joint is fixed but no fixed parent joint, conversion fails
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no <inertial> block defined, not modeled in "
        "sdf");
    ASSERT_FALSE(errors.empty());
    bool foundMissingLinkError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::MODEL_WITHOUT_LINK)
      {
        foundMissingLinkError = true;
        break;
      }
    }
    EXPECT_TRUE(foundMissingLinkError);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_FALSE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
  }
}

//////////////////////////////////////////////////
TEST(URDF2SDF, URDFConvertIntermediateLinkWithZeroMassToFrame)
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

  // intermediate link with no fixed joint
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
        <joint name='joint2_3' type='continuous'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // parent joint is not fixed, conversion to frame will fail
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, parent joint "
        "[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, not modeled in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
  }

  // intermediate link with parent fixed joint
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
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // lumping occurs therefore conversion of link2 to a frame was not
    // considered, link2 and joint1_2 dropped, joint2_3 parent set to link1
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("joint1_2"));
    const sdf::Joint *joint = model->JointByName("joint2_3");
    ASSERT_NE(nullptr, joint);
    EXPECT_EQ(std::string("link1"), joint->ParentLinkName());
    EXPECT_EQ(std::string("link3"), joint->ChildLinkName());
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }

  // intermediate link with parent fixed joint, with lumping turned off, don't
  // convert into revolute joints, preserve fixed joints
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
          <origin xyz='1 2 3' rpy='0.0 0.0 1.57'/>
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
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // link2 will be converted to a frame, relative to link1, joint1_2 will be
    // left out
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }

  // intermediate link with child fixed joint, with lumping turned off, don't
  // convert into revolute joints, preserve fixed joints
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
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // link2 will not be converted into a frame, as parent joint is not fixed
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, [1] children "
        "links ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, [1] children "
        "joints ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, parent joint "
        "[joint1_2] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, not modeled in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_FALSE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
  }

  // intermediate link with both fixed joint, with lumping turned off, don't
  // convert into revolute joints, preserve fixed joints
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
        <joint name='joint2_3' type='fixed'>
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
        <gazebo reference='joint2_3'>
          <disableFixedJointLumping>true</disableFixedJointLumping>
        </gazebo>
        <gazebo reference='joint2_3'>
          <preserveFixedJoint>true</preserveFixedJoint>
        </gazebo>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // link2 will be converted to a frame, attached to link1, pose relative to
    // link1, with joint1_2 dropped
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link2] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    EXPECT_FALSE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->FrameNameExists("joint1_2"));
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
    EXPECT_TRUE(model->LinkNameExists("link3"));
  }
}

//////////////////////////////////////////////////
TEST(URDF2SDF, URDFConvertLeafLinkWithZeroMassToFrame)
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

  // leaf link with non-fixed parent joint
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
        <link name='link3'/>
        <joint name='joint2_3' type='continuous'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // link3 will not be converted to a frame, as the parent joint is not fixed
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link3] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link3] has no <inertial> block defined, parent joint "
        "[joint2_3] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link3] has no <inertial> block defined, not modeled in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_FALSE(model->FrameNameExists("link3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
    EXPECT_FALSE(model->FrameNameExists("joint2_3"));
  }

  // leaf link with fixed parent joint
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
        <link name='link3'/>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::ParserConfig defaultConfig;
    sdf::Errors errors = root.LoadSdfString(urdfXml, defaultConfig);

    // lumping occurs therefore conversion of link3 to a frame was not
    // considered
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link3] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->FrameNameExists("link3"));
    EXPECT_TRUE(model->FrameNameExists("joint2_3"));
  }

  // leaf link with parent fixed joint, with lumping turned off, don't
  // convert into revolute joints, preserve fixed joints
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
        <link name='link3'/>
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
    sdf::ParserConfig config;
    config.URDFSetPreserveFixedJoint(true);
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);

    // link3 will be converted to a frame, attached to link2, pose relative to
    // link2, with joint2_3 dropped
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link3] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->FrameNameExists("link3"));
    EXPECT_FALSE(model->FrameNameExists("joint2_3"));
    EXPECT_FALSE(model->JointNameExists("joint2_3"));
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
        "urdf2sdf: link[link_1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link_1] has no <inertial> block defined, parent joint "
        "[joint_1] ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link_1] has no <inertial> block defined, not modeled in "
        "sdf");
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

    // link_1 converted to frame, attached to base_link, joint_1 dropped
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link_1] has no <inertial> block defined, but does not "
        "have a fixed parent joint, unable to be converted into a frame in "
        "sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("base_link"));
    EXPECT_TRUE(model->LinkNameExists("link_2"));
    EXPECT_TRUE(model->JointNameExists("joint_2"));
    const sdf::Frame *frame = model->FrameByName("link_1");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ(std::string("base_link"), frame->AttachedTo());
    EXPECT_FALSE(model->JointNameExists("joint_1"));
    EXPECT_FALSE(model->FrameNameExists("joint_1"));
  }
}
