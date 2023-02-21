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
TEST(URDF2SDF, WarnURDFLinkWithoutInertial)
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

  sdf::ParserConfig config;

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
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no inertia defined, not modeled in sdf");
    ASSERT_TRUE(errors.empty()) << errors;

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
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no inertia defined, [1] children links "
        "ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no inertia defined, [1] children joints "
        "ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no inertia defined, not modeled in sdf");
    EXPECT_FALSE(errors.empty()) << errors;

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
TEST(URDF2SDF, URDFConvertLinkWithNoInertiaToFrame)
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

  sdf::ParserConfig config;
  config.URDFSetPreserveFixedJoint(true);
  config.URDFSetConvertLinkWithNoMassToFrame(true);

  {
    // clear the contents of the buffer
    buffer.str("");

    // link2 has no inertia defined, however it will not be converted into a
    // frame due to it's parent joint being not a fixed joint, it will be
    // ignored, as well as as its parent joint
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
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] does not have a fixed parent joint, nor any "
        "fixed child joints, unable to convert to frame in sdf");
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

    // link3 has no inertia defined, it will be converted to a frame, attached
    // to the fixed joint joint2_3
    // when converting to sdf, we expect there to be a cycle error to occur, as
    // the converted frame has attached_to on the joint, while the joint is
    // already a parent joint
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
        <link name='link3'/>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link3] does not have a fixed parent joint, nor any "
        "fixed child joints, unable to convert to frame in sdf");
    EXPECT_FALSE(errors.empty()) << errors;

    bool foundFrameAttachedToCycleForJoint = false;
    bool foundFrameAttachedToCycleForLink = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::FRAME_ATTACHED_TO_CYCLE)
      {
        if (sdf::testing::contains(e.Message(), "test_robot::joint2_3"))
        {
          foundFrameAttachedToCycleForJoint = true;
        }
        else if (sdf::testing::contains(e.Message(), "test_robot::link3"))
        {
          foundFrameAttachedToCycleForLink = true;
        }
      }
    }
    EXPECT_TRUE(foundFrameAttachedToCycleForJoint);
    EXPECT_TRUE(foundFrameAttachedToCycleForLink);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_TRUE(model->FrameNameExists("link3"));
    const sdf::Frame *frame = model->FrameByName("link3");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("joint2_3", frame->AttachedTo());
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    // link2 has no inertia defined, it will be converted to a frame, attached
    // to the fixed parent joint joint1_2
    // when converting to sdf, we expect there to be a cycle error to occur, as
    // the converted frame has attached_to on the joint, while the joint is
    // already a child joint,
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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_FALSE(errors.empty());

    bool foundFrameAttachedToCycleForJoint = false;
    bool foundFrameAttachedToCycleForLink = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::FRAME_ATTACHED_TO_CYCLE)
      {
        if (sdf::testing::contains(e.Message(), "test_robot::joint1_2"))
        {
          foundFrameAttachedToCycleForJoint = true;
        }
        else if (sdf::testing::contains(e.Message(), "test_robot::link2"))
        {
          foundFrameAttachedToCycleForLink = true;
        }
      }
    }
    EXPECT_TRUE(foundFrameAttachedToCycleForJoint);
    EXPECT_TRUE(foundFrameAttachedToCycleForLink);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    const sdf::Frame *frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("joint1_2", frame->AttachedTo());
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->LinkNameExists("link3"));
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    // link2 has no inertia defined, it will be converted to a frame, attached
    // to the fixed child joint joint2_3
    // when converting to sdf, we expect there to be a cycle error to occur, as
    // the converted frame has attached_to on the joint, while the joint is
    // already a child joint,
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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_FALSE(errors.empty());

    bool foundJointParentSameAsChildError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::JOINT_PARENT_SAME_AS_CHILD)
      {
        foundJointParentSameAsChildError = true;
        break;
      }
    }
    EXPECT_TRUE(foundJointParentSameAsChildError);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    const sdf::Frame *frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("joint2_3", frame->AttachedTo());
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->LinkNameExists("link3"));
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
  }
}


//////////////////////////////////////////////////
TEST(URDF2SDF, URDFConvertLinkWitSmallMassToFrame)
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

  sdf::ParserConfig config;
  config.URDFSetPreserveFixedJoint(true);
  config.URDFSetMinimumAllowedLinkMass(1e-5);
  config.URDFSetConvertLinkWithNoMassToFrame(true);

  {
    // clear the contents of the buffer
    buffer.str("");

    // link2 has just enough mass, it will not be converted into a frame
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
            <mass value='1e-5' />
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
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link2] does not have a fixed parent joint, nor any "
        "fixed child joints, unable to convert to frame in sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    // link3 has a mass of 1e-6, it will be converted to a frame, attached
    // to the fixed joint joint2_3
    // when converting to sdf, we expect there to be a cycle error to occur, as
    // the converted frame has attached_to on the joint, while the joint is
    // already a parent joint
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
        <link name='link3'>
          <inertial>
            <mass value='1e-6' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint2_3' type='fixed'>
          <parent link='link2' />
          <child  link='link3' />
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_PRED2(sdf::testing::notContains, buffer.str(),
        "urdf2sdf: link[link3] does not have a fixed parent joint, nor any "
        "fixed child joints, unable to convert to frame in sdf");
    EXPECT_FALSE(errors.empty()) << errors;

    bool foundFrameAttachedToCycleForJoint = false;
    bool foundFrameAttachedToCycleForLink = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::FRAME_ATTACHED_TO_CYCLE)
      {
        if (sdf::testing::contains(e.Message(), "test_robot::joint2_3"))
        {
          foundFrameAttachedToCycleForJoint = true;
        }
        else if (sdf::testing::contains(e.Message(), "test_robot::link3"))
        {
          foundFrameAttachedToCycleForLink = true;
        }
      }
    }
    EXPECT_TRUE(foundFrameAttachedToCycleForJoint);
    EXPECT_TRUE(foundFrameAttachedToCycleForLink);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_FALSE(model->LinkNameExists("link3"));
    EXPECT_TRUE(model->FrameNameExists("link3"));
    const sdf::Frame *frame = model->FrameByName("link3");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("joint2_3", frame->AttachedTo());
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    // link2 has a mass of 1e-6, it will be converted to a frame, attached
    // to the fixed parent joint joint1_2
    // when converting to sdf, we expect there to be a cycle error to occur, as
    // the converted frame has attached_to on the joint, while the joint is
    // already a child joint,
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
            <mass value='1e-6' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='fixed'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_FALSE(errors.empty());

    bool foundFrameAttachedToCycleForJoint = false;
    bool foundFrameAttachedToCycleForLink = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::FRAME_ATTACHED_TO_CYCLE)
      {
        if (sdf::testing::contains(e.Message(), "test_robot::joint1_2"))
        {
          foundFrameAttachedToCycleForJoint = true;
        }
        else if (sdf::testing::contains(e.Message(), "test_robot::link2"))
        {
          foundFrameAttachedToCycleForLink = true;
        }
      }
    }
    EXPECT_TRUE(foundFrameAttachedToCycleForJoint);
    EXPECT_TRUE(foundFrameAttachedToCycleForLink);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    const sdf::Frame *frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("joint1_2", frame->AttachedTo());
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->LinkNameExists("link3"));
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    // link2 has a mass of 1e-6, it will be converted to a frame, attached
    // to the fixed child joint joint2_3
    // when converting to sdf, we expect there to be a cycle error to occur, as
    // the converted frame has attached_to on the joint, while the joint is
    // already a child joint,
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
            <mass value='1e-6' />
            <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
            <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
          </inertial>
        </link>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(urdfXml, config);
    EXPECT_FALSE(errors.empty());

    bool foundJointParentSameAsChildError = false;
    for (const auto &e : errors)
    {
      if (e.Code() == sdf::ErrorCode::JOINT_PARENT_SAME_AS_CHILD)
      {
        foundJointParentSameAsChildError = true;
        break;
      }
    }
    EXPECT_TRUE(foundJointParentSameAsChildError);

    const sdf::Model *model = root.Model();
    ASSERT_NE(nullptr, model);
    EXPECT_TRUE(model->LinkNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    const sdf::Frame *frame = model->FrameByName("link2");
    ASSERT_NE(nullptr, frame);
    EXPECT_EQ("joint2_3", frame->AttachedTo());
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
    EXPECT_TRUE(model->LinkNameExists("link3"));
    EXPECT_TRUE(model->JointNameExists("joint2_3"));
  }
}
