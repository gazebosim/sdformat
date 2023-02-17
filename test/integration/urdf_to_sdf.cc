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

  auto sdf = InitSDF();
  sdf::ParserConfig config;
  sdf::Errors errors;

  ASSERT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
  ASSERT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(sdf);

  sdf::Root root;
  errors = root.Load(sdf);
  ASSERT_TRUE(errors.empty()) << errors;

  const sdf::Model *model = root.Model();
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(model->LinkNameExists("link1"));
}

//////////////////////////////////////////////////
TEST(URDF2SDF, ValidJointedURDF)
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

  auto sdf = InitSDF();
  sdf::ParserConfig config;
  sdf::Errors errors;

  ASSERT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
  ASSERT_TRUE(errors.empty()) << errors;
  ASSERT_TRUE(sdf);

  sdf::Root root;
  errors = root.Load(sdf);
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
  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
  #endif

  sdf::ParserConfig config;

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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    auto sdf = InitSDF();
    sdf::Errors errors;

    ASSERT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link2] has no inertia defined, not modeled in sdf");
    EXPECT_TRUE(errors.empty()) << errors;

    sdf::Root root;
    errors = root.Load(sdf);
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

    auto sdf = InitSDF();
    sdf::Errors errors;

    EXPECT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no inertia defined, [1] children links "
        "ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no inertia defined, [1] children joints "
        "ignored");
    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "urdf2sdf: link[link1] has no inertia defined, not modeled in sdf");
    ASSERT_TRUE(errors.empty()) << errors;

    sdf::Root root;
    errors = root.Load(sdf);
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

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(true);
  #endif
}

//////////////////////////////////////////////////
TEST(URDF2SDF, URDFConvertLinkWithNoInertiaToFrame)
{
  // Capture sdferr output
  std::stringstream buffer;
  auto old = std::cerr.rdbuf(buffer.rdbuf());

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
  #endif

  sdf::ParserConfig config;
  config.URDFSetConvertLinkWithNoMassToFrame(true);

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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    auto sdf = InitSDF();
    sdf::Errors errors;
    ASSERT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
    ASSERT_TRUE(errors.empty()) << errors;

    sdf::Root root;
    errors = root.Load(sdf);
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
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
  }

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
          <origin xyz='1.0 2.0 3.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    auto sdf = InitSDF();
    sdf::Errors errors;
    ASSERT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
    ASSERT_TRUE(errors.empty()) << errors;

    sdf::Root root;
    errors = root.Load(sdf);
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
    EXPECT_FALSE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->FrameNameExists("link1"));
    EXPECT_TRUE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
  }

  {
    // clear the contents of the buffer
    buffer.str("");

    std::string urdfXml = R"(
      <robot name='test_robot'>
        <link name='link1'/>
        <link name='link2'/>
        <joint name='joint1_2' type='continuous'>
          <parent link='link1' />
          <child  link='link2' />
          <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
        </joint>
      </robot>)";

    auto sdf = InitSDF();
    sdf::Errors errors;
    ASSERT_TRUE(sdf::readString(urdfXml, config, sdf, errors));
    EXPECT_TRUE(errors.empty()) << errors;

    sdf::Root root;
    errors = root.Load(sdf);
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
    EXPECT_FALSE(model->LinkNameExists("link1"));
    EXPECT_TRUE(model->FrameNameExists("link1"));
    EXPECT_FALSE(model->LinkNameExists("link2"));
    EXPECT_TRUE(model->FrameNameExists("link2"));
    EXPECT_TRUE(model->JointNameExists("joint1_2"));
  }

  // Revert cerr rdbug so as to not interfere with other tests
  std::cerr.rdbuf(old);
  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(true);
  #endif
}
