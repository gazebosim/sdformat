/*
 * Copyright 2021 Open Source Robotics Foundation
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
#include <sstream>
#include <fstream>
#include <gtest/gtest.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"
#include "test_utils.hh"

//////////////////////////////////////////////////
TEST(Pose1_9, ModelPose)
{
  using Pose = ignition::math::Pose3d;

  const std::string testFile = sdf::testing::TestFile(
      "sdf", "pose_1_9.sdf");
  const double pi = 3.14159265358979323846;

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  ASSERT_TRUE(errors.empty()) << errors;
  EXPECT_EQ(SDF_PROTOCOL_VERSION, root.Version());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose_with_degrees", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose_with_quaternion", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(3);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_pose_no_attribute", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(4);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_pose_with_degrees", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, pi / 2, pi, pi * 3 / 2), model->RawPose());

  model = world->ModelByIndex(5);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_pose_with_radians", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(6);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_quaternion", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), model->RawPose());

  model = world->ModelByIndex(7);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_single_space_delimiter", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(8);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_newline_delimiter", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(9);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_newline_delimiter_quaternion", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), model->RawPose());

  model = world->ModelByIndex(10);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_messy_delimiters", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(11);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_messy_delimiters_quaternion", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), model->RawPose());
}

//////////////////////////////////////////////////
TEST(Pose1_9, SetQuatIntoDefaultFail)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_FALSE(poseValueParam->SetFromString(
      "1 2 3   0.7071068 0.7071068 0 0"));
}

//////////////////////////////////////////////////
TEST(Pose1_9, SetIntoRpyRadiansFail)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr rpyRadiansAttrib = poseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, rpyRadiansAttrib);
  ASSERT_TRUE(rpyRadiansAttrib->Set<std::string>("rpy_radians"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_FALSE(poseValueParam->SetFromString(
      "1 2 3   0.7071068 0.7071068 0 0"));
}

//////////////////////////////////////////////////
TEST(Pose1_9, SetIntoRpyDegreesFail)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr rpyDegreesAttrib = poseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, rpyDegreesAttrib);
  ASSERT_TRUE(rpyDegreesAttrib->Set<std::string>("rpy_degrees"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_FALSE(poseValueParam->SetFromString(
      "1 2 3   0.7071068 0.7071068 0 0"));
}

//////////////////////////////////////////////////
TEST(Pose1_9, SetIntoQuatFail)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr quatAttrib = poseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, quatAttrib);
  ASSERT_TRUE(quatAttrib->Set<std::string>("q_wxyz"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_FALSE(poseValueParam->SetFromString(
      "1 2 3   0.4 0.5 0.6"));
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseElementSetAndGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);
  poseElem->Set<ignition::math::Pose3d>(Pose(1, 2, 3, 0.4, 0.5, 0.6));

  Pose elemVal;
  ASSERT_TRUE(poseElem->Get<ignition::math::Pose3d>("", elemVal, Pose()));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), elemVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseElementSetAndParamGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);
  poseElem->Set<ignition::math::Pose3d>(Pose(1, 2, 3, 0.4, 0.5, 0.6));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  Pose paramVal;
  ASSERT_TRUE(poseValueParam->Get<ignition::math::Pose3d>(paramVal));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), paramVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetAndGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->Set<ignition::math::Pose3d>(
        Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  Pose paramVal;
  ASSERT_TRUE(poseValueParam->Get<ignition::math::Pose3d>(paramVal));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), paramVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetFromStringAndGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->SetFromString("1 2 3   0.4 0.5 0.6"));

  Pose paramVal;
  ASSERT_TRUE(poseValueParam->Get<ignition::math::Pose3d>(paramVal));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), paramVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetAndElemGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->Set<ignition::math::Pose3d>(
        Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  Pose elemVal;
  ASSERT_TRUE(poseElem->Get<ignition::math::Pose3d>("", elemVal, Pose()));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), elemVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetAndParentElemGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->Set<ignition::math::Pose3d>(
        Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  sdf::ElementPtr parentElem = poseValueParam->GetParentElement();
  ASSERT_NE(nullptr, parentElem);
  Pose parentElemVal;
  ASSERT_TRUE(parentElem->Get<ignition::math::Pose3d>(
      "", parentElemVal, Pose()));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), parentElemVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingParentPoseElement)
{
  const double pi = 3.14159265358979323846;
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("rotation_type", "string", "rpy_radians", false);
  ASSERT_TRUE(poseElem->Set<ignition::math::Pose3d>(
      Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  sdf::ElementPtr rpyDegreesPoseElem(new sdf::Element);
  rpyDegreesPoseElem->SetName("pose");
  rpyDegreesPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  rpyDegreesPoseElem->AddAttribute("relative_to", "string", "", false);
  rpyDegreesPoseElem->AddAttribute(
      "rotation_type", "string", "rpy_degrees", false);

  sdf::ParamPtr rpyDegreesAttrib = poseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, rpyDegreesAttrib);
  ASSERT_TRUE(rpyDegreesAttrib->Set<std::string>("rpy_degrees"));

  sdf::ElementPtr rpyRadiansPoseElem(new sdf::Element);
  rpyRadiansPoseElem->SetName("pose");
  rpyRadiansPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  rpyRadiansPoseElem->AddAttribute("relative_to", "string", "", false);
  rpyRadiansPoseElem->AddAttribute(
      "rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr rpyRadiansAttrib = poseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, rpyRadiansAttrib);
  ASSERT_TRUE(rpyRadiansAttrib->Set<std::string>("rpy_radians"));

  // Param from original default attibute
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);

  Pose val;
  ASSERT_TRUE(valParam->Get<ignition::math::Pose3d>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with rotation_type attribute as rpy_degrees
  valParam->SetParentElement(rpyDegreesPoseElem);
  ASSERT_TRUE(valParam->Get<ignition::math::Pose3d>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180),
      val);

  // Set parent to Element with rotation_type attribute as rpy_radians
  valParam->SetParentElement(rpyRadiansPoseElem);
  ASSERT_TRUE(valParam->Get<ignition::math::Pose3d>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Remove parent
  valParam->SetParentElement(nullptr);
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingParentPoseElementFromQuaternion)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr quatPoseElem(new sdf::Element);
  quatPoseElem->SetName("pose");
  quatPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  quatPoseElem->AddAttribute("relative_to", "string", "", false);
  quatPoseElem->AddAttribute(
      "rotation_type", "string", "rpy_degrees", false);

  sdf::ParamPtr quatAttrib = quatPoseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, quatAttrib);
  ASSERT_TRUE(quatAttrib->Set<std::string>("q_wxyz"));

  sdf::ElementPtr rpyRadiansPoseElem(new sdf::Element);
  rpyRadiansPoseElem->SetName("pose");
  rpyRadiansPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  rpyRadiansPoseElem->AddAttribute("relative_to", "string", "", false);
  rpyRadiansPoseElem->AddAttribute(
      "rotation_type", "string", "rpy_radians", false);

  sdf::ParamPtr rpyRadiansAttrib =
      rpyRadiansPoseElem->GetAttribute("rotation_type");
  ASSERT_NE(nullptr, rpyRadiansAttrib);
  ASSERT_TRUE(rpyRadiansAttrib->Set<std::string>("rpy_radians"));

  // Param from original default attibute
  sdf::ParamPtr valParam = quatPoseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1 2 3   0.7071068 0.7071068 0 0"));

  // Changing to parent Element with rotation_type attribute rpy_radians
  // The new value setting will fail since it can't parse 7 values into xyzrpy
  // TODO(AA): To capture error string.
  valParam->SetParentElement(rpyRadiansPoseElem);

  Pose val;
  ASSERT_TRUE(valParam->Get<ignition::math::Pose3d>(val));
  EXPECT_NE(Pose(1, 2, 3, 0.7071068, 0.7071068, 0.0), val);
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), val);
}

//////////////////////////////////////////////////
static bool contains(const std::string &_a, const std::string &_b)
{
  return _a.find(_b) != std::string::npos;
}

//////////////////////////////////////////////////
TEST(Pose1_9, InvalidRotationType)
{
  // Redirect sdferr output
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

  buffer.str("");
  const std::string testString = R"(<?xml version="1.0" ?>
    <sdf version="1.9">
      <world name="default">
        <model name="test_model">
          <pose rotation_type="rpy_xyz">
            1 2 3 0.4 0.5 0.6
          </pose>
          <link name="test_link"/>
        </model>
      </world>
    </sdf>)";

  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(testString);
  ASSERT_FALSE(errors.empty()) << errors;
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_EQ(5, errors[0].LineNumber());
  EXPECT_PRED2(contains, buffer.str(),
      "Invalid attribute //pose[@rotation_type='rpy_xyz']");
}

//////////////////////////////////////////////////
TEST(Pose1_9, InvalidNumberOfPoseValues)
{
  // Redirect sdferr output
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
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose>
              1 2 3 0.4 0.5
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 5 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose>
              1 2 3 0.4 0.5 0.6 0.7
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 7 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="rpy_radians">
              1 2 3 0.4 0.5
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 5 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="rpy_radians">
              1 2 3 0.4 0.5 0.6 0.7
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 7 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="rpy_degrees">
              1 2 3 0.4 0.5
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 5 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="rpy_degrees">
              1 2 3 0.4 0.5
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 5 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="rpy_degrees">
              1 2 3 0.4 0.5 0.6 0.7
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 6 values, but 7 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="q_wxyz">
              1 2 3 0.4 0.5 0.6
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 7 values, but 6 were found instead.");
  }
  {
    buffer.str("");
    const std::string testString = R"(<?xml version="1.0" ?>
      <sdf version="1.9">
        <world name="default">
          <model name="test_model">
            <pose rotation_type="q_wxyz">
              1 2 3 0.4 0.5 0.6 0.7 0.8
            </pose>
            <link name="test_link"/>
          </model>
        </world>
      </sdf>)";

    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(testString);
    ASSERT_FALSE(errors.empty()) << errors;
    EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
    EXPECT_EQ(5, errors[0].LineNumber());
    EXPECT_PRED2(contains, buffer.str(),
        "must have 7 values, but 8 were found instead.");
  }
}
