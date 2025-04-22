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

#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/Filesystem.hh"
#include "test_config.hh"
#include "test_utils.hh"

//////////////////////////////////////////////////
TEST(Pose1_9, PoseExpressionFormats)
{
  using Pose = gz::math::Pose3d;

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
  EXPECT_EQ("model_with_empty_pose", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_empty_pose_with_degrees_false", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_empty_pose_with_degrees_true", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(3);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_pose_no_attribute", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(4);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_pose_with_degrees_false", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(5);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_pose_with_degrees_true", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, pi / 2, pi, pi * 3 / 2), model->RawPose());

  model = world->ModelByIndex(6);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_single_space_delimiter", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(7);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_newline_delimiter", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(8);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_messy_delimiters", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(9);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_empty_pose_euler_rpy", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(10);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_empty_pose_euler_rpy_degrees_true", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(11);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_empty_pose_euler_rpy_degrees_false", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(12);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_euler_rpy", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(13);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_euler_rpy_degrees_false", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(14);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_euler_rpy_degrees_true", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, GZ_DTOR(90), GZ_DTOR(180), GZ_DTOR(270)),
      model->RawPose());

  model = world->ModelByIndex(15);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_quat_xyzw", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), model->RawPose());

  model = world->ModelByIndex(16);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_quat_xyzw_degrees_false", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), model->RawPose());

  // //inertial/pose
  model = world->ModelByIndex(17);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_with_inertia_pose", model->Name());
  {
    const auto link = model->LinkByIndex(0);
    ASSERT_NE(nullptr, link);
    EXPECT_EQ("link_euler_rpy_degrees_true", link->Name());
    EXPECT_EQ(Pose(1, 2, 3, GZ_DTOR(90), GZ_DTOR(180), GZ_DTOR(270)),
              link->Inertial().Pose());
  }
  {
    const auto link = model->LinkByIndex(1);
    ASSERT_NE(nullptr, link);
    EXPECT_EQ("link_quat_xyzw", link->Name());
    EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0),
              link->Inertial().Pose());
  }

  model = world->ModelByIndex(18);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_empty_quat_xyzw", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(19);
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("model_empty_quat_xyzw_degrees_false", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseStringOutput)
{
  std::ostringstream stream;
  stream
      << "<sdf version='1.9'>"
      << "<model name='parent'>"
      << "  <pose rotation_format='quat_xyzw'/>"
      << "  <link name='link'/>"
      << "</model>"
      << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::Errors errors;
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed, errors));
  ASSERT_TRUE(errors.empty()) << errors;

  sdf::Root root;
  errors = root.Load(sdfParsed);
  ASSERT_TRUE(errors.empty()) << errors;

  auto model = root.Model();
  ASSERT_NE(nullptr, model);

  auto elem = model->Element();
  ASSERT_NE(nullptr, elem);

  auto poseElem = elem->GetElement("pose");
  ASSERT_NE(nullptr, poseElem);

  auto poseParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseParam);

  const std::string strOutput = poseParam->GetAsString();
  EXPECT_EQ("0 0 0   0 0 0 1", strOutput);
}

//////////////////////////////////////////////////
TEST(Pose1_9, BadModelPoses)
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
    std::ostringstream stream;
    stream
        << "<sdf version='1.9'>"
        << "  <model name='bad_rotation_format'>"
        << "    <pose rotation_format='bad_rotation_format'></pose>"
        << "    <link name='link'/>"
        << "  </model>"
        << "</sdf>";

    sdf::SDFPtr sdfParsed(new sdf::SDF());
    sdf::init(sdfParsed);
    sdf::Errors errors;
    EXPECT_FALSE(sdf::readString(stream.str(), sdfParsed, errors));
    EXPECT_FALSE(errors.empty());

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "Undefined attribute //pose[@rotation_format='bad_rotation_format']");
  }

  {
    buffer.str("");
    std::ostringstream stream;
    stream
        << "<sdf version='1.9'>"
        << "  <model name='euler_rpy_wrong_number_of_values'>"
        << "    <pose rotation_format='euler_rpy'>"
        << "      1 2 3"
        << "    </pose>"
        << "    <link name='link'/>"
        << "  </model>"
        << "</sdf>";

    sdf::SDFPtr sdfParsed(new sdf::SDF());
    sdf::init(sdfParsed);
    sdf::Errors errors;
    EXPECT_FALSE(sdf::readString(stream.str(), sdfParsed, errors));
    EXPECT_FALSE(errors.empty());

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "//pose[@rotation_format='euler_rpy'] must have 6 values, "
        "but 3 were found");
  }

  {
    buffer.str("");
    std::ostringstream stream;
    stream
        << "<sdf version='1.9'>"
        << "  <model name='quat_xyzw_wrong_number_of_values'>"
        << "    <pose rotation_format='quat_xyzw'>"
        << "      1 2 3 4"
        << "    </pose>"
        << "    <link name='link'/>"
        << "  </model>"
        << "</sdf>";

    sdf::SDFPtr sdfParsed(new sdf::SDF());
    sdf::init(sdfParsed);
    sdf::Errors errors;
    EXPECT_FALSE(sdf::readString(stream.str(), sdfParsed, errors));
    EXPECT_FALSE(errors.empty());

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "//pose[@rotation_format='quat_xyzw'] must have 7 values, "
        "but 4 were found");
  }

  {
    buffer.str("");
    std::ostringstream stream;
    stream
        << "<sdf version='1.9'>"
        << "  <model name='quat_xyzw_with_degrees_true'>"
        << "    <pose rotation_format='quat_xyzw' degrees='true'>"
        << "      1 2 3 0 0 0 1"
        << "    </pose>"
        << "    <link name='link'/>"
        << "  </model>"
        << "</sdf>";

    sdf::SDFPtr sdfParsed(new sdf::SDF());
    sdf::init(sdfParsed);
    sdf::Errors errors;
    EXPECT_FALSE(sdf::readString(stream.str(), sdfParsed, errors));
    EXPECT_FALSE(errors.empty());

    EXPECT_PRED2(sdf::testing::contains, buffer.str(),
        "//pose[@degrees='true'] does not apply when parsing quaternions");
  }
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseSet8ValuesFail)
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

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "quat_xyzw", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_FALSE(poseValueParam->SetFromString(
      "1 2 3   0.7071068 0.7071068 0 0   0"));
  EXPECT_PRED2(sdf::testing::contains, buffer.str(),
      "The value for //pose[@rotation_format='quat_xyzw'] must have 7 values, "
      "but more than that were found in '1 2 3   0.7071068 0.7071068 0 0   0'");
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseElementSetAndGet)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);
  poseElem->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6));

  Pose elemVal;
  ASSERT_TRUE(poseElem->Get<Pose>("", elemVal, Pose()));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), elemVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseElementSetAndParamGet)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);
  poseElem->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  Pose paramVal;
  ASSERT_TRUE(poseValueParam->Get<Pose>(paramVal));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), paramVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetAndGet)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  Pose paramVal;
  ASSERT_TRUE(poseValueParam->Get<Pose>(paramVal));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), paramVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetFromStringAndGet)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->SetFromString("1 2 3   0.4 0.5 0.6"));

  Pose paramVal;
  ASSERT_TRUE(poseValueParam->Get<Pose>(paramVal));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), paramVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetAndElemGet)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  Pose elemVal;
  ASSERT_TRUE(poseElem->Get<Pose>("", elemVal, Pose()));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), elemVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseParamSetAndParentElemGet)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);

  ASSERT_TRUE(poseValueParam->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  sdf::ElementPtr parentElem = poseValueParam->GetParentElement();
  ASSERT_NE(nullptr, parentElem);
  EXPECT_EQ(poseElem, parentElem);
  Pose parentElemVal;
  ASSERT_TRUE(parentElem->Get<Pose>("", parentElemVal, Pose()));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), parentElemVal);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingParentPoseElementAfterSet)
{
  // Since the values are explicitly set using the Set<T> function,
  // reparsing should not change their values, even when parent elements have
  // been changed.

  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);
  ASSERT_TRUE(poseElem->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  sdf::ElementPtr degreesPoseElem(new sdf::Element);
  degreesPoseElem->SetName("pose");
  degreesPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  degreesPoseElem->AddAttribute("relative_to", "string", "", false);
  degreesPoseElem->AddAttribute("degrees", "bool", "true", false);
  degreesPoseElem->AddAttribute(
      "rotation_format", "string", "euler_rpy", false);

  sdf::ElementPtr radiansPoseElem(new sdf::Element);
  radiansPoseElem->SetName("pose");
  radiansPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  radiansPoseElem->AddAttribute("relative_to", "string", "", false);
  radiansPoseElem->AddAttribute("degrees", "bool", "false", false);
  radiansPoseElem->AddAttribute(
      "rotation_format", "string", "euler_rpy", false);

  sdf::ElementPtr quatPoseElem(new sdf::Element);
  quatPoseElem->SetName("pose");
  quatPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  quatPoseElem->AddAttribute("relative_to", "string", "", false);
  quatPoseElem->AddAttribute("degrees", "bool", "false", false);
  quatPoseElem->AddAttribute(
      "rotation_format", "string", "quat_xyzw", false);

  // Param from original default attribute
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute true, as euler_rpy.
  ASSERT_TRUE(valParam->SetParentElement(degreesPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute false, as euler_rpy.
  ASSERT_TRUE(valParam->SetParentElement(radiansPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute false, as quat_xyzw.
  ASSERT_TRUE(valParam->SetParentElement(quatPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Remove parent
  ASSERT_TRUE(valParam->SetParentElement(nullptr));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingParentPoseElementAfterParamSetFromString)
{
  // Since the values are set using the SetFromString function, reparsing
  // should change their values, when parent elements have been changed.

  const double pi = 3.14159265358979323846;
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ElementPtr degreesPoseElem(new sdf::Element);
  degreesPoseElem->SetName("pose");
  degreesPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  degreesPoseElem->AddAttribute("relative_to", "string", "", false);
  degreesPoseElem->AddAttribute("degrees", "bool", "true", false);
  degreesPoseElem->AddAttribute(
      "rotation_format", "string", "euler_rpy", false);

  sdf::ElementPtr radiansPoseElem(new sdf::Element);
  radiansPoseElem->SetName("pose");
  radiansPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  radiansPoseElem->AddAttribute("relative_to", "string", "", false);
  radiansPoseElem->AddAttribute("degrees", "bool", "false", false);
  radiansPoseElem->AddAttribute(
      "rotation_format", "string", "euler_rpy", false);

  sdf::ElementPtr quatPoseElem(new sdf::Element);
  quatPoseElem->SetName("pose");
  quatPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  quatPoseElem->AddAttribute("relative_to", "string", "", false);
  quatPoseElem->AddAttribute("degrees", "bool", "false", false);
  quatPoseElem->AddAttribute(
      "rotation_format", "string", "quat_xyzw", false);

  // Param from original default attribute
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1, 2, 3, 0.4, 0.5, 0.6"));

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute true, in euler_rpy.
  ASSERT_TRUE(valParam->SetParentElement(degreesPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Set parent to Element with degrees attribute false, in euler_rpy.
  ASSERT_TRUE(valParam->SetParentElement(radiansPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute false, in quat_xyzw, will
  // fail, as the string that was previously set was only 6 values. The value
  // will remain the same as before and the parent Element will still be the
  // previous one.
  ASSERT_FALSE(valParam->SetParentElement(quatPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  auto parent = valParam->GetParentElement();
  EXPECT_EQ(parent, radiansPoseElem);

  // Remove parent
  ASSERT_TRUE(valParam->SetParentElement(nullptr));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingAttributeOfParentElement)
{
  const double pi = 3.14159265358979323846;
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  // Param value in radians
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1, 2, 3, 0.4, 0.5, 0.6"));

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Changing @degrees to true without reparsing, value will remain the same
  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Values will change to be degrees after reparsing
  ASSERT_TRUE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Changing @rotation_format to euler_rpy without reparsing, value remains the
  // same
  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("euler_rpy"));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Values will still remain the same after reparsing
  ASSERT_TRUE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Changing @rotation_format to quat_xyzw without reparsing, value remains the
  // same
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("quat_xyzw"));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Reparsing will fail, value remains the same as before
  EXPECT_FALSE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Changing @rotation_format to something invalid without reparsing, value
  // remains the same
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("invalid_format"));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Reparsing will fail, value remains the same as before
  EXPECT_FALSE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Changing back to default values
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("euler_rpy"));
  ASSERT_TRUE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, QuatXYZWSetDegreesTrueFail)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0 0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);

  // Set rotation_format to quat_xyzw
  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("quat_xyzw"));

  // Param rotation format in quaternion
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1 2 3 0.7071068 0 0 0.7071068"));

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), val);

  // Changing @degrees to true without reparsing, value will remain the same
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), val);

  // Reparsing will fail due to setting degrees as true, values remain the same
  EXPECT_FALSE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.7071068, 0.7071068, 0, 0), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithoutAttrib)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithDegreesFalse)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "degrees='false'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithDegreesTrue)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "degrees='true'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithEulerRPY)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "", false);

  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("euler_rpy"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "rotation_format='euler_rpy'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithEulerRPYDegreesTrue)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));

  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("euler_rpy"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "degrees='true'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "rotation_format='euler_rpy'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithQuatXYZ)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("quat_xyzw"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3   0.7071068 0 0 0.7071068"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  // The string output has changed as it was parsed from the value, instead of
  // the original string.
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "rotation_format='quat_xyzw'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.707107 0 0 0.707107");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithQuatXYZWDegreesFalse)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));

  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("quat_xyzw"));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3   0.7071068 0 0 0.7071068"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  // The string output has changed as it was parsed from the value, instead of
  // the original string.
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "degrees='false'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "rotation_format='quat_xyzw'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.707107 0 0 0.707107");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringAfterChangingDegreeAttribute)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  // Param value in radians
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1 2 3 0.4 0.5 0.6"));

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  std::string elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");

  // Changing to attribute to degrees, however this does not modify the
  // value of the underlying Param. Reparse needs to be called, which uses
  // the input from SetFromString, to get a new value.
  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  EXPECT_TRUE(valParam->Reparse());

  elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "degrees='true'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");

  // Changing back to radians
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));
  EXPECT_TRUE(valParam->Reparse());
  elemStr = poseElem->ToString("", config);
  EXPECT_PRED2(sdf::testing::contains, elemStr, "degrees='false'");
  EXPECT_PRED2(sdf::testing::contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("integration", "model", _input);
}

//////////////////////////////////////////////////
TEST(Pose1_9, IncludePoseInModelString)
{
  using Pose = gz::math::Pose3d;

  sdf::setFindCallback(findFileCb);

  std::ostringstream stream;
  stream
      << "<sdf version='1.9'>"
      << "  <model name='parent'>"
      << "    <include>"
      << "      <uri>box</uri>"
      << "      <pose degrees='true'>0 10 0 90 0 0</pose>"
      << "    </include>"
      << "  </model>"
      << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::Errors errors;
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed, errors));
  ASSERT_TRUE(errors.empty()) << errors;

  sdf::Root root;
  errors = root.Load(sdfParsed);
  ASSERT_TRUE(errors.empty()) << errors;

  auto model = root.Model();
  ASSERT_NE(nullptr, model);

  auto boxModel = model->ModelByName("box");
  ASSERT_NE(nullptr, boxModel);
  EXPECT_EQ(Pose(0, 10, 0, GZ_DTOR(90), GZ_DTOR(0), GZ_DTOR(0)),
      boxModel->RawPose());
}

//////////////////////////////////////////////////
TEST(Pose1_9, IncludeEulerRPYPoseInModelString)
{
  using Pose = gz::math::Pose3d;

  sdf::setFindCallback(findFileCb);

  std::ostringstream stream;
  stream
      << "<sdf version='1.9'>"
      << "  <model name='parent'>"
      << "    <include>"
      << "      <uri>box</uri>"
      << "      <pose degrees='true' rotation_format='euler_rpy'>"
      << "        0 10 0 90 0 0"
      << "      </pose>"
      << "    </include>"
      << "  </model>"
      << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::Errors errors;
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed, errors));
  ASSERT_TRUE(errors.empty()) << errors;

  sdf::Root root;
  errors = root.Load(sdfParsed);
  ASSERT_TRUE(errors.empty()) << errors;

  auto model = root.Model();
  ASSERT_NE(nullptr, model);

  auto boxModel = model->ModelByName("box");
  ASSERT_NE(nullptr, boxModel);
  EXPECT_EQ(Pose(0, 10, 0, GZ_DTOR(90), GZ_DTOR(0), GZ_DTOR(0)),
      boxModel->RawPose());
}

//////////////////////////////////////////////////
TEST(Pose1_9, IncludeQuatXYZWPoseIn)
{
  using Pose = gz::math::Pose3d;

  sdf::setFindCallback(findFileCb);

  std::ostringstream stream;
  stream
      << "<sdf version='1.9'>"
      << "  <model name='parent'>"
      << "    <include>"
      << "      <uri>box</uri>"
      << "      <pose rotation_format='quat_xyzw'>"
      << "        0 10 0 0.7071068 0 0 0.7071068"
      << "      </pose>"
      << "    </include>"
      << "  </model>"
      << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::Errors errors;
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed, errors));
  ASSERT_TRUE(errors.empty()) << errors;

  sdf::Root root;
  errors = root.Load(sdfParsed);
  ASSERT_TRUE(errors.empty()) << errors;

  auto model = root.Model();
  ASSERT_NE(nullptr, model);

  auto boxModel = model->ModelByName("box");
  ASSERT_NE(nullptr, boxModel);
  EXPECT_EQ(Pose(0, 10, 0, 0.7071068, 0.7071068, 0, 0),
      boxModel->RawPose());
}

//////////////////////////////////////////////////
TEST(Pose1_9, IncludePoseInWorld)
{
  using Pose = gz::math::Pose3d;

  sdf::setFindCallback(findFileCb);
  const std::string testFile = sdf::testing::TestFile(
      "sdf", "include_pose_1_9.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  ASSERT_TRUE(errors.empty()) << errors;
  EXPECT_EQ(SDF_PROTOCOL_VERSION, root.Version());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("first_box", model->Name());
  EXPECT_EQ(Pose(0, 10, 0, GZ_DTOR(90), GZ_DTOR(0), GZ_DTOR(0)),
            model->RawPose());

  model = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("second_box", model->Name());
  EXPECT_EQ(Pose(0, 10, 0, 0.7071068, 0.7071068, 0, 0), model->RawPose());

  model = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("third_box", model->Name());
  EXPECT_EQ(Pose(0, 10, 0, GZ_DTOR(90), GZ_DTOR(0), GZ_DTOR(0)),
            model->RawPose());

  model = world->ModelByIndex(3);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("forth_box", model->Name());
  EXPECT_EQ(Pose(0, 10, 0, GZ_DTOR(90), GZ_DTOR(0), GZ_DTOR(0)),
            model->RawPose());

  model = world->ModelByIndex(4);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("fifth_box", model->Name());
  EXPECT_EQ(Pose(0, 10, 0, GZ_DTOR(90), GZ_DTOR(0), GZ_DTOR(0)),
            model->RawPose());
}
