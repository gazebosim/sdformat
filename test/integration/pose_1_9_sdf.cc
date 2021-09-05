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
  ASSERT_EQ("model_with_empty_pose_with_degrees_false", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose_with_degrees_true", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(3);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_pose_no_attribute", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(4);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_pose_with_degrees_false", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(5);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_pose_with_degrees_true", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, pi / 2, pi, pi * 3 / 2), model->RawPose());

  model = world->ModelByIndex(6);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_single_space_delimiter", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(7);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_newline_delimiter", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());

  model = world->ModelByIndex(8);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_messy_delimiters", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), model->RawPose());
}

//////////////////////////////////////////////////
static bool contains(const std::string &_a, const std::string &_b)
{
  return _a.find(_b) != std::string::npos;
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseSet7ValuesFail)
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
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_FALSE(poseValueParam->SetFromString(
      "1 2 3   0.7071068 0.7071068 0 0"));
  EXPECT_PRED2(contains, buffer.str(), "can only accept 6 values");
}

//////////////////////////////////////////////////
TEST(Pose1_9, PoseElementSetAndGet)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6));

  Pose elemVal;
  ASSERT_TRUE(poseElem->Get<Pose>("", elemVal, Pose()));
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
  poseElem->AddAttribute("degrees", "bool", "false", false);
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
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

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
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

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
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

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
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

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

  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  ASSERT_TRUE(poseElem->Set<Pose>(Pose(1, 2, 3, 0.4, 0.5, 0.6)));

  sdf::ElementPtr degreesPoseElem(new sdf::Element);
  degreesPoseElem->SetName("pose");
  degreesPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  degreesPoseElem->AddAttribute("relative_to", "string", "", false);
  degreesPoseElem->AddAttribute("degrees", "bool", "true", false);

  sdf::ElementPtr radiansPoseElem(new sdf::Element);
  radiansPoseElem->SetName("pose");
  radiansPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  radiansPoseElem->AddAttribute("relative_to", "string", "", false);
  radiansPoseElem->AddAttribute("degrees", "bool", "false", false);

  // Param from original default attibute
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute true.
  valParam->SetParentElement(degreesPoseElem);
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute false.
  valParam->SetParentElement(radiansPoseElem);
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Remove parent
  valParam->SetParentElement(nullptr);
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingParentPoseElementAfterParamSetFromString)
{
  // Since the values are set using the SetFromString function, reparsing
  // should change their values, when parent elements have been changed.

  const double pi = 3.14159265358979323846;
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  sdf::ElementPtr degreesPoseElem(new sdf::Element);
  degreesPoseElem->SetName("pose");
  degreesPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  degreesPoseElem->AddAttribute("relative_to", "string", "", false);
  degreesPoseElem->AddAttribute("degrees", "bool", "true", false);

  sdf::ElementPtr radiansPoseElem(new sdf::Element);
  radiansPoseElem->SetName("pose");
  radiansPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  radiansPoseElem->AddAttribute("relative_to", "string", "", false);
  radiansPoseElem->AddAttribute("degrees", "bool", "false", false);

  // Param from original default attibute
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1, 2, 3, 0.4, 0.5, 0.6"));

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Set parent to Element with degrees attribute true.
  valParam->SetParentElement(degreesPoseElem);
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Set parent to Element with degrees attribute false.
  valParam->SetParentElement(radiansPoseElem);
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Remove parent
  valParam->SetParentElement(nullptr);
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ChangingAttributeOfParentElement)
{
  const double pi = 3.14159265358979323846;
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  // Param value in radians
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1, 2, 3, 0.4, 0.5, 0.6"));

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Changing to degrees without reparsing, value will remain the same
  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  // Values will change to be degrees after reparsing
  ASSERT_TRUE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4 * pi / 180, 0.5 * pi / 180, 0.6 * pi / 180), val);

  // Changing back to radians
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));
  ASSERT_TRUE(valParam->Reparse());
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithoutAttrib)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  std::string elemStr = poseElem->ToString("");
  EXPECT_PRED2(contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithDegreesFalse)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  std::string elemStr = poseElem->ToString("");
  EXPECT_PRED2(contains, elemStr, "degrees='0'");
  EXPECT_PRED2(contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringWithDegreesTrue)
{
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));

  sdf::ParamPtr poseValueParam = poseElem->GetValue();
  ASSERT_NE(nullptr, poseValueParam);
  EXPECT_TRUE(poseValueParam->SetFromString("1 2 3  0.4 0.5 0.6"));

  std::string elemStr = poseElem->ToString("");
  EXPECT_PRED2(contains, elemStr, "degrees='1'");
  EXPECT_PRED2(contains, elemStr, "0.4 0.5 0.6");
}

//////////////////////////////////////////////////
TEST(Pose1_9, ToStringAfterChangingDegreeAttribute)
{
  using Pose = ignition::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);

  // Param value in radians
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1 2 3 0.4 0.5 0.6"));

  std::string elemStr = poseElem->ToString("");
  EXPECT_PRED2(contains, elemStr, "0.4 0.5 0.6");

  // Changing to degrees
  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  elemStr = poseElem->ToString("");
  EXPECT_PRED2(contains, elemStr, "degrees='1'");
  EXPECT_PRED2(contains, elemStr, "0.4 0.5 0.6");

  // Changing back to radians
  ASSERT_TRUE(degreesAttrib->Set<bool>(false));
  elemStr = poseElem->ToString("");
  EXPECT_PRED2(contains, elemStr, "degrees='0'");
  EXPECT_PRED2(contains, elemStr, "0.4 0.5 0.6");
}
