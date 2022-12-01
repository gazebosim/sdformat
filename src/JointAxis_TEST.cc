/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <limits>
#include "sdf/JointAxis.hh"
#include "sdf/Root.hh"

/////////////////////////////////////////////////
TEST(DOMJointAxis, Construction)
{
  sdf::JointAxis axis;
  EXPECT_EQ(nullptr, axis.Element());
  EXPECT_EQ(gz::math::Vector3d::UnitZ, axis.Xyz());
  EXPECT_TRUE(axis.XyzExpressedIn().empty());
  EXPECT_DOUBLE_EQ(0.0, axis.Damping());
  EXPECT_DOUBLE_EQ(0.0, axis.Friction());
  EXPECT_DOUBLE_EQ(0.0, axis.SpringReference());
  EXPECT_DOUBLE_EQ(0.0, axis.SpringStiffness());
  EXPECT_DOUBLE_EQ(-std::numeric_limits<double>::infinity(), axis.Lower());
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), axis.Upper());
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), axis.Effort());
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), axis.MaxVelocity());
  EXPECT_DOUBLE_EQ(1e8, axis.Stiffness());
  EXPECT_DOUBLE_EQ(1.0, axis.Dissipation());

  {
    sdf::Errors errors = axis.SetXyz(gz::math::Vector3d(0, 1, 0));
    EXPECT_TRUE(errors.empty());
    EXPECT_EQ(gz::math::Vector3d::UnitY, axis.Xyz());
  }

  axis.SetXyzExpressedIn("__model__");
  EXPECT_EQ("__model__", axis.XyzExpressedIn());

  // expect errors when trying to resolve axis without graph
  gz::math::Vector3d vec3;
  EXPECT_FALSE(axis.ResolveXyz(vec3).empty());

  axis.SetDamping(0.2);
  EXPECT_DOUBLE_EQ(0.2, axis.Damping());

  axis.SetFriction(1.3);
  EXPECT_DOUBLE_EQ(1.3, axis.Friction());

  axis.SetSpringReference(2.4);
  EXPECT_DOUBLE_EQ(2.4, axis.SpringReference());

  axis.SetSpringStiffness(-1.2);
  EXPECT_DOUBLE_EQ(-1.2, axis.SpringStiffness());

  axis.SetLower(-10.8);
  EXPECT_DOUBLE_EQ(-10.8, axis.Lower());

  axis.SetUpper(123.4);
  EXPECT_DOUBLE_EQ(123.4, axis.Upper());

  axis.SetEffort(3.2);
  EXPECT_DOUBLE_EQ(3.2, axis.Effort());

  axis.SetMaxVelocity(54.2);
  EXPECT_DOUBLE_EQ(54.2, axis.MaxVelocity());

  axis.SetStiffness(1e2);
  EXPECT_DOUBLE_EQ(1e2, axis.Stiffness());

  axis.SetDissipation(1.5);
  EXPECT_DOUBLE_EQ(1.5, axis.Dissipation());

  sdf::MimicJointContainer mimic("test_joint", 5.0, 1.0, 2.0);

  EXPECT_FALSE(axis.MimicJoint());
  axis.SetMimicJoint(mimic);
  EXPECT_TRUE(axis.MimicJoint());
  EXPECT_EQ(axis.MimicJoint()->Joint(), "test_joint");
  EXPECT_DOUBLE_EQ(axis.MimicJoint()->Multiplier(), 5.0);
  EXPECT_DOUBLE_EQ(axis.MimicJoint()->Offset(), 1.0);
  EXPECT_DOUBLE_EQ(axis.MimicJoint()->Reference(), 2.0);
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, CopyConstructor)
{
  sdf::JointAxis jointAxis;
  EXPECT_TRUE(jointAxis.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());

  sdf::JointAxis jointAxisCopy(jointAxis);
  EXPECT_EQ(jointAxis.Xyz(), jointAxisCopy.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, AssignmentOperator)
{
  sdf::JointAxis jointAxis;
  EXPECT_TRUE(jointAxis.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());

  sdf::JointAxis jointAxisCopy;
  jointAxisCopy = jointAxis;
  EXPECT_EQ(jointAxis.Xyz(), jointAxisCopy.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, MoveConstructor)
{
  gz::math::Vector3d axis{0, 1, 0};
  sdf::JointAxis jointAxis;
  EXPECT_TRUE(jointAxis.SetXyz(axis).empty());

  sdf::JointAxis jointAxisMoved(std::move(jointAxis));
  EXPECT_EQ(axis, jointAxisMoved.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, MoveAssignmentOperator)
{
  gz::math::Vector3d axis{0, 1, 0};
  sdf::JointAxis jointAxis;
  EXPECT_TRUE(jointAxis.SetXyz(axis).empty());

  sdf::JointAxis jointAxisMoved;
  jointAxisMoved = std::move(jointAxis);
  EXPECT_EQ(axis, jointAxisMoved.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, CopyAssignmentAfterMove)
{
  gz::math::Vector3d axis1{0, 1, 0};
  sdf::JointAxis jointAxis1;
  EXPECT_TRUE(jointAxis1.SetXyz(axis1).empty());

  gz::math::Vector3d axis2{1, 0, 0};
  sdf::JointAxis jointAxis2;
  EXPECT_TRUE(jointAxis2.SetXyz(axis2).empty());

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::JointAxis tmp = std::move(jointAxis1);
  jointAxis1 = jointAxis2;
  jointAxis2 = tmp;

  EXPECT_EQ(axis2, jointAxis1.Xyz());
  EXPECT_EQ(axis1, jointAxis2.Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, ZeroNormVectorReturnsError)
{
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz({1.0, 0, 0}).empty());

  sdf::Errors errors = axis.SetXyz(gz::math::Vector3d::Zero);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Message(), "The norm of the xyz vector cannot be zero");
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, ParseMimic)
{
  std::string sdf =
    "<?xml version='1.0' ?>"
    "<sdf version='1.10'>"
    "  <model name='test'>"
    "    <link name='link1'/>"
    "    <link name='link2'/>"
    "    <joint name='revolute_joint' type='revolute'>"
    "      <pose>1 0 0 0 0 0</pose>"
    "      <child>link1</child>"
    "      <parent>link2</parent>"
    "      <axis>"
    "        <xyz>0 0 1</xyz>"
    "        <mimic joint='test_joint'>"
    "          <multiplier>4</multiplier>"
    "          <offset>2</offset>"
    "          <reference>3</reference>"
    "        </mimic>"
    "      </axis>"
    "    </joint>"
    "  </model>"
    "</sdf>";

  sdf::Root root;
  root.LoadSdfString(sdf);
  auto jointElement = root.Element()->GetElement("model")->GetElement("joint");
  EXPECT_NE(nullptr, jointElement);

  // Test loading from sdf element pointer.
  sdf::JointAxis jointAxis;
  jointAxis.Load(jointElement->GetElement("axis"));
  EXPECT_EQ(jointAxis.MimicJoint()->Joint(), "test_joint");
  EXPECT_TRUE(jointAxis.MimicJoint());
  EXPECT_DOUBLE_EQ(jointAxis.MimicJoint()->Multiplier(), 4);
  EXPECT_DOUBLE_EQ(jointAxis.MimicJoint()->Offset(), 2);
  EXPECT_DOUBLE_EQ(jointAxis.MimicJoint()->Reference(), 3);

  // Test conversion back to sdf element.
  auto mimicElement = jointAxis.ToElement()->GetElement("mimic");
  EXPECT_NE(nullptr, mimicElement);
  EXPECT_EQ(mimicElement->Get<std::string>("joint"), "test_joint");
  EXPECT_EQ(mimicElement->Get<double>("offset"), 2);
  EXPECT_EQ(mimicElement->Get<double>("multiplier"), 4);
  EXPECT_EQ(mimicElement->Get<double>("reference"), 3);
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, ParseInvalidSelfMimic)
{
  std::string sdf =
    "<?xml version='1.0' ?>"
    "<sdf version='1.10'>"
    "  <model name='test'>"
    "    <link name='link1'/>"
    "    <link name='link2'/>"
    "    <joint name='self_mimic' type='universal'>"
    "      <pose>1 0 0 0 0 0</pose>"
    "      <child>link1</child>"
    "      <parent>link2</parent>"
    "      <axis>"
    "        <xyz>0 0 1</xyz>"
    "        <mimic joint='self_mimic'>"
    "          <multiplier>4</multiplier>"
    "          <offset>2</offset>"
    "          <reference>3</reference>"
    "        </mimic>"
    "      </axis>"
    "      <axis2>"
    "        <xyz>1 0 0</xyz>"
    "        <mimic joint='self_mimic'>"
    "          <multiplier>4</multiplier>"
    "          <offset>2</offset>"
    "          <reference>3</reference>"
    "        </mimic>"
    "      </axis2>"
    "    </joint>"
    "  </model>"
    "</sdf>";

  sdf::Root root;
  auto errors = root.LoadSdfString(sdf);
  EXPECT_EQ(errors.size(), 4) << errors;
  for (const auto &error : errors)
  {
    std::stringstream ss;
    ss << error;
    const std::string errorMsg = "Error Code 39: Msg: Joint with name "
      "[self_mimic] cannot mimic itself.";
    EXPECT_EQ(ss.str(), errorMsg);
  }
}

/////////////////////////////////////////////////
TEST(DOMJointAxis, ParseMimicInvalidJointName)
{
  std::string sdf =
    "<?xml version='1.0' ?>"
    "<sdf version='1.10'>"
    "  <model name='test'>"
    "    <link name='link1'/>"
    "    <link name='link2'/>"
    "    <joint name='joint' type='universal'>"
    "      <pose>1 0 0 0 0 0</pose>"
    "      <child>link1</child>"
    "      <parent>link2</parent>"
    "      <axis>"
    "        <xyz>0 0 1</xyz>"
    "        <mimic joint='invalid'>"
    "          <multiplier>4</multiplier>"
    "          <offset>2</offset>"
    "          <reference>3</reference>"
    "        </mimic>"
    "      </axis>"
    "      <axis2>"
    "        <xyz>1 0 0</xyz>"
    "        <mimic joint='invalid'>"
    "          <multiplier>4</multiplier>"
    "          <offset>2</offset>"
    "          <reference>3</reference>"
    "        </mimic>"
    "      </axis2>"
    "    </joint>"
    "  </model>"
    "</sdf>";

  sdf::Root root;
  auto errors = root.LoadSdfString(sdf);
  EXPECT_EQ(errors.size(), 2) << errors;
  for (const auto &error : errors)
  {
    std::stringstream ss;
    ss << error;
    const std::string errorMsg = "Error Code 39: Msg: A joint with"
      " name[invalid] specified by an axis mimic in joint with name[joint] not"
      " found in model with name[test].";
    EXPECT_EQ(ss.str(), errorMsg);
  }
}
