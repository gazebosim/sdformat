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
#include "test_utils.hh"

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

  sdf::MimicConstraint mimic("test_joint", "axis", 5.0, 1.0, 2.0);

  EXPECT_FALSE(axis.Mimic());
  axis.SetMimic(mimic);
  EXPECT_TRUE(axis.Mimic());
  EXPECT_EQ(axis.Mimic()->Joint(), "test_joint");
  EXPECT_EQ(axis.Mimic()->Axis(), "axis");
  EXPECT_DOUBLE_EQ(axis.Mimic()->Multiplier(), 5.0);
  EXPECT_DOUBLE_EQ(axis.Mimic()->Offset(), 1.0);
  EXPECT_DOUBLE_EQ(axis.Mimic()->Reference(), 2.0);
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
TEST(DOMJointAxis, ToElement)
{
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

  sdf::JointAxis axis;
  sdf::Errors errors;

  errors = axis.SetXyz(gz::math::Vector3d(0, 1, 0));
  ASSERT_TRUE(errors.empty());
  axis.SetXyzExpressedIn("test");
  ASSERT_TRUE(errors.empty());

  axis.SetDamping(0.2);
  axis.SetFriction(1.3);
  axis.SetSpringReference(2.4);
  axis.SetSpringStiffness(-1.2);
  axis.SetLower(-10.8);
  axis.SetUpper(123.4);
  axis.SetEffort(3.2);
  axis.SetMaxVelocity(54.2);
  axis.SetStiffness(1e2);
  axis.SetDissipation(1.5);

  sdf::MimicConstraint mimic("test_joint", "axis2", 5.0, 1.0, 2.0);
  axis.SetMimic(mimic);

  sdf::ElementPtr elem = axis.ToElement(errors);
  ASSERT_TRUE(errors.empty());

  // Check //axis/xyz
  sdf::ElementPtr xyzElem = elem->GetElement("xyz", errors);
  ASSERT_TRUE(errors.empty());
  gz::math::Vector3d xyz = elem->Get<gz::math::Vector3d>(
      errors, "xyz", gz::math::Vector3d()).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(gz::math::Vector3d::UnitY, xyz);
  std::string expressedIn = elem->GetElement("xyz", errors)->Get<std::string>(
      errors, "expressed_in");
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ("test", expressedIn);

  // Check //axis/dynamics
  sdf::ElementPtr dynElem = elem->GetElement("dynamics", errors);
  ASSERT_TRUE(errors.empty());

  double damping = 0;
  damping = dynElem->Get<double>(errors, "damping", damping).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(0.2, damping);

  double friction = 0;
  friction = dynElem->Get<double>(errors, "friction", friction).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(1.3, friction);

  double springReference = 0;
  springReference = dynElem->Get<double>(
      errors, "spring_reference", springReference).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(2.4, springReference);

  double springStiffness = 0;
  springStiffness = dynElem->Get<double>(
      errors, "spring_stiffness", springStiffness).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(-1.2, springStiffness);

  // Check //axis/limit
  sdf::ElementPtr limitElem = elem->GetElement("limit", errors);
  double lower = 0;
  lower = limitElem->Get<double>(errors, "lower", lower).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(-10.8, lower);

  double upper = 0;
  upper = limitElem->Get<double>(errors, "upper", upper).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(123.4, upper);

  double effort = 0;
  effort = limitElem->Get<double>(errors, "effort", effort).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(3.2, effort);

  double maxVel = 0;
  maxVel = limitElem->Get<double>(errors, "velocity", maxVel).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(54.2, maxVel);

  double stiffness = 0;
  stiffness = limitElem->Get<double>(errors, "stiffness", stiffness).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(1e2, stiffness);

  double dissipation = 0;
  dissipation = limitElem->Get<double>(
      errors, "dissipation", dissipation).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(1.5, dissipation);

  // Check //axis/mimic
  sdf::ElementPtr mimicElem = elem->FindElement("mimic");
  ASSERT_NE(nullptr, mimicElem);
  std::string mimicJointName;
  mimicJointName = mimicElem->Get<std::string>(
      errors, "joint", mimicJointName).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ("test_joint", mimicJointName);

  std::string mimicAxisName;
  mimicAxisName = mimicElem->Get<std::string>(
      errors, "axis", mimicAxisName).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ("axis2", mimicAxisName);

  double multiplier = 0.0;
  multiplier = mimicElem->Get<double>(
      errors, "multiplier", multiplier).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(5.0, multiplier);

  double offset = 0.0;
  offset = mimicElem->Get<double>(
      errors, "offset", offset).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(1.0, offset);

  double reference = 0.0;
  reference = mimicElem->Get<double>(
      errors, "reference", reference).first;
  ASSERT_TRUE(errors.empty());
  EXPECT_DOUBLE_EQ(2.0, reference);

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
