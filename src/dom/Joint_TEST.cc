/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "sdf/dom/Joint.hh"

/////////////////////////////////////////////////
TEST(DOMJoint, Construction)
{
  sdf::Joint joint;

  EXPECT_EQ(joint.Type(), sdf::JointType::UNKNOWN);
  EXPECT_EQ(joint.Typename(), "unknown");
}

/////////////////////////////////////////////////
TEST(DOMJoint, Set)
{
  sdf::Joint joint;

  EXPECT_EQ(joint.Type(), sdf::JointType::UNKNOWN);
  EXPECT_EQ(joint.Typename(), "unknown");

  joint.SetType(sdf::JointType::BALL);
  EXPECT_EQ(joint.Type(), sdf::JointType::BALL);
  EXPECT_EQ(joint.Typename(), "ball");

  joint.SetType(sdf::JointType::GEARBOX);
  EXPECT_EQ(joint.Type(), sdf::JointType::GEARBOX);
  EXPECT_EQ(joint.Typename(), "gearbox");

  joint.SetType(sdf::JointType::PRISMATIC);
  EXPECT_EQ(joint.Type(), sdf::JointType::PRISMATIC);
  EXPECT_EQ(joint.Typename(), "prismatic");

  joint.SetType(sdf::JointType::REVOLUTE);
  EXPECT_EQ(joint.Type(), sdf::JointType::REVOLUTE);
  EXPECT_EQ(joint.Typename(), "revolute");

  joint.SetType(sdf::JointType::REVOLUTE2);
  EXPECT_EQ(joint.Type(), sdf::JointType::REVOLUTE2);
  EXPECT_EQ(joint.Typename(), "revolute2");

  joint.SetType(sdf::JointType::SCREW);
  EXPECT_EQ(joint.Type(), sdf::JointType::SCREW);
  EXPECT_EQ(joint.Typename(), "screw");

  joint.SetType(sdf::JointType::UNIVERSAL);
  EXPECT_EQ(joint.Type(), sdf::JointType::UNIVERSAL);
  EXPECT_EQ(joint.Typename(), "universal");
}
