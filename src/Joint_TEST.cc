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
#include <ignition/math/Pose3.hh>
#include "sdf/Joint.hh"

/////////////////////////////////////////////////
TEST(DOMJoint, Construction)
{
  sdf::Joint joint;
  EXPECT_TRUE(joint.Name().empty());
  EXPECT_EQ(sdf::JointType::INVALID, joint.Type());
  EXPECT_TRUE(joint.ParentLinkName().empty());
  EXPECT_TRUE(joint.ChildLinkName().empty());
  EXPECT_EQ(ignition::math::Pose3d::Zero, joint.Pose());
  EXPECT_TRUE(joint.PoseFrame().empty());
  EXPECT_EQ(nullptr, joint.Element());

  joint.SetPose({-1, -2, -3, IGN_PI, IGN_PI, 0});
  EXPECT_EQ(ignition::math::Pose3d(-1, -2, -3, IGN_PI, IGN_PI, 0),
            joint.Pose());

  joint.SetPoseFrame("link");
  EXPECT_EQ("link", joint.PoseFrame());

  joint.SetName("test_joint");
  EXPECT_EQ("test_joint", joint.Name());

  joint.SetParentLinkName("parent");
  EXPECT_EQ("parent", joint.ParentLinkName());

  joint.SetChildLinkName("child");
  EXPECT_EQ("child", joint.ChildLinkName());

  joint.SetType(sdf::JointType::BALL);
  EXPECT_EQ(sdf::JointType::BALL, joint.Type());
  joint.SetType(sdf::JointType::CONTINUOUS);
  EXPECT_EQ(sdf::JointType::CONTINUOUS, joint.Type());
  joint.SetType(sdf::JointType::GEARBOX);
  EXPECT_EQ(sdf::JointType::GEARBOX, joint.Type());
  joint.SetType(sdf::JointType::PRISMATIC);
  EXPECT_EQ(sdf::JointType::PRISMATIC, joint.Type());
  joint.SetType(sdf::JointType::REVOLUTE);
  EXPECT_EQ(sdf::JointType::REVOLUTE, joint.Type());
  joint.SetType(sdf::JointType::REVOLUTE2);
  EXPECT_EQ(sdf::JointType::REVOLUTE2, joint.Type());
  joint.SetType(sdf::JointType::SCREW);
  EXPECT_EQ(sdf::JointType::SCREW, joint.Type());
  joint.SetType(sdf::JointType::UNIVERSAL);
  EXPECT_EQ(sdf::JointType::UNIVERSAL, joint.Type());
}
