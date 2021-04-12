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
#include "sdf/JointAxis.hh"

/////////////////////////////////////////////////
TEST(DOMJoint, Construction)
{
  sdf::Joint joint;
  EXPECT_TRUE(joint.Name().empty());
  EXPECT_EQ(sdf::JointType::INVALID, joint.Type());
  EXPECT_TRUE(joint.ParentLinkName().empty());
  EXPECT_TRUE(joint.ChildLinkName().empty());
  EXPECT_EQ(ignition::math::Pose3d::Zero, joint.RawPose());
  EXPECT_TRUE(joint.PoseRelativeTo().empty());
  EXPECT_EQ(nullptr, joint.Element());
  {
    auto semanticPose = joint.SemanticPose();
    EXPECT_EQ(ignition::math::Pose3d::Zero, semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose without graph
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  joint.SetRawPose({-1, -2, -3, IGN_PI, IGN_PI, 0});
  EXPECT_EQ(ignition::math::Pose3d(-1, -2, -3, IGN_PI, IGN_PI, 0),
            joint.RawPose());

  joint.SetPoseRelativeTo("link");
  EXPECT_EQ("link", joint.PoseRelativeTo());
  {
    auto semanticPose = joint.SemanticPose();
    EXPECT_EQ(joint.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose without graph
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  joint.SetName("test_joint");
  EXPECT_EQ("test_joint", joint.Name());

  joint.SetParentLinkName("parent");
  EXPECT_EQ("parent", joint.ParentLinkName());

  joint.SetChildLinkName("child");
  EXPECT_EQ("child", joint.ChildLinkName());

  std::string body;
  EXPECT_FALSE(joint.ResolveChildLink(body).empty());
  EXPECT_TRUE(body.empty());
  EXPECT_FALSE(joint.ResolveParentLink(body).empty());
  EXPECT_TRUE(body.empty());

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

  EXPECT_EQ(nullptr, joint.Axis(0));
  EXPECT_EQ(nullptr, joint.Axis(1));
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz(ignition::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(ignition::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);
  ASSERT_TRUE(nullptr != joint.Axis(0));
  ASSERT_TRUE(nullptr != joint.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint.Axis(1)->Xyz());

  EXPECT_DOUBLE_EQ(1.0, joint.ThreadPitch());
  const double threadPitch = 0.1;
  joint.SetThreadPitch(threadPitch);
  EXPECT_DOUBLE_EQ(threadPitch, joint.ThreadPitch());

  EXPECT_EQ(0u, joint.SensorCount());
  EXPECT_EQ(nullptr, joint.SensorByIndex(0));
  EXPECT_EQ(nullptr, joint.SensorByIndex(1));
  EXPECT_EQ(nullptr, joint.SensorByName("empty"));
  EXPECT_FALSE(joint.SensorNameExists(""));
  EXPECT_FALSE(joint.SensorNameExists("default"));
}

/////////////////////////////////////////////////
TEST(DOMJoint, MoveConstructor)
{
  sdf::Joint joint;
  joint.SetName("test_joint");
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz(ignition::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(ignition::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);

  sdf::Joint joint2(std::move(joint));

  EXPECT_EQ("test_joint", joint2.Name());
  ASSERT_TRUE(nullptr != joint2.Axis(0));
  ASSERT_TRUE(nullptr != joint2.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint2.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint2.Axis(1)->Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJoint, CopyConstructor)
{
  sdf::Joint joint;
  joint.SetName("test_joint");
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz(ignition::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(ignition::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);

  sdf::Joint joint2(joint);

  EXPECT_EQ("test_joint", joint.Name());
  ASSERT_TRUE(nullptr != joint.Axis(0));
  ASSERT_TRUE(nullptr != joint.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint.Axis(1)->Xyz());

  EXPECT_EQ("test_joint", joint2.Name());
  ASSERT_TRUE(nullptr != joint2.Axis(0));
  ASSERT_TRUE(nullptr != joint2.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint2.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint2.Axis(1)->Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJoint, MoveAssignment)
{
  sdf::Joint joint;
  joint.SetName("test_joint");
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz(ignition::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(ignition::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);

  sdf::Joint joint2;
  joint2 = std::move(joint);

  EXPECT_EQ("test_joint", joint2.Name());
  ASSERT_TRUE(nullptr != joint2.Axis(0));
  ASSERT_TRUE(nullptr != joint2.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint2.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint2.Axis(1)->Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJoint, CopyAssignment)
{
  sdf::Joint joint;
  joint.SetName("test_joint");
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz(ignition::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(ignition::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);

  sdf::Joint joint2;
  joint2 = joint;

  EXPECT_EQ("test_joint", joint.Name());
  ASSERT_TRUE(nullptr != joint.Axis(0));
  ASSERT_TRUE(nullptr != joint.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint.Axis(1)->Xyz());

  EXPECT_EQ("test_joint", joint2.Name());
  ASSERT_TRUE(nullptr != joint2.Axis(0));
  ASSERT_TRUE(nullptr != joint2.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint2.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint2.Axis(1)->Xyz());
}

/////////////////////////////////////////////////
TEST(DOMJoint, CopyAssignmentAfterMove)
{
  sdf::Joint joint1;
  joint1.SetName("test_joint1");
  sdf::JointAxis joint1Axis;
  EXPECT_TRUE(joint1Axis.SetXyz(ignition::math::Vector3d(1, 0, 0)).empty());
  joint1.SetAxis(0, joint1Axis);
  sdf::JointAxis joint1Axis1;
  EXPECT_TRUE(joint1Axis1.SetXyz(ignition::math::Vector3d(0, 1, 0)).empty());
  joint1.SetAxis(1, joint1Axis1);

  sdf::Joint joint2;
  joint2.SetName("test_joint2");
  sdf::JointAxis joint2Axis;
  EXPECT_TRUE(joint2Axis.SetXyz(ignition::math::Vector3d(0, 0, 1)).empty());
  joint2.SetAxis(0, joint2Axis);
  sdf::JointAxis joint2Axis1;
  EXPECT_TRUE(joint2Axis1.SetXyz(ignition::math::Vector3d(-1, 0, 0)).empty());
  joint2.SetAxis(1, joint2Axis1);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Joint tmp = std::move(joint1);
  joint1 = joint2;
  joint2 = tmp;

  EXPECT_EQ("test_joint2", joint1.Name());
  ASSERT_TRUE(nullptr != joint1.Axis(0));
  ASSERT_TRUE(nullptr != joint1.Axis(1));
  EXPECT_EQ(joint2Axis.Xyz(), joint1.Axis(0)->Xyz());
  EXPECT_EQ(joint2Axis1.Xyz(), joint1.Axis(1)->Xyz());

  EXPECT_EQ("test_joint1", joint2.Name());
  ASSERT_TRUE(nullptr != joint2.Axis(0));
  ASSERT_TRUE(nullptr != joint2.Axis(1));
  EXPECT_EQ(joint1Axis.Xyz(), joint2.Axis(0)->Xyz());
  EXPECT_EQ(joint1Axis1.Xyz(), joint2.Axis(1)->Xyz());
}
