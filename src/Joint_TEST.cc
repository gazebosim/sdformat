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
#include <gz/math/Pose3.hh>
#include "sdf/Joint.hh"
#include "sdf/Sensor.hh"
#include "sdf/JointAxis.hh"

/////////////////////////////////////////////////
TEST(DOMJoint, Construction)
{
  sdf::Joint joint;
  EXPECT_TRUE(joint.Name().empty());
  EXPECT_EQ(sdf::JointType::INVALID, joint.Type());

  EXPECT_TRUE(joint.ParentName().empty());
  EXPECT_TRUE(joint.ChildName().empty());

  EXPECT_EQ(gz::math::Pose3d::Zero, joint.RawPose());
  EXPECT_TRUE(joint.PoseRelativeTo().empty());
  EXPECT_EQ(nullptr, joint.Element());
  {
    auto semanticPose = joint.SemanticPose();
    EXPECT_EQ(gz::math::Pose3d::Zero, semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose without graph
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  joint.SetRawPose({-1, -2, -3, GZ_PI, GZ_PI, 0});
  EXPECT_EQ(gz::math::Pose3d(-1, -2, -3, GZ_PI, GZ_PI, 0),
            joint.RawPose());

  joint.SetPoseRelativeTo("link");
  EXPECT_EQ("link", joint.PoseRelativeTo());
  {
    auto semanticPose = joint.SemanticPose();
    EXPECT_EQ(joint.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("link", semanticPose.RelativeTo());
    gz::math::Pose3d pose;
    // expect errors when trying to resolve pose without graph
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  joint.SetName("test_joint");
  EXPECT_EQ("test_joint", joint.Name());

  joint.SetParentName("parent2");
  EXPECT_EQ("parent2", joint.ParentName());

  joint.SetChildName("child2");
  EXPECT_EQ("child2", joint.ChildName());

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
  EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);
  ASSERT_TRUE(nullptr != joint.Axis(0));
  ASSERT_TRUE(nullptr != joint.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint.Axis(1)->Xyz());

  // Default thread pitch
  EXPECT_DOUBLE_EQ(1.0, joint.ScrewThreadPitch());
  EXPECT_DOUBLE_EQ(-2*GZ_PI, joint.ThreadPitch());

  // Set and check thread pitch
  const double threadPitch = 0.1;
  joint.SetScrewThreadPitch(threadPitch);
  EXPECT_DOUBLE_EQ(threadPitch, joint.ScrewThreadPitch());
  EXPECT_DOUBLE_EQ(-2*GZ_PI / threadPitch, joint.ThreadPitch());
  joint.SetThreadPitch(threadPitch);
  EXPECT_DOUBLE_EQ(threadPitch, joint.ThreadPitch());
  EXPECT_DOUBLE_EQ(-2*GZ_PI / threadPitch, joint.ScrewThreadPitch());

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
  EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
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
  EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
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
  EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
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
  EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
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
  EXPECT_TRUE(joint1Axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint1.SetAxis(0, joint1Axis);
  sdf::JointAxis joint1Axis1;
  EXPECT_TRUE(joint1Axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
  joint1.SetAxis(1, joint1Axis1);

  sdf::Joint joint2;
  joint2.SetName("test_joint2");
  sdf::JointAxis joint2Axis;
  EXPECT_TRUE(joint2Axis.SetXyz(gz::math::Vector3d(0, 0, 1)).empty());
  joint2.SetAxis(0, joint2Axis);
  sdf::JointAxis joint2Axis1;
  EXPECT_TRUE(joint2Axis1.SetXyz(gz::math::Vector3d(-1, 0, 0)).empty());
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

/////////////////////////////////////////////////
TEST(DOMJoint, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Joint joint;
  joint.SetRawPose({-1, -2, -3, 0, GZ_PI, 0});
  joint.SetPoseRelativeTo("link");
  joint.SetName("test_joint");
  joint.SetParentName("parent");
  joint.SetChildName("child");
  joint.SetType(sdf::JointType::BALL);
  sdf::JointAxis axis;
  EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d(1, 0, 0)).empty());
  joint.SetAxis(0, axis);
  sdf::JointAxis axis1;
  EXPECT_TRUE(axis1.SetXyz(gz::math::Vector3d(0, 1, 0)).empty());
  joint.SetAxis(1, axis1);

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      sdf::Sensor sensor;
      sensor.SetName("sensor" + std::to_string(i));
      EXPECT_TRUE(joint.AddSensor(sensor));
      EXPECT_FALSE(joint.AddSensor(sensor));
    }
    if (j == 0)
      joint.ClearSensors();
  }

  sdf::ElementPtr jointElem = joint.ToElement();
  EXPECT_NE(nullptr, jointElem);
  EXPECT_EQ(nullptr, joint.Element());

  // verify values after loading the element back
  sdf::Joint joint2;
  joint2.Load(jointElem);

  EXPECT_EQ(gz::math::Pose3d(-1, -2, -3, 0, GZ_PI, 0),
            joint2.RawPose());
  EXPECT_EQ("link", joint2.PoseRelativeTo());
  EXPECT_EQ("test_joint", joint2.Name());
  EXPECT_EQ("parent", joint2.ParentName());
  EXPECT_EQ("child", joint2.ChildName());
  EXPECT_EQ(sdf::JointType::BALL, joint2.Type());
  ASSERT_TRUE(nullptr != joint2.Axis(0));
  ASSERT_TRUE(nullptr != joint2.Axis(1));
  EXPECT_EQ(axis.Xyz(), joint2.Axis(0)->Xyz());
  EXPECT_EQ(axis1.Xyz(), joint2.Axis(1)->Xyz());

  EXPECT_EQ(joint.SensorCount(), joint.SensorCount());
  for (uint64_t i = 0; i < joint.SensorCount(); ++i)
    EXPECT_NE(nullptr, joint.SensorByIndex(i));

  // make changes to DOM and verify ToElement produces updated values
  joint2.SetParentName("new_parent");
  sdf::ElementPtr joint2Elem = joint2.ToElement();
  EXPECT_NE(nullptr, joint2Elem);
  sdf::Joint joint3;
  joint3.Load(joint2Elem);
  EXPECT_EQ("new_parent", joint3.ParentName());
}

/////////////////////////////////////////////////
TEST(DOMJoint, MutableByIndex)
{
  sdf::Joint joint;

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(joint.AddSensor(sensor));

  // Modify the sensor
  sdf::Sensor *s = joint.SensorByIndex(0);
  ASSERT_NE(nullptr, s);
  EXPECT_EQ("sensor1", s->Name());
  s->SetName("sensor2");
  EXPECT_EQ("sensor2", joint.SensorByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST(DOMJoint, MutableByName)
{
  sdf::Joint joint;

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(joint.AddSensor(sensor));

  // Modify the sensor
  sdf::Sensor *s = joint.SensorByName("sensor1");
  ASSERT_NE(nullptr, s);
  EXPECT_EQ("sensor1", s->Name());
  s->SetName("sensor2");
  EXPECT_EQ("sensor2", joint.SensorByName("sensor2")->Name());
}
