/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <vector>
#include <gtest/gtest.h>

#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Types.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMJointAxis, Complete)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "joint_complete.sdf");

  // Load the SDF file
  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_NE(nullptr, model);

  // The model should have nine joints.
  EXPECT_EQ(9u, model->JointCount());

  std::vector<sdf::JointType> jointTypes =
  {
    sdf::JointType::REVOLUTE,
    sdf::JointType::BALL,
    sdf::JointType::CONTINUOUS,
    sdf::JointType::FIXED,
    sdf::JointType::GEARBOX,
    sdf::JointType::PRISMATIC,
    sdf::JointType::REVOLUTE2,
    sdf::JointType::SCREW,
    sdf::JointType::UNIVERSAL,
  };
  for (size_t i = 0; i < jointTypes.size(); ++i)
  {
    EXPECT_EQ(jointTypes[i], model->JointByIndex(i)->Type()) << i;
  }

  // Get the joint
  const sdf::Joint *joint = model->JointByIndex(0);
  ASSERT_NE(nullptr, joint);
  ASSERT_NE(nullptr, joint->Element());
  EXPECT_EQ(sdf::JointType::REVOLUTE, joint->Type());

  // Get the first axis
  const sdf::JointAxis *axis = joint->Axis();
  ASSERT_NE(nullptr, axis);
  ASSERT_NE(nullptr, axis->Element());

  // Get the second axis
  const sdf::JointAxis *axis2 = joint->Axis(1);
  ASSERT_NE(nullptr, axis2);

  EXPECT_EQ(ignition::math::Vector3d::UnitZ, axis->Xyz());
  EXPECT_EQ(ignition::math::Vector3d::UnitY, axis2->Xyz());

  EXPECT_TRUE(axis->UseParentModelFrame());
  EXPECT_FALSE(axis2->UseParentModelFrame());

  EXPECT_DOUBLE_EQ(-0.5, axis->Lower());
  EXPECT_DOUBLE_EQ(0.5, axis->Upper());
  EXPECT_DOUBLE_EQ(-1.0, axis2->Lower());
  EXPECT_DOUBLE_EQ(1.0, axis2->Upper());

  EXPECT_DOUBLE_EQ(123.4, axis->Effort());
  EXPECT_DOUBLE_EQ(0.5, axis2->Effort());

  EXPECT_DOUBLE_EQ(12.0, axis->MaxVelocity());
  EXPECT_DOUBLE_EQ(200.0, axis2->MaxVelocity());

  EXPECT_DOUBLE_EQ(0.1, axis->Damping());
  EXPECT_DOUBLE_EQ(0.0, axis2->Damping());

  EXPECT_DOUBLE_EQ(0.2, axis->Friction());
  EXPECT_DOUBLE_EQ(0.0, axis2->Friction());

  EXPECT_DOUBLE_EQ(1.3, axis->SpringReference());
  EXPECT_DOUBLE_EQ(0.0, axis2->SpringReference());

  EXPECT_DOUBLE_EQ(10.6, axis->SpringStiffness());
  EXPECT_DOUBLE_EQ(0.0, axis2->SpringStiffness());
}
