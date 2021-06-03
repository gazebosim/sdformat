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
#include <gtest/gtest.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(Pose1_9, ModelPoses)
{
  using Pose = ignition::math::Pose3d;

  const std::string testFile = sdf::testing::TestFile(
      "sdf", "pose_1_9.sdf");

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  for (const auto e : errors)
    std::cout << e << std::endl;
  // std::cout << errors << std::endl;
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(SDF_PROTOCOL_VERSION, root.Version());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  std::cout << "modelWithEmptyPose" << std::endl;
  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_pose", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "modelWithPoseValue" << std::endl;
  model = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_pose_value", model->Name());
  EXPECT_EQ(Pose(1, 2, 3, 1, 2, 3), model->RawPose());

  std::cout << "modelWithEmptyTranslation" << std::endl;
  model = world->ModelByIndex(2);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_translation", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "modelWithTranslationValue" << std::endl;
  model = world->ModelByIndex(3);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_translation_value", model->Name());
  EXPECT_EQ(Pose(2, 3, 4, 0, 0, 0), model->RawPose());

  std::cout << "modelWithEmptyRPYDeg" << std::endl;
  model = world->ModelByIndex(4);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_rpy_deg", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "modelWithRPYDegValue" << std::endl;
  model = world->ModelByIndex(5);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_deg_value", model->Name());
  EXPECT_EQ(Pose(0, 0, 0, 0, IGN_DTOR(90), IGN_DTOR(180)), model->RawPose());

  std::cout << "modelWithEmptyRPYRad" << std::endl;
  model = world->ModelByIndex(6);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_rpy_rad", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "modelWithRPYRadValue" << std::endl;
  model = world->ModelByIndex(7);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_rpy_rad_value", model->Name());
  EXPECT_EQ(Pose(0, 0, 0, 1, 2, 3), model->RawPose());

  std::cout << "modelWithEmptyQuat" << std::endl;
  model = world->ModelByIndex(8);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_empty_quat", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  std::cout << "modelWithQuatValue" << std::endl;
  model = world->ModelByIndex(9);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_with_quat_value", model->Name());
  EXPECT_EQ(Pose(0, 0, 0, 0.7071068, 0.7071068, 0, 0), model->RawPose());
}

