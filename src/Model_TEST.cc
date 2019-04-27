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
#include "sdf/Link.hh"
#include "sdf/Model.hh"

/////////////////////////////////////////////////
TEST(DOMModel, Construction)
{
  sdf::Model model;
  EXPECT_EQ(nullptr, model.Element());
  EXPECT_TRUE(model.Name().empty());

  model.SetName("test_model");
  EXPECT_EQ("test_model", model.Name());

  EXPECT_FALSE(model.Static());
  model.SetStatic(true);
  EXPECT_TRUE(model.Static());

  EXPECT_FALSE(model.SelfCollide());
  model.SetSelfCollide(true);
  EXPECT_TRUE(model.SelfCollide());

  EXPECT_TRUE(model.AllowAutoDisable());
  model.SetAllowAutoDisable(false);
  EXPECT_FALSE(model.AllowAutoDisable());

  EXPECT_FALSE(model.EnableWind());
  model.SetEnableWind(true);
  EXPECT_TRUE(model.EnableWind());

  EXPECT_EQ(0u, model.LinkCount());
  EXPECT_EQ(nullptr, model.LinkByIndex(0));
  EXPECT_EQ(nullptr, model.LinkByIndex(1));
  EXPECT_FALSE(model.LinkNameExists(""));
  EXPECT_FALSE(model.LinkNameExists("default"));

  EXPECT_EQ(0u, model.JointCount());
  EXPECT_EQ(nullptr, model.JointByIndex(0));
  EXPECT_EQ(nullptr, model.JointByIndex(1));
  EXPECT_FALSE(model.JointNameExists(""));
  EXPECT_FALSE(model.JointNameExists("default"));

  EXPECT_EQ(ignition::math::Pose3d::Zero, model.Pose());
  EXPECT_TRUE(model.PoseFrame().empty());

  model.SetPose({1, 2, 3, 0, 0, IGN_PI});
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, IGN_PI), model.Pose());

  model.SetPoseFrame("world");
  EXPECT_EQ("world", model.PoseFrame());
}

/////////////////////////////////////////////////
TEST(DOMModel, CopyConstructor)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2(model);
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, CopyAssignmentOperator)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2;
  model2 = model;
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, MoveConstructor)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2(model);
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, MoveAssignmentOperator)
{
  sdf::Model model;
  model.SetName("test_model");

  sdf::Model model2;
  model2 = std::move(model);
  EXPECT_EQ("test_model", model2.Name());
}

/////////////////////////////////////////////////
TEST(DOMModel, CopyAssignmentAfterMove)
{
  sdf::Model model1;
  model1.SetName("model1");

  sdf::Model model2;
  model2.SetName("model2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Model tmp = std::move(model1);
  model1 = model2;
  model2 = tmp;

  EXPECT_EQ("model2", model1.Name());
  EXPECT_EQ("model1", model2.Name());
}
