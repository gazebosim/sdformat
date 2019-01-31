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
#include "sdf/parser.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Collision.hh"
#include "sdf/Geometry.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Visual.hh"
#include "test_config.h"

using namespace ignition::math;

/////////////////////////////////////////////////
TEST(Urdf, Construction)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "double_pendulum.sdf");

  std::string urdfString = sdf::toUrdf(testFile);

  sdf::Root root;
  root.LoadSdfString(urdfString);

  /*sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  sdf::readString(urdfString, sdfParsed);
  sdfParsed->PrintValues();
  */

  const sdf::Model *model = root.ModelByIndex(0);
  EXPECT_NE(nullptr, model);
  // EXPECT_EQ(Pose3d(1, 0, 0, 0, 0, 0), *model->Pose());

  // Get the base link
  const sdf::Link *baseLink = model->LinkByName("base");
  ASSERT_NE(nullptr, baseLink);
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), baseLink->Pose());
  EXPECT_DOUBLE_EQ(100, baseLink->Inertial().MassMatrix().Mass());

  // Get the base link's vis_plate_on_ground visual
  const sdf::Visual *visPlate = baseLink->VisualByName(
      "base_fixed_joint_lump__vis_plate_on_ground_visual");
  ASSERT_NE(nullptr, visPlate);
  EXPECT_EQ(Pose3d(0, 0, 0.01, 0, 0, 0), visPlate->Pose());
  EXPECT_DOUBLE_EQ(0.8, visPlate->Geom()->CylinderShape()->Radius());
  EXPECT_DOUBLE_EQ(0.02, visPlate->Geom()->CylinderShape()->Length());

  // Get the base link's col_plate_on_ground collision
  const sdf::Collision *colPlate = baseLink->CollisionByName(
      "base_fixed_joint_lump__col_plate_on_ground_collision");
  ASSERT_NE(nullptr, colPlate);
  EXPECT_EQ(Pose3d(0, 0, 0.01, 0, 0, 0), colPlate->Pose());
  EXPECT_DOUBLE_EQ(0.8, colPlate->Geom()->CylinderShape()->Radius());
  EXPECT_DOUBLE_EQ(0.02, colPlate->Geom()->CylinderShape()->Length());

  // Get the base link's vis_pole visual
  const sdf::Visual *visPole = baseLink->VisualByName(
      "base_fixed_joint_lump__vis_pole_visual_1");
  ASSERT_NE(nullptr, visPole);
  EXPECT_EQ(Pose3d(-0.275, 0, 1.1, 0, 0, 0), visPole->Pose());
  EXPECT_DOUBLE_EQ(0.2, visPole->Geom()->BoxShape()->Size().X());
  EXPECT_DOUBLE_EQ(0.2, visPole->Geom()->BoxShape()->Size().Y());
  EXPECT_DOUBLE_EQ(2.2, visPole->Geom()->BoxShape()->Size().Z());

  // Get the base link's col_pole collision
  const sdf::Collision *colPole = baseLink->CollisionByName(
      "base_fixed_joint_lump__col_pole_collision_1");
  ASSERT_NE(nullptr, colPole);
  EXPECT_EQ(Pose3d(-0.275, 0, 1.1, 0, 0, 0), colPole->Pose());
  EXPECT_DOUBLE_EQ(0.2, colPole->Geom()->BoxShape()->Size().X());
  EXPECT_DOUBLE_EQ(0.2, colPole->Geom()->BoxShape()->Size().Y());
  EXPECT_DOUBLE_EQ(2.2, colPole->Geom()->BoxShape()->Size().Z());

  // Get the upper link
  const sdf::Link *upperLink = model->LinkByName("upper_link");
  ASSERT_NE(nullptr, upperLink);

  EXPECT_EQ(Pose3d(0, 0, 2.1, -1.5708, 0, 0), upperLink->Pose());
  EXPECT_EQ(Pose3d(0, 0, 2.1, -1.5708, 0, 0), upperLink->Pose("base_link"));

  const sdf::Joint *upperJoint = model->JointByName("upper_joint");
  EXPECT_EQ(Pose3d(0, 0, 0, 0, 0, 0), upperJoint->Pose("upper_link"));
  EXPECT_EQ("upper_joint", upperJoint->Name());
}
