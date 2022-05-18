/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "sdf/Link.hh"
#include "sdf/SemanticPose.hh"

/////////////////////////////////////////////////
TEST(SemanticPose, Construction)
{
  sdf::Link link;
  gz::math::Pose3d rawPose(1, 0, 0, 0, 0, 0);
  link.SetRawPose(rawPose);
  const sdf::SemanticPose &semPose = link.SemanticPose();
  EXPECT_EQ(rawPose, semPose.RawPose());
}

/////////////////////////////////////////////////
TEST(SemanticPose, CopyConstructor)
{
  sdf::Link link;
  gz::math::Pose3d rawPose(1, 0, 0, 0, 0, 0);
  link.SetRawPose(rawPose);
  sdf::SemanticPose semPose1 = link.SemanticPose();

  sdf::SemanticPose semPose2(semPose1);
  EXPECT_EQ(rawPose, semPose2.RawPose());
}

/////////////////////////////////////////////////
TEST(SemanticPose, CopyAssignmentOperator)
{
  sdf::Link link;
  gz::math::Pose3d rawPose(1, 0, 0, 0, 0, 0);
  link.SetRawPose(rawPose);
  sdf::SemanticPose semPose1 = link.SemanticPose();

  // Create another SemanticPose object from another Link;
  sdf::Link link2;
  sdf::SemanticPose semPose2 = link2.SemanticPose();
  EXPECT_EQ(gz::math::Pose3d::Zero, semPose2.RawPose());

  semPose2 = semPose1;
  EXPECT_EQ(rawPose, semPose2.RawPose());
}

/////////////////////////////////////////////////
TEST(SemanticPose, MoveConstructor)
{
  sdf::Link link;
  gz::math::Pose3d rawPose(1, 0, 0, 0, 0, 0);
  link.SetRawPose(rawPose);
  sdf::SemanticPose semPose1 = link.SemanticPose();

  sdf::SemanticPose semPose2(std::move(semPose1));
  EXPECT_EQ(rawPose, semPose2.RawPose());
}

/////////////////////////////////////////////////
TEST(SemanticPose, MoveAssignmentOperator)
{
  sdf::Link link;
  gz::math::Pose3d rawPose(1, 0, 0, 0, 0, 0);
  link.SetRawPose(rawPose);
  sdf::SemanticPose semPose1 = link.SemanticPose();

  // Create another SemanticPose object from another Link;
  sdf::Link link2;
  sdf::SemanticPose semPose2 = link2.SemanticPose();
  EXPECT_EQ(gz::math::Pose3d::Zero, semPose2.RawPose());

  semPose2 = std::move(semPose1);
  EXPECT_EQ(rawPose, semPose2.RawPose());
}

/////////////////////////////////////////////////
TEST(SemanticPose, CopyAssignmentAfterMove)
{
  sdf::Link link;
  gz::math::Pose3d rawPose(1, 0, 0, 0, 0, 0);
  link.SetRawPose(rawPose);
  sdf::SemanticPose semPose1 = link.SemanticPose();

  // Create another SemanticPose object from another Link;
  sdf::Link link2;
  sdf::SemanticPose semPose2 = link2.SemanticPose();
  EXPECT_EQ(gz::math::Pose3d::Zero, semPose2.RawPose());

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::SemanticPose tmp = std::move(semPose1);
  semPose1 = semPose2;
  semPose2 = tmp;

  EXPECT_EQ(gz::math::Pose3d::Zero, semPose1.RawPose());
  EXPECT_EQ(rawPose, semPose2.RawPose());
}
