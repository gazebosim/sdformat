/*
 * Copyright 2020 Open Source Robotics Foundation
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

#include "sdf/Collision.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/Surface.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"

#include "test_config.hh"

//////////////////////////////////////////////////
TEST(DOMSurface, Shapes)
{
  const auto testFile =
    sdf::testing::TestFile("sdf", "shapes.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  const auto model = root.Model();
  ASSERT_NE(nullptr, model);

  const auto link = model->LinkByIndex(0);
  ASSERT_NE(nullptr, link);

  const auto boxCol = link->CollisionByName("box_col");
  ASSERT_NE(nullptr, boxCol);
  ASSERT_NE(nullptr, boxCol->Surface());
  ASSERT_NE(nullptr, boxCol->Surface()->Contact());
  EXPECT_EQ(boxCol->Surface()->Contact()->CollideBitmask(), 0xAB);
  EXPECT_DOUBLE_EQ(boxCol->Surface()->Friction()->ODE()->Mu(), 0.6);
  EXPECT_DOUBLE_EQ(boxCol->Surface()->Friction()->ODE()->Mu2(), 0.7);
  EXPECT_DOUBLE_EQ(boxCol->Surface()->Friction()->ODE()->Slip1(), 4);
  EXPECT_DOUBLE_EQ(boxCol->Surface()->Friction()->ODE()->Slip2(), 5);
  EXPECT_EQ(boxCol->Surface()->Friction()->ODE()->Fdir1(),
            gz::math::Vector3d(1.2, 3.4, 5.6));

  EXPECT_DOUBLE_EQ(1.6,
      boxCol->Surface()->Friction()->BulletFriction()->Friction());
  EXPECT_DOUBLE_EQ(1.7,
      boxCol->Surface()->Friction()->BulletFriction()->Friction2());
  EXPECT_EQ(boxCol->Surface()->Friction()->BulletFriction()->Fdir1(),
            gz::math::Vector3d(6.5, 4.3, 2.1));
  EXPECT_DOUBLE_EQ(1.5,
      boxCol->Surface()->Friction()->BulletFriction()->RollingFriction());

  EXPECT_DOUBLE_EQ(5.1,
                   boxCol->Surface()->Friction()->Torsional()->Coefficient());
  EXPECT_FALSE(boxCol->Surface()->Friction()->Torsional()->UsePatchRadius());
  EXPECT_DOUBLE_EQ(1.3,
                   boxCol->Surface()->Friction()->Torsional()->PatchRadius());
  EXPECT_DOUBLE_EQ(3.7,
                   boxCol->Surface()->Friction()->Torsional()->SurfaceRadius());
  EXPECT_DOUBLE_EQ(3.1, boxCol->Surface()->Friction()->Torsional()->ODESlip());
}
