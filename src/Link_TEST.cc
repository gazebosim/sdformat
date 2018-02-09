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
#include "sdf/Collision.hh"
#include "sdf/Link.hh"
#include "sdf/Visual.hh"

/////////////////////////////////////////////////
TEST(DOMLink, Construction)
{
  sdf::Link link;
  EXPECT_TRUE(link.Name().empty());

  link.SetName("test_link");
  EXPECT_EQ("test_link", link.Name());

  EXPECT_EQ(0u, link.VisualCount());
  EXPECT_EQ(nullptr, link.VisualByIndex(0));
  EXPECT_EQ(nullptr, link.VisualByIndex(1));
  EXPECT_FALSE(link.VisualNameExists(""));
  EXPECT_FALSE(link.VisualNameExists("default"));

  // Get the default interial
  const ignition::math::Inertiald inertial = link.Inertial();
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Z());

  EXPECT_EQ(0u, link.CollisionCount());
  EXPECT_EQ(nullptr, link.CollisionByIndex(0));
  EXPECT_EQ(nullptr, link.CollisionByIndex(1));
  EXPECT_FALSE(link.CollisionNameExists(""));
  EXPECT_FALSE(link.CollisionNameExists("default"));

  ignition::math::Inertiald inertial2 {
    {2.3, ignition::math::Vector3d(0.1, 0.2, 0.3),
      ignition::math::Vector3d(1.2, 2.3, 3.4)},
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0)};

  link.SetInertial(inertial2);

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(1.2, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.4, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
}
