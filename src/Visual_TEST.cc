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
#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMVisual, Construction)
{
  sdf::Visual visual;
  EXPECT_TRUE(visual.Name().empty());

  visual.SetName("test_visual");
  EXPECT_EQ(visual.Name(), "test_visual");

  ASSERT_NE(nullptr, visual.Geom());
  EXPECT_EQ(sdf::GeometryType::EMPTY, visual.Geom()->Type());
  EXPECT_EQ(nullptr, visual.Geom()->BoxShape());
  EXPECT_EQ(nullptr, visual.Geom()->CylinderShape());
  EXPECT_EQ(nullptr, visual.Geom()->PlaneShape());
  EXPECT_EQ(nullptr, visual.Geom()->SphereShape());
}
