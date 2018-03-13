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
#include <gtest/gtest.h>

#include "sdf/Box.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Element.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Plane.hh"
#include "sdf/Root.hh"
#include "sdf/Sphere.hh"
#include "sdf/Types.hh"
#include "sdf/World.hh"
#include "sdf/Visual.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMGeometry, Shapes)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "shapes.sdf");

  // Load the SDF file
  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());

  // Get the first model
  const sdf::Model *model = root.ModelByIndex(0);
  ASSERT_TRUE(model != nullptr);

  const sdf::Link *link = model->LinkByIndex(0);
  ASSERT_TRUE(link != nullptr);

  // Test box visual and collision
  const sdf::Visual *boxVis = link->VisualByName("box_vis");
  ASSERT_TRUE(boxVis != nullptr);

  const sdf::Box *boxVisGeom = boxVis->BoxGeom();
  ASSERT_TRUE(boxVisGeom != nullptr);
  EXPECT_EQ(ignition::math::Vector3d(1, 2, 3), boxVisGeom->Size());

  // Test cylinder visual and collision
  const sdf::Visual *cylinderVis = link->VisualByName("cylinder_vis");
  ASSERT_TRUE(cylinderVis != nullptr);

  const sdf::Cylinder *cylinderVisGeom = cylinderVis->CylinderGeom();
  ASSERT_TRUE(cylinderVisGeom != nullptr);
  EXPECT_DOUBLE_EQ(2.1, cylinderVisGeom->Radius());
  EXPECT_DOUBLE_EQ(10.2, cylinderVisGeom->Length());

  // Test sphere visual and collision
  const sdf::Visual *sphereVis = link->VisualByName("sphere_vis");
  ASSERT_TRUE(sphereVis != nullptr);

  const sdf::Sphere *sphereVisGeom = sphereVis->SphereGeom();
  ASSERT_TRUE(sphereVisGeom != nullptr);
  EXPECT_DOUBLE_EQ(100.2, sphereVisGeom->Radius());

  // Test plane visual and collision
  const sdf::Visual *planeVis = link->VisualByName("plane_vis");
  ASSERT_TRUE(planeVis != nullptr);

  const sdf::Plane *planeVisGeom = planeVis->PlaneGeom();
  ASSERT_TRUE(planeVisGeom != nullptr);
  EXPECT_EQ(ignition::math::Vector3d::UnitY, planeVisGeom->Normal());
  EXPECT_EQ(ignition::math::Vector2d(2, 4), planeVisGeom->Size());
}
