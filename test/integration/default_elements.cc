/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>

#include "sdf/SDFImpl.hh"
#include "sdf/parser.hh"
#include "sdf/Frame.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(ExplicitlySetInFile, EmptyRoadSphCoords)
{
  const std::string test_file =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty_road_sph_coords.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(test_file);
  EXPECT_TRUE(errors.empty());

  sdf::ElementPtr elementPtr = root.Element();
  EXPECT_TRUE(root.Element()->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  sdf::ElementPtr road_ptr = elementPtr->GetFirstElement();
  EXPECT_FALSE(road_ptr->GetExplicitlySetInFile());

  road_ptr = road_ptr->GetNextElement();
  EXPECT_FALSE(road_ptr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  sdf::ElementPtr spherical_coords_ptr = elementPtr->GetFirstElement();
  EXPECT_FALSE(spherical_coords_ptr->GetExplicitlySetInFile());

  spherical_coords_ptr = elementPtr->GetNextElement();
  EXPECT_FALSE(spherical_coords_ptr->GetExplicitlySetInFile());

  spherical_coords_ptr = elementPtr->GetNextElement();
  EXPECT_FALSE(spherical_coords_ptr->GetExplicitlySetInFile());

  spherical_coords_ptr = elementPtr->GetNextElement();
  EXPECT_FALSE(spherical_coords_ptr->GetExplicitlySetInFile());

  spherical_coords_ptr = elementPtr->GetNextElement();
  EXPECT_FALSE(spherical_coords_ptr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  sdf::ElementPtr physics_ptr = elementPtr->GetFirstElement();
  EXPECT_FALSE(physics_ptr->GetExplicitlySetInFile());

  physics_ptr = physics_ptr->GetNextElement();
  EXPECT_FALSE(physics_ptr->GetExplicitlySetInFile());

  physics_ptr = physics_ptr->GetNextElement();
  EXPECT_FALSE(physics_ptr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  sdf::ElementPtr scene_ptr = elementPtr->GetFirstElement();
  EXPECT_FALSE(scene_ptr->GetExplicitlySetInFile());

  scene_ptr = scene_ptr->GetNextElement();
  EXPECT_FALSE(scene_ptr->GetExplicitlySetInFile());

  scene_ptr = scene_ptr->GetNextElement();
  EXPECT_FALSE(scene_ptr->GetExplicitlySetInFile());
}

//////////////////////////////////////////////////
TEST(ExplicitlySetInFile, EmptyAxis)
{
  const std::string test_file =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty_axis.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(test_file);
  EXPECT_TRUE(errors.empty());

  sdf::ElementPtr elementPtr = root.Element();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_TRUE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetFirstElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());

  elementPtr = elementPtr->GetNextElement();
  EXPECT_FALSE(elementPtr->GetExplicitlySetInFile());
}
