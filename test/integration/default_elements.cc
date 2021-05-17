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

  sdf::ElementPtr rootPtr = root.Element();
  EXPECT_TRUE(rootPtr->GetExplicitlySetInFile());

  sdf::ElementPtr worldPtr = rootPtr->GetFirstElement();
  EXPECT_TRUE(worldPtr->GetExplicitlySetInFile());

  sdf::ElementPtr roadPtr = worldPtr->GetFirstElement();
  EXPECT_TRUE(roadPtr->GetExplicitlySetInFile());

  sdf::ElementPtr roadWidthPtr = roadPtr->GetFirstElement();
  EXPECT_FALSE(roadWidthPtr->GetExplicitlySetInFile());

  sdf::ElementPtr roadPointPtr = roadWidthPtr->GetNextElement();
  EXPECT_FALSE(roadPointPtr->GetExplicitlySetInFile());

  sdf::ElementPtr sphericalCoordsPtr = roadPtr->GetNextElement();
  EXPECT_TRUE(sphericalCoordsPtr->GetExplicitlySetInFile());

  sdf::ElementPtr surfaceModel = sphericalCoordsPtr->GetFirstElement();
  EXPECT_FALSE(surfaceModel->GetExplicitlySetInFile());

  sdf::ElementPtr latitudeDegPtr = surfaceModel->GetNextElement();
  EXPECT_FALSE(latitudeDegPtr->GetExplicitlySetInFile());

  sdf::ElementPtr longitudeDegPtr = latitudeDegPtr->GetNextElement();
  EXPECT_FALSE(longitudeDegPtr->GetExplicitlySetInFile());

  sdf::ElementPtr elevationPtr = longitudeDegPtr->GetNextElement();
  EXPECT_FALSE(elevationPtr->GetExplicitlySetInFile());

  sdf::ElementPtr headingDegPtr = elevationPtr->GetNextElement();
  EXPECT_FALSE(headingDegPtr->GetExplicitlySetInFile());

  sdf::ElementPtr gravityPtr = sphericalCoordsPtr->GetNextElement();
  EXPECT_FALSE(gravityPtr->GetExplicitlySetInFile());

  sdf::ElementPtr magneticFieldPtr = gravityPtr->GetNextElement();
  EXPECT_FALSE(magneticFieldPtr->GetExplicitlySetInFile());

  sdf::ElementPtr atmosphereTypePtr = magneticFieldPtr->GetNextElement();
  EXPECT_FALSE(atmosphereTypePtr->GetExplicitlySetInFile());

  sdf::ElementPtr physicsPtr = atmosphereTypePtr->GetNextElement();
  EXPECT_FALSE(physicsPtr->GetExplicitlySetInFile());

  sdf::ElementPtr maxStepSizePtr = physicsPtr->GetFirstElement();
  EXPECT_FALSE(maxStepSizePtr->GetExplicitlySetInFile());

  sdf::ElementPtr realTimeFactorPtr = maxStepSizePtr->GetNextElement();
  EXPECT_FALSE(realTimeFactorPtr->GetExplicitlySetInFile());

  sdf::ElementPtr realTimeUpdate = realTimeFactorPtr->GetNextElement();
  EXPECT_FALSE(realTimeUpdate->GetExplicitlySetInFile());

  sdf::ElementPtr scenePtr = physicsPtr->GetNextElement();
  EXPECT_FALSE(scenePtr->GetExplicitlySetInFile());

  sdf::ElementPtr ambientPtr = scenePtr->GetFirstElement();
  EXPECT_FALSE(ambientPtr->GetExplicitlySetInFile());

  sdf::ElementPtr backgroundPtr = ambientPtr->GetNextElement();
  EXPECT_FALSE(backgroundPtr->GetExplicitlySetInFile());

  sdf::ElementPtr shadowsPtr = backgroundPtr->GetNextElement();
  EXPECT_FALSE(shadowsPtr->GetExplicitlySetInFile());
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

  sdf::ElementPtr rootPtr = root.Element();
  EXPECT_TRUE(rootPtr->GetExplicitlySetInFile());

  sdf::ElementPtr modelPtr = rootPtr->GetFirstElement();
  EXPECT_TRUE(modelPtr->GetExplicitlySetInFile());

  sdf::ElementPtr link1Ptr = modelPtr->GetFirstElement();
  EXPECT_TRUE(link1Ptr->GetExplicitlySetInFile());

  sdf::ElementPtr link2Ptr = link1Ptr->GetNextElement();
  EXPECT_TRUE(link2Ptr->GetExplicitlySetInFile());

  sdf::ElementPtr jointPtr = link2Ptr->GetNextElement();
  EXPECT_TRUE(jointPtr->GetExplicitlySetInFile());

  sdf::ElementPtr parentPtr = jointPtr->GetFirstElement();
  EXPECT_TRUE(parentPtr->GetExplicitlySetInFile());

  sdf::ElementPtr childPtr = parentPtr->GetNextElement();
  EXPECT_TRUE(childPtr->GetExplicitlySetInFile());

  sdf::ElementPtr axisPtr = childPtr->GetNextElement();
  EXPECT_TRUE(axisPtr->GetExplicitlySetInFile());

  sdf::ElementPtr xyzPtr = axisPtr->GetFirstElement();
  EXPECT_FALSE(xyzPtr->GetExplicitlySetInFile());

  sdf::ElementPtr limitPtr = xyzPtr->GetNextElement();
  EXPECT_FALSE(limitPtr->GetExplicitlySetInFile());

  sdf::ElementPtr lowerLimitPtr = limitPtr->GetFirstElement();
  EXPECT_FALSE(lowerLimitPtr->GetExplicitlySetInFile());

  sdf::ElementPtr upperLimitPtr = lowerLimitPtr->GetNextElement();
  EXPECT_FALSE(upperLimitPtr->GetExplicitlySetInFile());
}
