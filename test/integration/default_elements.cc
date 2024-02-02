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

#include "test_config.hh"

//////////////////////////////////////////////////
TEST(ExplicitlySetInFile, EmptyRoadSphCoords)
{
  const auto testFile =
    sdf::testing::TestFile("sdf", "empty_road_sph_coords.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
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
  const auto testFile =
    sdf::testing::TestFile("sdf", "empty_axis.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
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

//////////////////////////////////////////////////
TEST(ExplicitlySetInFile, ToString)
{
  const auto testFile =
    sdf::testing::TestFile("sdf", "empty_road_sph_coords.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(root.Element()->ToString(""),
            root.Element()->ToString("", true, false));

  std::ostringstream stream;
  std::string version = SDF_VERSION;
  stream
    << "<sdf version='" << version << "'>\n"
    << "  <world name='default'>\n"
    << "    <road name='empty_road'>\n"
    << "    </road>\n"
    << "    <spherical_coordinates>\n"
    << "    </spherical_coordinates>\n"
    << "  </world>\n"
    << "</sdf>\n";

  EXPECT_EQ(root.Element()->ToString("", false, false), stream.str());

  stream.str(std::string());
  stream
    << "<sdf version='" << version << "'>\n"
    << "  <world name='default'>\n"
    << "    <road name='empty_road'>\n"
    << "      <width>1</width>\n"
    << "      <point>0 0 0</point>\n"
    << "    </road>\n"
    << "    <spherical_coordinates>\n"
    << "      <surface_model>EARTH_WGS84</surface_model>\n"
    << "      <latitude_deg>0</latitude_deg>\n"
    << "      <longitude_deg>0</longitude_deg>\n"
    << "      <elevation>0</elevation>\n"
    << "      <heading_deg>0</heading_deg>\n"
    << "    </spherical_coordinates>\n"
    << "    <gravity>0 0 -9.8</gravity>\n"
    << "    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05"
    << "</magnetic_field>\n"
    << "    <atmosphere type='adiabatic'/>\n"
    << "    <physics type='ode'>\n"
    << "      <max_step_size>0.001</max_step_size>\n"
    << "      <real_time_factor>1</real_time_factor>\n"
    << "      <real_time_update_rate>1000</real_time_update_rate>\n"
    << "    </physics>\n"
    << "    <scene>\n"
    << "      <ambient>0.4 0.4 0.4 1</ambient>\n"
    << "      <background>0.7 0.7 0.7 1</background>\n"
    << "      <shadows>true</shadows>\n"
    << "    </scene>\n"
    << "  </world>\n"
    << "</sdf>\n";

  sdf::PrintConfig config;
  config.SetOutPrecision(6);
  EXPECT_EQ(root.Element()->ToString("", config), stream.str());
  EXPECT_EQ(root.Element()->ToString("", true, false, config), stream.str());

  stream.str(std::string());
  stream
    << "<sdf version='" << version << "'>\n"
    << "  <world name='default'>\n"
    << "    <road name='empty_road'>\n"
    << "      <width>1</width>\n"
    << "      <point>0 0 0</point>\n"
    << "    </road>\n"
    << "    <spherical_coordinates>\n"
    << "      <surface_model>EARTH_WGS84</surface_model>\n"
    << "      <latitude_deg>0</latitude_deg>\n"
    << "      <longitude_deg>0</longitude_deg>\n"
    << "      <elevation>0</elevation>\n"
    << "      <heading_deg>0</heading_deg>\n"
    << "    </spherical_coordinates>\n"
    << "    <gravity>0 0 -9.8</gravity>\n"
    << "    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05"
    << "</magnetic_field>\n"
    << "    <atmosphere type='adiabatic'/>\n"
    << "    <physics name='default_physics' default='false' type='ode'>\n"
    << "      <max_step_size>0.001</max_step_size>\n"
    << "      <real_time_factor>1</real_time_factor>\n"
    << "      <real_time_update_rate>1000</real_time_update_rate>\n"
    << "    </physics>\n"
    << "    <scene>\n"
    << "      <ambient>0.4 0.4 0.4 1</ambient>\n"
    << "      <background>0.7 0.7 0.7 1</background>\n"
    << "      <shadows>true</shadows>\n"
    << "    </scene>\n"
    << "  </world>\n"
    << "</sdf>\n";

  EXPECT_EQ(root.Element()->ToString("", true, true, config), stream.str());
}
