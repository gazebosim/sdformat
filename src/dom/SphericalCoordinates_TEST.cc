/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "sdf/dom/SphericalCoordinates.hh"

/////////////////////////////////////////////////
TEST(DOMSphericalCoordinates, Construction)
{
  sdf::SphericalCoordinates sc;
  EXPECT_EQ(sc.SurfaceModel(), "EARTH_WGS84");
  EXPECT_EQ(sc.WorldFrameOrientation(),
      sdf::SphericalCoordinatesWorldFrameOrientation::ENU);
  EXPECT_EQ(sc.WorldFrameOrientationName(), "enu");

  EXPECT_DOUBLE_EQ(sc.Latitude(), 0.0);
  EXPECT_DOUBLE_EQ(sc.Longitude(), 0.0);
  EXPECT_DOUBLE_EQ(sc.Elevation(), 0.0);
  EXPECT_DOUBLE_EQ(sc.Heading(), 0.0);
}

/////////////////////////////////////////////////
TEST(DOMSphericalCoordinates, Set)
{
  sdf::SphericalCoordinates sc;

  sc.SetSurfaceModel("MARS_WGS84");
  EXPECT_EQ(sc.SurfaceModel(), "MARS_WGS84");

  sc.SetWorldFrameOrientation(
      sdf::SphericalCoordinatesWorldFrameOrientation::NED);
  EXPECT_EQ(sc.WorldFrameOrientation(),
      sdf::SphericalCoordinatesWorldFrameOrientation::NED);
  EXPECT_EQ(sc.WorldFrameOrientationName(), "ned");


  sc.SetWorldFrameOrientation(
      sdf::SphericalCoordinatesWorldFrameOrientation::NWU);
  EXPECT_EQ(sc.WorldFrameOrientation(),
      sdf::SphericalCoordinatesWorldFrameOrientation::NWU);
  EXPECT_EQ(sc.WorldFrameOrientationName(), "nwu");

  sc.SetWorldFrameOrientation(
      sdf::SphericalCoordinatesWorldFrameOrientation::UNKNOWN);
  EXPECT_EQ(sc.WorldFrameOrientation(),
      sdf::SphericalCoordinatesWorldFrameOrientation::UNKNOWN);
  EXPECT_EQ(sc.WorldFrameOrientationName(), "unknown");

  sc.SetLatitude(12.3);
  EXPECT_DOUBLE_EQ(sc.Latitude(), 12.3);

  sc.SetLongitude(-12.3);
  EXPECT_DOUBLE_EQ(sc.Longitude(), -12.3);

  sc.SetElevation(34.5);
  EXPECT_DOUBLE_EQ(sc.Elevation(), 34.5);

  sc.SetHeading(45.1);
  EXPECT_DOUBLE_EQ(sc.Heading(), 45.1);
}
