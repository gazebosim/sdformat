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
#include <gz/math/Angle.hh>
#include "sdf/Lidar.hh"

/////////////////////////////////////////////////
TEST(DOMLidar, Construction)
{
  sdf::Lidar lidar;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, lidar.LidarNoise());
}

/////////////////////////////////////////////////
TEST(DOMLidar, Set)
{
  sdf::Lidar lidar;

  lidar.SetHorizontalScanSamples(123);
  EXPECT_EQ(lidar.HorizontalScanSamples(), 123u);
  lidar.SetHorizontalScanResolution(0.45);
  EXPECT_DOUBLE_EQ(lidar.HorizontalScanResolution(), 0.45);
  lidar.SetHorizontalScanMinAngle(gz::math::Angle(0.67));
  EXPECT_DOUBLE_EQ(*(lidar.HorizontalScanMinAngle()), 0.67);
  lidar.SetHorizontalScanMaxAngle(gz::math::Angle(0.89));
  EXPECT_DOUBLE_EQ(*(lidar.HorizontalScanMaxAngle()), 0.89);

  lidar.SetVerticalScanSamples(98);
  EXPECT_EQ(lidar.VerticalScanSamples(), 98u);
  lidar.SetVerticalScanResolution(0.76);
  EXPECT_DOUBLE_EQ(lidar.VerticalScanResolution(), 0.76);
  lidar.SetVerticalScanMinAngle(gz::math::Angle(0.54));
  EXPECT_DOUBLE_EQ(*(lidar.VerticalScanMinAngle()), 0.54);
  lidar.SetVerticalScanMaxAngle(gz::math::Angle(0.321));
  EXPECT_DOUBLE_EQ(*(lidar.VerticalScanMaxAngle()), 0.321);

  lidar.SetRangeMin(1.2);
  EXPECT_DOUBLE_EQ(lidar.RangeMin(), 1.2);
  lidar.SetRangeMax(3.4);
  EXPECT_DOUBLE_EQ(lidar.RangeMax(), 3.4);
  lidar.SetRangeResolution(5.6);
  EXPECT_DOUBLE_EQ(lidar.RangeResolution(), 5.6);

  sdf::Noise noise;
  noise.SetMean(6.5);
  noise.SetStdDev(3.79);
  lidar.SetLidarNoise(noise);
  EXPECT_EQ(noise, lidar.LidarNoise());

  lidar.SetHorizontalScanSamples(111);
  lidar.SetHorizontalScanResolution(2.2);

  // Inequality operator
  sdf::Lidar lidar2;
  EXPECT_NE(lidar2, lidar);

  // Copy constructor
  sdf::Lidar lidarCopied(lidar);
  EXPECT_EQ(lidarCopied, lidar);

  // Assignment operator
  sdf::Lidar lidarAssigned;
  lidarAssigned = lidar;
  EXPECT_EQ(lidarAssigned, lidar);

  // Move constructor
  sdf::Lidar lidarMoved = std::move(lidar);
  EXPECT_EQ(lidarCopied, lidarMoved);
  // Test nullptr private class
  lidar = lidarMoved;
  EXPECT_EQ(lidarCopied, lidar);

  // Move assignment operator
  sdf::Lidar lidarMoveAssigned;
  lidarMoveAssigned = std::move(lidarCopied);
  EXPECT_EQ(lidarAssigned, lidarMoveAssigned);
  // Test nullptr private class
  lidarCopied = lidarMoveAssigned;
  EXPECT_EQ(lidarAssigned, lidarCopied);
}

/////////////////////////////////////////////////
TEST(DOMLidar, Load)
{
  sdf::Lidar lidar;
  sdf::Errors errors;

  // Null element
  errors = lidar.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, lidar.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = lidar.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, lidar.Element());

  // The Lidar::Load function is tested more thouroughly in the
  // link_dom.cc integration test.
}
