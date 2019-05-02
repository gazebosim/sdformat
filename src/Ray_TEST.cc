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
#include <ignition/math/Angle.hh>
#include "sdf/Ray.hh"

/////////////////////////////////////////////////
TEST(DOMRay, Construction)
{
  sdf::Ray ray;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, ray.RayNoise());
}

/////////////////////////////////////////////////
TEST(DOMRay, Set)
{
  sdf::Ray ray;

  ray.SetHorizontalScanSamples(123);
  EXPECT_EQ(ray.HorizontalScanSamples(), 123u);
  ray.SetHorizontalScanResolution(0.45);
  EXPECT_DOUBLE_EQ(ray.HorizontalScanResolution(), 0.45);
  ray.SetHorizontalScanMinAngle(math::Angle(0.67));
  EXPECT_DOUBLE_EQ(*(ray.HorizontalScanMinAngle()), 0.67);
  ray.SetHorizontalScanMaxAngle(math::Angle(0.89));
  EXPECT_DOUBLE_EQ(*(ray.HorizontalScanMaxAngle()), 0.89);

  ray.SetVerticalScanSamples(98);
  EXPECT_EQ(ray.VerticalScanSamples(), 98u);
  ray.SetVerticalScanResolution(0.76);
  EXPECT_DOUBLE_EQ(ray.VerticalScanResolution(), 0.76);
  ray.SetVerticalScanMinAngle(math::Angle(0.54));
  EXPECT_DOUBLE_EQ(*(ray.VerticalScanMinAngle()), 0.54);
  ray.SetVerticalScanMaxAngle(math::Angle(0.321));
  EXPECT_DOUBLE_EQ(*(ray.VerticalScanMaxAngle()), 0.321);

  ray.SetMinRange(1.2);
  EXPECT_DOUBLE_EQ(ray.MinRange(), 1.2);
  ray.SetMaxRange(3.4);
  EXPECT_DOUBLE_EQ(ray.MaxRange(), 3.4);
  ray.SetRangeResolution(5.6);
  EXPECT_DOUBLE_EQ(ray.RangeResolution(), 5.6);

  sdf::Noise noise;
  noise.SetMean(6.5);
  noise.SetStdDev(3.79);
  ray.SetRayNoise(noise);
  EXPECT_EQ(noise, ray.RayNoise());

  ray.SetHorizontalScanSamples(111);
  ray.SetHorizontalScanResolution(2.2);

  // Inequality operator
  sdf::Ray ray2;
  EXPECT_NE(ray2, ray);

  // Copy constructor
  sdf::Ray rayCopied(ray);
  EXPECT_EQ(rayCopied, ray);

  // Assignment operator
  sdf::Ray rayAssigned;
  rayAssigned = ray;
  EXPECT_EQ(rayAssigned, ray);

  // Move constructor
  sdf::Ray rayMoved = std::move(ray);
  EXPECT_EQ(rayCopied, rayMoved);
  // Test nullptr private class
  ray = rayMoved;
  EXPECT_EQ(rayCopied, ray);

  // Move assignment operator
  sdf::Ray rayMoveAssigned;
  rayMoveAssigned = std::move(rayCopied);
  EXPECT_EQ(rayAssigned, rayMoveAssigned);
  // Test nullptr private class
  rayCopied = rayMoveAssigned;
  EXPECT_EQ(rayAssigned, rayCopied);
}

/////////////////////////////////////////////////
TEST(DOMRay, Load)
{
  sdf::Ray ray;
  sdf::Errors errors;

  // Null element
  errors = ray.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, ray.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = ray.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, ray.Element());

  // The Ray::Load function is tested more thouroughly in the
  // link_dom.cc integration test.
}
