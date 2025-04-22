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
#include "sdf/Magnetometer.hh"

/////////////////////////////////////////////////
TEST(DOMMagnetometer, Construction)
{
  sdf::Magnetometer mag;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, mag.XNoise());
  EXPECT_EQ(defaultNoise, mag.YNoise());
  EXPECT_EQ(defaultNoise, mag.ZNoise());
}

/////////////////////////////////////////////////
TEST(DOMMagnetometer, Set)
{
  sdf::Magnetometer mag;
  sdf::Noise defaultNoise, noise;
  EXPECT_EQ(defaultNoise, mag.XNoise());
  EXPECT_EQ(defaultNoise, mag.YNoise());
  EXPECT_EQ(defaultNoise, mag.ZNoise());

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);

  mag.SetXNoise(noise);
  EXPECT_EQ(noise, mag.XNoise());
  EXPECT_EQ(defaultNoise, mag.YNoise());
  EXPECT_EQ(defaultNoise, mag.ZNoise());

  mag.SetYNoise(noise);
  EXPECT_EQ(noise, mag.XNoise());
  EXPECT_EQ(noise, mag.YNoise());
  EXPECT_EQ(defaultNoise, mag.ZNoise());

  mag.SetZNoise(noise);
  EXPECT_EQ(noise, mag.XNoise());
  EXPECT_EQ(noise, mag.YNoise());
  EXPECT_EQ(noise, mag.ZNoise());

  // Copy Constructor
  sdf::Magnetometer mag2(mag);
  EXPECT_EQ(mag, mag2);

  // Copy operator
  sdf::Magnetometer mag3;
  mag3 = mag;
  EXPECT_EQ(mag, mag3);

  // Move Constructor
  sdf::Magnetometer mag4(std::move(mag));
  EXPECT_EQ(mag2, mag4);

  // Move operator
  sdf::Magnetometer mag5;
  mag5 = std::move(mag2);
  EXPECT_EQ(mag3, mag5);

  // inequality
  sdf::Magnetometer mag6;
  EXPECT_NE(mag3, mag6);

  // Copy assignment after move
  sdf::Magnetometer mag7;
  mag7.SetXNoise(defaultNoise);
  sdf::Magnetometer mag8;
  mag8.SetXNoise(noise);
  sdf::Magnetometer tmp = std::move(mag7);
  mag7 = mag8;
  mag8 = tmp;
  EXPECT_EQ(noise, mag7.XNoise());
  EXPECT_EQ(defaultNoise, mag8.XNoise());
}

/////////////////////////////////////////////////
TEST(DOMMagnetometer, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  // No <noise> element
  sdf::Magnetometer mag;
  sdf::Errors errors = mag.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <magnetometer>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, mag.Element());
  EXPECT_EQ(sdf.get(), mag.Element().get());

  // The Magnetometer::Load function is test more thoroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMMagnetometer, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Magnetometer mag;
  sdf::Noise noise;
  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  mag.SetXNoise(noise);
  mag.SetYNoise(noise);
  mag.SetZNoise(noise);

  sdf::ElementPtr magElem = mag.ToElement();
  EXPECT_NE(nullptr, magElem);
  EXPECT_EQ(nullptr, mag.Element());

  // verify values after loading the element back
  sdf::Magnetometer mag2;
  mag2.Load(magElem);

  EXPECT_EQ(noise, mag2.XNoise());
  EXPECT_EQ(noise, mag2.YNoise());
  EXPECT_EQ(noise, mag2.ZNoise());

  // make changes to DOM and verify ToElement produces updated values
  noise.SetBiasMean(11.22);
  mag2.SetXNoise(noise);
  sdf::ElementPtr mag2Elem = mag2.ToElement();
  EXPECT_NE(nullptr, mag2Elem);
  sdf::Magnetometer mag3;
  mag3.Load(mag2Elem);
  EXPECT_EQ(noise, mag3.XNoise());
  EXPECT_NE(noise, mag.XNoise());
}
