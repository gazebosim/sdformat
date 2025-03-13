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
#include "sdf/AirSpeed.hh"

/////////////////////////////////////////////////
TEST(DOMAirSpeed, Construction)
{
  sdf::AirSpeed alt;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, alt.PressureNoise());
}

/////////////////////////////////////////////////
TEST(DOMAirSpeed, Set)
{
  sdf::AirSpeed alt;
  sdf::Noise defaultNoise, noise;
  EXPECT_EQ(defaultNoise, alt.PressureNoise());

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);

  alt.SetPressureNoise(noise);
  EXPECT_EQ(noise, alt.PressureNoise());

  // Copy Constructor
  sdf::AirSpeed alt2(alt);
  EXPECT_EQ(alt, alt2);

  // Copy operator
  sdf::AirSpeed alt3;
  alt3 = alt;
  EXPECT_EQ(alt, alt3);

  // Move Constructor
  sdf::AirSpeed alt4(std::move(alt));
  EXPECT_EQ(alt2, alt4);

  alt = alt4;
  EXPECT_EQ(alt2, alt);

  // Move operator
  sdf::AirSpeed alt5;
  alt5 = std::move(alt2);
  EXPECT_EQ(alt3, alt5);

  alt2 = alt5;
  EXPECT_EQ(alt3, alt2);

  // inequality
  sdf::AirSpeed alt6;
  EXPECT_NE(alt3, alt6);
  alt6.SetPressureNoise(alt3.PressureNoise());
  EXPECT_EQ(alt3, alt6);
}

/////////////////////////////////////////////////
TEST(DOMAirSpeed, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  // No <noise> element
  sdf::AirSpeed alt;
  sdf::Errors errors = alt.Load(sdf);
  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <air_speed>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, alt.Element());
  EXPECT_EQ(sdf.get(), alt.Element().get());

  // The AirSpeed::Load function is test more thoroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMAirSpeed, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::AirSpeed alt;
  sdf::Noise defaultNoise, noise;
  EXPECT_EQ(defaultNoise, alt.PressureNoise());

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  alt.SetPressureNoise(noise);

  sdf::ElementPtr altElem = alt.ToElement();
  EXPECT_NE(nullptr, altElem);
  EXPECT_EQ(nullptr, alt.Element());

  // verify values after loading the element back
  sdf::AirSpeed alt2;
  alt2.Load(altElem);

  EXPECT_EQ(noise, alt2.PressureNoise());

  // make changes to DOM and verify ToElement produces updated values
  noise.SetMean(2.3);
  alt2.SetPressureNoise(noise);
  sdf::ElementPtr alt2Elem = alt2.ToElement();
  EXPECT_NE(nullptr, alt2Elem);
  sdf::AirSpeed alt3;
  alt3.Load(alt2Elem);
  EXPECT_EQ(noise, alt3.PressureNoise());
}
