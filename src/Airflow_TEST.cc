/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include "sdf/AirFlow.hh"
#include "sdf/Noise.hh"

/////////////////////////////////////////////////
TEST(DOMAirFlow, Construction)
{
  sdf::AirFlow alt;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, alt.DirectionNoise());
}

/////////////////////////////////////////////////
TEST(DOMAirFlow, Set)
{
  sdf::AirFlow alt;
  sdf::Noise defaultNoise, dir_noise, speed_noise;
  EXPECT_EQ(defaultNoise, alt.DirectionNoise());
  EXPECT_EQ(defaultNoise, alt.SpeedNoise());

  speed_noise.SetType(sdf::NoiseType::GAUSSIAN);
  speed_noise.SetMean(1.2);
  speed_noise.SetStdDev(2.3);
  speed_noise.SetBiasMean(4.5);
  speed_noise.SetBiasStdDev(6.7);
  speed_noise.SetPrecision(8.9);

  dir_noise.SetType(sdf::NoiseType::GAUSSIAN);
  dir_noise.SetMean(1.2);
  dir_noise.SetStdDev(2.3);
  dir_noise.SetBiasMean(4.5);
  dir_noise.SetBiasStdDev(6.7);
  dir_noise.SetPrecision(8.9);

  alt.SetDirectionNoise(dir_noise);
  EXPECT_EQ(dir_noise, alt.DirectionNoise());

  alt.SetSpeedNoise(speed_noise);
  EXPECT_EQ(speed_noise, alt.SpeedNoise());

  // Copy Constructor
  sdf::AirFlow alt2(alt);
  EXPECT_EQ(alt, alt2);

  // Copy operator
  sdf::AirFlow alt3;
  alt3 = alt;
  EXPECT_EQ(alt, alt3);

  // Move Constructor
  sdf::AirFlow alt4(std::move(alt));
  EXPECT_EQ(alt2, alt4);

  alt = alt4;
  EXPECT_EQ(alt2, alt);

  // Move operator
  sdf::AirFlow alt5;
  alt5 = std::move(alt2);
  EXPECT_EQ(alt3, alt5);

  alt2 = alt5;
  EXPECT_EQ(alt3, alt2);

  // inequality
  sdf::AirFlow alt6;
  EXPECT_NE(alt3, alt6);
  alt6.SetDirectionNoise(alt3.DirectionNoise());
  EXPECT_NE(alt3, alt6);
  alt6.SetSpeedNoise(alt3.SpeedNoise());
  EXPECT_EQ(alt3, alt6);
}

/////////////////////////////////////////////////
TEST(DOMAirFlow, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  // No <noise> element
  sdf::AirFlow alt;
  sdf::Errors errors = alt.Load(sdf);
  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <air_flow>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, alt.Element());
  EXPECT_EQ(sdf.get(), alt.Element().get());

  // The AirFlow::Load function is test more thouroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMAirFlow, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::AirFlow alt;
  sdf::Noise defaultNoise, dir_noise, speed_noise;
  EXPECT_EQ(defaultNoise, alt.DirectionNoise());

  dir_noise.SetType(sdf::NoiseType::GAUSSIAN);
  dir_noise.SetMean(1.2);
  dir_noise.SetStdDev(2.3);
  dir_noise.SetBiasMean(4.5);
  dir_noise.SetBiasStdDev(6.7);
  dir_noise.SetPrecision(8.9);

  speed_noise.SetType(sdf::NoiseType::GAUSSIAN);
  speed_noise.SetMean(1.2);
  speed_noise.SetStdDev(2.3);
  speed_noise.SetBiasMean(4.5);
  speed_noise.SetBiasStdDev(6.7);
  speed_noise.SetPrecision(8.9);

  alt.SetDirectionNoise(dir_noise);
  alt.SetSpeedNoise(speed_noise);

  sdf::ElementPtr altElem = alt.ToElement();
  EXPECT_NE(nullptr, altElem);
  EXPECT_EQ(nullptr, alt.Element());

  // verify values after loading the element back
  sdf::AirFlow alt2;
  alt2.Load(altElem);

  EXPECT_EQ(dir_noise, alt2.DirectionNoise());
  EXPECT_EQ(speed_noise, alt2.SpeedNoise());

  // make changes to DOM and verify ToElement produces updated values
  dir_noise.SetMean(2.3);
  alt2.SetDirectionNoise(dir_noise);
  sdf::ElementPtr alt2Elem = alt2.ToElement();
  EXPECT_NE(nullptr, alt2Elem);
  sdf::AirFlow alt3;
  alt3.Load(alt2Elem);
  EXPECT_EQ(dir_noise, alt3.DirectionNoise());
  EXPECT_EQ(speed_noise, alt3.SpeedNoise());
}
