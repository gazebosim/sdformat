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
#include "sdf/Altimeter.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMAltimeter, Construction)
{
  sdf::Altimeter alt;
  sdf::Noise defaultNoise;
  EXPECT_EQ(defaultNoise, alt.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, alt.VerticalVelocityNoise());
}

/////////////////////////////////////////////////
TEST(DOMAltimeter, Set)
{
  sdf::Altimeter alt;
  sdf::Noise defaultNoise, noise;
  EXPECT_EQ(defaultNoise, alt.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, alt.VerticalVelocityNoise());

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);

  alt.SetVerticalPositionNoise(noise);
  EXPECT_EQ(noise, alt.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, alt.VerticalVelocityNoise());

  alt.SetVerticalVelocityNoise(noise);
  EXPECT_EQ(noise, alt.VerticalPositionNoise());
  EXPECT_EQ(noise, alt.VerticalVelocityNoise());

  // Copy Constructor
  sdf::Altimeter alt2(alt);
  EXPECT_EQ(alt, alt2);

  // Copy operator
  sdf::Altimeter alt3;
  alt3 = alt;
  EXPECT_EQ(alt, alt3);

  // Move Constructor
  sdf::Altimeter alt4(std::move(alt));
  EXPECT_EQ(alt2, alt4);

  alt = alt4;
  EXPECT_EQ(alt2, alt);

  // Move operator
  sdf::Altimeter alt5;
  alt5 = std::move(alt2);
  EXPECT_EQ(alt3, alt5);

  alt2 = alt5;
  EXPECT_EQ(alt3, alt2);

  // inequality
  sdf::Altimeter alt6;
  EXPECT_NE(alt3, alt6);
  // set position noise but velocity noise should still be different
  alt6.SetVerticalPositionNoise(alt3.VerticalPositionNoise());
  EXPECT_NE(alt3, alt6);
}

/////////////////////////////////////////////////
TEST(DOMAltimeter, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  // No <noise> element
  sdf::Altimeter alt;
  sdf::Errors errors = alt.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <altimeter>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, alt.Element());
  EXPECT_EQ(sdf.get(), alt.Element().get());

  // The Altimeter::Load function is test more thoroughly in the
  // link_dom.cc integration test.
}

/////////////////////////////////////////////////
TEST(DOMAltimeter, ToElement)
{
  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Altimeter alt;
  sdf::Noise defaultNoise, noise;
  EXPECT_EQ(defaultNoise, alt.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, alt.VerticalVelocityNoise());

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  alt.SetVerticalPositionNoise(noise);
  alt.SetVerticalVelocityNoise(noise);

  sdf::ElementPtr altElem = alt.ToElement();
  EXPECT_NE(nullptr, altElem);
  EXPECT_EQ(nullptr, alt.Element());

  // verify values after loading the element back
  sdf::Altimeter alt2;
  alt2.Load(altElem);

  EXPECT_EQ(noise, alt2.VerticalPositionNoise());
  EXPECT_EQ(noise, alt2.VerticalVelocityNoise());

  // make changes to DOM and verify ToElement produces updated values
  noise.SetMean(2.3);
  alt2.SetVerticalPositionNoise(noise);
  sdf::ElementPtr alt2Elem = alt2.ToElement();
  EXPECT_NE(nullptr, alt2Elem);
  sdf::Altimeter alt3;
  alt3.Load(alt2Elem);
  EXPECT_EQ(noise, alt3.VerticalPositionNoise());
}

/////////////////////////////////////////////////
TEST(DOMAltimeter, ToElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  #ifdef _WIN32
    sdf::Console::Instance()->SetQuiet(false);
    sdf::testing::ScopeExit revertSetQuiet(
      []
      {
        sdf::Console::Instance()->SetQuiet(true);
      });
  #endif

  // test calling ToElement on a DOM object constructed without calling Load
  sdf::Altimeter alt;
  sdf::Errors errors;
  sdf::Noise defaultNoise, noise;
  EXPECT_EQ(defaultNoise, alt.VerticalPositionNoise());
  EXPECT_EQ(defaultNoise, alt.VerticalVelocityNoise());

  noise.SetType(sdf::NoiseType::GAUSSIAN);
  noise.SetMean(1.2);
  noise.SetStdDev(2.3);
  noise.SetBiasMean(4.5);
  noise.SetBiasStdDev(6.7);
  noise.SetPrecision(8.9);
  alt.SetVerticalPositionNoise(noise);
  alt.SetVerticalVelocityNoise(noise);

  sdf::ElementPtr altElem = alt.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_NE(nullptr, altElem);
  EXPECT_EQ(nullptr, alt.Element());

  // verify values after loading the element back
  sdf::Altimeter alt2;
  errors = alt2.Load(altElem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(noise, alt2.VerticalPositionNoise());
  EXPECT_EQ(noise, alt2.VerticalVelocityNoise());

  // make changes to DOM and verify ToElement produces updated values
  noise.SetMean(2.3);
  alt2.SetVerticalPositionNoise(noise);
  sdf::ElementPtr alt2Elem = alt2.ToElement();
  EXPECT_NE(nullptr, alt2Elem);
  sdf::Altimeter alt3;
  alt3.Load(alt2Elem);
  EXPECT_EQ(noise, alt3.VerticalPositionNoise());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
