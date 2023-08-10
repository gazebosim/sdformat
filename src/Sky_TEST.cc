/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include "sdf/Sky.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMSky, Construction)
{
  sdf::Sky sky;
  EXPECT_DOUBLE_EQ(10.0, sky.Time());
  EXPECT_DOUBLE_EQ(6.0, sky.Sunrise());
  EXPECT_DOUBLE_EQ(20.0, sky.Sunset());
  EXPECT_DOUBLE_EQ(0.6, sky.CloudSpeed());
  EXPECT_EQ(gz::math::Angle(), sky.CloudDirection());
  EXPECT_DOUBLE_EQ(0.5, sky.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.5, sky.CloudMeanSize());
  EXPECT_EQ(gz::math::Color(0.8f, 0.8f, 0.8f),
      sky.CloudAmbient());
  EXPECT_EQ("", sky.CubemapUri());
}

/////////////////////////////////////////////////
TEST(DOMSky, CopyConstruction)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::Sky sky;
  sky.Load(sdf);
  sky.SetTime(1.0);
  sky.SetSunrise(5.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(0.3);
  sky.SetCloudDirection(gz::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(gz::math::Color::Blue);
  sky.SetCubemapUri("dummyUri");

  sdf::Sky sky2(sky);
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(gz::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(gz::math::Color::Blue, sky2.CloudAmbient());
  EXPECT_EQ("dummyUri", sky2.CubemapUri());

  EXPECT_NE(nullptr, sky2.Element());
  EXPECT_EQ(sky.Element(), sky2.Element());
}

/////////////////////////////////////////////////
TEST(DOMSky, MoveConstruction)
{
  sdf::Sky sky;
  sky.SetTime(1.0);
  sky.SetSunrise(5.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(0.3);
  sky.SetCloudDirection(gz::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(gz::math::Color::Blue);
  sky.SetCubemapUri("dummyUri");

  sdf::Sky sky2(std::move(sky));
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(gz::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(gz::math::Color::Blue, sky2.CloudAmbient());
  EXPECT_EQ("dummyUri", sky2.CubemapUri());
}

/////////////////////////////////////////////////
TEST(DOMSky, MoveAssignmentOperator)
{
  sdf::Sky sky;
  sky.SetTime(1.0);
  sky.SetSunrise(5.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(0.3);
  sky.SetCloudDirection(gz::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(gz::math::Color::Blue);
  sky.SetCubemapUri("dummyUri");

  sdf::Sky sky2;
  sky2 = std::move(sky);
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(gz::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(gz::math::Color::Blue, sky2.CloudAmbient());
  EXPECT_EQ("dummyUri", sky2.CubemapUri());
}

/////////////////////////////////////////////////
TEST(DOMSky, AssignmentOperator)
{
  sdf::Sky sky;
  sky.SetTime(1.0);
  sky.SetSunrise(5.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(0.3);
  sky.SetCloudDirection(gz::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(gz::math::Color::Blue);
  sky.SetCubemapUri("dummyUri");

  sdf::Sky sky2;
  sky2 = sky;
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(gz::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(gz::math::Color::Blue, sky2.CloudAmbient());
  EXPECT_EQ("dummyUri", sky2.CubemapUri());
}

/////////////////////////////////////////////////
TEST(DOMSky, CopyAssignmentAfterMove)
{
  sdf::Sky sky1;
  sky1.SetTime(21.0);

  sdf::Sky sky2;
  sky2.SetTime(1.0);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Sky tmp = std::move(sky1);
  sky1 = sky2;
  sky2 = tmp;

  EXPECT_DOUBLE_EQ(1.0, sky1.Time());
  EXPECT_DOUBLE_EQ(21.0, sky2.Time());
}

/////////////////////////////////////////////////
TEST(DOMSky, Set)
{
  sdf::Sky sky;

  sky.SetTime(1.0);
  EXPECT_DOUBLE_EQ(1.0, sky.Time());

  sky.SetSunrise(5.0);
  EXPECT_DOUBLE_EQ(5.0, sky.Sunrise());

  sky.SetSunset(15.0);
  EXPECT_DOUBLE_EQ(15.0, sky.Sunset());

  sky.SetCloudSpeed(0.3);
  EXPECT_DOUBLE_EQ(0.3, sky.CloudSpeed());

  sky.SetCloudDirection(gz::math::Angle(1.2));
  EXPECT_EQ(gz::math::Angle(1.2), sky.CloudDirection());

  sky.SetCloudHumidity(0.9);
  EXPECT_DOUBLE_EQ(0.9, sky.CloudHumidity());

  sky.SetCloudMeanSize(0.123);
  EXPECT_DOUBLE_EQ(0.123, sky.CloudMeanSize());

  sky.SetCloudAmbient(gz::math::Color(0.1f, 0.2f, 0.3f));
  EXPECT_EQ(gz::math::Color(0.1f, 0.2f, 0.3f),
      sky.CloudAmbient());

  sky.SetCubemapUri("dummyUri");
  EXPECT_EQ("dummyUri", sky.CubemapUri());
}

/////////////////////////////////////////////////
TEST(DOMSky, ToElement)
{
  sdf::Sky sky;

  sky.SetTime(1.2);
  sky.SetSunrise(0.5);
  sky.SetSunset(10.2);
  sky.SetCloudSpeed(100.2);
  sky.SetCloudDirection(1.56);
  sky.SetCloudHumidity(0.2);
  sky.SetCloudMeanSize(0.5);
  sky.SetCloudAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  sky.SetCubemapUri("dummyUri");

  sdf::ElementPtr elem = sky.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Sky sky2;
  sky2.Load(elem);

  EXPECT_DOUBLE_EQ(sky.Time(), sky2.Time());
  EXPECT_DOUBLE_EQ(sky.Sunrise(), sky2.Sunrise());
  EXPECT_DOUBLE_EQ(sky.Sunset(), sky2.Sunset());
  EXPECT_DOUBLE_EQ(sky.CloudSpeed(), sky2.CloudSpeed());
  EXPECT_EQ(sky.CloudDirection(), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(sky.CloudHumidity(), sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(sky.CloudMeanSize(), sky2.CloudMeanSize());
  EXPECT_EQ(sky.CloudAmbient(), sky2.CloudAmbient());
  EXPECT_EQ(sky.CubemapUri(), sky2.CubemapUri());
}

/////////////////////////////////////////////////
TEST(DOMSky, ToElementErrorOutput)
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

  sdf::Sky sky;
  sdf::Errors errors;

  sky.SetTime(1.2);
  sky.SetSunrise(0.5);
  sky.SetSunset(10.2);
  sky.SetCloudSpeed(100.2);
  sky.SetCloudDirection(1.56);
  sky.SetCloudHumidity(0.2);
  sky.SetCloudMeanSize(0.5);
  sky.SetCloudAmbient(gz::math::Color(0.1f, 0.2f, 0.3f, 1.0f));
  sky.SetCubemapUri("dummyUri");

  sdf::ElementPtr elem = sky.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Sky sky2;
  errors = sky2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(sky.Time(), sky2.Time());
  EXPECT_DOUBLE_EQ(sky.Sunrise(), sky2.Sunrise());
  EXPECT_DOUBLE_EQ(sky.Sunset(), sky2.Sunset());
  EXPECT_DOUBLE_EQ(sky.CloudSpeed(), sky2.CloudSpeed());
  EXPECT_EQ(sky.CloudDirection(), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(sky.CloudHumidity(), sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(sky.CloudMeanSize(), sky2.CloudMeanSize());
  EXPECT_EQ(sky.CloudAmbient(), sky2.CloudAmbient());
  EXPECT_EQ(sky.CubemapUri(), sky2.CubemapUri());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
