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

/////////////////////////////////////////////////
TEST(DOMSky, Construction)
{
  sdf::Sky sky;
  EXPECT_DOUBLE_EQ(10.0, sky.Time());
  EXPECT_DOUBLE_EQ(6.0, sky.Sunrise());
  EXPECT_DOUBLE_EQ(20.0, sky.Sunset());
  EXPECT_DOUBLE_EQ(0.6, sky.CloudSpeed());
  EXPECT_EQ(ignition::math::Angle(), sky.CloudDirection());
  EXPECT_DOUBLE_EQ(0.5, sky.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.5, sky.CloudMeanSize());
  EXPECT_EQ(ignition::math::Color(0.8f, 0.8f, 0.8f),
      sky.CloudAmbient());
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
  sky.SetCloudDirection(ignition::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(ignition::math::Color::Blue);

  sdf::Sky sky2(sky);
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(ignition::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(ignition::math::Color::Blue, sky2.CloudAmbient());

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
  sky.SetCloudDirection(ignition::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(ignition::math::Color::Blue);

  sdf::Sky sky2(std::move(sky));
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(ignition::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(ignition::math::Color::Blue, sky2.CloudAmbient());
}

/////////////////////////////////////////////////
TEST(DOMSky, MoveAssignmentOperator)
{
  sdf::Sky sky;
  sky.SetTime(1.0);
  sky.SetSunrise(5.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(0.3);
  sky.SetCloudDirection(ignition::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(ignition::math::Color::Blue);

  sdf::Sky sky2;
  sky2 = std::move(sky);
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(ignition::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(ignition::math::Color::Blue, sky2.CloudAmbient());
}

/////////////////////////////////////////////////
TEST(DOMSky, AssignmentOperator)
{
  sdf::Sky sky;
  sky.SetTime(1.0);
  sky.SetSunrise(5.0);
  sky.SetSunset(15.0);
  sky.SetCloudSpeed(0.3);
  sky.SetCloudDirection(ignition::math::Angle(1.2));
  sky.SetCloudHumidity(0.9);
  sky.SetCloudMeanSize(0.123);
  sky.SetCloudAmbient(ignition::math::Color::Blue);

  sdf::Sky sky2;
  sky2 = sky;
  EXPECT_DOUBLE_EQ(1.0, sky2.Time());
  EXPECT_DOUBLE_EQ(5.0, sky2.Sunrise());
  EXPECT_DOUBLE_EQ(15.0, sky2.Sunset());
  EXPECT_DOUBLE_EQ(0.3, sky2.CloudSpeed());
  EXPECT_EQ(ignition::math::Angle(1.2), sky2.CloudDirection());
  EXPECT_DOUBLE_EQ(0.9, sky2.CloudHumidity());
  EXPECT_DOUBLE_EQ(0.123, sky2.CloudMeanSize());
  EXPECT_EQ(ignition::math::Color::Blue, sky2.CloudAmbient());
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

  sky.SetCloudDirection(ignition::math::Angle(1.2));
  EXPECT_EQ(ignition::math::Angle(1.2), sky.CloudDirection());

  sky.SetCloudHumidity(0.9);
  EXPECT_DOUBLE_EQ(0.9, sky.CloudHumidity());

  sky.SetCloudMeanSize(0.123);
  EXPECT_DOUBLE_EQ(0.123, sky.CloudMeanSize());

  sky.SetCloudAmbient(ignition::math::Color(0.1f, 0.2f, 0.3f));
  EXPECT_EQ(ignition::math::Color(0.1f, 0.2f, 0.3f),
      sky.CloudAmbient());
}
