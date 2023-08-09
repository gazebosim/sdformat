/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <sstream>
#include <utility>

#include <sdf/Atmosphere.hh>
#include <gz/math/Temperature.hh>
#include <gz/math/Vector3.hh>
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMAtmosphere, Construction)
{
  sdf::Atmosphere atmosphere;
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere.Type());
  EXPECT_DOUBLE_EQ(288.15, atmosphere.Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(-0.0065, atmosphere.TemperatureGradient());
  EXPECT_DOUBLE_EQ(101325, atmosphere.Pressure());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, Set)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetType(sdf::AtmosphereType::ADIABATIC);
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere.Type());

  atmosphere.SetTemperature(123.23);
  EXPECT_DOUBLE_EQ(123.23, atmosphere.Temperature().Kelvin());

  atmosphere.SetTemperatureGradient(-1.65);
  EXPECT_DOUBLE_EQ(-1.65, atmosphere.TemperatureGradient());

  atmosphere.SetPressure(76531.3);
  EXPECT_DOUBLE_EQ(76531.3, atmosphere.Pressure());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, MoveConstructor)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetTemperature(123.23);
  atmosphere.SetTemperatureGradient(-1.65);
  atmosphere.SetPressure(76531.3);

  sdf::Atmosphere atmosphere2(std::move(atmosphere));
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere2.Type());
  EXPECT_DOUBLE_EQ(123.23, atmosphere2.Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(-1.65, atmosphere2.TemperatureGradient());
  EXPECT_DOUBLE_EQ(76531.3, atmosphere2.Pressure());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, CopyConstructor)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetTemperature(123.23);
  atmosphere.SetTemperatureGradient(-1.65);
  atmosphere.SetPressure(76531.3);

  sdf::Atmosphere atmosphere2(atmosphere);
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere2.Type());
  EXPECT_DOUBLE_EQ(123.23, atmosphere2.Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(-1.65, atmosphere2.TemperatureGradient());
  EXPECT_DOUBLE_EQ(76531.3, atmosphere2.Pressure());

  EXPECT_TRUE(atmosphere == atmosphere2);
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, MoveAssignment)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetTemperature(123.23);
  atmosphere.SetTemperatureGradient(-1.65);
  atmosphere.SetPressure(76531.3);

  sdf::Atmosphere atmosphere2;
  atmosphere2 = std::move(atmosphere);
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere2.Type());
  EXPECT_DOUBLE_EQ(123.23, atmosphere2.Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(-1.65, atmosphere2.TemperatureGradient());
  EXPECT_DOUBLE_EQ(76531.3, atmosphere2.Pressure());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, CopyAssignment)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetTemperature(123.23);
  atmosphere.SetTemperatureGradient(-1.65);
  atmosphere.SetPressure(76531.3);

  sdf::Atmosphere atmosphere2;
  atmosphere2 = atmosphere;
  EXPECT_EQ(sdf::AtmosphereType::ADIABATIC, atmosphere2.Type());
  EXPECT_DOUBLE_EQ(123.23, atmosphere2.Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(-1.65, atmosphere2.TemperatureGradient());
  EXPECT_DOUBLE_EQ(76531.3, atmosphere2.Pressure());

  EXPECT_TRUE(atmosphere == atmosphere2);
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, CopyAssignmentAfterMove)
{
  sdf::Atmosphere atmosphere1;
  atmosphere1.SetTemperature(100.0);

  sdf::Atmosphere atmosphere2;
  atmosphere2.SetTemperature(200.0);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Atmosphere tmp = std::move(atmosphere1);
  atmosphere1 = atmosphere2;
  atmosphere2 = tmp;

  EXPECT_DOUBLE_EQ(200.0, atmosphere1.Temperature().Kelvin());
  EXPECT_DOUBLE_EQ(100.0, atmosphere2.Temperature().Kelvin());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, ToElement)
{
  sdf::Atmosphere atmosphere;
  atmosphere.SetType(sdf::AtmosphereType::ADIABATIC);
  atmosphere.SetTemperature(gz::math::Temperature(123));
  atmosphere.SetTemperatureGradient(1.34);
  atmosphere.SetPressure(2.65);

  sdf::ElementPtr elem = atmosphere.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Atmosphere atmosphere2;
  atmosphere2.Load(elem);

  // verify values after loading the element back
  EXPECT_EQ(atmosphere.Temperature(), atmosphere2.Temperature());
  EXPECT_DOUBLE_EQ(atmosphere.TemperatureGradient(),
      atmosphere2.TemperatureGradient());
  EXPECT_DOUBLE_EQ(atmosphere.Pressure(), atmosphere2.Pressure());
}

/////////////////////////////////////////////////
TEST(DOMAtmosphere, ToElementErrorOutput)
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

  sdf::Atmosphere atmosphere;
  sdf::Errors errors;
  atmosphere.SetType(sdf::AtmosphereType::ADIABATIC);
  atmosphere.SetTemperature(gz::math::Temperature(123));
  atmosphere.SetTemperatureGradient(1.34);
  atmosphere.SetPressure(2.65);

  sdf::ElementPtr elem = atmosphere.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Atmosphere atmosphere2;
  errors = atmosphere2.Load(elem);
  EXPECT_TRUE(errors.empty());

  // verify values after loading the element back
  EXPECT_EQ(atmosphere.Temperature(), atmosphere2.Temperature());
  EXPECT_DOUBLE_EQ(atmosphere.TemperatureGradient(),
      atmosphere2.TemperatureGradient());
  EXPECT_DOUBLE_EQ(atmosphere.Pressure(), atmosphere2.Pressure());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
