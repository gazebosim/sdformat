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
#include "sdf/Battery.hh"

/////////////////////////////////////////////////
TEST(DOMBattery, Construction)
{
  sdf::Battery battery;
  EXPECT_EQ(nullptr, battery.Element());

  battery.SetName("battery1");
  EXPECT_EQ(battery.Name(), "battery1");

  EXPECT_DOUBLE_EQ(battery.Voltage(), 0.0);
  battery.SetVoltage(1.0);
  EXPECT_DOUBLE_EQ(battery.Voltage(), 1.0);
}

/////////////////////////////////////////////////
TEST(DOMBattery, CopyConstructor)
{
  sdf::Battery battery;
  battery.SetName("battery1");
  battery.SetVoltage(1.0);

  auto batteryCopied = sdf::Battery(battery);
  EXPECT_EQ(batteryCopied.Name(), "battery1");
  EXPECT_DOUBLE_EQ(batteryCopied.Voltage(), 1.0);
}

/////////////////////////////////////////////////
TEST(DOMBattery, AssignmentOperator)
{
  sdf::Battery battery;
  battery.SetName("battery1");
  battery.SetVoltage(1.0);

  sdf::Battery batteryAssigned;
  batteryAssigned = battery;
  EXPECT_EQ(batteryAssigned.Name(), "battery1");
  EXPECT_DOUBLE_EQ(batteryAssigned.Voltage(), 1.0);
}

/////////////////////////////////////////////////
TEST(DOMBattery, MoveConstructor)
{
  sdf::Battery battery;
  battery.SetName("battery1");
  battery.SetVoltage(1.0);

  sdf::Battery batteryMoved = std::move(battery);
  EXPECT_EQ(batteryMoved.Name(), "battery1");
  EXPECT_DOUBLE_EQ(batteryMoved.Voltage(), 1.0);
}

/////////////////////////////////////////////////
TEST(DOMBattery, MoveAssignmentOperator)
{
  sdf::Battery battery;
  battery.SetName("battery1");
  battery.SetVoltage(1.0);

  sdf::Battery batteryMoved;
  batteryMoved = std::move(battery);
  EXPECT_EQ(batteryMoved.Name(), "battery1");
  EXPECT_DOUBLE_EQ(batteryMoved.Voltage(), 1.0);
}

/////////////////////////////////////////////////
TEST(DOMBattery, Load)
{
  sdf::Battery battery;
  sdf::Errors errors;

  // Null element name
  errors = battery.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, battery.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = battery.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, battery.Element());

  // No battery name
  sdf->SetName("battery");
  errors = battery.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_NE(nullptr, battery.Element());

  // No voltage
  sdf->SetName("battery");
  sdf->AddAttribute("name", "string", "battery1", true);
  sdf::ParamPtr param = sdf->GetAttribute("name");
  EXPECT_NE(nullptr, param);
  param->SetFromString("battery1");
  errors = battery.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ATTRIBUTE_MISSING, errors[0].Code());
  EXPECT_NE(nullptr, battery.Element());

  // No missing attributes
  sdf->SetName("battery");
  param = sdf->GetAttribute("name");
  EXPECT_NE(nullptr, param);
  param->SetFromString("battery2");
  sdf->AddAttribute("voltage", "float", "12.0", true);
  param = sdf->GetAttribute("voltage");
  EXPECT_NE(nullptr, param);
  param->SetFromString("12");
  errors = battery.Load(sdf);
  ASSERT_EQ(0u, errors.size());
  EXPECT_NE(nullptr, battery.Element());
}
