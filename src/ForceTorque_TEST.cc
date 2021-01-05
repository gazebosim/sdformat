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
#include "sdf/ForceTorque.hh"

/////////////////////////////////////////////////
TEST(DOMForceTorque, Construction)
{
  sdf::ForceTorque ft;

  EXPECT_EQ(ft.Frame(), sdf::ForceTorqueFrame::CHILD);
  ft.SetFrame(sdf::ForceTorqueFrame::PARENT);
  EXPECT_EQ(ft.Frame(), sdf::ForceTorqueFrame::PARENT);

  EXPECT_EQ(ft.MeasureDirection(),
            sdf::ForceTorqueMeasureDirection::CHILD_TO_PARENT);
  ft.SetMeasureDirection(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD);
  EXPECT_EQ(ft.MeasureDirection(),
            sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD);

  // Copy constructor
  sdf::ForceTorque ft2(ft);
  EXPECT_EQ(ft2, ft);

  // Copy operator
  sdf::ForceTorque ft3;
  ft3 = ft;
  EXPECT_EQ(ft3, ft);

  // Move constructor
  sdf::ForceTorque ft4(std::move(ft));
  EXPECT_EQ(ft4, ft2);
  ft = ft4;
  EXPECT_EQ(ft, ft2);

  // Move operator
  sdf::ForceTorque ft5;
  ft5 = std::move(ft2);
  EXPECT_EQ(ft5, ft3);
  ft2 = ft5;
  EXPECT_EQ(ft2, ft3);

  // Inequality
  sdf::ForceTorque ft6;
  EXPECT_NE(ft6, ft3);
}

/////////////////////////////////////////////////
TEST(DOMForceTorque, Load)
{
  sdf::ElementPtr sdf(std::make_shared<sdf::Element>());

  sdf::ForceTorque ft;
  sdf::Errors errors = ft.Load(sdf);
  EXPECT_FALSE(errors.empty());
  EXPECT_TRUE(errors[0].Message().find("is not a <force_torque>")
      != std::string::npos) << errors[0].Message();

  EXPECT_NE(nullptr, ft.Element());
  EXPECT_EQ(sdf.get(), ft.Element().get());

  // The ForceTorque::Load function is tested more thouroughly in the
  // link_dom.cc integration test.
}
