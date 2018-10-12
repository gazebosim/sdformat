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
#include "sdf/Box.hh"
#include "sdf/Element.hh"

/////////////////////////////////////////////////
TEST(DOMBox, Construction)
{
  sdf::Box box;
  EXPECT_EQ(nullptr, box.Element());

  EXPECT_EQ(ignition::math::Vector3d::One, box.Size());

  box.SetSize(ignition::math::Vector3d::Zero);
  EXPECT_EQ(ignition::math::Vector3d::Zero, box.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, Load)
{
  sdf::Box box;
  sdf::Errors errors;

  // Null element name
  errors = box.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = box.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, box.Element());

  // Missing <size> element
  sdf->SetName("box");
  errors = box.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <size>"));
  EXPECT_NE(nullptr, box.Element());
}
