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
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
#include "sdf/Plane.hh"

/////////////////////////////////////////////////
TEST(DOMPlane, Construction)
{
  sdf::Plane plane;
  EXPECT_EQ(nullptr, plane.Element());

  EXPECT_EQ(ignition::math::Vector3d::UnitZ, plane.Normal());
  EXPECT_EQ(ignition::math::Vector2d::One, plane.Size());

  plane.SetNormal({1, 0, 0});
  EXPECT_EQ(ignition::math::Vector3d::UnitX, plane.Normal());

  plane.SetNormal({1, 0, 1});
  EXPECT_EQ(ignition::math::Vector3d(0.707107, 0, 0.707107), plane.Normal());

  plane.SetSize({1.2, 3.4});
  EXPECT_EQ(ignition::math::Vector2d(1.2, 3.4), plane.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, Load)
{
  sdf::Plane plane;
  sdf::Errors errors;

  // Null element name
  errors = plane.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, plane.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = plane.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, plane.Element());

  // Missing <normal> and <size> elements
  sdf->SetName("plane");
  errors = plane.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <normal>"));
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find("missing a <size>"));
  EXPECT_NE(nullptr, plane.Element());

  // Add a normal element
  sdf::ElementPtr normalDesc(new sdf::Element());
  normalDesc->SetName("normal");
  normalDesc->AddValue("vector3", "0 0 1", "1", "normal");
  sdf->AddElementDescription(normalDesc);
  sdf::ElementPtr normalElem = sdf->AddElement("normal");
  normalElem->Set<ignition::math::Vector3d>({1, 0, 0});

  // Missing <size> element
  sdf->SetName("plane");
  errors = plane.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <size>"));
}
