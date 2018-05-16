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
#include "sdf/Geometry.hh"

/////////////////////////////////////////////////
TEST(DOMGeometry, Construction)
{
  sdf::Geometry geom;
  EXPECT_EQ(nullptr, geom.Element());
  EXPECT_EQ(sdf::GeometryType::EMPTY, geom.Type());

  geom.SetType(sdf::GeometryType::BOX);
  EXPECT_EQ(sdf::GeometryType::BOX, geom.Type());

  geom.SetType(sdf::GeometryType::CYLINDER);
  EXPECT_EQ(sdf::GeometryType::CYLINDER, geom.Type());

  geom.SetType(sdf::GeometryType::PLANE);
  EXPECT_EQ(sdf::GeometryType::PLANE, geom.Type());

  geom.SetType(sdf::GeometryType::SPHERE);
  EXPECT_EQ(sdf::GeometryType::SPHERE, geom.Type());
}

/////////////////////////////////////////////////
TEST(DOMGeometry, Load)
{
  sdf::Geometry geom;
  sdf::Errors errors;

  // Null element name
  errors = geom.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, geom.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = geom.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, geom.Element());
}
