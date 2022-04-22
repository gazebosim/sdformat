/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include "sdf/Polyline.hh"

/////////////////////////////////////////////////
TEST(DOMPolyline, Construction)
{
  sdf::Polyline polyline;
  EXPECT_EQ(nullptr, polyline.Element());

  EXPECT_TRUE(polyline.Points().empty());
  EXPECT_DOUBLE_EQ(1.0, polyline.Height());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, MoveConstructor)
{
  sdf::Polyline polyline;
  polyline.SetHeight(0.2);

  sdf::Polyline polyline2(std::move(polyline));
  EXPECT_DOUBLE_EQ(0.2, polyline2.Height());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, CopyConstructor)
{
  sdf::Polyline polyline;
  polyline.SetHeight(0.2);

  sdf::Polyline polyline2(polyline);
  EXPECT_DOUBLE_EQ(0.2, polyline2.Height());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, CopyAssignmentOperator)
{
  sdf::Polyline polyline;
  polyline.SetHeight(0.2);

  sdf::Polyline polyline2;
  polyline2 = polyline;
  EXPECT_DOUBLE_EQ(0.2, polyline2.Height());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, MoveAssignmentOperator)
{
  sdf::Polyline polyline;
  polyline.SetHeight(0.2);

  sdf::Polyline polyline2;
  polyline2 = std::move(polyline);
  EXPECT_DOUBLE_EQ(0.2, polyline2.Height());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, CopyAssignmentAfterMove)
{
  sdf::Polyline polyline1;
  polyline1.SetHeight(0.1);

  sdf::Polyline polyline2;
  polyline2.SetHeight(0.2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Polyline tmp = std::move(polyline1);
  polyline1 = polyline2;
  polyline2 = tmp;

  EXPECT_DOUBLE_EQ(0.2, polyline1.Height());
  EXPECT_DOUBLE_EQ(0.1, polyline2.Height());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, Load)
{
  sdf::Polyline polyline;
  sdf::Errors errors;

  // Null element
  errors = polyline.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, polyline.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("banana");
  errors = polyline.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, polyline.Element());

  // Missing <height> element
  sdf->SetName("polyline");
  errors = polyline.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <height>"));
  EXPECT_NE(nullptr, polyline.Element());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, Points)
{
  sdf::Polyline polyline;
  EXPECT_TRUE(polyline.Points().empty());

  ignition::math::Vector2d p1{1, 2};
  ignition::math::Vector2d p2{3, 4};
  ignition::math::Vector2d p3{5, 6};

  polyline.AddPoint(p1);
  polyline.AddPoint(p2);
  polyline.AddPoint(p3);

  EXPECT_EQ(3u, polyline.Points().size());
  EXPECT_EQ(p1, polyline.Points()[0]);
  EXPECT_EQ(p2, polyline.Points()[1]);
  EXPECT_EQ(p3, polyline.Points()[2]);
}
