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
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMPolyline, Construction)
{
  sdf::Polyline polyline;
  EXPECT_EQ(nullptr, polyline.Element());

  EXPECT_EQ(0u, polyline.PointCount());
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
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <height>"));
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_ERROR, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "Missing element description for [point]"));
  EXPECT_NE(nullptr, polyline.Element());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, Points)
{
  sdf::Polyline polyline;
  EXPECT_EQ(0u, polyline.PointCount());

  gz::math::Vector2d p1{1, 2};
  gz::math::Vector2d p2{3, 4};
  gz::math::Vector2d p3{5, 6};
  gz::math::Vector2d p4{7, 8};

  EXPECT_TRUE(polyline.AddPoint(p1));
  EXPECT_TRUE(polyline.AddPoint(p2));
  EXPECT_TRUE(polyline.AddPoint(p3));

  EXPECT_EQ(3u, polyline.PointCount());
  EXPECT_EQ(3u, polyline.Points().size());
  EXPECT_EQ(p1, *polyline.PointByIndex(0));
  EXPECT_EQ(p2, *polyline.PointByIndex(1));
  EXPECT_EQ(p3, *polyline.PointByIndex(2));
  EXPECT_EQ(p1, polyline.Points()[0]);
  EXPECT_EQ(p2, polyline.Points()[1]);
  EXPECT_EQ(p3, polyline.Points()[2]);
  EXPECT_EQ(nullptr, polyline.PointByIndex(3));

  *polyline.PointByIndex(0) = p4;
  EXPECT_EQ(p4, *polyline.PointByIndex(0));

  polyline.ClearPoints();
  EXPECT_EQ(0u, polyline.PointCount());
}

/////////////////////////////////////////////////
TEST(DOMPolyline, ToElement)
{
  sdf::Polyline polyline;
  polyline.SetHeight(1.2);

  gz::math::Vector2d p1{1, 2};
  gz::math::Vector2d p2{3, 4};
  EXPECT_TRUE(polyline.AddPoint(p1));
  EXPECT_TRUE(polyline.AddPoint(p2));

  auto elem = polyline.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Polyline polyline2;
  polyline2.Load(elem);

  EXPECT_DOUBLE_EQ(polyline.Height(), polyline2.Height());
  ASSERT_EQ(polyline.PointCount(), polyline2.PointCount());

  EXPECT_EQ(*polyline.PointByIndex(0), *polyline2.PointByIndex(0));
  EXPECT_EQ(*polyline.PointByIndex(1), *polyline2.PointByIndex(1));
}

/////////////////////////////////////////////////
TEST(DOMPolyline, ToElementErrorOutput)
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

  sdf::Polyline polyline;
  sdf::Errors errors;
  polyline.SetHeight(1.2);

  gz::math::Vector2d p1{1, 2};
  gz::math::Vector2d p2{3, 4};
  EXPECT_TRUE(polyline.AddPoint(p1));
  EXPECT_TRUE(polyline.AddPoint(p2));

  auto elem = polyline.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Polyline polyline2;
  errors = polyline2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(polyline.Height(), polyline2.Height());
  ASSERT_EQ(polyline.PointCount(), polyline2.PointCount());

  EXPECT_EQ(*polyline.PointByIndex(0), *polyline2.PointByIndex(0));
  EXPECT_EQ(*polyline.PointByIndex(1), *polyline2.PointByIndex(1));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
