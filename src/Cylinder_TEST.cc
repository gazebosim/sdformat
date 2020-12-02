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
#include "sdf/Cylinder.hh"

/////////////////////////////////////////////////
TEST(DOMCylinder, Construction)
{
  sdf::Cylinder cylinder;
  EXPECT_EQ(nullptr, cylinder.Element());
  // A default cylinder has a length of 1 meter and radius if 0.5 meters.
  EXPECT_DOUBLE_EQ(IGN_PI * std::pow(0.5, 2) * 1.0, cylinder.Shape().Volume());

  EXPECT_DOUBLE_EQ(0.5, cylinder.Radius());
  EXPECT_DOUBLE_EQ(1.0, cylinder.Length());

  cylinder.SetRadius(0.5);
  cylinder.SetLength(2.3);

  EXPECT_DOUBLE_EQ(0.5, cylinder.Radius());
  EXPECT_DOUBLE_EQ(2.3, cylinder.Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, MoveConstructor)
{
  sdf::Cylinder cylinder;
  cylinder.SetRadius(0.2);
  cylinder.SetLength(3.0);

  sdf::Cylinder cylinder2(std::move(cylinder));
  EXPECT_DOUBLE_EQ(0.2, cylinder2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cylinder2.Length());

  EXPECT_DOUBLE_EQ(IGN_PI * std::pow(0.2, 2) * 3.0, cylinder2.Shape().Volume());
  EXPECT_DOUBLE_EQ(0.2, cylinder2.Shape().Radius());
  EXPECT_DOUBLE_EQ(3.0, cylinder2.Shape().Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, CopyConstructor)
{
  sdf::Cylinder cylinder;
  cylinder.SetRadius(0.2);
  cylinder.SetLength(3.0);

  sdf::Cylinder cylinder2(cylinder);
  EXPECT_DOUBLE_EQ(0.2, cylinder2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cylinder2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, CopyAssignmentOperator)
{
  sdf::Cylinder cylinder;
  cylinder.SetRadius(0.2);
  cylinder.SetLength(3.0);

  sdf::Cylinder cylinder2;
  cylinder2 = cylinder;
  EXPECT_DOUBLE_EQ(0.2, cylinder2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cylinder2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, MoveAssignmentConstructor)
{
  sdf::Cylinder cylinder;
  cylinder.SetRadius(0.2);
  cylinder.SetLength(3.0);

  sdf::Cylinder cylinder2;
  cylinder2 = std::move(cylinder);
  EXPECT_DOUBLE_EQ(0.2, cylinder2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cylinder2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, CopyAssignmentAfterMove)
{
  sdf::Cylinder cylinder1;
  cylinder1.SetRadius(0.2);
  cylinder1.SetLength(3.0);

  sdf::Cylinder cylinder2;
  cylinder2.SetRadius(2);
  cylinder2.SetLength(30.0);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Cylinder tmp = std::move(cylinder1);
  cylinder1 = cylinder2;
  cylinder2 = tmp;

  EXPECT_DOUBLE_EQ(2, cylinder1.Radius());
  EXPECT_DOUBLE_EQ(30, cylinder1.Length());

  EXPECT_DOUBLE_EQ(0.2, cylinder2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cylinder2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, Load)
{
  sdf::Cylinder cylinder;
  sdf::Errors errors;

  // Null element name
  errors = cylinder.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, cylinder.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = cylinder.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, cylinder.Element());

  // Missing <radius> and <length> elements
  sdf->SetName("cylinder");
  errors = cylinder.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <radius>"))
      << errors[0].Message();
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find("Invalid <length>"))
      << errors[1].Message();
  EXPECT_NE(nullptr, cylinder.Element());

  // Add a radius element
  sdf::ElementPtr radiusDesc(new sdf::Element());
  radiusDesc->SetName("radius");
  radiusDesc->AddValue("double", "1.0", "1", "radius");
  sdf->AddElementDescription(radiusDesc);
  sdf::ElementPtr radiusElem = sdf->AddElement("radius");
  radiusElem->Set<double>(2.0);

  // Missing <length> element
  sdf->SetName("cylinder");
  errors = cylinder.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <length>"))
      << errors[0].Message();
}

/////////////////////////////////////////////////
TEST(DOMCylinder, Shape)
{
  sdf::Cylinder cylinder;
  EXPECT_DOUBLE_EQ(0.5, cylinder.Radius());
  EXPECT_DOUBLE_EQ(1.0, cylinder.Length());

  cylinder.Shape().SetRadius(0.123);
  cylinder.Shape().SetLength(0.456);
  EXPECT_DOUBLE_EQ(0.123, cylinder.Radius());
  EXPECT_DOUBLE_EQ(0.456, cylinder.Length());
}
