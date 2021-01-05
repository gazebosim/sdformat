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
#include "sdf/Ellipsoid.hh"

/////////////////////////////////////////////////
TEST(DOMEllipsoid, Construction)
{
  sdf::Ellipsoid ellipsoid;
  EXPECT_EQ(nullptr, ellipsoid.Element());
  // A default ellipsoid has all three radii set to 1
  EXPECT_DOUBLE_EQ(IGN_PI * 4. / 3., ellipsoid.Shape().Volume());
  EXPECT_EQ(ignition::math::Vector3d::One, ellipsoid.Shape().Radii());

  const ignition::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);
  EXPECT_EQ(expectedRadii, ellipsoid.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, MoveConstructor)
{
  sdf::Ellipsoid ellipsoid;
  const ignition::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2(std::move(ellipsoid));
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CopyConstructor)
{
  sdf::Ellipsoid ellipsoid;
  const ignition::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2(ellipsoid);
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CopyAssignmentOperator)
{
  sdf::Ellipsoid ellipsoid;
  const ignition::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2;
  ellipsoid2 = ellipsoid;
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, MoveAssignmentConstructor)
{
  sdf::Ellipsoid ellipsoid;
  const ignition::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2;
  ellipsoid2 = std::move(ellipsoid);
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CopyAssignmentAfterMove)
{
  sdf::Ellipsoid ellipsoid1;
  const ignition::math::Vector3d expectedRadii1(1.0, 2.0, 3.0);
  ellipsoid1.SetRadii(expectedRadii1);

  sdf::Ellipsoid ellipsoid2;
  const ignition::math::Vector3d expectedRadii2(10.0, 20.0, 30.0);
  ellipsoid2.SetRadii(expectedRadii2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Ellipsoid tmp = std::move(ellipsoid1);
  ellipsoid1 = ellipsoid2;
  ellipsoid2 = tmp;

  EXPECT_EQ(expectedRadii1, ellipsoid2.Shape().Radii());
  EXPECT_EQ(expectedRadii2, ellipsoid1.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, Load)
{
  sdf::Ellipsoid ellipsoid;
  sdf::Errors errors;

  // Null element name
  errors = ellipsoid.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, ellipsoid.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = ellipsoid.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, ellipsoid.Element());

  // Missing <radii> element
  sdf->SetName("ellipsoid");
  errors = ellipsoid.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <radii>"))
      << errors[0].Message();
  EXPECT_NE(nullptr, ellipsoid.Element());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, Shape)
{
  sdf::Ellipsoid ellipsoid;
  EXPECT_EQ(ignition::math::Vector3d::One, ellipsoid.Radii());

  const ignition::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.Shape().SetRadii(expectedRadii);
  EXPECT_EQ(expectedRadii, ellipsoid.Radii());
}
