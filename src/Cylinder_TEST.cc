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

#include <optional>
#include <gtest/gtest.h>
#include "sdf/Cylinder.hh"
#include "test_utils.hh"
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMCylinder, Construction)
{
  sdf::Cylinder cylinder;
  EXPECT_EQ(nullptr, cylinder.Element());
  // A default cylinder has a length of 1 meter and radius if 0.5 meters.
  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.5, 2) * 1.0, cylinder.Shape().Volume());

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

  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.2, 2) * 3.0, cylinder2.Shape().Volume());
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
  radiusDesc->AddValue("double", "1.0", true, "radius");
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

/////////////////////////////////////////////////
TEST(DOMCylinder, CalculateInertial)
{
  sdf::Cylinder cylinder;

  // density of aluminium
  const double density = 2170;

  // Invalid dimensions leading to std::nullopt return in
  // CalculateInertial()
  cylinder.SetLength(-1);
  cylinder.SetRadius(0);
  auto invalidCylinderInertial = cylinder.CalculateInertial(density);
  ASSERT_EQ(std::nullopt, invalidCylinderInertial);

  const double l = 2.0;
  const double r = 0.1;

  cylinder.SetLength(l);
  cylinder.SetRadius(r);

  double expectedMass = cylinder.Shape().Volume() * density;
  double ixxIyy = (1/12.0) * expectedMass * (3*r*r + l*l);
  double izz = 0.5 * expectedMass * r * r;

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixxIyy, ixxIyy, izz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  auto cylinderInertial = cylinder.CalculateInertial(density);
  EXPECT_EQ(cylinder.Shape().Mat().Density(), density);
  ASSERT_NE(std::nullopt, cylinderInertial);
  EXPECT_EQ(expectedInertial, *cylinderInertial);
  EXPECT_EQ(expectedInertial.MassMatrix().DiagonalMoments(),
    cylinderInertial->MassMatrix().DiagonalMoments());
  EXPECT_EQ(expectedInertial.MassMatrix().Mass(),
    cylinderInertial->MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.Pose(), cylinderInertial->Pose());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, ToElement)
{
  sdf::Cylinder cylinder;

  cylinder.SetRadius(1.2);
  cylinder.SetLength(0.5);

  sdf::ElementPtr elem = cylinder.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Cylinder cylinder2;
  cylinder2.Load(elem);

  EXPECT_DOUBLE_EQ(cylinder.Radius(), cylinder2.Radius());
  EXPECT_DOUBLE_EQ(cylinder.Length(), cylinder2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCylinder, ToElementErrorOutput)
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

  sdf::Cylinder cylinder;
  sdf::Errors errors;

  cylinder.SetRadius(1.2);
  cylinder.SetLength(0.5);

  sdf::ElementPtr elem = cylinder.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Cylinder cylinder2;
  errors = cylinder2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(cylinder.Radius(), cylinder2.Radius());
  EXPECT_DOUBLE_EQ(cylinder.Length(), cylinder2.Length());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMCylinder, AxisAlignedBox)
{
  sdf::Cylinder cylinder;
  cylinder.SetRadius(0.25);
  cylinder.SetLength(2.5);

  auto aabb = cylinder.AxisAlignedBox();
  EXPECT_EQ(gz::math::Vector3d(0.5, 0.5, 2.5), aabb.Size());
  EXPECT_EQ(gz::math::Vector3d(-0.25, -0.25, -1.25), aabb.Min());
  EXPECT_EQ(gz::math::Vector3d(0.25, 0.25, 1.25), aabb.Max());
}
