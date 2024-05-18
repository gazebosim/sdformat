/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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
#include "sdf/Cone.hh"
#include "test_utils.hh"
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMCone, Construction)
{
  sdf::Cone cone;
  EXPECT_EQ(nullptr, cone.Element());
  // A default cone has a length of 1 meter and radius if 0.5 meters.
  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.5, 2) * 1.0 / 3.0,
                   cone.Shape().Volume());

  EXPECT_DOUBLE_EQ(0.5, cone.Radius());
  EXPECT_DOUBLE_EQ(1.0, cone.Length());

  cone.SetRadius(0.5);
  cone.SetLength(2.3);

  EXPECT_DOUBLE_EQ(0.5, cone.Radius());
  EXPECT_DOUBLE_EQ(2.3, cone.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, MoveConstructor)
{
  sdf::Cone cone;
  cone.SetRadius(0.2);
  cone.SetLength(3.0);

  sdf::Cone cone2(std::move(cone));
  EXPECT_DOUBLE_EQ(0.2, cone2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cone2.Length());

  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.2, 2) * 3.0 / 3.0,
                   cone2.Shape().Volume());
  EXPECT_DOUBLE_EQ(0.2, cone2.Shape().Radius());
  EXPECT_DOUBLE_EQ(3.0, cone2.Shape().Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, CopyConstructor)
{
  sdf::Cone cone;
  cone.SetRadius(0.2);
  cone.SetLength(3.0);

  sdf::Cone cone2(cone);
  EXPECT_DOUBLE_EQ(0.2, cone2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cone2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, CopyAssignmentOperator)
{
  sdf::Cone cone;
  cone.SetRadius(0.2);
  cone.SetLength(3.0);

  sdf::Cone cone2;
  cone2 = cone;
  EXPECT_DOUBLE_EQ(0.2, cone2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cone2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, MoveAssignmentConstructor)
{
  sdf::Cone cone;
  cone.SetRadius(0.2);
  cone.SetLength(3.0);

  sdf::Cone cone2;
  cone2 = std::move(cone);
  EXPECT_DOUBLE_EQ(0.2, cone2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cone2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, CopyAssignmentAfterMove)
{
  sdf::Cone cone1;
  cone1.SetRadius(0.2);
  cone1.SetLength(3.0);

  sdf::Cone cone2;
  cone2.SetRadius(2);
  cone2.SetLength(30.0);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Cone tmp = std::move(cone1);
  cone1 = cone2;
  cone2 = tmp;

  EXPECT_DOUBLE_EQ(2, cone1.Radius());
  EXPECT_DOUBLE_EQ(30, cone1.Length());

  EXPECT_DOUBLE_EQ(0.2, cone2.Radius());
  EXPECT_DOUBLE_EQ(3.0, cone2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, Load)
{
  sdf::Cone cone;
  sdf::Errors errors;

  // Null element name
  errors = cone.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, cone.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = cone.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, cone.Element());

  // Missing <radius> and <length> elements
  sdf->SetName("cone");
  errors = cone.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <radius>"))
      << errors[0].Message();
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find("Invalid <length>"))
      << errors[1].Message();
  EXPECT_NE(nullptr, cone.Element());

  // Add a radius element
  sdf::ElementPtr radiusDesc(new sdf::Element());
  radiusDesc->SetName("radius");
  radiusDesc->AddValue("double", "1.0", true, "radius");
  sdf->AddElementDescription(radiusDesc);
  sdf::ElementPtr radiusElem = sdf->AddElement("radius");
  radiusElem->Set<double>(2.0);

  // Missing <length> element
  sdf->SetName("cone");
  errors = cone.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <length>"))
      << errors[0].Message();
}

/////////////////////////////////////////////////
TEST(DOMCone, Shape)
{
  sdf::Cone cone;
  EXPECT_DOUBLE_EQ(0.5, cone.Radius());
  EXPECT_DOUBLE_EQ(1.0, cone.Length());

  cone.Shape().SetRadius(0.123);
  cone.Shape().SetLength(0.456);
  EXPECT_DOUBLE_EQ(0.123, cone.Radius());
  EXPECT_DOUBLE_EQ(0.456, cone.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, CalculateInertial)
{
  sdf::Cone cone;

  // density of aluminium
  const double density = 2170;

  // Invalid dimensions leading to std::nullopt return in
  // CalculateInertial()
  cone.SetLength(-1);
  cone.SetRadius(0);
  auto invalidConeInertial = cone.CalculateInertial(density);
  ASSERT_EQ(std::nullopt, invalidConeInertial);

  const double l = 2.0;
  const double r = 0.1;

  cone.SetLength(l);
  cone.SetRadius(r);

  double expectedMass = cone.Shape().Volume() * density;
  double ixxIyy = (3/80.0) * expectedMass * (4*r*r + l*l);
  double izz = 3.0 * expectedMass * r * r / 10.0;

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixxIyy, ixxIyy, izz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  auto coneInertial = cone.CalculateInertial(density);
  EXPECT_EQ(cone.Shape().Mat().Density(), density);
  ASSERT_NE(std::nullopt, coneInertial);
  EXPECT_EQ(expectedInertial, *coneInertial);
  EXPECT_EQ(expectedInertial.MassMatrix().DiagonalMoments(),
    coneInertial->MassMatrix().DiagonalMoments());
  EXPECT_EQ(expectedInertial.MassMatrix().Mass(),
    coneInertial->MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.Pose(), coneInertial->Pose());
}

/////////////////////////////////////////////////
TEST(DOMCone, ToElement)
{
  sdf::Cone cone;

  cone.SetRadius(1.2);
  cone.SetLength(0.5);

  sdf::ElementPtr elem = cone.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Cone cone2;
  cone2.Load(elem);

  EXPECT_DOUBLE_EQ(cone.Radius(), cone2.Radius());
  EXPECT_DOUBLE_EQ(cone.Length(), cone2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCone, ToElementErrorOutput)
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

  sdf::Cone cone;
  sdf::Errors errors;

  cone.SetRadius(1.2);
  cone.SetLength(0.5);

  sdf::ElementPtr elem = cone.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Cone cone2;
  errors = cone2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(cone.Radius(), cone2.Radius());
  EXPECT_DOUBLE_EQ(cone.Length(), cone2.Length());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
