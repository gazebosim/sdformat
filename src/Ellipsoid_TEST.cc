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

#include <optional>
#include <gtest/gtest.h>
#include "sdf/Ellipsoid.hh"
#include "test_utils.hh"
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMEllipsoid, Construction)
{
  sdf::Ellipsoid ellipsoid;
  EXPECT_EQ(nullptr, ellipsoid.Element());
  // A default ellipsoid has all three radii set to 1
  EXPECT_DOUBLE_EQ(GZ_PI * 4. / 3., ellipsoid.Shape().Volume());
  EXPECT_EQ(gz::math::Vector3d::One, ellipsoid.Shape().Radii());

  const gz::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);
  EXPECT_EQ(expectedRadii, ellipsoid.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, MoveConstructor)
{
  sdf::Ellipsoid ellipsoid;
  const gz::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2(std::move(ellipsoid));
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CopyConstructor)
{
  sdf::Ellipsoid ellipsoid;
  const gz::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2(ellipsoid);
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CopyAssignmentOperator)
{
  sdf::Ellipsoid ellipsoid;
  const gz::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2;
  ellipsoid2 = ellipsoid;
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, MoveAssignmentConstructor)
{
  sdf::Ellipsoid ellipsoid;
  const gz::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  sdf::Ellipsoid ellipsoid2;
  ellipsoid2 = std::move(ellipsoid);
  EXPECT_EQ(expectedRadii, ellipsoid2.Shape().Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CopyAssignmentAfterMove)
{
  sdf::Ellipsoid ellipsoid1;
  const gz::math::Vector3d expectedRadii1(1.0, 2.0, 3.0);
  ellipsoid1.SetRadii(expectedRadii1);

  sdf::Ellipsoid ellipsoid2;
  const gz::math::Vector3d expectedRadii2(10.0, 20.0, 30.0);
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

  // Negative <radii> element
  sdf::ElementPtr radiiElem = sdf->AddElement("radii");
  radiiElem->Set<gz::math::Vector3d>(gz::math::Vector3d(-1, -1, -1));
  errors = ellipsoid.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <radii>"))
      << errors[0].Message();
  EXPECT_NE(nullptr, ellipsoid.Element());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, Shape)
{
  sdf::Ellipsoid ellipsoid;
  EXPECT_EQ(gz::math::Vector3d::One, ellipsoid.Radii());

  const gz::math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.Shape().SetRadii(expectedRadii);
  EXPECT_EQ(expectedRadii, ellipsoid.Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, CalculateInertial)
{
  sdf::Ellipsoid ellipsoid;

  // density of Aluminum
  const double density = 2170;

  // Invalid dimensions leading to std::nullopt return in
  // CalculateInertial()
  ellipsoid.SetRadii(gz::math::Vector3d(-1, 2, 0));
  auto invalidEllipsoidInertial = ellipsoid.CalculateInertial(density);
  ASSERT_EQ(std::nullopt, invalidEllipsoidInertial);

  const double a = 1.0;
  const double b = 10.0;
  const double c = 100.0;

  ellipsoid.SetRadii(gz::math::Vector3d(a, b, c));

  double expectedMass = ellipsoid.Shape().Volume() * density;
  double ixx = (expectedMass / 5.0) * (b*b + c*c);
  double iyy = (expectedMass / 5.0) * (a*a + c*c);
  double izz = (expectedMass / 5.0) * (a*a + b*b);

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixx, iyy, izz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  auto ellipsoidInertial = ellipsoid.CalculateInertial(density);
  EXPECT_EQ(ellipsoid.Shape().Mat().Density(), density);
  ASSERT_NE(std::nullopt, ellipsoidInertial);
  EXPECT_EQ(expectedInertial, *ellipsoidInertial);
  EXPECT_EQ(expectedInertial.MassMatrix().DiagonalMoments(),
    ellipsoidInertial->MassMatrix().DiagonalMoments());
  EXPECT_EQ(expectedInertial.MassMatrix().Mass(),
    ellipsoidInertial->MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.Pose(), ellipsoidInertial->Pose());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, ToElement)
{
  sdf::Ellipsoid ellipsoid;

  ellipsoid.SetRadii(gz::math::Vector3d(0.1, 1.2, 3.4));

  sdf::ElementPtr elem = ellipsoid.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Ellipsoid ellipsoid2;
  ellipsoid2.Load(elem);

  EXPECT_EQ(ellipsoid.Radii(), ellipsoid2.Radii());
}

/////////////////////////////////////////////////
TEST(DOMEllipsoid, ToElementErrorOutput)
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

  sdf::Ellipsoid ellipsoid;
  sdf::Errors errors;

  ellipsoid.SetRadii(gz::math::Vector3d(0.1, 1.2, 3.4));

  sdf::ElementPtr elem = ellipsoid.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Ellipsoid ellipsoid2;

  errors = ellipsoid2.Load(elem);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(ellipsoid.Radii(), ellipsoid2.Radii());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
