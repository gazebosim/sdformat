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
#include "sdf/Capsule.hh"
#include "test_utils.hh"
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMCapsule, Construction)
{
  sdf::Capsule capsule;
  EXPECT_EQ(nullptr, capsule.Element());
  // A default capsule has a length of 1 meter and radius if 0.5 meters.
  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.5, 2) * (1.0 + 4./3. * 0.5),
                   capsule.Shape().Volume());

  EXPECT_DOUBLE_EQ(0.5, capsule.Radius());
  EXPECT_DOUBLE_EQ(1.0, capsule.Length());

  capsule.SetRadius(0.5);
  capsule.SetLength(2.3);

  EXPECT_DOUBLE_EQ(0.5, capsule.Radius());
  EXPECT_DOUBLE_EQ(2.3, capsule.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, MoveConstructor)
{
  sdf::Capsule capsule;
  capsule.SetRadius(0.2);
  capsule.SetLength(3.0);
  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.2, 2) * (3.0 + 4./3. * 0.2),
                   capsule.Shape().Volume());

  sdf::Capsule capsule2(std::move(capsule));
  EXPECT_DOUBLE_EQ(0.2, capsule2.Radius());
  EXPECT_DOUBLE_EQ(3.0, capsule2.Length());

  EXPECT_DOUBLE_EQ(GZ_PI * std::pow(0.2, 2) * (3.0 + 4./3. * 0.2),
                   capsule2.Shape().Volume());
  EXPECT_DOUBLE_EQ(0.2, capsule2.Shape().Radius());
  EXPECT_DOUBLE_EQ(3.0, capsule2.Shape().Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, CopyConstructor)
{
  sdf::Capsule capsule;
  capsule.SetRadius(0.2);
  capsule.SetLength(3.0);

  sdf::Capsule capsule2(capsule);
  EXPECT_DOUBLE_EQ(0.2, capsule2.Radius());
  EXPECT_DOUBLE_EQ(3.0, capsule2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, CopyAssignmentOperator)
{
  sdf::Capsule capsule;
  capsule.SetRadius(0.2);
  capsule.SetLength(3.0);

  sdf::Capsule capsule2;
  capsule2 = capsule;
  EXPECT_DOUBLE_EQ(0.2, capsule2.Radius());
  EXPECT_DOUBLE_EQ(3.0, capsule2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, MoveAssignmentConstructor)
{
  sdf::Capsule capsule;
  capsule.SetRadius(0.2);
  capsule.SetLength(3.0);

  sdf::Capsule capsule2;
  capsule2 = std::move(capsule);
  EXPECT_DOUBLE_EQ(0.2, capsule2.Radius());
  EXPECT_DOUBLE_EQ(3.0, capsule2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, CopyAssignmentAfterMove)
{
  sdf::Capsule capsule1;
  capsule1.SetRadius(0.2);
  capsule1.SetLength(3.0);

  sdf::Capsule capsule2;
  capsule2.SetRadius(2);
  capsule2.SetLength(30.0);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Capsule tmp = std::move(capsule1);
  capsule1 = capsule2;
  capsule2 = tmp;

  EXPECT_DOUBLE_EQ(2, capsule1.Radius());
  EXPECT_DOUBLE_EQ(30, capsule1.Length());

  EXPECT_DOUBLE_EQ(0.2, capsule2.Radius());
  EXPECT_DOUBLE_EQ(3.0, capsule2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, Load)
{
  sdf::Capsule capsule;
  sdf::Errors errors;

  // Null element name
  errors = capsule.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, capsule.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = capsule.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, capsule.Element());

  // Missing <radius> and <length> elements
  sdf->SetName("capsule");
  errors = capsule.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <radius>"))
      << errors[0].Message();
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find("Invalid <length>"))
      << errors[1].Message();
  EXPECT_NE(nullptr, capsule.Element());

  // Add <radius> element description
  sdf::ElementPtr radiusDesc(new sdf::Element());
  radiusDesc->SetName("radius");
  radiusDesc->AddValue("double", "1.0", true, "radius");
  sdf->AddElementDescription(radiusDesc);

  // Add radius element
  sdf::ElementPtr radiusElem = sdf->AddElement("radius");
  radiusElem->Set<double>(2.0);

  // Missing <length> element
  sdf->SetName("capsule");
  errors = capsule.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <length>"))
      << errors[0].Message();

  // Add <length> element description
  sdf::ElementPtr lengthDesc(new sdf::Element());
  lengthDesc->SetName("length");
  lengthDesc->AddValue("double", "1.0", true, "length");
  sdf->AddElementDescription(lengthDesc);

  // Add length element and test negative radius
  sdf::ElementPtr lengthElem = sdf->AddElement("length");
  lengthElem->Set<double>(3.0);
  radiusElem->Set<double>(-1.0);
  errors = capsule.Load(sdf);
  ASSERT_EQ(0u, errors.size());
  EXPECT_NE(nullptr, capsule.Element());
  EXPECT_DOUBLE_EQ(0.5, capsule.Radius());

  // Test negative length
  radiusElem->Set<double>(1.0);
  lengthElem->Set<double>(-1.0);
  errors = capsule.Load(sdf);
  ASSERT_EQ(0u, errors.size());
  EXPECT_NE(nullptr, capsule.Element());
  EXPECT_DOUBLE_EQ(1.0, capsule.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, Shape)
{
  sdf::Capsule capsule;
  EXPECT_DOUBLE_EQ(0.5, capsule.Radius());
  EXPECT_DOUBLE_EQ(1.0, capsule.Length());

  capsule.Shape().SetRadius(0.123);
  capsule.Shape().SetLength(0.456);
  EXPECT_DOUBLE_EQ(0.123, capsule.Radius());
  EXPECT_DOUBLE_EQ(0.456, capsule.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, CalculateInertial)
{
  sdf::Capsule capsule;

  // density of Aluminum
  const double density = 2710;
  const double l = 2.0;
  const double r = 0.1;

  capsule.SetLength(l);
  capsule.SetRadius(r);

  double expectedMass = capsule.Shape().Volume() * density;
  const double cylinderVolume = GZ_PI * r*r * l;
  const double sphereVolume = GZ_PI * 4. / 3. * r*r*r;
  const double volume = cylinderVolume + sphereVolume;
  const double cylinderMass = expectedMass * cylinderVolume / volume;
  const double sphereMass = expectedMass * sphereVolume / volume;

  double ixxIyy = (1/12.0) * cylinderMass * (3*r*r + l*l)
    + sphereMass * (0.4*r*r + 0.375*r*l + 0.25*l*l);
  double izz = r*r * (0.5 * cylinderMass + 0.4 * sphereMass);

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixxIyy, ixxIyy, izz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  auto capsuleInertial = capsule.CalculateInertial(density);
  EXPECT_EQ(capsule.Shape().Mat().Density(), density);
  ASSERT_NE(std::nullopt, capsuleInertial);
  EXPECT_EQ(expectedInertial, *capsuleInertial);
  EXPECT_EQ(expectedInertial.MassMatrix().DiagonalMoments(),
    capsuleInertial->MassMatrix().DiagonalMoments());
  EXPECT_EQ(expectedInertial.MassMatrix().Mass(),
    capsuleInertial->MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.Pose(), capsuleInertial->Pose());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, ToElement)
{
  sdf::Capsule capsule;

  capsule.SetRadius(1.2);
  capsule.SetLength(0.5);

  sdf::ElementPtr elem = capsule.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Capsule capsule2;
  capsule2.Load(elem);

  EXPECT_DOUBLE_EQ(capsule.Radius(), capsule2.Radius());
  EXPECT_DOUBLE_EQ(capsule.Length(), capsule2.Length());
}

/////////////////////////////////////////////////
TEST(DOMCapsule, ToElementErrorOutput)
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

  sdf::Capsule capsule;
  sdf::Errors errors;

  capsule.SetRadius(1.2);
  capsule.SetLength(0.5);

  sdf::ElementPtr elem = capsule.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Capsule capsule2;
  errors = capsule2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(capsule.Radius(), capsule2.Radius());
  EXPECT_DOUBLE_EQ(capsule.Length(), capsule2.Length());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

/////////////////////////////////////////////////
TEST(DOMCapsule, AxisAlignedBox)
{
  sdf::Capsule capsule;
  capsule.SetRadius(0.5);
  capsule.SetLength(2.0);

  auto aabb = capsule.AxisAlignedBox();
  EXPECT_EQ(gz::math::Vector3d(1.0, 1.0, 3.0), aabb.Size());
  EXPECT_EQ(gz::math::Vector3d(-0.5, -0.5, -1.5), aabb.Min());
  EXPECT_EQ(gz::math::Vector3d(0.5, 0.5, 1.5), aabb.Max());
}
