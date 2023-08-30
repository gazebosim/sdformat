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
#include "sdf/Sphere.hh"
#include "test_utils.hh"
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMSphere, Construction)
{
  sdf::Sphere sphere;
  EXPECT_EQ(nullptr, sphere.Element());

  EXPECT_DOUBLE_EQ(1.0, sphere.Radius());

  sphere.SetRadius(0.5);

  EXPECT_DOUBLE_EQ(0.5, sphere.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, MoveConstructor)
{
  sdf::Sphere sphere;
  sphere.SetRadius(0.2);

  sdf::Sphere sphere2(std::move(sphere));
  EXPECT_DOUBLE_EQ(0.2, sphere2.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, CopyConstructor)
{
  sdf::Sphere sphere;
  sphere.SetRadius(0.2);

  sdf::Sphere sphere2(sphere);
  EXPECT_DOUBLE_EQ(0.2, sphere2.Radius());

  EXPECT_DOUBLE_EQ(4.0/3.0*GZ_PI*std::pow(0.2, 3), sphere2.Shape().Volume());
  EXPECT_DOUBLE_EQ(0.2, sphere2.Shape().Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, CopyAssignmentOperator)
{
  sdf::Sphere sphere;
  sphere.SetRadius(0.2);

  sdf::Sphere sphere2;
  sphere2 = sphere;
  EXPECT_DOUBLE_EQ(0.2, sphere2.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, MoveAssignmentOperator)
{
  sdf::Sphere sphere;
  sphere.SetRadius(0.2);

  sdf::Sphere sphere2;
  sphere2 = std::move(sphere);
  EXPECT_DOUBLE_EQ(0.2, sphere2.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, CopyAssignmentAfterMove)
{
  sdf::Sphere sphere1;
  sphere1.SetRadius(0.1);

  sdf::Sphere sphere2;
  sphere2.SetRadius(0.2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Sphere tmp = std::move(sphere1);
  sphere1 = sphere2;
  sphere2 = tmp;

  EXPECT_DOUBLE_EQ(0.2, sphere1.Radius());
  EXPECT_DOUBLE_EQ(0.1, sphere2.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, Load)
{
  sdf::Sphere sphere;
  sdf::Errors errors;

  // Null element name
  errors = sphere.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, sphere.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = sphere.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, sphere.Element());

  // Missing <radius> element
  sdf->SetName("sphere");
  errors = sphere.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <radius>"));
  EXPECT_NE(nullptr, sphere.Element());
}

/////////////////////////////////////////////////
TEST(DOMSphere, Shape)
{
  sdf::Sphere sphere;
  EXPECT_DOUBLE_EQ(1.0, sphere.Radius());

  sphere.Shape().SetRadius(0.123);
  EXPECT_DOUBLE_EQ(0.123, sphere.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, CalculateInertial)
{
  sdf::Sphere sphere;

  // density of aluminium
  const double density = 2170;

  sphere.SetRadius(-2);
  auto invalidSphereInertial = sphere.CalculateInertial(density);
  ASSERT_EQ(std::nullopt, invalidSphereInertial);

  const double r = 0.1;

  sphere.SetRadius(r);

  double expectedMass = sphere.Shape().Volume() * density;
  double ixxIyyIzz = 0.4 * expectedMass * r * r;

  gz::math::MassMatrix3d expectedMassMat(
    expectedMass,
    gz::math::Vector3d(ixxIyyIzz, ixxIyyIzz, ixxIyyIzz),
    gz::math::Vector3d::Zero
  );

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  auto sphereInertial = sphere.CalculateInertial(density);
  EXPECT_EQ(sphere.Shape().Material().Density(), density);
  ASSERT_NE(std::nullopt, sphereInertial);
  EXPECT_EQ(expectedInertial, *sphereInertial);
  EXPECT_EQ(expectedInertial.MassMatrix().DiagonalMoments(),
    sphereInertial->MassMatrix().DiagonalMoments());
  EXPECT_EQ(expectedInertial.MassMatrix().Mass(),
    sphereInertial->MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.Pose(), sphereInertial->Pose());
}

/////////////////////////////////////////////////
TEST(DOMSphere, ToElement)
{
  sdf::Sphere sphere;

  sphere.SetRadius(1.2);

  sdf::ElementPtr elem = sphere.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Sphere sphere2;
  sphere2.Load(elem);

  EXPECT_DOUBLE_EQ(sphere.Radius(), sphere2.Radius());
}

/////////////////////////////////////////////////
TEST(DOMSphere, ToElementErrorOutput)
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

  sdf::Sphere sphere;
  sdf::Errors errors;

  sphere.SetRadius(1.2);

  sdf::ElementPtr elem = sphere.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Sphere sphere2;
  errors = sphere2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_DOUBLE_EQ(sphere.Radius(), sphere2.Radius());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
