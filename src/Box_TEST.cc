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
#include "sdf/Box.hh"
#include "sdf/Element.hh"
#include "test_utils.hh"
#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/MassMatrix3.hh>

/////////////////////////////////////////////////
TEST(DOMBox, Construction)
{
  sdf::Box box;
  EXPECT_EQ(nullptr, box.Element());

  EXPECT_EQ(gz::math::Vector3d::One, box.Size());

  box.SetSize(gz::math::Vector3d::Zero);
  EXPECT_EQ(gz::math::Vector3d::Zero, box.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, MoveConstructor)
{
  const gz::math::Vector3d size(1, 2, 3);

  sdf::Box box;
  box.SetSize(size);

  sdf::Box box2(std::move(box));
  EXPECT_EQ(size, box2.Size());

  EXPECT_DOUBLE_EQ(1 * 2 * 3, box2.Shape().Volume());
  EXPECT_EQ(size, box2.Shape().Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, CopyConstructor)
{
  const gz::math::Vector3d size(0.1, 0.2, 0.3);

  sdf::Box box;
  box.SetSize(size);

  sdf::Box box2(box);
  EXPECT_EQ(size, box2.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, CopyAssigmentOperator)
{
  const gz::math::Vector3d size(0.2, 0.3, 0.4);

  sdf::Box box;
  box.SetSize(size);

  sdf::Box box2;
  box2 = box;
  EXPECT_EQ(size, box2.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, MoveAssignmentConstructor)
{
  const gz::math::Vector3d size(1, 2, 3);

  sdf::Box box;
  box.SetSize(size);

  sdf::Box box2;
  box2 = std::move(box);
  EXPECT_EQ(size, box2.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, CopyAssignmentAfterMove)
{
  const gz::math::Vector3d size1(1, 2, 3);
  const gz::math::Vector3d size2(4, 5, 6);

  sdf::Box box1;
  box1.SetSize(size1);

  sdf::Box box2;
  box2.SetSize(size2);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Box tmp = std::move(box1);
  box1 = box2;
  box2 = tmp;

  EXPECT_EQ(size2, box1.Size());
  EXPECT_EQ(size1, box2.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, Load)
{
  sdf::Box box;
  sdf::Errors errors;

  // Null element name
  errors = box.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = box.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, box.Element());

  // Missing <size> element
  sdf->SetName("box");
  errors = box.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <size>"));
  EXPECT_NE(nullptr, box.Element());

  // Define <size> element description
  sdf::ElementPtr sizeDesc(new sdf::Element());
  sizeDesc->SetName("size");
  sizeDesc->AddValue("vector3", "1 1 1", true, "size vector");
  sdf->AddElementDescription(sizeDesc);

  // Now add the element and assign a negative value
  sdf::ElementPtr sizeElem = sdf->AddElement("size");
  sizeElem->Set<gz::math::Vector3d>(gz::math::Vector3d(-1, -1, -1));

  // Load and check behavior
  errors = box.Load(sdf);
  ASSERT_EQ(0u, errors.size());
  EXPECT_NE(nullptr, box.Element());
  EXPECT_EQ(gz::math::Vector3d::One, box.Size()); // Defaulted to 1,1,1
}

/////////////////////////////////////////////////
TEST(DOMBox, Shape)
{
  sdf::Box box;
  EXPECT_EQ(gz::math::Vector3d::One, box.Size());

  box.Shape().SetSize(gz::math::Vector3d(1, 2, 3));
  EXPECT_EQ(gz::math::Vector3d(1, 2, 3), box.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, CalculateInertial)
{
  sdf::Box box;

  // density of Aluminum
  double density = 2710;

  // Invalid dimensions leading to std::nullopt return in
  // CalculateInertial()
  box.SetSize(gz::math::Vector3d(-1, 1, 0));
  auto invalidBoxInertial = box.CalculateInertial(density);
  ASSERT_EQ(std::nullopt, invalidBoxInertial);

  const double l = 2;
  const double w = 2;
  const double h = 2;
  box.SetSize(gz::math::Vector3d(l, w, h));

  double expectedMass = box.Shape().Volume() * density;
  double ixx = (1.0 / 12.0) * expectedMass * (w * w + h * h);
  double iyy = (1.0 / 12.0) * expectedMass * (l * l + h * h);
  double izz = (1.0 / 12.0) * expectedMass * (l * l + w * w);

  gz::math::MassMatrix3d expectedMassMat(
      expectedMass,
      gz::math::Vector3d(ixx, iyy, izz),
      gz::math::Vector3d::Zero);

  gz::math::Inertiald expectedInertial;
  expectedInertial.SetMassMatrix(expectedMassMat);
  expectedInertial.SetPose(gz::math::Pose3d::Zero);

  auto boxInertial = box.CalculateInertial(density);
  EXPECT_EQ(box.Shape().Material().Density(), density);
  ASSERT_NE(std::nullopt, boxInertial);
  EXPECT_EQ(expectedInertial, *boxInertial);
  EXPECT_EQ(expectedInertial.MassMatrix().DiagonalMoments(),
            boxInertial->MassMatrix().DiagonalMoments());
  EXPECT_EQ(expectedInertial.MassMatrix().Mass(),
            boxInertial->MassMatrix().Mass());
  EXPECT_EQ(expectedInertial.Pose(), boxInertial->Pose());
}

/////////////////////////////////////////////////
TEST(DOMBox, ToElement)
{
  sdf::Box box;

  box.SetSize(gz::math::Vector3d(1, 2, 3));

  sdf::ElementPtr elem = box.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Box box2;
  box2.Load(elem);

  EXPECT_EQ(box.Size(), box2.Size());
}

/////////////////////////////////////////////////
TEST(DOMBox, ToElementErrorOutput)
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

  sdf::Box box;
  sdf::Errors errors;

  box.SetSize(gz::math::Vector3d(1, 2, 3));

  sdf::ElementPtr elem = box.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Box box2;
  errors = box2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(box.Size(), box2.Size());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
