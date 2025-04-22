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
#include <gz/math/Vector3.hh>
#include <gz/math/Vector2.hh>
#include "sdf/Plane.hh"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(DOMPlane, Construction)
{
  sdf::Plane plane;
  EXPECT_EQ(nullptr, plane.Element());

  EXPECT_EQ(gz::math::Vector3d::UnitZ, plane.Normal());
  EXPECT_EQ(gz::math::Vector2d::One, plane.Size());

  plane.SetNormal({1, 0, 0});
  EXPECT_EQ(gz::math::Vector3d::UnitX, plane.Normal());

  plane.SetNormal({1, 0, 1});
  EXPECT_EQ(gz::math::Vector3d(0.707107, 0, 0.707107), plane.Normal());

  plane.SetSize({1.2, 3.4});
  EXPECT_EQ(gz::math::Vector2d(1.2, 3.4), plane.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, MoveConstructor)
{
  sdf::Plane plane;
  plane.SetNormal({1, 0, 0});
  plane.SetSize({1.2, 3.4});

  sdf::Plane plane2(std::move(plane));
  EXPECT_EQ(gz::math::Vector3d::UnitX, plane2.Normal());
  EXPECT_EQ(gz::math::Vector2d(1.2, 3.4), plane2.Size());

  EXPECT_EQ(gz::math::Vector3d::UnitX, plane2.Shape().Normal());
  EXPECT_EQ(gz::math::Vector2d(1.2, 3.4), plane2.Shape().Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, CopyConstructor)
{
  sdf::Plane plane;
  plane.SetNormal({1, 0, 0});
  plane.SetSize({1.2, 3.4});

  sdf::Plane plane2(plane);
  EXPECT_EQ(gz::math::Vector3d::UnitX, plane2.Normal());
  EXPECT_EQ(gz::math::Vector2d(1.2, 3.4), plane2.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, CopyAssignmentOperator)
{
  sdf::Plane plane;
  plane.SetNormal({1, 0, 0});
  plane.SetSize({1.2, 3.4});

  sdf::Plane plane2;
  plane2 = plane;
  EXPECT_EQ(gz::math::Vector3d::UnitX, plane2.Normal());
  EXPECT_EQ(gz::math::Vector2d(1.2, 3.4), plane2.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, MoveAssignmentOperator)
{
  sdf::Plane plane;
  plane.SetNormal({1, 0, 0});
  plane.SetSize({1.2, 3.4});

  sdf::Plane plane2;
  plane2 = std::move(plane);
  EXPECT_EQ(gz::math::Vector3d::UnitX, plane2.Normal());
  EXPECT_EQ(gz::math::Vector2d(1.2, 3.4), plane2.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, CopyAssignmentAfterMove)
{
  sdf::Plane plane1;
  plane1.SetNormal(gz::math::Vector3d::UnitX);

  sdf::Plane plane2;
  plane2.SetNormal(gz::math::Vector3d::UnitY);

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Plane tmp = std::move(plane1);
  plane1 = plane2;
  plane2 = tmp;

  EXPECT_EQ(gz::math::Vector3d::UnitY, plane1.Normal());
  EXPECT_EQ(gz::math::Vector3d::UnitX, plane2.Normal());
}

/////////////////////////////////////////////////
TEST(DOMPlane, Load)
{
  sdf::Plane plane;
  sdf::Errors errors;

  // Null element name
  errors = plane.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, plane.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = plane.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, plane.Element());

  // Missing <normal> and <size> elements
  sdf->SetName("plane");
  errors = plane.Load(sdf);
  ASSERT_EQ(2u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <normal>"));
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[1].Code());
  EXPECT_NE(std::string::npos, errors[1].Message().find("missing a <size>"));
  EXPECT_NE(nullptr, plane.Element());

  // Add <normal> element description
  sdf::ElementPtr normalDesc(new sdf::Element());
  normalDesc->SetName("normal");
  normalDesc->AddValue("vector3", "0 0 1", true, "normal");
  sdf->AddElementDescription(normalDesc);

  // Add normal element
  sdf::ElementPtr normalElem = sdf->AddElement("normal");
  normalElem->Set<gz::math::Vector3d>({1, 0, 0});

  // Missing <size> element
  sdf->SetName("plane");
  errors = plane.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <size>"));

  // Now add <size> element description
  sdf::ElementPtr sizeDesc(new sdf::Element());
  sizeDesc->SetName("size");
  sizeDesc->AddValue("vector2d", "1 1", true, "size");
  sdf->AddElementDescription(sizeDesc);

  // Negative <size> element
  sdf::ElementPtr sizeElem = sdf->AddElement("size");
  sizeElem->Set<gz::math::Vector2d>(gz::math::Vector2d(-1, -1));
  errors = plane.Load(sdf);
  ASSERT_EQ(0u, errors.size());
  EXPECT_NE(nullptr, plane.Element());
  EXPECT_EQ(gz::math::Vector2d::One, plane.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, Shape)
{
  sdf::Plane plane;
  EXPECT_EQ(gz::math::Vector2d::One, plane.Size());

  plane.Shape().Set(plane.Shape().Normal(), gz::math::Vector2d(1, 2),
      plane.Shape().Offset());
  EXPECT_EQ(gz::math::Vector2d(1, 2), plane.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, ToElement)
{
  sdf::Plane plane;

  plane.SetNormal(gz::math::Vector3d(0, 1, 0));
  plane.SetSize(gz::math::Vector2d(2, 4));

  sdf::ElementPtr elem = plane.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Plane plane2;
  plane2.Load(elem);

  EXPECT_EQ(plane.Normal(), plane2.Normal());
  EXPECT_EQ(plane.Size(), plane2.Size());
}

/////////////////////////////////////////////////
TEST(DOMPlane, ToElementErrorOutput)
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

  sdf::Plane plane;
  sdf::Errors errors;

  plane.SetNormal(gz::math::Vector3d(0, 1, 0));
  plane.SetSize(gz::math::Vector2d(2, 4));

  sdf::ElementPtr elem = plane.ToElement(errors);
  EXPECT_TRUE(errors.empty());
  ASSERT_NE(nullptr, elem);

  sdf::Plane plane2;
  errors = plane2.Load(elem);
  EXPECT_TRUE(errors.empty());

  EXPECT_EQ(plane.Normal(), plane2.Normal());
  EXPECT_EQ(plane.Size(), plane2.Size());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
