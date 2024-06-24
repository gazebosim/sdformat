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
#include "sdf/Capsule.hh"
#include "test_utils.hh"

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

  // Add a radius element
  sdf::ElementPtr radiusDesc(new sdf::Element());
  radiusDesc->SetName("radius");
  radiusDesc->AddValue("double", "1.0", true, "radius");
  sdf->AddElementDescription(radiusDesc);
  sdf::ElementPtr radiusElem = sdf->AddElement("radius");
  radiusElem->Set<double>(2.0);

  // Missing <length> element
  sdf->SetName("capsule");
  errors = capsule.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INVALID, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("Invalid <length>"))
      << errors[0].Message();
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
