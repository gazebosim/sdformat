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
#include "sdf/Mesh.hh"

/////////////////////////////////////////////////
TEST(DOMMesh, Construction)
{
  sdf::Mesh mesh;
  EXPECT_EQ(nullptr, mesh.Element());

  EXPECT_EQ(std::string(), mesh.Uri());
  EXPECT_EQ(std::string(), mesh.Submesh());
  EXPECT_TRUE(ignition::math::Vector3d(1, 1, 1) == mesh.Scale());
  EXPECT_FALSE( mesh.CenterSubmesh());
}

/////////////////////////////////////////////////
TEST(DOMMesh, Load)
{
  sdf::Mesh mesh;
  sdf::Errors errors;

  // Null element name
  errors = mesh.Load(nullptr);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_EQ(nullptr, mesh.Element());

  // Bad element name
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("bad");
  errors = mesh.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_INCORRECT_TYPE, errors[0].Code());
  EXPECT_NE(nullptr, mesh.Element());

  // Missing <uri> element
  sdf->SetName("mesh");
  errors = mesh.Load(sdf);
  ASSERT_EQ(1u, errors.size());
  EXPECT_EQ(sdf::ErrorCode::ELEMENT_MISSING, errors[0].Code());
  EXPECT_NE(std::string::npos, errors[0].Message().find("missing a <uri>"));
  EXPECT_NE(nullptr, mesh.Element());
}
