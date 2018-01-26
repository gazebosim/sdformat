/*
 * Copyright 2018 Open Source Robotics Foundation
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

#include <string>
#include <gtest/gtest.h>

#include "sdf/Element.hh"
#include "sdf/Joint.hh"
#include "sdf/Types.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(DOMJoint, NotAJoint)
{
  // Create an Element that is not a joint
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("world");
  sdf::Joint joint;
  sdf::Errors errors = joint.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_INCORRECT_TYPE);
  EXPECT_TRUE(errors[0].Message().find("Attempting to load a Joint") !=
               std::string::npos);
}

//////////////////////////////////////////////////
TEST(DOMJoint, NoName)
{
  // Create a "joint" with no name
  sdf::ElementPtr element(new sdf::Element);
  element->SetName("joint");

  element->PrintValues("  ");
  sdf::Joint joint;
  sdf::Errors errors = joint.Load(element);
  ASSERT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ATTRIBUTE_MISSING);
  EXPECT_TRUE(errors[0].Message().find("joint name is required") !=
               std::string::npos);
}
