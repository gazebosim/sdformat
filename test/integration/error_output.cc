/*
 * Copyright 2022 Open Source Robotics Foundation
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

#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Param.hh"
#include "sdf/Types.hh"
#include "test_utils.hh"

////////////////////////////////////////
// Test Param class for sdf::Errors outputs
TEST(Error, ErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
    sdf::Console::Instance()->GetMsgStream(), &buffer);

  sdf::Errors errors;
  ASSERT_NO_THROW(sdf::Param param1("key", "not_valid_type", "true", false,
        errors, "description"));
  ASSERT_GE(2u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE);
  EXPECT_NE(std::string::npos,
      errors[0].Message().find(
        "Unknown parameter type[not_valid_type]"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos,
      errors[1].Message().find("Invalid parameter"));

  errors.clear();
  ASSERT_NO_THROW(sdf::Param param2("key", "not_valid_type", "true",
                                    false, "", "", errors, "description"));
  ASSERT_GE(2u, errors.size());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE);
  EXPECT_NE(std::string::npos,
   errors[0].Message().find("Unknown parameter type[not_valid_type]"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos,
   errors[1].Message().find("Invalid parameter"));

  errors.clear();
  sdf::Param param3("key", "bool", "true", false, errors, "description");
  // Check no new errors were added
  ASSERT_EQ(errors.size(), 0u);
  param3.Set(4, errors);
  ASSERT_GE(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos,
   errors[0].Message().find("Invalid boolean value"));

  errors.clear();
  ignition::math::Pose3d pose;
  param3.Get<ignition::math::Pose3d>(pose, errors);
  ASSERT_GE(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
    "The value for //pose[@rotation_format='euler_rpy'] must have 6 "
    "values, but 1 were found instead in '1'."));

  errors.clear();
  param3.Update(errors);
  ASSERT_GE(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
    "[updateFunc] is not set."));

  errors.clear();
  sdf::Param requiredParam("key", "int", "1", true, "2", "4", errors,
                           "description");
  // Check no new errors were added
  ASSERT_EQ(errors.size(), 0u);
  requiredParam.SetFromString("", errors);
  ASSERT_GE(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
    "Empty string used when setting a required parameter. Key[key]"));
  EXPECT_FALSE(requiredParam.ValidateValue(errors));
  ASSERT_GE(errors.size(), 2u);
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[1].Message().find(
    "The value [1] is less than the minimum allowed value of [2] for "
    "key [key]"));


  errors.clear();
  // Adding a parent with @rotation_format to something invalid
  // will make reparse fail
  sdf::Param poseParam("", "pose", "1 2 3 0.4 0.5 0.6", false, "description");
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->AddAttribute("rotation_format", "string", "invalid_format", false);
  EXPECT_FALSE(poseParam.SetParentElement(poseElem, errors));
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
    "Undefined attribute //pose[@rotation_format='invalid_format'], "
    "only 'euler_rpy' and 'quat_xyzw' is supported."));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[1].Message().find(
    "Failed to set value '1 2 3 0.40000000000000002 0.5 "
    "0.59999999999999987' to key [] for new parent element of name '',"
    " reverting to previous value '1 2 3 0.40000000000000002 0.5 "
    "0.59999999999999987'."));

  errors.clear();
  sdf::Param param4("key", "bool", "15", false, "a", "b", errors,
                    "description");
  ASSERT_EQ(errors.size(), 6u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
    "Invalid boolean value"));
  EXPECT_EQ(errors[1].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[1].Message().find(
    "Invalid parameter"));
  EXPECT_EQ(errors[2].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[2].Message().find(
    "Invalid boolean value"));
  EXPECT_EQ(errors[3].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[3].Message().find(
    "Invalid [min] parameter in SDFormat description of [key]"));
  EXPECT_EQ(errors[4].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[4].Message().find(
    "Invalid boolean value"));
  EXPECT_EQ(errors[5].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  EXPECT_NE(std::string::npos, errors[5].Message().find(
    "Invalid [max] parameter in SDFormat description of [key]"));

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
