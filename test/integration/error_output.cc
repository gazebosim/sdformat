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
  ASSERT_EQ(errors[0].Code(), sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE);
  ASSERT_NE(std::string::npos,
   errors[0].Message().find(
     "Unknown parameter type[not_valid_type]"));
  ASSERT_EQ(errors[1].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos,
   errors[1].Message().find("Invalid parameter"));

  ASSERT_NO_THROW(sdf::Param param2("key", "not_valid_type", "true",
                                    false, "", "", errors, "description"));
  ASSERT_EQ(errors[2].Code(), sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE);
  ASSERT_NE(std::string::npos,
   errors[2].Message().find("Unknown parameter type[not_valid_type]"));
  ASSERT_EQ(errors[3].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos,
   errors[3].Message().find("Invalid parameter"));

  sdf::Param param3("key", "bool", "true", false, errors, "description");
  // Check no new errors were added
  ASSERT_EQ(errors.size(), 4u);
  param3.Set(4, errors);
  ASSERT_EQ(errors[4].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos,
   errors[4].Message().find("Invalid boolean value"));

  ignition::math::Pose3d pose;
  param3.Get<ignition::math::Pose3d>(pose, errors);
  ASSERT_EQ(errors[5].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[5].Message().find(
    "The value for //pose[@rotation_format='euler_rpy'] must have 6 "
    "values, but 1 were found instead in '1'."));

  param3.Update(errors);
  ASSERT_EQ(errors[6].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[6].Message().find(
    "[updateFunc] is not set."));

  sdf::Param requiredParam("key", "int", "1", true, "2", "4", errors,
                           "description");
  // Check no new errors were added
  ASSERT_EQ(errors.size(), 7u);
  requiredParam.SetFromString("", errors);
  ASSERT_EQ(errors[7].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[7].Message().find(
    "Empty string used when setting a required parameter. Key[key]"));
  ASSERT_FALSE(requiredParam.ValidateValue(errors));
  ASSERT_EQ(errors[8].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[8].Message().find(
    "The value [1] is less than the minimum allowed value of [2] for "
    "key [key]"));


  // Adding a parent with @rotation_format to something invalid
  // will make reparse fail
  sdf::Param poseParam("", "pose", "1 2 3 0.4 0.5 0.6", false, "description");
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->AddAttribute("rotation_format", "string", "invalid_format", false);
  ASSERT_FALSE(poseParam.SetParentElement(poseElem, errors));
  ASSERT_EQ(errors[9].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[9].Message().find(
    "Undefined attribute //pose[@rotation_format='invalid_format'], "
    "only 'euler_rpy' and 'quat_xyzw' is supported."));
  ASSERT_EQ(errors[10].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[10].Message().find(
    "Failed to set value '1 2 3 0.40000000000000002 0.5 "
    "0.59999999999999987' to key [] for new parent element of name '',"
    " reverting to previous value '1 2 3 0.40000000000000002 0.5 "
    "0.59999999999999987'."));

  sdf::Param param4("key", "bool", "15", false, "a", "b", errors,
                    "description");
  ASSERT_EQ(errors[11].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[11].Message().find(
    "Invalid boolean value"));
  ASSERT_EQ(errors[12].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[12].Message().find(
    "Invalid parameter"));
  ASSERT_EQ(errors[13].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[13].Message().find(
    "Invalid boolean value"));
  ASSERT_EQ(errors[14].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[14].Message().find(
    "Invalid [min] parameter in SDFormat description of [key]"));
  ASSERT_EQ(errors[15].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[15].Message().find(
    "Invalid boolean value"));
  ASSERT_EQ(errors[16].Code(), sdf::ErrorCode::PARAMETER_ERROR);
  ASSERT_NE(std::string::npos, errors[16].Message().find(
    "Invalid [max] parameter in SDFormat description of [key]"));

  // Check nothing has been printed
  ASSERT_TRUE(buffer.str().empty()) << buffer.str();
}
