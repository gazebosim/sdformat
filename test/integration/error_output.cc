/*
 * Copyright 2015 Open Source Robotics Foundation
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


////////////////////////////////////////
// Test Param class for sdf::Errors outputs
TEST(Error, ErrorOutput)
{
  std::stringstream buffer;
  std::streambuf * old = std::cerr.rdbuf(buffer.rdbuf());
  sdf::Errors errors;
  sdf::Error error;
  EXPECT_NO_THROW(sdf::Param param1("key", "not_valid_type", "true", false,
                                    errors, "description"));
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE, error.Code());
  ASSERT_EQ("Unknown parameter type[not_valid_type]", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid parameter", error.Message());

  EXPECT_NO_THROW(sdf::Param param2("key", "not_valid_type", "true",
                                    false, "", "", errors, "description"));
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::UNKNOWN_PARAMETER_TYPE, error.Code());
  ASSERT_EQ("Unknown parameter type[not_valid_type]", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid parameter", error.Message());

  sdf::Param param3("key", "bool", "true", false, errors, "description");
  ASSERT_TRUE(errors.empty());
  param3.Set(4, errors);
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid boolean value", error.Message());

  ignition::math::Pose3d pose;
  param3.Get<ignition::math::Pose3d>(pose, errors);
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("The value for //pose[@rotation_format='euler_rpy'] must have 6 "
            "values, but 1 were found instead in '1'.", error.Message());

  param3.Update(errors);
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("[updateFunc] is not set.", error.Message());

  sdf::Param requiredParam("key", "int", "1", true, "2", "4", errors,
                           "description");
  EXPECT_TRUE(errors.empty());
  requiredParam.SetFromString("", errors);
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Empty string used when setting a required parameter. Key[key]",
            error.Message());
  ASSERT_FALSE(requiredParam.ValidateValue(errors));
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("The value [1] is less than the minimum allowed value of [2] for"
            " key [key]", error.Message());


  // Adding a parent with @rotation_format to something invalid
  // will make reparse fail
  sdf::Param poseParam("", "pose", "1 2 3 0.4 0.5 0.6", false, "description");
  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->AddAttribute("rotation_format", "string", "invalid_format", false);
  ASSERT_FALSE(poseParam.SetParentElement(poseElem, errors));
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Undefined attribute //pose[@rotation_format='invalid_format'], "
            "only 'euler_rpy' and 'quat_xyzw' is supported.", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Failed to set value '1 2 3 0.40000000000000002 0.5 "
            "0.59999999999999987' to key [] for new parent element of name '',"
            " reverting to previous value '1 2 3 0.40000000000000002 0.5 "
            "0.59999999999999987'."
            , error.Message());

  sdf::Param param4("key", "bool", "15", false, "a", "b", errors,
                    "description");
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid boolean value", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid parameter", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid boolean value", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid [min] parameter in SDFormat description of [key]",
            error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid boolean value", error.Message());
  error = errors.front();
  errors.erase(errors.begin());
  ASSERT_EQ(sdf::ErrorCode::PARAMETER_ERROR, error.Code());
  ASSERT_EQ("Invalid [max] parameter in SDFormat description of [key]",
            error.Message());

  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty());
  std::cerr.rdbuf(old);
}
