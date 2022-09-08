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
#include "sdf/World.hh"
#include "test_utils.hh"
#include "test_config.hh"

////////////////////////////////////////
// Test Param class for sdf::Errors outputs
TEST(ErrorOutput, ParamErrorOutput)
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
  gz::math::Pose3d pose;
  param3.Get<gz::math::Pose3d>(pose, errors);
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

////////////////////////////////////////
// Test Element class for sdf::Errors outputs
TEST(ErrorOutput, ElementErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  sdf::Errors errors;
  sdf::ElementPtr elem = std::make_shared<sdf::Element>();
  elem->SetName("testElement");

  elem->GetAny(errors, "test");
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::ELEMENT_ERROR);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Unable to find value for key [test]"));

  errors.clear();
  elem->GetElement("missingElement", errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Missing element description for [missingElement]"));

  errors.clear();
  elem->AddAttribute(
      "invalidAttribute", "int", "invalidFormat", false, errors);
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Invalid argument. Unable to set value [invalidFormat]"
      " for key[invalidAttribute]"));
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "Invalid parameter"));

  errors.clear();
  elem->AddValue("type", "value", true, "a", "b", errors);
  ASSERT_EQ(errors.size(), 9u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Unknown parameter type[type]"));
  EXPECT_NE(std::string::npos, errors[1].Message().find(
      "Invalid parameter"));
  EXPECT_NE(std::string::npos, errors[2].Message().find(
      "Unknown parameter type[type]"));
  EXPECT_NE(std::string::npos, errors[3].Message().find(
      "Invalid [min] parameter in SDFormat description of [testElement]"));
  EXPECT_NE(std::string::npos, errors[4].Message().find(
      "Unknown parameter type[type]"));
  EXPECT_NE(std::string::npos, errors[5].Message().find(
      "Invalid [max] parameter in SDFormat description of [testElement]"));
  EXPECT_NE(std::string::npos, errors[6].Message().find(
      "Unknown parameter type[type]"));
  EXPECT_NE(std::string::npos, errors[7].Message().find(
      "Failed to set value '0' to key [testElement] for new parent element"
      " of name 'testElement', reverting to previous value '0'."));
  EXPECT_NE(std::string::npos, errors[8].Message().find(
      "Cannot set parent Element of value to itself."));
  errors.clear();

  elem->GetElement("nonExistentElement", errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Missing element description for [nonExistentElement]"));
  errors.clear();

  elem->RemoveChild(sdf::ElementPtr(), errors);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Cannot remove a nullptr child pointer"));
  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

////////////////////////////////////////
// Test PrintConfig class for sdf::Errors outputs
TEST(ErrorOutput, PrintConfigErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
      sdf::Console::Instance()->GetMsgStream(), &buffer);

  sdf::Errors errors;
  sdf::PrintConfig printConfig;
  ASSERT_FALSE(printConfig.SetRotationSnapToDegrees(361, 300, errors));
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Interval value to snap to must be larger than 0,"
      " and less than or equal to 360."));
  errors.clear();

  ASSERT_FALSE(printConfig.SetRotationSnapToDegrees(300, 300, errors));
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
      "Tolerance must be larger than 0, less than or equal to 360, "
      "and less than the provided interval."));
  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}

////////////////////////////////////////
// Test World class for sdf::Errors outputs
TEST(ErrorOutput, WorldErrorOutput)
{
  std::stringstream buffer;
  sdf::testing::RedirectConsoleStream redir(
    sdf::Console::Instance()->GetMsgStream(), &buffer);

  sdf::Errors errors;

  std::ostringstream stream;
  stream << "<?xml version=\"1.0\"?>"
         << "<sdf version='1.8'>"
         << "  <world name='test_world'>"
         << "    <model name='common_name'>"
         << "      <link name='a'>"
         << "      </link>"
         << "    </model>"
         << "    <model name='common_name'>"
         << "      <link name='a'>"
         << "      </link>"
         << "    </model>"
         << "    <frame name='common_name'/>"
         << "  </world>"
         << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  sdf::ParserConfig parserConfig;
  parserConfig.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  sdf::readString(stream.str(), parserConfig, sdfParsed, errors);
  EXPECT_TRUE(errors.empty());

  sdf::World world;
  errors = world.Load(sdfParsed->Root()->GetElement("world"), parserConfig);
  ASSERT_EQ(errors.size(), 3u);
  EXPECT_NE(std::string::npos, errors[0].Message().find(
    "Non-unique name[common_name] detected 3 times in XML children of world"
    " with name[test_world]."));
  EXPECT_NE(std::string::npos, errors[1].Message().find(
    "model with name[common_name] already exists."));
  EXPECT_NE(std::string::npos, errors[2].Message().find(
    "Frame with name [common_name] in world with name [test_world] has a name"
    " collision, changing frame name to [common_name_frame]."));
  // Check nothing has been printed
  EXPECT_TRUE(buffer.str().empty()) << buffer.str();
}
