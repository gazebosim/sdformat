/*
 * Copyright 2014 Open Source Robotics Foundation
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
#include "sdf/Param.hh"

////////////////////////////////////////////////////
/// Test getting a bool using true/false and 1/0.
TEST(Param, Bool)
{
  sdf::Param boolParam("key", "bool", "true", false, "description");
  bool value;
  boolParam.Get<bool>(value);
  EXPECT_TRUE(value);

  boolParam.Set(false);
  boolParam.Get<bool>(value);
  EXPECT_FALSE(value);

  // String parameter that represents a boolean.
  sdf::Param strParam("key", "string", "true", false, "description");

  strParam.Get<bool>(value);
  EXPECT_TRUE(value);

  strParam.Set("false");
  strParam.Get<bool>(value);
  EXPECT_FALSE(value);

  strParam.Set("1");
  strParam.Get<bool>(value);
  EXPECT_TRUE(value);

  strParam.Set("0");
  strParam.Get<bool>(value);
  EXPECT_FALSE(value);

  // Anything other than 1 or true is treated as a false value
  strParam.Set("%");
  strParam.Get<bool>(value);
  EXPECT_FALSE(value);

  boolParam.Set(true);
  boost::any anyValue;
  EXPECT_TRUE(boolParam.GetAny(anyValue));
  try
  {
    value = boost::any_cast<bool>(anyValue);
  }
  catch(boost::bad_any_cast &_e)
  {
    FAIL();
  }
  EXPECT_TRUE(value);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
