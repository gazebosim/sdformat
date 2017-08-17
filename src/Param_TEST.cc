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

#include <cstdint>

#include <gtest/gtest.h>

#include <boost/version.hpp>

#include <ignition/math/Angle.hh>

#include "sdf/Exception.hh"
#include "sdf/Param.hh"

bool check_double(std::string num)
{
  const std::string name = "number";
  const std::string type = "double";
  const std::string def = "0.0";

  sdf::Param param(name, type, def, true);
  return param.SetFromString(num);
}

////////////////////////////////////////////////////
/// Test getting a bool using true/false and 1/0.
TEST(Param, Bool)
{
  sdf::Param boolParam("key", "bool", "true", false, "description");
  bool value = true;
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

  strParam.Set("True");
  strParam.Get<bool>(value);
  EXPECT_TRUE(value);

  strParam.Set("TRUE");
  strParam.Get<bool>(value);
  EXPECT_TRUE(value);

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
  catch(boost::bad_any_cast &/*_e*/)
  {
    FAIL();
  }
  EXPECT_TRUE(value);
}

////////////////////////////////////////////////////
/// Test decimal number
TEST(SetFromString, Decimals)
{
  ASSERT_TRUE(check_double("0.2345"));
}

////////////////////////////////////////////////////
/// Test setting and reading hex int values.
TEST(Param, HexInt)
{
  sdf::Param intParam("key", "int", "0", false, "description");
  int value;
  EXPECT_TRUE(intParam.Get<int>(value));
  EXPECT_EQ(value, 0);

  EXPECT_TRUE(intParam.SetFromString("0x01"));
  EXPECT_TRUE(intParam.Get<int>(value));
  EXPECT_EQ(value, 1);

  EXPECT_TRUE(intParam.SetFromString("0xff"));
  EXPECT_TRUE(intParam.Get<int>(value));
  EXPECT_EQ(value, 255);

  EXPECT_TRUE(intParam.SetFromString("0x00002"));
  EXPECT_TRUE(intParam.Get<int>(value));
  EXPECT_EQ(value, 2);

  EXPECT_FALSE(intParam.SetFromString("0xffffffffffffffffffffffffffffffffff"));
  EXPECT_TRUE(intParam.Get<int>(value));
  EXPECT_EQ(value, 2);
}

////////////////////////////////////////////////////
/// Test setting and reading hex unsigned int values.
TEST(Param, HexUInt)
{
  sdf::Param uintParam("key", "unsigned int", "0", false, "description");
  unsigned int value;
  EXPECT_TRUE(uintParam.Get<unsigned int>(value));
  EXPECT_EQ(value, 0u);

  EXPECT_TRUE(uintParam.SetFromString("0x01"));
  EXPECT_TRUE(uintParam.Get<unsigned int>(value));
  EXPECT_EQ(value, 1u);

  EXPECT_TRUE(uintParam.SetFromString("0xff"));
  EXPECT_TRUE(uintParam.Get<unsigned int>(value));
  EXPECT_EQ(value, 255u);

  EXPECT_TRUE(uintParam.SetFromString("0x00002"));
  EXPECT_TRUE(uintParam.Get<unsigned int>(value));
  EXPECT_EQ(value, 2u);

  EXPECT_FALSE(uintParam.SetFromString("0xffffffffffffffffffffffffffffffffff"));
  EXPECT_TRUE(uintParam.Get<unsigned int>(value));
  EXPECT_EQ(value, 2u);
}

////////////////////////////////////////////////////
/// Test setting and reading hex and non-hex float values.
TEST(Param, HexFloat)
{
// Microsoft does not parse hex values properly.
// https://bitbucket.org/osrf/sdformat/issues/114
#ifndef _MSC_VER
  sdf::Param floatParam("key", "float", "0", false, "description");
  float value;
  EXPECT_TRUE(floatParam.Get<float>(value));
  EXPECT_FLOAT_EQ(value, 0.0f);

  EXPECT_TRUE(floatParam.SetFromString("0x01"));
  EXPECT_TRUE(floatParam.Get<float>(value));
  EXPECT_FLOAT_EQ(value, 1.0f);

  EXPECT_TRUE(floatParam.SetFromString("0X2A"));
  EXPECT_TRUE(floatParam.Get<float>(value));
  EXPECT_FLOAT_EQ(value, 42.0f);

  EXPECT_TRUE(floatParam.SetFromString("0.123"));
  EXPECT_TRUE(floatParam.Get<float>(value));
  EXPECT_FLOAT_EQ(value, 0.123f);

  EXPECT_FALSE(floatParam.SetFromString("1.0e100"));
  EXPECT_TRUE(floatParam.Get<float>(value));
  EXPECT_FLOAT_EQ(value, 0.123f);
#endif
}

////////////////////////////////////////////////////
/// Test setting and reading hex and non-hex double values.
TEST(Param, HexDouble)
{
  sdf::Param doubleParam("key", "double", "0", false, "description");
  double value;
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 0.0);

// Microsoft does not parse hex values properly.
// https://bitbucket.org/osrf/sdformat/issues/114
#ifndef _MSC_VER
  EXPECT_TRUE(doubleParam.SetFromString("0x01"));
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 1.0);

  EXPECT_TRUE(doubleParam.SetFromString("0X2A"));
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 42.0);
#endif
  EXPECT_TRUE(doubleParam.SetFromString("0.123456789"));
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 0.123456789);

  EXPECT_FALSE(doubleParam.SetFromString("1.0e1000"));
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 0.123456789);
}

////////////////////////////////////////////////////
/// Test setting and reading uint64_t values.
TEST(Param, uint64t)
{
  sdf::Param uint64tParam("key", "uint64_t", "1", false, "description");
  std::uint64_t value;
  EXPECT_TRUE(uint64tParam.Get<std::uint64_t>(value));
  EXPECT_EQ(value, 1u);

  // Max uint64_t
  EXPECT_TRUE(uint64tParam.SetFromString("18446744073709551615"));
  EXPECT_TRUE(uint64tParam.Get<std::uint64_t>(value));
  EXPECT_EQ(value, UINT64_MAX);
}

////////////////////////////////////////////////////
/// Unknown type, should fall back to stream operators
TEST(Param, UnknownType)
{
  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  ignition::math::Angle value;
  EXPECT_TRUE(doubleParam.Get<ignition::math::Angle>(value));
  EXPECT_DOUBLE_EQ(value.Radian(), 1.0);
}

////////////////////////////////////////////////////
TEST(Param, Vector2i)
{
  sdf::Param vect2iParam("key", "vector2i", "0 0", false, "description");
  ignition::math::Vector2i value;

  EXPECT_TRUE(vect2iParam.Get<ignition::math::Vector2i>(value));
  EXPECT_EQ(value, ignition::math::Vector2i(0, 0));
}

////////////////////////////////////////////////////
TEST(Param, InvalidConstructor)
{
  ASSERT_THROW(sdf::Param badParam("key", "badtype", "0", false, "description"),
               sdf::AssertionInternalError);
}

////////////////////////////////////////////////////
TEST(Param, SetDescription)
{
  sdf::Param uint64Param("key", "uint64_t", "1", false, "description");

  uint64Param.SetDescription("new desc");

  ASSERT_EQ("new desc", uint64Param.GetDescription());
}

////////////////////////////////////////////////////
TEST(Param, Reset)
{
  sdf::Param uint64Param("key", "uint64_t", "1", false, "description");
  uint64_t val;

  uint64Param.SetFromString("89");

  uint64Param.Get<uint64_t>(val);
  ASSERT_EQ(89UL, val);

  uint64Param.Reset();

  uint64Param.Get<uint64_t>(val);
  ASSERT_EQ(1UL, val);
}

////////////////////////////////////////////////////
TEST(Param, EmptyRequiredSetFromString)
{
  sdf::Param uint64Param("key", "uint64_t", "1", true, "description");
  uint64_t val;

  ASSERT_FALSE(uint64Param.SetFromString(""));
  uint64Param.Get<uint64_t>(val);

  ASSERT_EQ(1UL, val);
}

////////////////////////////////////////////////////
TEST(Param, EmptySetFromString)
{
  sdf::Param uint64Param("key", "uint64_t", "1", false, "description");

  uint64_t val;

  uint64Param.SetFromString("89");

  ASSERT_TRUE(uint64Param.SetFromString(""));
  uint64Param.Get<uint64_t>(val);

  ASSERT_EQ(1UL, val);
}

////////////////////////////////////////////////////
TEST(Param, InvalidBool)
{
  sdf::Param boolParam("key", "bool", "true", false, "description");

  ASSERT_FALSE(boolParam.SetFromString("false1"));
}

////////////////////////////////////////////////////
TEST(Param, InvalidInt)
{
  sdf::Param intParam("key", "int", "1", false, "description");

  ASSERT_FALSE(intParam.SetFromString("abc"));
}

////////////////////////////////////////////////////
TEST(Param, GetAny)
{
  boost::any anyValue;

  sdf::Param intParam("key", "int", "true", false, "description");
  EXPECT_TRUE(intParam.GetAny(anyValue));

  sdf::Param uint64Param("key", "uint64_t", "1", false, "description");
  EXPECT_TRUE(uint64Param.GetAny(anyValue));

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  EXPECT_TRUE(doubleParam.GetAny(anyValue));

  sdf::Param floatParam("key", "float", "1.0", false, "description");
  EXPECT_TRUE(floatParam.GetAny(anyValue));

  sdf::Param boolParam("key", "bool", "true", false, "description");
  EXPECT_TRUE(boolParam.GetAny(anyValue));

  sdf::Param stringParam("key", "string", "hello", false, "description");
  EXPECT_TRUE(stringParam.GetAny(anyValue));

  sdf::Param unsignedParam("key", "unsigned int", "1", false, "description");
  EXPECT_TRUE(unsignedParam.GetAny(anyValue));

  sdf::Param charParam("key", "char", "a", false, "description");
  EXPECT_TRUE(charParam.GetAny(anyValue));

  sdf::Param timeParam("key", "time", "8 20", false, "description");
  EXPECT_TRUE(timeParam.GetAny(anyValue));

  sdf::Param colorParam("key", "color", "8 20 67 23", false, "description");
  EXPECT_TRUE(colorParam.GetAny(anyValue));

  sdf::Param vector3Param("key", "vector3", "8.1 20.24 67.7", false,
                          "description");
  EXPECT_TRUE(vector3Param.GetAny(anyValue));

  sdf::Param vector2iParam("key", "vector2i", "8 20", false, "description");
  EXPECT_TRUE(vector2iParam.GetAny(anyValue));

  sdf::Param vector2dParam("key", "vector2d", "8.1 20.24", false,
                           "description");
  EXPECT_TRUE(vector2dParam.GetAny(anyValue));

  sdf::Param poseParam("key", "pose", "1 2 3 4 5 6", false, "description");
  EXPECT_TRUE(poseParam.GetAny(anyValue));

  sdf::Param quatParam("key", "quaternion", "1 2 3 4", false, "description");
  EXPECT_TRUE(quatParam.GetAny(anyValue));
}

////////////////////////////////////////////////////
TEST(Param, SetTemplate)
{
  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  double value;

  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 1.0);

  doubleParam.Set<double>(25.456);

  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 25.456);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
