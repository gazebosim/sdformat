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

#include <any>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include <gz/math/Angle.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>

#include "sdf/Exception.hh"
#include "sdf/Element.hh"
#include "sdf/Param.hh"
#include "sdf/parser.hh"

bool check_double(const std::string &num)
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
  EXPECT_TRUE(boolParam.Get<bool>(value));
  EXPECT_TRUE(value);

  EXPECT_TRUE(boolParam.Set(false));
  EXPECT_TRUE(boolParam.Get<bool>(value));
  EXPECT_FALSE(value);

  // String parameter that represents a boolean.
  sdf::Param strParam("key", "string", "true", false, "description");

  EXPECT_TRUE(strParam.Get<bool>(value));
  EXPECT_TRUE(value);

  EXPECT_TRUE(strParam.Set("false"));
  EXPECT_TRUE(strParam.Get<bool>(value));
  EXPECT_FALSE(value);

  EXPECT_TRUE(strParam.Set("1"));
  EXPECT_TRUE(strParam.Get<bool>(value));
  EXPECT_TRUE(value);

  EXPECT_TRUE(strParam.Set("0"));
  EXPECT_TRUE(strParam.Get<bool>(value));
  EXPECT_FALSE(value);

  EXPECT_TRUE(strParam.Set("True"));
  EXPECT_TRUE(strParam.Get<bool>(value));
  EXPECT_TRUE(value);

  EXPECT_TRUE(strParam.Set("TRUE"));
  EXPECT_TRUE(strParam.Get<bool>(value));
  EXPECT_TRUE(value);

  // Anything other than "0", "1", "true", or "false" (any capitalization) will
  // cause Get<bool> to fail.
  EXPECT_TRUE(strParam.Set("%"));
  EXPECT_FALSE(strParam.Get<bool>(value));

  EXPECT_TRUE(boolParam.Set(true));
  std::any anyValue;
  EXPECT_TRUE(boolParam.GetAny(anyValue));
  try
  {
    value = std::any_cast<bool>(anyValue);
  }
  catch(std::bad_any_cast &/*_e*/)
  {
    FAIL();
  }
  EXPECT_TRUE(value);
}

////////////////////////////////////////////////////
/// Test getting and setting a floating point value
TEST(Param, Double)
{
  sdf::Param doubleParam("key", "double", "0.0", false, "description");
  double value = -1.0;
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(0.0, value);

  doubleParam.Set(1.0);
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(1.0, value);

  const double sixteenDigits = 12345678.87654321;
  doubleParam.Set(sixteenDigits);
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(sixteenDigits, value);
}

////////////////////////////////////////////////////
/// Test getting and setting a Vector3d
TEST(Param, Vector3d)
{
  sdf::Param vectorParam("key", "vector3", "0 0 0", false, "description");
  gz::math::Vector3d value{1.0, 2.0, 3.0};
  EXPECT_TRUE(vectorParam.Get<gz::math::Vector3d>(value));
  EXPECT_DOUBLE_EQ(0.0, value.X());
  EXPECT_DOUBLE_EQ(0.0, value.Y());
  EXPECT_DOUBLE_EQ(0.0, value.Z());

  vectorParam.Set(gz::math::Vector3d::One);
  EXPECT_TRUE(vectorParam.Get<gz::math::Vector3d>(value));
  EXPECT_DOUBLE_EQ(1.0, value.X());
  EXPECT_DOUBLE_EQ(1.0, value.Y());
  EXPECT_DOUBLE_EQ(1.0, value.Z());

  const double fifteenDigits = 0.123456789012345;
  const gz::math::Vector3d manyDigits{
      fifteenDigits, 1.0 + fifteenDigits, 2.0 + fifteenDigits};
  vectorParam.Set(manyDigits);
  EXPECT_TRUE(vectorParam.Get<gz::math::Vector3d>(value));
  EXPECT_DOUBLE_EQ(0.0 + fifteenDigits, value.X());
  EXPECT_DOUBLE_EQ(1.0 + fifteenDigits, value.Y());
  EXPECT_DOUBLE_EQ(2.0 + fifteenDigits, value.Z());
}

////////////////////////////////////////////////////
/// Test getting and setting a Pose3d
TEST(Param, Pose3d)
{
  sdf::Param poseParam("key", "pose", "0 0 0 0 0 0", false, "description");
  gz::math::Pose3d value{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
  EXPECT_TRUE(poseParam.Get<gz::math::Pose3d>(value));
  EXPECT_DOUBLE_EQ(0.0, value.Pos().X());
  EXPECT_DOUBLE_EQ(0.0, value.Pos().Y());
  EXPECT_DOUBLE_EQ(0.0, value.Pos().Z());
  EXPECT_DOUBLE_EQ(0.0, value.Rot().Euler().X());
  EXPECT_DOUBLE_EQ(0.0, value.Rot().Euler().Y());
  EXPECT_DOUBLE_EQ(0.0, value.Rot().Euler().Z());

  poseParam.Set(gz::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  EXPECT_TRUE(poseParam.Get<gz::math::Pose3d>(value));
  EXPECT_DOUBLE_EQ(1.0, value.Pos().X());
  EXPECT_DOUBLE_EQ(2.0, value.Pos().Y());
  EXPECT_DOUBLE_EQ(3.0, value.Pos().Z());
  EXPECT_DOUBLE_EQ(0.1, value.Rot().Euler().X());
  EXPECT_DOUBLE_EQ(0.2, value.Rot().Euler().Y());
  EXPECT_DOUBLE_EQ(0.3, value.Rot().Euler().Z());

  const double fifteenDigits = 0.123456789012345;
  const gz::math::Pose3d manyDigits{
      fifteenDigits, 1.0 + fifteenDigits, 2.0 + fifteenDigits,
      0.5 * fifteenDigits, fifteenDigits, 2.0 * fifteenDigits};
  poseParam.Set(manyDigits);
  EXPECT_TRUE(poseParam.Get<gz::math::Pose3d>(value));
  EXPECT_DOUBLE_EQ(0.0 + fifteenDigits, value.Pos().X());
  EXPECT_DOUBLE_EQ(1.0 + fifteenDigits, value.Pos().Y());
  EXPECT_DOUBLE_EQ(2.0 + fifteenDigits, value.Pos().Z());
  EXPECT_DOUBLE_EQ(0.5 * fifteenDigits, value.Rot().Euler().X());
  EXPECT_DOUBLE_EQ(1.0 * fifteenDigits, value.Rot().Euler().Y());
  EXPECT_DOUBLE_EQ(2.0 * fifteenDigits, value.Rot().Euler().Z());
}

////////////////////////////////////////////////////
/// Test decimal number
TEST(SetFromString, Decimals)
{
  ASSERT_TRUE(check_double("0.2345"));
}

////////////////////////////////////////////////////
/// Test setting param as a string but getting it as different type
TEST(Param, StringTypeGet)
{
  sdf::Param stringParam("key", "string", "", false, "description");

  // pose type
  gz::math::Pose3d pose;
  EXPECT_TRUE(stringParam.SetFromString("1 1 1 0 0 0"));
  EXPECT_TRUE(stringParam.Get<gz::math::Pose3d>(pose));
  EXPECT_EQ(gz::math::Pose3d(1, 1, 1, 0, 0, 0), pose);
  EXPECT_EQ("string", stringParam.GetTypeName());

  // color type
  gz::math::Color color;
  EXPECT_TRUE(stringParam.SetFromString("0 0 1 1"));
  EXPECT_TRUE(stringParam.Get<gz::math::Color>(color));
  EXPECT_EQ(gz::math::Color(0, 0, 1, 1), color);
  EXPECT_EQ("string", stringParam.GetTypeName());
}

////////////////////////////////////////////////////
/// Test Inf
TEST(SetFromString, DoublePositiveInf)
{
  ASSERT_TRUE(std::numeric_limits<double>::has_infinity);
  std::vector<std::string> positiveInfStrings{
    "inf", "Inf", "INF", "+inf", "+Inf", "+INF"};
  for (const auto &infString : positiveInfStrings)
  {
    sdf::Param doubleParam("key", "double", "0", false, "description");
    double value = 0.;
    EXPECT_TRUE(doubleParam.SetFromString(infString));
    doubleParam.Get<double>(value);
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), value);

    sdf::Param stringParam("key", "string", "0", false, "description");
    value = 0;
    EXPECT_TRUE(stringParam.SetFromString(infString));
    EXPECT_TRUE(stringParam.Get<double>(value));
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), value);
  }
}

////////////////////////////////////////////////////
/// Test -Inf
TEST(SetFromString, DoubleNegativeInf)
{
  ASSERT_TRUE(std::numeric_limits<double>::is_iec559);
  std::vector<std::string> negativeInfStrings{
    "-inf", "-Inf", "-INF"};
  for (const auto &infString : negativeInfStrings)
  {
    sdf::Param doubleParam("key", "double", "0", false, "description");
    double value = 0.;
    EXPECT_TRUE(doubleParam.SetFromString(infString));
    doubleParam.Get<double>(value);
    EXPECT_DOUBLE_EQ(-std::numeric_limits<double>::infinity(), value);

    sdf::Param stringParam("key", "string", "0", false, "description");
    value = 0;
    EXPECT_TRUE(stringParam.SetFromString(infString));
    EXPECT_TRUE(stringParam.Get<double>(value));
    EXPECT_DOUBLE_EQ(-std::numeric_limits<double>::infinity(), value);
  }
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
// https://github.com/osrf/sdformat/issues/114
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
// https://github.com/osrf/sdformat/issues/114
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
  gz::math::Angle value;
  EXPECT_TRUE(doubleParam.Get<gz::math::Angle>(value));
  EXPECT_DOUBLE_EQ(value.Radian(), 1.0);
}

////////////////////////////////////////////////////
TEST(Param, Vector2i)
{
  sdf::Param vect2iParam("key", "vector2i", "0 0", false, "description");
  gz::math::Vector2i value;

  EXPECT_TRUE(vect2iParam.Get<gz::math::Vector2i>(value));
  EXPECT_EQ(value, gz::math::Vector2i(0, 0));
}

////////////////////////////////////////////////////
TEST(Param, InvalidConstructor)
{
  ASSERT_THROW(sdf::Param badParam("key", "badtype", "0", false, "description"),
               sdf::AssertionInternalError);
}

////////////////////////////////////////////////////
TEST(Param, InvalidColorConstructor)
{
  ASSERT_THROW(sdf::Param("key", "color", "8 20 67 23", false, "description"),
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
  std::any anyValue;

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

  sdf::Param colorParam("key", "color", "0 0.1 0.2 0.3", false, "description");
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

////////////////////////////////////////////////////
TEST(Param, MinMaxViolation)
{
  sdf::Param doubleParam("key", "double", "1.0", false, "0", "10.0",
                         "description");
  {
    double value;
    EXPECT_TRUE(doubleParam.Get<double>(value));
    EXPECT_DOUBLE_EQ(value, 1.0);
  }

  EXPECT_FALSE(doubleParam.Set<double>(-1.));
  EXPECT_FALSE(doubleParam.Set<double>(11.));
  EXPECT_TRUE(doubleParam.Set<double>(5.));

  {
    double value;
    EXPECT_TRUE(doubleParam.Get<double>(value));
    EXPECT_DOUBLE_EQ(value, 5.0);
  }
}

//////////////////////////////////////////////////
TEST(Param, SettingParentElement)
{
  sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  ASSERT_TRUE(doubleParam.SetParentElement(parentElement));

  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  EXPECT_EQ(parentElement, doubleParam.GetParentElement());

  // Set a new parent Element
  sdf::ElementPtr newParentElement = std::make_shared<sdf::Element>();

  ASSERT_TRUE(doubleParam.SetParentElement(newParentElement));
  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  EXPECT_EQ(newParentElement, doubleParam.GetParentElement());

  // Remove the parent Element
  ASSERT_TRUE(doubleParam.SetParentElement(nullptr));
  EXPECT_EQ(nullptr, doubleParam.GetParentElement());
}

//////////////////////////////////////////////////
TEST(Param, CopyConstructor)
{
  sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  ASSERT_TRUE(doubleParam.SetParentElement(parentElement));

  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  EXPECT_EQ(parentElement, doubleParam.GetParentElement());

  sdf::Param newParam(doubleParam);
  ASSERT_NE(nullptr, newParam.GetParentElement());
  EXPECT_EQ(parentElement, newParam.GetParentElement());
}

//////////////////////////////////////////////////
TEST(Param, EqualOperator)
{
  sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  ASSERT_TRUE(doubleParam.SetParentElement(parentElement));

  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  EXPECT_EQ(parentElement, doubleParam.GetParentElement());

  sdf::Param newParam = doubleParam;
  ASSERT_NE(nullptr, newParam.GetParentElement());
  EXPECT_EQ(parentElement, newParam.GetParentElement());
}

//////////////////////////////////////////////////
TEST(Param, DestroyParentElementAfterConstruct)
{
  sdf::ParamPtr param = nullptr;

  {
    sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();

    param = std::make_shared<sdf::Param>(
        "key", "double", "1.0", false, "description");
    ASSERT_TRUE(param->SetParentElement(parentElement));
  }

  ASSERT_NE(nullptr, param);
  EXPECT_EQ(nullptr, param->GetParentElement());
}

//////////////////////////////////////////////////
TEST(Param, ReparsingAfterSetDouble)
{
  sdf::Param doubleParam("key", "double", "1.0", false, "description");

  double value;
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 1.0);

  // Reparsing after setting a value will pass.
  ASSERT_TRUE(doubleParam.Set<double>(5.));
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 5.0);
  EXPECT_TRUE(doubleParam.Reparse());

  // Value will be as expected, as the value was set using the explcit Set
  // function, and no parent element attributes are meant to change the value
  // for type double.
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 5.0);
}

//////////////////////////////////////////////////
TEST(Param, ReparsingAfterSetFromStringDouble)
{
  sdf::Param doubleParam("key", "double", "1.0", false, "description");

  double value;
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 1.0);

  // Reparsing after setting a value from string will pass.
  ASSERT_TRUE(doubleParam.SetFromString("5.0"));
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 5.0);
  EXPECT_TRUE(doubleParam.Reparse());

  // Value will be as expected, as no parent element attributes are meant to
  // change the value for type double.
  EXPECT_TRUE(doubleParam.Get<double>(value));
  EXPECT_DOUBLE_EQ(value, 5.0);
}

/////////////////////////////////////////////////
TEST(Param, ReparsingAfterSetPose)
{
  using Pose = gz::math::Pose3d;

  sdf::Param poseParam("", "pose", "1 2 3 0.4 0.5 0.6", false, "description");
  Pose value;
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), value);

  // Reparsing after setting values will pass. Note that the value was
  // explicitly set using the Set function, reparsing will always yield the same
  // value moving forwards.
  EXPECT_TRUE(poseParam.Set<Pose>(Pose(2, 3, 4, 0.5, 0.6, 0.7)));
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);

  // Setting parent with @degrees as false, value will remain the same.
  ASSERT_TRUE(poseParam.SetParentElement(poseElem));
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // Changing attribute @degrees to true, value remains the same as it was
  // explicitly Set.
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // After reparse, value will still remain the same, as the explicit Set
  // function was used.
  EXPECT_TRUE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // Changing parent @rotation_format to euler_rpy, value remains the same
  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("euler_rpy"));
  EXPECT_TRUE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // Changing parent @rotation_format to quat_xyzw, reparse will pass, but value
  // remains the same as before, as it was explicitly set
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("quat_xyzw"));
  EXPECT_TRUE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // Changing parent @rotation_format to something invalid, reparse will pass,
  // value remains the same as before, as it was explicitly set
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("invalid_format"));
  EXPECT_TRUE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);
}

/////////////////////////////////////////////////
TEST(Param, ReparsingAfterSetFromStringPose)
{
  using Pose = gz::math::Pose3d;

  sdf::Param poseParam("", "pose", "1 2 3 0.4 0.5 0.6", false, "description");
  Pose value;
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), value);

  // Reparsing after setting values will pass. Note that the value was
  // set using SetFromString.
  EXPECT_TRUE(poseParam.SetFromString("2 3 4 0.5 0.6 0.7"));
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0   0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  sdf::ParamPtr degreesAttrib = poseElem->GetAttribute("degrees");
  ASSERT_NE(nullptr, degreesAttrib);

  // Setting parent with @degrees as false, value will remain the same.
  ASSERT_TRUE(poseParam.SetParentElement(poseElem));
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // Changing attribute @degrees to true, value remains the same before
  // reparsing.
  ASSERT_TRUE(degreesAttrib->Set<bool>(true));
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), value);

  // After reparse, rotation values will be parsed as degrees.
  EXPECT_TRUE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, GZ_DTOR(0.5), GZ_DTOR(0.6), GZ_DTOR(0.7)), value);

  // Changing parent @rotation_format to euler_rpy, value remains the same
  sdf::ParamPtr rotationFormatAttrib =
      poseElem->GetAttribute("rotation_format");
  ASSERT_NE(nullptr, rotationFormatAttrib);
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("euler_rpy"));
  EXPECT_TRUE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, GZ_DTOR(0.5), GZ_DTOR(0.6), GZ_DTOR(0.7)), value);

  // Changing parent @rotation_format to quat_xyzw, reparse will fail, value
  // remains the same as before
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("quat_xyzw"));
  EXPECT_FALSE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, GZ_DTOR(0.5), GZ_DTOR(0.6), GZ_DTOR(0.7)), value);

  // Changing parent @rotation_format to something invalid, reparse will fail,
  // value remains the same as before
  ASSERT_TRUE(rotationFormatAttrib->Set<std::string>("invalid_format"));
  EXPECT_FALSE(poseParam.Reparse());
  EXPECT_TRUE(poseParam.Get<Pose>(value));
  EXPECT_EQ(Pose(2, 3, 4, GZ_DTOR(0.5), GZ_DTOR(0.6), GZ_DTOR(0.7)), value);
}

/////////////////////////////////////////////////
TEST(Param, IgnoresParentElementAttribute)
{
  {
    // Without parent
    sdf::Param doubleParam("key", "double", "1.0", false, "description");
    EXPECT_TRUE(doubleParam.IgnoresParentElementAttribute());
  }

  {
    // With parent
    sdf::Param doubleParam("key", "double", "1.0", false, "description");
    sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();
    ASSERT_TRUE(doubleParam.SetParentElement(parentElement));
    EXPECT_FALSE(doubleParam.IgnoresParentElementAttribute());
  }

  {
    // Param from parent element
    sdf::ElementPtr elem(new sdf::Element);
    elem->SetName("double");
    elem->AddValue("double", "0", true);

    sdf::ParamPtr valParam = elem->GetValue();
    ASSERT_NE(nullptr, valParam);
    EXPECT_FALSE(valParam->IgnoresParentElementAttribute());
  }

  {
    // With parent using Set and SetFromString
    sdf::Param doubleParam("key", "double", "1.0", false, "description");
    sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();
    ASSERT_TRUE(doubleParam.SetParentElement(parentElement));
    ASSERT_TRUE(doubleParam.Set<double>(23.4));
    EXPECT_TRUE(doubleParam.IgnoresParentElementAttribute());

    ASSERT_TRUE(doubleParam.SetFromString("34.5"));
    EXPECT_FALSE(doubleParam.IgnoresParentElementAttribute());

    ASSERT_TRUE(doubleParam.SetFromString("45.6", true));
    EXPECT_TRUE(doubleParam.IgnoresParentElementAttribute());

    ASSERT_TRUE(doubleParam.SetFromString("56.7", false));
    EXPECT_FALSE(doubleParam.IgnoresParentElementAttribute());

    ASSERT_TRUE(doubleParam.Set<double>(67.8));
    EXPECT_TRUE(doubleParam.IgnoresParentElementAttribute());
  }
}

/////////////////////////////////////////////////
TEST(Param, WithoutParentElementSetParentElementFail)
{
  using Pose = gz::math::Pose3d;

  sdf::Param poseParam("", "pose", "0 0 0 0 0 0", false, "description");
  EXPECT_TRUE(poseParam.SetFromString("2 3 4 0.5 0.6 0.7"));

  Pose val;
  EXPECT_TRUE(poseParam.Get<Pose>(val));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), val);

  sdf::ElementPtr quatPoseElem(new sdf::Element);
  quatPoseElem->SetName("pose");
  quatPoseElem->AddValue("pose", "0 0 0 0 0 0", true);
  quatPoseElem->AddAttribute("relative_to", "string", "", false);
  quatPoseElem->AddAttribute("degrees", "bool", "false", false);
  quatPoseElem->AddAttribute(
      "rotation_format", "string", "quat_xyzw", false);

  // Set parent to Element with degrees attribute false, in quat_xyzw, will
  // fail, as the string that was previously set was only 6 values. The value
  // will remain the same as before and the parent Element will still be the
  // previous one, in this case a nullptr.
  ASSERT_FALSE(poseParam.SetParentElement(quatPoseElem));
  ASSERT_TRUE(poseParam.Get<Pose>(val));
  EXPECT_EQ(Pose(2, 3, 4, 0.5, 0.6, 0.7), val);

  auto parent = poseParam.GetParentElement();
  EXPECT_EQ(nullptr, parent);
}

/////////////////////////////////////////////////
TEST(Param, ChangeParentElementFail)
{
  using Pose = gz::math::Pose3d;

  sdf::ElementPtr poseElem(new sdf::Element);
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "0 0 0 0 0 0", true);
  poseElem->AddAttribute("relative_to", "string", "", false);
  poseElem->AddAttribute("degrees", "bool", "false", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  // Param from original default attibute
  sdf::ParamPtr valParam = poseElem->GetValue();
  ASSERT_NE(nullptr, valParam);
  ASSERT_TRUE(valParam->SetFromString("1, 2, 3, 0.4, 0.5, 0.6"));

  Pose val;
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  sdf::ElementPtr quatPoseElem(new sdf::Element);
  quatPoseElem->SetName("pose");
  quatPoseElem->AddValue("pose", "0 0 0   0 0 0", true);
  quatPoseElem->AddAttribute("relative_to", "string", "", false);
  quatPoseElem->AddAttribute("degrees", "bool", "false", false);
  quatPoseElem->AddAttribute(
      "rotation_format", "string", "quat_xyzw", false);

  // Set parent to Element with degrees attribute false, in quat_xyzw, will
  // fail, as the string that was previously set was only 6 values. The value
  // will remain the same as before and the parent Element will still be the
  // previous one.
  ASSERT_FALSE(valParam->SetParentElement(quatPoseElem));
  ASSERT_TRUE(valParam->Get<Pose>(val));
  EXPECT_EQ(Pose(1, 2, 3, 0.4, 0.5, 0.6), val);

  auto parent = valParam->GetParentElement();
  EXPECT_EQ(parent, poseElem);
}

/////////////////////////////////////////////////
TEST(Param, PoseWithDefaultValue)
{
  using Pose = gz::math::Pose3d;

  auto poseElem = std::make_shared<sdf::Element>();
  poseElem->SetName("pose");
  poseElem->AddValue("pose", "1 0 0 0 0 0", false);
  poseElem->AddAttribute("rotation_format", "string", "euler_rpy", false);

  // Clone poseElem for testing Parm::SetFromString
  auto poseElemClone = poseElem->Clone();

  const std::string testString = R"(
  <sdf version="1.9">
    <pose rotation_format="quat_xyzw"/>
  </sdf>)";

  sdf::Errors errors;
  EXPECT_TRUE(sdf::readString(testString, poseElem, errors));
  EXPECT_TRUE(errors.empty()) << errors;

  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), poseElem->Get<Pose>());
  const std::string expectedString =
      "<pose rotation_format='quat_xyzw'>1 0 0   0 0 0 1</pose>\n";
  EXPECT_STREQ(expectedString.c_str(), poseElem->ToString("").c_str());

  // same test but using Param::SetFromString
  poseElemClone->GetAttribute("rotation_format")->SetFromString("quat_xyzw");
  auto value = poseElemClone->GetValue();
  value->SetFromString("");
  value->Reparse();

  EXPECT_EQ(Pose(1, 0, 0, 0, 0, 0), poseElemClone->Get<Pose>());
  EXPECT_STREQ(expectedString.c_str(), poseElemClone->ToString("").c_str());
}
